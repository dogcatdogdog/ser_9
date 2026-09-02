//! HTTP 层集成测试 (W7) — tokio 起真实服务 + reqwest POST /plan
//!
//! 1. JSON 往返: 正常请求 → 200 + 合法 RoutePlanResp
//! 2. 错误码: BAD_REQUEST (400) / 非法 JSON
//! 3. golden: Python 固定种子输出 (solver_golden.json) → Rust 服务响应逐字段比对
//!    (sequence 精确; 浮点字段容差 0.02, 容忍 round2 舍入方向差异)

use a3_rust::dto::RoutePlanResp;
use a3_rust::http::app;
use axum::serve;
use tokio::net::TcpListener;

/// 起真实 TCP 服务 (随机端口), 返回 base url
async fn spawn_server() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").await.expect("bind");
    let addr = listener.local_addr().expect("local_addr");
    tokio::spawn(async move {
        serve(listener, app()).await.expect("serve");
    });
    format!("http://{addr}")
}

/// 正常请求 → 200 + 合法响应 (JSON 往返)
#[tokio::test]
async fn post_plan_roundtrip_returns_200() {
    let base = spawn_server().await;
    let client = reqwest::Client::new();
    let body = r#"{
        "targets": [
            {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0},
            {"id": "c2", "location": {"x": 0.0, "y": 100.0}, "demand": 5.0},
            {"id": "c3", "location": {"x": 100.0, "y": 100.0}, "demand": 5.0}
        ],
        "home": {"x": 0.0, "y": 0.0},
        "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                   "alpha": 0.1, "beta": 0.005}
    }"#;
    let resp = client
        .post(format!("{base}/plan"))
        .header("content-type", "application/json")
        .body(body)
        .send()
        .await
        .expect("send");
    assert_eq!(resp.status(), 200);
    let plan: RoutePlanResp = resp.json().await.expect("parse");
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 3);
    assert_eq!(plan.segments.len(), 4); // home→c?→...→home
    assert!(plan.total_equiv_distance > 0.0);
}

/// 空 targets → 400 BAD_REQUEST (错误码契约)
#[tokio::test]
async fn post_plan_empty_targets_returns_400() {
    let base = spawn_server().await;
    let client = reqwest::Client::new();
    let body = r#"{
        "targets": [],
        "home": {"x": 0.0, "y": 0.0},
        "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                   "alpha": 0.1, "beta": 0.005}
    }"#;
    let resp = client
        .post(format!("{base}/plan"))
        .header("content-type", "application/json")
        .body(body)
        .send()
        .await
        .expect("send");
    assert_eq!(resp.status(), 400);
    let err: serde_json::Value = resp.json().await.expect("parse");
    assert_eq!(err["code"], "BAD_REQUEST");
}

/// 非法 JSON → 400
#[tokio::test]
async fn post_plan_malformed_json_returns_400() {
    let base = spawn_server().await;
    let client = reqwest::Client::new();
    let resp = client
        .post(format!("{base}/plan"))
        .header("content-type", "application/json")
        .body(r#"{"targets": [broken"#)
        .send()
        .await
        .expect("send");
    assert_eq!(resp.status(), 400);
}

// ====================================================================
// golden: Python 固定种子输出 vs Rust 服务响应 (DEVPLAN 强制对齐)
// ====================================================================

#[derive(serde::Deserialize)]
struct Golden {
    instances: Vec<GoldenInstance>,
}

#[derive(serde::Deserialize)]
struct GoldenInstance {
    fixture: String,
    home: serde_json::Value,
    drone: serde_json::Value,
    targets: Vec<serde_json::Value>,
    expected: GoldenExpected,
}

#[derive(serde::Deserialize)]
struct GoldenExpected {
    sequence: Vec<String>,
    segments: Vec<serde_json::Value>,
    total_geo_distance: f64,
    total_equiv_distance: f64,
    total_energy_consumed: f64,
    remaining_energy: f64,
    total_payload_delivered: f64,
    feasible: bool,
    warnings: Vec<String>,
}

fn load_golden() -> Golden {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/solver_golden.json"
    );
    let raw = std::fs::read_to_string(path)
        .expect("golden 缺失 — 先运行 scripts/gen_solver_golden.py");
    serde_json::from_str(&raw).expect("golden 解析失败")
}

/// 逐字段比对 6 个 fixture 的 Python 输出 vs Rust HTTP 响应
#[tokio::test]
async fn golden_matches_python_output() {
    let base = spawn_server().await;
    let client = reqwest::Client::new();
    let golden = load_golden();

    // 浮点容差: round2 半值舍入方向差异 ≤ 0.01/元素
    const TOL: f64 = 0.02;

    for inst in &golden.instances {
        let mut req = serde_json::Map::new();
        req.insert("home".into(), inst.home.clone());
        req.insert("drone".into(), inst.drone.clone());
        req.insert("targets".into(), serde_json::Value::Array(inst.targets.clone()));

        let resp = client
            .post(format!("{base}/plan"))
            .header("content-type", "application/json")
            .json(&serde_json::Value::Object(req))
            .send()
            .await
            .expect("send");
        assert_eq!(resp.status(), 200, "{}: 应返回 200", inst.fixture);
        let plan: RoutePlanResp = resp.json().await.expect("parse");

        // sequence: 精确相等
        assert_eq!(plan.sequence, inst.expected.sequence, "{}: sequence", inst.fixture);

        // 浮点总量: 容差比对
        let feq = |a: f64, b: f64, what: &str| {
            assert!(
                (a - b).abs() < TOL,
                "{}: {what} rust={a:.4} python={b:.4}",
                inst.fixture
            );
        };
        feq(plan.total_geo_distance, inst.expected.total_geo_distance, "total_geo_distance");
        feq(plan.total_equiv_distance, inst.expected.total_equiv_distance, "total_equiv_distance");
        feq(plan.total_energy_consumed, inst.expected.total_energy_consumed, "total_energy_consumed");
        feq(plan.remaining_energy, inst.expected.remaining_energy, "remaining_energy");
        feq(plan.total_payload_delivered, inst.expected.total_payload_delivered, "total_payload_delivered");

        // feasible / warnings 数量
        assert_eq!(plan.feasible, inst.expected.feasible, "{}: feasible", inst.fixture);
        assert_eq!(plan.warnings.len(), inst.expected.warnings.len(), "{}: warnings len", inst.fixture);

        // segments: 逐字段容差比对
        assert_eq!(plan.segments.len(), inst.expected.segments.len(), "{}: segments len", inst.fixture);
        for (idx, (rust_seg, py_seg)) in
            plan.segments.iter().zip(inst.expected.segments.iter()).enumerate()
        {
            assert_eq!(rust_seg.from_id, py_seg["from_id"].as_str().unwrap(), "{}: seg{idx} from_id", inst.fixture);
            assert_eq!(rust_seg.to_id, py_seg["to_id"].as_str().unwrap(), "{}: seg{idx} to_id", inst.fixture);
            for field in [
                "geo_distance",
                "equiv_distance",
                "energy_consumed",
                "payload_before",
                "payload_after",
                "battery_before",
                "battery_after",
            ] {
                let r = match field {
                    "geo_distance" => rust_seg.geo_distance,
                    "equiv_distance" => rust_seg.equiv_distance,
                    "energy_consumed" => rust_seg.energy_consumed,
                    "payload_before" => rust_seg.payload_before,
                    "payload_after" => rust_seg.payload_after,
                    "battery_before" => rust_seg.battery_before,
                    _ => rust_seg.battery_after,
                };
                let p = py_seg[field].as_f64().expect("golden f64");
                assert!(
                    (r - p).abs() < TOL,
                    "{}: seg{idx} {field} rust={r:.4} python={p:.4}",
                    inst.fixture
                );
            }
        }
    }
}

// ====================================================================
// W8 硬化验证: 并发请求 (spawn_blocking 隔离后应有真实并发能力)
// ====================================================================

/// 10 并发请求 (20 点 Solomon 实例) → 全部 200 + 结果一致 (确定性)
#[tokio::test]
async fn concurrent_requests_all_succeed() {
    let base = spawn_server().await;
    let golden = load_golden();
    // 选最重的实例 (solomon_rc101_n20) 最大化并发下的计算负载
    let inst = golden
        .instances
        .iter()
        .find(|i| i.fixture == "solomon_rc101_n20.json")
        .expect("rc101 in golden");
    let mut req = serde_json::Map::new();
    req.insert("home".into(), inst.home.clone());
    req.insert("drone".into(), inst.drone.clone());
    req.insert("targets".into(), serde_json::Value::Array(inst.targets.clone()));
    let body = serde_json::Value::Object(req).to_string();

    let client = reqwest::Client::new();
    let mut handles = Vec::new();
    for _ in 0..10 {
        let client = client.clone();
        let url = format!("{base}/plan");
        let body = body.clone();
        handles.push(tokio::spawn(async move {
            client
                .post(&url)
                .header("content-type", "application/json")
                .body(body)
                .send()
                .await
        }));
    }

    for (idx, h) in handles.into_iter().enumerate() {
        let resp = h.await.expect("task join").expect("send");
        assert_eq!(resp.status(), 200, "并发请求 {idx} 应成功");
        let plan: RoutePlanResp = resp.json().await.expect("parse");
        assert_eq!(plan.sequence.len(), 20, "并发请求 {idx} 应覆盖全部 20 点");
        assert_eq!(
            plan.sequence,
            inst.expected.sequence,
            "并发请求 {idx} 结果应与串行一致 (确定性)"
        );
    }
}

/// 并发中的非法请求 (alpha=0) → 400, 不影响其他请求
#[tokio::test]
async fn concurrent_bad_request_isolated() {
    let base = spawn_server().await;
    let client = reqwest::Client::new();
    let good = r#"{
        "targets": [{"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0}],
        "home": {"x": 0.0, "y": 0.0},
        "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
                   "alpha": 0.1, "beta": 0.005}
    }"#;
    let bad = good.replace(r#""alpha": 0.1"#, r#""alpha": 0.0"#);

    let mut handles = Vec::new();
    for _ in 0..5 {
        let (client, good, bad, url) = (
            client.clone(),
            good.to_string(),
            bad.to_string(),
            format!("{base}/plan"),
        );
        handles.push(tokio::spawn(async move {
            let g = client
                .post(&url)
                .header("content-type", "application/json")
                .body(good)
                .send()
                .await
                .expect("good send");
            let b = client
                .post(&url)
                .header("content-type", "application/json")
                .body(bad)
                .send()
                .await
                .expect("bad send");
            (g.status(), b.status())
        }));
    }
    for h in handles {
        let (good_status, bad_status) = h.await.expect("task join");
        assert_eq!(good_status, 200, "好请求应成功");
        assert_eq!(bad_status, 400, "坏请求应 400 (校验隔离)");
    }
}
