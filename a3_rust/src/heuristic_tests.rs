//! heuristic 单测 1:1 移植 — 对齐 `a3_python/tests/test_heuristic.py` (54 例)
//!
//! 跳过 Python 的 `TestW5FullEval` 4 例 (full_eval 是 W5 benchmark 材料,
//! Rust 生产路径只实现增量评估), 移植 50 例。

use std::collections::{HashMap, HashSet};

use crate::dto::{DroneSpecDto, GeoPointDto, RoutePlanResp, TargetDto};
use crate::energy::simulate_route_energy;
use crate::heuristic::{
    build_route_plan, construct_nn, construct_savings, local_search_2opt,
    local_search_or_opt, local_search_vnd, segment_payloads, try_2opt_move,
    try_or_opt_move,
};

// ====================================================================
// 辅助函数
// ====================================================================

fn make_targets(coords_demands: &[(f64, f64, f64)]) -> Vec<TargetDto> {
    coords_demands
        .iter()
        .enumerate()
        .map(|(i, (x, y, d))| TargetDto {
            id: format!("c{}", i + 1),
            location: GeoPointDto { x: *x, y: *y },
            demand: *d,
            ..Default::default()
        })
        .collect()
}

fn make_map(targets: &[TargetDto]) -> HashMap<String, &TargetDto> {
    targets.iter().map(|t| (t.id.clone(), t)).collect()
}

/// id 列表 → Vec<String> (对齐 Python sequence: list[str])
fn seq(ids: &[&str]) -> Vec<String> {
    ids.iter().map(|s| s.to_string()).collect()
}

fn drone(payload: f64, battery: f64, alpha: f64, beta: f64) -> DroneSpecDto {
    DroneSpecDto {
        payload_capacity: payload,
        battery_capacity: battery,
        alpha,
        beta,
        ..Default::default()
    }
}

const HOME: GeoPointDto = GeoPointDto { x: 0.0, y: 0.0 };

/// 验证 RoutePlan 满足载重约束和电量约束 (对齐 Python `_verify_constraints`)
fn verify_constraints(plan: &RoutePlanResp, capacity: f64, alpha: f64, beta: f64) {
    for seg in &plan.segments {
        assert!(seg.payload_before <= capacity, "Payload {} exceeds capacity {capacity}", seg.payload_before);
        assert!(seg.payload_before >= 0.0, "Negative payload {}", seg.payload_before);
        assert!(seg.battery_after >= 0.0, "Battery exhausted at {}->{}: {}", seg.from_id, seg.to_id, seg.battery_after);
        let expected_energy = seg.geo_distance * (alpha + beta * seg.payload_before);
        assert!(
            (seg.energy_consumed - expected_energy).abs() < 0.01,
            "Energy mismatch: {} vs expected {expected_energy}",
            seg.energy_consumed
        );
    }
}

fn verify_all(plan: &RoutePlanResp, targets: &[TargetDto], drone: &DroneSpecDto) {
    verify_constraints(plan, drone.payload_capacity, drone.alpha, drone.beta);
    let ids: HashSet<&str> = targets.iter().map(|t| t.id.as_str()).collect();
    let visited: HashSet<&str> = plan.sequence.iter().map(|s| s.as_str()).collect();
    assert_eq!(visited, ids, "All targets must be visited");
}

/// fixture 加载 (扁平格式: home{x,y}, targets[{id,x,y,demand}])
/// 对齐 a3_python/tests/fixtures/*.json
fn load_fixture(name: &str) -> (GeoPointDto, Vec<TargetDto>) {
    #[derive(serde::Deserialize)]
    struct FxPoint {
        x: f64,
        y: f64,
    }
    #[derive(serde::Deserialize)]
    struct FxTarget {
        id: String,
        x: f64,
        y: f64,
        demand: f64,
    }
    #[derive(serde::Deserialize)]
    struct FxData {
        home: FxPoint,
        targets: Vec<FxTarget>,
    }
    let path = format!(
        "{}/../a3_python/tests/fixtures/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let raw = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("fixture {name} 加载失败: {e}"));
    let data: FxData = serde_json::from_str(&raw).expect("fixture JSON 解析失败");
    let home = GeoPointDto { x: data.home.x, y: data.home.y };
    let targets = data
        .targets
        .into_iter()
        .map(|t| TargetDto {
            id: t.id,
            location: GeoPointDto { x: t.x, y: t.y },
            demand: t.demand,
            ..Default::default()
        })
        .collect();
    (home, targets)
}

// ====================================================================
// 正例 (positive) — 正常场景, 应找到可行解
// ====================================================================

#[test]
fn nn_three_points_line() {
    let targets = make_targets(&[(100.0, 0.0, 5.0), (200.0, 0.0, 5.0), (300.0, 0.0, 5.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible, "Expected feasible, got warnings: {:?}", plan.warnings);
    assert_eq!(plan.sequence.len(), 3);
    verify_all(&plan, &targets, &drone);
}

#[test]
fn nn_five_points_scattered() {
    let targets = make_targets(&[
        (100.0, 0.0, 3.0), (50.0, 80.0, 4.0), (200.0, 50.0, 3.0),
        (150.0, 200.0, 5.0), (300.0, 100.0, 2.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 5);
    verify_all(&plan, &targets, &drone);
}

#[test]
fn nn_all_points_visited() {
    let targets = make_targets(&[(100.0, 0.0, 2.0), (200.0, 0.0, 2.0), (50.0, 100.0, 2.0)]);
    let drone = drone(20.0, 5000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
    verify_all(&plan, &targets, &drone);
}

#[test]
fn nn_single_point() {
    let targets = make_targets(&[(100.0, 0.0, 5.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence, vec!["c1".to_string()]);
    verify_constraints(&plan, drone.payload_capacity, drone.alpha, drone.beta);
}

#[test]
fn nn_improves_over_input_order() {
    let targets = make_targets(&[
        (500.0, 500.0, 1.0), (100.0, 100.0, 1.0), (400.0, 400.0, 1.0),
        (200.0, 200.0, 1.0), (300.0, 300.0, 1.0),
    ]);
    let drone = drone(20.0, 20000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
    // NN 应产生比输入顺序更好的路线
    let map = make_map(&targets);
    let bad_seq = seq(&["c1", "c2", "c3", "c4", "c5"]);
    let sim = simulate_route_energy(&bad_seq, &map, &HOME, &drone);
    assert!(
        plan.total_equiv_distance <= sim.total_equiv,
        "NN ({}) should not be worse than input order ({})",
        plan.total_equiv_distance,
        sim.total_equiv
    );
}

#[test]
fn savings_three_points_line() {
    let targets = make_targets(&[(100.0, 0.0, 5.0), (200.0, 0.0, 5.0), (300.0, 0.0, 5.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_savings(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 3);
    verify_constraints(&plan, drone.payload_capacity, drone.alpha, drone.beta);
}

#[test]
fn savings_five_points() {
    let targets = make_targets(&[
        (100.0, 0.0, 3.0), (50.0, 80.0, 4.0), (200.0, 50.0, 3.0),
        (150.0, 200.0, 5.0), (300.0, 100.0, 2.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let plan = construct_savings(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 5);
    verify_constraints(&plan, drone.payload_capacity, drone.alpha, drone.beta);
}

#[test]
fn savings_single_point() {
    let targets = make_targets(&[(100.0, 0.0, 5.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_savings(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence, vec!["c1".to_string()]);
}

// ====================================================================
// 退化 (degenerate) — 不可行场景
// ====================================================================

#[test]
fn nn_over_capacity() {
    let targets = make_targets(&[(100.0, 0.0, 30.0), (200.0, 0.0, 30.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005); // capacity=50, demand=60
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(!plan.feasible);
    assert!(plan.warnings[0].to_lowercase().contains("capacity"));
}

#[test]
#[should_panic(expected = "cannot be empty")]
fn nn_empty_targets() {
    construct_nn(&[], &HOME, &drone(50.0, 5000.0, 0.1, 0.005));
}

#[test]
fn nn_tight_battery_partial() {
    // c1 很近, c2 很远 → 至少访问了近点
    let targets = make_targets(&[(100.0, 0.0, 1.0), (50000.0, 0.0, 1.0)]);
    let drone = drone(10.0, 500.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.sequence.contains(&"c1".to_string()));
}

#[test]
fn savings_over_capacity() {
    let targets = make_targets(&[(100.0, 0.0, 30.0), (200.0, 0.0, 30.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_savings(&targets, &HOME, &drone);
    assert!(!plan.feasible);
    assert!(plan.warnings[0].to_lowercase().contains("capacity"));
}

#[test]
#[should_panic(expected = "cannot be empty")]
fn savings_empty_targets() {
    construct_savings(&[], &HOME, &drone(50.0, 5000.0, 0.1, 0.005));
}

// ====================================================================
// 边界 (boundary)
// ====================================================================

#[test]
fn nn_zero_demand() {
    // 巡检场景: 所有点 demand=0 → 等效距离 = 几何距离
    let targets = make_targets(&[(100.0, 0.0, 0.0), (200.0, 0.0, 0.0), (300.0, 0.0, 0.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 3);
    for seg in &plan.segments {
        assert!(
            (seg.geo_distance - seg.equiv_distance).abs() < 0.01,
            "Zero demand should make equiv=geo, got {} vs {}",
            seg.equiv_distance,
            seg.geo_distance
        );
    }
}

#[test]
fn max_capacity_exact_match() {
    // 载重恰好等于 capacity → 应可行
    let targets = make_targets(&[(100.0, 0.0, 25.0), (200.0, 0.0, 25.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
}

#[test]
fn nn_same_location_points() {
    // 两个点在同一位置 → 应正确处理
    let targets = vec![
        TargetDto { id: "c1".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 5.0, ..Default::default() },
        TargetDto { id: "c2".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 5.0, ..Default::default() },
    ];
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let plan = construct_nn(&targets, &HOME, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 2);
}

// ====================================================================
// 一致性 (consistency)
// ====================================================================

#[test]
fn nn_deterministic() {
    let targets = make_targets(&[
        (100.0, 0.0, 3.0), (50.0, 80.0, 4.0), (200.0, 50.0, 3.0),
        (150.0, 200.0, 5.0), (300.0, 100.0, 2.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let p1 = construct_nn(&targets, &HOME, &drone);
    let p2 = construct_nn(&targets, &HOME, &drone);
    assert_eq!(p1.sequence, p2.sequence);
    assert_eq!(p1.total_equiv_distance, p2.total_equiv_distance);
    assert_eq!(p1.feasible, p2.feasible);
}

#[test]
fn savings_deterministic() {
    let targets = make_targets(&[(100.0, 0.0, 3.0), (50.0, 80.0, 4.0), (200.0, 50.0, 3.0)]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let p1 = construct_savings(&targets, &HOME, &drone);
    let p2 = construct_savings(&targets, &HOME, &drone);
    assert_eq!(p1.sequence, p2.sequence);
    assert_eq!(p1.total_equiv_distance, p2.total_equiv_distance);
}

/// 在 fixture 数据上验证所有约束 (NN)
#[test]
fn nn_constraints_satisfied_on_fixtures() {
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    for name in ["custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json"] {
        let (home, targets) = load_fixture(name);
        let plan = construct_nn(&targets, &home, &drone);
        if plan.feasible {
            verify_constraints(&plan, drone.payload_capacity, drone.alpha, drone.beta);
        }
    }
}

/// 在 fixture 数据上验证所有约束 (Savings)
#[test]
fn savings_constraints_satisfied_on_fixtures() {
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    for name in ["custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json"] {
        let (home, targets) = load_fixture(name);
        let plan = construct_savings(&targets, &home, &drone);
        if plan.feasible {
            verify_constraints(&plan, drone.payload_capacity, drone.alpha, drone.beta);
        }
    }
}

// ====================================================================
// 回归 (regression) — 锁定已知预期值
// ====================================================================

#[test]
fn nn_custom_5_heavy_expected_value() {
    let (home, targets) = load_fixture("custom_5_heavy.json");
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    let plan = construct_nn(&targets, &home, &drone);
    assert!(plan.feasible, "Expected feasible but got: {:?}", plan.warnings);
    assert_eq!(plan.sequence.len(), 5);
    // 锁定值: 当前实现产出 (2026-08-05), 与 Python 断言一致
    assert!(
        (plan.total_equiv_distance - 1051.5).abs() / 1051.5 < 0.01,
        "equiv={}",
        plan.total_equiv_distance
    );
}

#[test]
fn nn_custom_15_mixed_all_visited() {
    let (home, targets) = load_fixture("custom_15_mixed.json");
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    let plan = construct_nn(&targets, &home, &drone);
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 15);
}

/// 在所有 6 个实例上, NN 的 total_equiv_dist <= Savings (至少不差)
#[test]
fn nn_vs_savings_nn_wins() {
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    let fixtures = [
        "custom_5_heavy.json",
        "custom_10_tight.json",
        "custom_15_mixed.json",
        "solomon_r101_n20.json",
        "solomon_c101_n20.json",
        "solomon_rc101_n20.json",
    ];
    for name in fixtures {
        let (home, targets) = load_fixture(name);
        let nn = construct_nn(&targets, &home, &drone);
        let sv = construct_savings(&targets, &home, &drone);
        if nn.feasible && sv.feasible {
            assert!(
                nn.total_equiv_distance <= sv.total_equiv_distance,
                "{name}: NN ({}) should not be worse than Savings ({})",
                nn.total_equiv_distance,
                sv.total_equiv_distance
            );
        }
    }
}

// ====================================================================
// 增量评估辅助函数
// ====================================================================

#[test]
fn segment_payloads_correct() {
    let targets = vec![
        TargetDto { id: "c1".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 3.0, ..Default::default() },
        TargetDto { id: "c2".into(), location: GeoPointDto { x: 200.0, y: 0.0 }, demand: 5.0, ..Default::default() },
        TargetDto { id: "c3".into(), location: GeoPointDto { x: 300.0, y: 0.0 }, demand: 2.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let payloads = segment_payloads(&seq(&["c1", "c2", "c3"]), &map, 10.0);
    // full_path: home→c1→c2→c3→home, 4 segments
    assert_eq!(payloads.len(), 4);
    assert_eq!(payloads[0], 10.0); // home→c1: full load
    assert_eq!(payloads[1], 7.0); // c1→c2: after delivering c1 (3)
    assert_eq!(payloads[2], 2.0); // c2→c3: after delivering c2 (5)
    assert_eq!(payloads[3], 0.0); // c3→home: all delivered
}

#[test]
fn segment_payloads_zero_demand() {
    let targets = vec![
        TargetDto { id: "c1".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 0.0, ..Default::default() },
        TargetDto { id: "c2".into(), location: GeoPointDto { x: 200.0, y: 0.0 }, demand: 0.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let payloads = segment_payloads(&seq(&["c1", "c2"]), &map, 0.0);
    assert_eq!(payloads, vec![0.0, 0.0, 0.0]);
}

#[test]
fn try_2opt_move_finds_improvement() {
    let targets = vec![
        TargetDto { id: "A".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "B".into(), location: GeoPointDto { x: 200.0, y: 100.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "C".into(), location: GeoPointDto { x: 100.0, y: 100.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "D".into(), location: GeoPointDto { x: 200.0, y: 0.0 }, demand: 1.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let sequence = seq(&["A", "C", "B", "D"]);
    let current_equiv = 650.0;

    let result = try_2opt_move(&sequence, 0, 3, &map, &HOME, &drone, 4.0, current_equiv);
    assert!(result.is_some());
    let (new_seq, new_equiv) = result.unwrap();
    assert!(new_equiv < current_equiv);
    assert_eq!(new_seq.len(), 4);
    let set: HashSet<&str> = new_seq.iter().map(|s| s.as_str()).collect();
    assert_eq!(set, ["A", "B", "C", "D"].into_iter().collect());
}

#[test]
fn try_2opt_move_rejects_worse_move() {
    let targets = vec![
        TargetDto { id: "A".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "B".into(), location: GeoPointDto { x: 200.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "C".into(), location: GeoPointDto { x: 300.0, y: 0.0 }, demand: 1.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    // On a line: A→B→C is already optimal
    let sequence = seq(&["A", "B", "C"]);
    let sim = simulate_route_energy(&sequence, &map, &HOME, &drone);

    let result = try_2opt_move(&sequence, 0, 2, &map, &HOME, &drone, 3.0, sim.total_equiv);
    assert!(result.is_none());
}

#[test]
fn try_2opt_move_rejects_battery_infeasible() {
    let targets = vec![
        TargetDto { id: "A".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "B".into(), location: GeoPointDto { x: 10000.0, y: 0.0 }, demand: 1.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let drone = drone(50.0, 500.0, 0.1, 0.005); // 低电池
    let sequence = seq(&["A", "B"]);
    let sim = simulate_route_energy(&sequence, &map, &HOME, &drone);

    let result = try_2opt_move(&sequence, 0, 1, &map, &HOME, &drone, 2.0, sim.total_equiv);
    if let Some((new_seq, _)) = result {
        let sim2 = simulate_route_energy(&new_seq, &map, &HOME, &drone);
        assert!(sim2.feasible);
    }
}

// ====================================================================
// 2-opt 搜索
// ====================================================================

#[test]
fn opt2_improves_crossing_route() {
    let targets = make_targets(&[
        (100.0, 0.0, 1.0), (200.0, 100.0, 1.0), (100.0, 100.0, 1.0), (200.0, 0.0, 1.0),
    ]);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    assert!(nn.feasible);
    let map = make_map(&targets);
    let opt2 = local_search_2opt(&nn, &map, &HOME, &drone, 100);
    assert!(opt2.feasible);
    assert_eq!(opt2.sequence.len(), targets.len());
    verify_all(&opt2, &targets, &drone);
    // 2-opt should not make things worse
    assert!(opt2.total_equiv_distance <= nn.total_equiv_distance + 0.01);
}

#[test]
fn opt2_improves_scattered_points() {
    let targets = make_targets(&[
        (500.0, 500.0, 1.0), (100.0, 100.0, 1.0), (400.0, 400.0, 1.0),
        (200.0, 200.0, 1.0), (300.0, 300.0, 1.0),
    ]);
    let drone = drone(20.0, 20000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let opt2 = local_search_2opt(&nn, &map, &HOME, &drone, 100);
    assert!(opt2.feasible);
    verify_all(&opt2, &targets, &drone);
    assert!(opt2.total_equiv_distance <= nn.total_equiv_distance + 0.01);
}

#[test]
fn opt2_preserves_all_points() {
    let targets = make_targets(&[
        (100.0, 0.0, 2.0), (50.0, 80.0, 3.0), (200.0, 50.0, 2.0),
        (150.0, 200.0, 4.0), (300.0, 100.0, 1.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let opt2 = local_search_2opt(&nn, &map, &HOME, &drone, 100);
    assert_eq!(opt2.sequence.len(), targets.len());
    let a: HashSet<&str> = opt2.sequence.iter().map(|s| s.as_str()).collect();
    let b: HashSet<&str> = nn.sequence.iter().map(|s| s.as_str()).collect();
    assert_eq!(a, b);
}

#[test]
fn opt2_empty_route() {
    let route = RoutePlanResp {
        sequence: Vec::new(),
        segments: Vec::new(),
        total_geo_distance: 0.0,
        total_equiv_distance: 0.0,
        total_energy_consumed: 0.0,
        remaining_energy: 1000.0,
        total_payload_delivered: 0.0,
        feasible: false,
        warnings: Vec::new(),
    };
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let map: HashMap<String, &TargetDto> = HashMap::new();
    let result = local_search_2opt(&route, &map, &HOME, &drone, 100);
    assert!(result.sequence.is_empty());
    assert_eq!(result.total_equiv_distance, 0.0);
}

#[test]
fn opt2_single_point() {
    let targets = make_targets(&[(100.0, 0.0, 5.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let opt2 = local_search_2opt(&nn, &map, &HOME, &drone, 100);
    assert_eq!(opt2.sequence, nn.sequence);
    assert_eq!(opt2.total_equiv_distance, nn.total_equiv_distance);
}

#[test]
fn opt2_max_iterations_exhausted() {
    let targets = make_targets(&[
        (100.0, 0.0, 1.0), (200.0, 100.0, 1.0), (100.0, 100.0, 1.0), (200.0, 0.0, 1.0),
    ]);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let opt2 = local_search_2opt(&nn, &map, &HOME, &drone, 1);
    assert!(opt2.feasible);
}

// ====================================================================
// Or-opt 搜索
// ====================================================================

#[test]
fn or_opt_improves_suboptimal_route() {
    let targets = make_targets(&[
        (100.0, 0.0, 1.0), (200.0, 100.0, 1.0), (100.0, 100.0, 1.0),
        (200.0, 0.0, 1.0), (300.0, 50.0, 1.0),
    ]);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let or_opt = local_search_or_opt(&nn, &map, &HOME, &drone, 3, 100);
    assert!(or_opt.feasible);
    verify_all(&or_opt, &targets, &drone);
    assert!(or_opt.total_equiv_distance <= nn.total_equiv_distance + 0.01);
}

#[test]
fn or_opt_preserves_all_points() {
    let targets = make_targets(&[
        (100.0, 0.0, 2.0), (200.0, 50.0, 3.0), (50.0, 80.0, 2.0), (150.0, 200.0, 1.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let or_opt = local_search_or_opt(&nn, &map, &HOME, &drone, 3, 100);
    assert_eq!(or_opt.sequence.len(), targets.len());
    let a: HashSet<&str> = or_opt.sequence.iter().map(|s| s.as_str()).collect();
    let b: HashSet<&str> = nn.sequence.iter().map(|s| s.as_str()).collect();
    assert_eq!(a, b);
}

#[test]
fn or_opt_single_point() {
    let targets = make_targets(&[(100.0, 0.0, 5.0)]);
    let drone = drone(50.0, 5000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let or_opt = local_search_or_opt(&nn, &map, &HOME, &drone, 3, 100);
    assert_eq!(or_opt.sequence, nn.sequence);
}

#[test]
fn or_opt_segment_size_one() {
    let targets = make_targets(&[
        (100.0, 0.0, 1.0), (200.0, 100.0, 1.0), (100.0, 100.0, 1.0), (200.0, 0.0, 1.0),
    ]);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let or_opt = local_search_or_opt(&nn, &map, &HOME, &drone, 1, 100);
    assert!(or_opt.feasible);
    assert!(or_opt.total_equiv_distance <= nn.total_equiv_distance + 0.01);
}

// ====================================================================
// VND 搜索
// ====================================================================

#[test]
fn vnd_improves_nn_solution() {
    let targets = make_targets(&[
        (100.0, 0.0, 1.0), (200.0, 100.0, 1.0), (100.0, 100.0, 1.0),
        (200.0, 0.0, 1.0), (300.0, 50.0, 1.0),
    ]);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let vnd = local_search_vnd(&nn, &map, &HOME, &drone, 20, 3);
    assert!(vnd.feasible);
    verify_all(&vnd, &targets, &drone);
    assert!(vnd.total_equiv_distance <= nn.total_equiv_distance + 0.01);
}

#[test]
fn vnd_better_than_or_equal_to_nn() {
    let targets = make_targets(&[
        (500.0, 500.0, 1.0), (100.0, 100.0, 1.0), (400.0, 400.0, 1.0),
        (200.0, 200.0, 1.0), (300.0, 300.0, 1.0),
    ]);
    let drone = drone(20.0, 20000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let vnd = local_search_vnd(&nn, &map, &HOME, &drone, 20, 3);
    assert!(vnd.total_equiv_distance <= nn.total_equiv_distance + 0.01);
    verify_constraints(&vnd, drone.payload_capacity, drone.alpha, drone.beta);
}

#[test]
fn vnd_multi_iteration_converges() {
    let targets = make_targets(&[
        (100.0, 0.0, 1.0), (200.0, 100.0, 1.0), (100.0, 100.0, 1.0),
        (200.0, 0.0, 1.0), (150.0, 200.0, 1.0),
    ]);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let vnd1 = local_search_vnd(&nn, &map, &HOME, &drone, 5, 3);
    let vnd2 = local_search_vnd(&nn, &map, &HOME, &drone, 20, 3);
    assert_eq!(vnd1.sequence, vnd2.sequence);
    assert_eq!(vnd1.total_equiv_distance, vnd2.total_equiv_distance);
}

#[test]
fn vnd_deterministic() {
    let targets = make_targets(&[
        (100.0, 0.0, 2.0), (50.0, 80.0, 3.0), (200.0, 50.0, 2.0),
        (150.0, 200.0, 4.0), (300.0, 100.0, 1.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let v1 = local_search_vnd(&nn, &map, &HOME, &drone, 20, 3);
    let v2 = local_search_vnd(&nn, &map, &HOME, &drone, 20, 3);
    assert_eq!(v1.sequence, v2.sequence);
    assert_eq!(v1.total_equiv_distance, v2.total_equiv_distance);
    assert_eq!(v1.total_energy_consumed, v2.total_energy_consumed);
}

#[test]
fn vnd_constraints_satisfied_on_fixtures() {
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    for name in ["custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json"] {
        let (home, targets) = load_fixture(name);
        let nn = construct_nn(&targets, &home, &drone);
        let map = make_map(&targets);
        if nn.feasible {
            let vnd = local_search_vnd(&nn, &map, &home, &drone, 20, 3);
            if vnd.feasible {
                verify_constraints(&vnd, drone.payload_capacity, drone.alpha, drone.beta);
                assert!(vnd.total_equiv_distance <= nn.total_equiv_distance + 0.01);
            }
        }
    }
}

#[test]
fn opt2_deterministic() {
    let targets = make_targets(&[
        (100.0, 0.0, 2.0), (50.0, 80.0, 3.0), (200.0, 50.0, 2.0),
        (150.0, 200.0, 4.0), (300.0, 100.0, 1.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let a = local_search_2opt(&nn, &map, &HOME, &drone, 100);
    let b = local_search_2opt(&nn, &map, &HOME, &drone, 100);
    assert_eq!(a.sequence, b.sequence);
    assert_eq!(a.total_equiv_distance, b.total_equiv_distance);
}

#[test]
fn or_opt_deterministic() {
    let targets = make_targets(&[
        (100.0, 0.0, 2.0), (50.0, 80.0, 3.0), (200.0, 50.0, 2.0),
    ]);
    let drone = drone(50.0, 10000.0, 0.1, 0.005);
    let nn = construct_nn(&targets, &HOME, &drone);
    let map = make_map(&targets);
    let a = local_search_or_opt(&nn, &map, &HOME, &drone, 3, 100);
    let b = local_search_or_opt(&nn, &map, &HOME, &drone, 3, 100);
    assert_eq!(a.sequence, b.sequence);
    assert_eq!(a.total_equiv_distance, b.total_equiv_distance);
}

// ====================================================================
// 增量评估 vs 全量模拟 (专利创新点 3 验证)
// ====================================================================

#[test]
fn opt2_incremental_matches_full_simulation() {
    let targets = vec![
        TargetDto { id: "A".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "B".into(), location: GeoPointDto { x: 200.0, y: 100.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "C".into(), location: GeoPointDto { x: 100.0, y: 100.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "D".into(), location: GeoPointDto { x: 200.0, y: 0.0 }, demand: 1.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let sequence = seq(&["A", "C", "B", "D"]);
    let sim = simulate_route_energy(&sequence, &map, &HOME, &drone);
    let result = try_2opt_move(&sequence, 0, 3, &map, &HOME, &drone, 4.0, sim.total_equiv);
    assert!(result.is_some());
    let (new_seq, inc_new_equiv) = result.unwrap();
    let full = simulate_route_energy(&new_seq, &map, &HOME, &drone);
    assert!(
        (inc_new_equiv - full.total_equiv).abs() < 1.0,
        "Incremental equiv {inc_new_equiv:.2} vs full {:.2}",
        full.total_equiv
    );
}

#[test]
fn or_opt_incremental_matches_full_simulation() {
    let targets = vec![
        TargetDto { id: "A".into(), location: GeoPointDto { x: 100.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "B".into(), location: GeoPointDto { x: 200.0, y: 100.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "C".into(), location: GeoPointDto { x: 100.0, y: 100.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "D".into(), location: GeoPointDto { x: 200.0, y: 0.0 }, demand: 1.0, ..Default::default() },
        TargetDto { id: "E".into(), location: GeoPointDto { x: 300.0, y: 50.0 }, demand: 1.0, ..Default::default() },
    ];
    let map = make_map(&targets);
    let drone = drone(50.0, 50000.0, 0.1, 0.005);
    let sequence = seq(&["A", "C", "B", "D", "E"]);
    let sim = simulate_route_energy(&sequence, &map, &HOME, &drone);
    let result = try_or_opt_move(&sequence, 3, 4, 0, &map, &HOME, &drone, 5.0, sim.total_equiv);
    if let Some((new_seq, inc_new_equiv)) = result {
        let full = simulate_route_energy(&new_seq, &map, &HOME, &drone);
        assert!(
            (inc_new_equiv - full.total_equiv).abs() < 1.0,
            "Incremental equiv {inc_new_equiv:.2} vs full {:.2}",
            full.total_equiv
        );
    }
}

// ====================================================================
// W4 回归 — 锁定已知预期值 (含 plan_multistop)
// ====================================================================

#[test]
fn vnd_custom_5_heavy_expected() {
    use crate::solver::plan_multistop;
    use crate::dto::{Defaults, MultiStopReq};
    let (home, targets) = load_fixture("custom_5_heavy.json");
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    let req = MultiStopReq { targets, home, drone };
    let cfg = Defaults { max_iterations: 20, time_limit_secs: 5.0, seed: 42 };
    let plan = plan_multistop(&req, &cfg).expect("ok");
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 5);
    let nn = construct_nn(&req.targets, &req.home, &req.drone);
    assert!(plan.total_equiv_distance <= nn.total_equiv_distance + 0.01);
}

#[test]
fn vnd_custom_15_mixed_all_visited() {
    use crate::solver::plan_multistop;
    use crate::dto::{Defaults, MultiStopReq};
    let (home, targets) = load_fixture("custom_15_mixed.json");
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    let req = MultiStopReq { targets, home, drone };
    let cfg = Defaults { max_iterations: 20, time_limit_secs: 5.0, seed: 42 };
    let plan = plan_multistop(&req, &cfg).expect("ok");
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 15);
}

#[test]
fn vnd_on_solomon_r101() {
    use crate::solver::plan_multistop;
    use crate::dto::{Defaults, MultiStopReq};
    let (home, targets) = load_fixture("solomon_r101_n20.json");
    let drone = drone(500.0, 100000.0, 0.08, 0.002);
    let req = MultiStopReq { targets, home, drone };
    let cfg = Defaults { max_iterations: 20, time_limit_secs: 5.0, seed: 42 };
    let plan = plan_multistop(&req, &cfg).expect("ok");
    assert!(plan.feasible);
    assert_eq!(plan.sequence.len(), 20);
    verify_constraints(&plan, drone.payload_capacity, drone.alpha, drone.beta);
}

// 保持 build_route_plan 引用避免 unused (heuristic 内部使用, 此处为直接测试)
#[allow(dead_code)]
fn _use_build_route_plan(
    seq: &[String],
    map: &HashMap<String, &TargetDto>,
    drone: &DroneSpecDto,
) -> RoutePlanResp {
    build_route_plan(seq, map, &HOME, drone)
}

