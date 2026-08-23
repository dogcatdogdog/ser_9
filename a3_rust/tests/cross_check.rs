//! W6 交叉验证 (DEVPLAN 强制对齐) — Python numpy vs Rust 等效距离矩阵
//!
//! golden 由 a3_rust/scripts/gen_energy_golden.py 生成 (env312 python),
//! 覆盖 6 个标准 fixture (5/10/15/20 点 × 几何/聚集/混合)。
//! 断言: 两版矩阵逐元素误差 < 1e-6。

use a3_rust::dto::GeoPointDto;
use a3_rust::energy::{compute_equiv_matrix, geo_distance};

/// 强制对齐容差 (DEVPLAN: 等效距离矩阵误差 < 1e-6)
const TOL: f64 = 1e-6;

#[derive(serde::Deserialize)]
struct Golden {
    alpha: f64,
    beta: f64,
    instances: Vec<GoldenInstance>,
}

#[derive(serde::Deserialize)]
struct GoldenInstance {
    name: String,
    n: usize,
    locations: Vec<Vec<f64>>,
    demands: Vec<f64>,
    geo_matrix: Vec<f64>,
    equiv_matrix: Vec<f64>,
}

fn load_golden() -> Golden {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/energy_golden.json"
    );
    let raw = std::fs::read_to_string(path)
        .expect("golden 文件缺失 — 先运行 scripts/gen_energy_golden.py");
    serde_json::from_str(&raw).expect("golden JSON 解析失败")
}

fn rust_geo_matrix(inst: &GoldenInstance) -> Vec<f64> {
    let n = inst.n;
    let pts: Vec<GeoPointDto> = inst
        .locations
        .iter()
        .map(|p| GeoPointDto { x: p[0], y: p[1] })
        .collect();
    let mut m = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            m[i * n + j] = geo_distance(&pts[i], &pts[j]);
        }
    }
    m
}

/// 几何距离矩阵: Rust 独立重算 (f64::sqrt) vs Python numpy — 逐元素 < 1e-6
#[test]
fn geo_matrix_matches_python_numpy() {
    let golden = load_golden();
    for inst in &golden.instances {
        let rust_geo = rust_geo_matrix(inst);
        for (idx, (rust_val, py_val)) in
            rust_geo.iter().zip(inst.geo_matrix.iter()).enumerate()
        {
            let diff = (rust_val - py_val).abs();
            assert!(
                diff < TOL,
                "{} geo[{}]: rust={:.12} python={:.12} diff={:.2e}",
                inst.name,
                idx,
                rust_val,
                py_val,
                diff
            );
        }
    }
}

/// 等效距离矩阵: Rust compute_equiv_matrix vs Python compute_equiv_matrix — 逐元素 < 1e-6
#[test]
fn equiv_matrix_matches_python_numpy() {
    let golden = load_golden();
    for inst in &golden.instances {
        let rust_geo = rust_geo_matrix(inst);
        let rust_equiv = compute_equiv_matrix(&rust_geo, &inst.demands, golden.alpha, golden.beta);
        for (idx, (rust_val, py_val)) in
            rust_equiv.iter().zip(inst.equiv_matrix.iter()).enumerate()
        {
            let diff = (rust_val - py_val).abs();
            assert!(
                diff < TOL,
                "{} equiv[{}]: rust={:.12} python={:.12} diff={:.2e}",
                inst.name,
                idx,
                rust_val,
                py_val,
                diff
            );
        }
    }
}
