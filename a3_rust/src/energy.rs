//! 载重-能耗耦合模型 + 等效距离变换 — 对齐 Python `a3_python/energy_model.py`
//!
//! 核心公式 (A3_REQUIREMENTS.md §3.2):
//!   E(i→j) = geo_dist(i→j) × (α + β × load_before_departure(i)) / α
//!
//! R6.1 结论: 使用标准库 `f64::sqrt()`, 与 numpy 的 libm 同为 0.5 ULP 精度
//! (IEEE 754), 交叉验证误差预期 ≪ 1e-6。不引入 geo/nalgebra crate。

use crate::dto::GeoPointDto;

/// 两点间欧几里得距离 (米) — 对齐 Python `euclidean_distance`
pub fn geo_distance(a: &GeoPointDto, b: &GeoPointDto) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}

/// 等效电量距离变换 (专利创新点 1) — 对齐 Python `compute_equiv_distance`
///
/// equiv = geo_dist × (α + β × payload) / α
///
/// 语义: 载重 payload 越大, 等效距离越长 → 构造启发式天然偏向
/// "先送远的、后送重的"。
pub fn equiv_distance(geo_dist: f64, payload: f64, alpha: f64, beta: f64) -> f64 {
    assert!(alpha > 0.0, "alpha must be > 0, got {alpha}");
    geo_dist * (alpha + beta * payload) / alpha
}

/// 一段路径的能耗 (Wh) — 对齐 Python `compute_energy_for_segment`
///
/// energy = equiv_distance × alpha = geo_dist × (α + β × payload)
pub fn energy_for_segment(geo_dist: f64, payload: f64, alpha: f64, beta: f64) -> f64 {
    equiv_distance(geo_dist, payload, alpha, beta) * alpha
}

/// 等效电量距离矩阵 — 对齐 Python `compute_equiv_matrix`
///
/// 注意: 等效距离是状态依赖的 (payload_before 取决于访问顺序)。
/// 本函数以「从 i 出发时载重 = total_demand - demands[i]」为近似,
/// 供构造启发式使用; 精确值由 W7 的 `simulate_route_energy` 逐段计算。
///
/// Args:
///   geo_matrix: N×N 几何距离矩阵, 行主序 (row-major)
///   demands:    各点需求量 (长度 N, 第 0 项为 home = 0)
///   alpha, beta: 能耗率参数
///
/// Returns:
///   N×N 等效距离矩阵 (行主序)
pub fn compute_equiv_matrix(
    geo_matrix: &[f64],
    demands: &[f64],
    alpha: f64,
    beta: f64,
) -> Vec<f64> {
    assert!(alpha > 0.0, "alpha must be > 0, got {alpha}");
    let n = demands.len();
    assert_eq!(geo_matrix.len(), n * n, "geo_matrix must be N×N");

    let total_demand: f64 = demands.iter().sum();
    let mut equiv = vec![0.0; n * n];

    for i in 0..n {
        // 从 i 出发时的剩余载重 (不含 i 自身, 因为 i 点已投递)
        let payload = total_demand - demands[i];
        let factor = (alpha + beta * payload) / alpha;
        for j in 0..n {
            equiv[i * n + j] = geo_matrix[i * n + j] * factor;
        }
    }
    equiv
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 3-4-5 直角三角形距离
    #[test]
    fn geo_distance_pythagorean_triple() {
        let a = GeoPointDto { x: 0.0, y: 0.0 };
        let b = GeoPointDto { x: 3.0, y: 4.0 };
        assert_eq!(geo_distance(&a, &b), 5.0);
    }

    /// 退化: 同点距离为 0
    #[test]
    fn geo_distance_identical_points() {
        let a = GeoPointDto { x: 1.5, y: -2.0 };
        assert_eq!(geo_distance(&a, &a), 0.0);
    }

    /// 载重为 0 → 等效距离 = 几何距离
    #[test]
    fn equiv_distance_zero_payload_is_geo() {
        assert_eq!(equiv_distance(100.0, 0.0, 0.1, 0.005), 100.0);
    }

    /// 正载重 → 等效距离 > 几何距离
    #[test]
    fn equiv_distance_positive_payload_inflates() {
        // 100 × (0.1 + 0.005×20) / 0.1 = 100 × 2 = 200
        assert_eq!(equiv_distance(100.0, 20.0, 0.1, 0.005), 200.0);
    }

    /// 载重为负 (不该出现) — 公式按数学定义仍成立, 不 panic
    #[test]
    fn equiv_distance_negative_payload_ok() {
        let d = equiv_distance(10.0, -1.0, 0.1, 0.005);
        assert_eq!(d, 10.0 * (0.1 - 0.005) / 0.1);
    }

    /// alpha 非正 → panic (对齐 Python ValueError)
    #[test]
    #[should_panic(expected = "alpha must be > 0")]
    fn equiv_distance_alpha_zero_panics() {
        equiv_distance(10.0, 5.0, 0.0, 0.005);
    }

    /// energy = equiv × alpha
    #[test]
    fn energy_for_segment_consistency() {
        let e = energy_for_segment(100.0, 20.0, 0.1, 0.005);
        assert_eq!(e, 200.0 * 0.1); // equiv=200, alpha=0.1
    }

    /// 矩阵版 vs 标量版逐元素一致 (对齐 Python compute_equiv_matrix 语义)
    #[test]
    fn equiv_matrix_matches_scalar() {
        // 3 点 (home + 2 targets), 与 Python 文档示例一致
        let geo: Vec<f64> = vec![0.0, 3.0, 4.0, 3.0, 0.0, 5.0, 4.0, 5.0, 0.0];
        let demands = vec![0.0, 8.0, 7.0];
        let alpha = 0.1;
        let beta = 0.005;
        let m = compute_equiv_matrix(&geo, &demands, alpha, beta);
        // m[i*3+j]; i=1 (target 0): payload = 15-8 = 7 → factor = (0.1+0.035)/0.1 = 1.35
        assert!((m[3] - 3.0 * 1.35).abs() < 1e-12);
        // 对角线 (i=2, j=2): 几何距离 0 × factor = 0 (非 NaN)
        assert!(m[8].abs() < 1e-12);
        // i=0 (home): payload = 15 → factor = 1.75
        assert!((m[2] - 4.0 * 1.75).abs() < 1e-12);
    }

    /// 维度不匹配 → panic
    #[test]
    #[should_panic(expected = "must be N×N")]
    fn equiv_matrix_wrong_shape_panics() {
        compute_equiv_matrix(&[1.0, 2.0], &[0.0, 1.0, 2.0], 0.1, 0.005);
    }
}
