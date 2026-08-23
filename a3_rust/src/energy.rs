//! 载重-能耗耦合模型 + 等效距离变换 — 对齐 Python `a3_python/energy_model.py`
//!
//! 核心公式 (A3_REQUIREMENTS.md §3.2):
//!   E(i→j) = geo_dist(i→j) × (α + β × load_before_departure(i)) / α
//!
//! R6.1 结论: 使用标准库 `f64::sqrt()`, 与 numpy 的 libm 同为 0.5 ULP 精度
//! (IEEE 754), 交叉验证误差预期 ≪ 1e-6。不引入 geo/nalgebra crate。

use std::collections::HashMap;

use crate::dto::{DroneSpecDto, GeoPointDto, SegmentDto, TargetDto};

/// 保留 2 位小数 — 对齐 Python `round(x, 2)` 的输出形态 (DTO 对齐 §1)
///
/// 注: Python round 是 round-half-even, Rust round() 是 half-away-from-zero,
/// 精确半值时有 ±0.01 差异; 对路线规划结果无实质影响, golden 比对用容差。
pub fn round2(x: f64) -> f64 {
    let r = (x * 100.0).round() / 100.0;
    if r == 0.0 {
        0.0 // 归一化 -0.0
    } else {
        r
    }
}

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

/// 全量路线模拟结果 — 对齐 Python `simulate_route_energy` 的 7 元组
pub struct SimulatedRoute {
    pub segments: Vec<SegmentDto>,
    pub total_geo: f64,
    pub total_equiv: f64,
    pub total_energy: f64,
    pub remaining_energy: f64,
    pub feasible: bool,
    pub warnings: Vec<String>,
}

/// 全量后验证: 逐段模拟 payload 变化 + 精确计算各段能耗
/// — 对齐 Python `simulate_route_energy`
///
/// 注意 (与 Python 一致): 电量不足只记录 warning 不中断模拟;
/// feasible 由 warnings 是否为空 + 载重约束决定。
pub fn simulate_route_energy(
    sequence: &[String],
    targets: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
) -> SimulatedRoute {
    let mut segments: Vec<SegmentDto> = Vec::new();
    let mut total_geo = 0.0;
    let mut total_equiv = 0.0;
    let mut total_energy = 0.0;
    let mut warnings: Vec<String> = Vec::new();

    // 初始状态
    let total_demand: f64 = targets.values().map(|t| t.demand).sum();
    let mut payload = total_demand;
    let mut battery = drone.battery_capacity;

    if payload > drone.payload_capacity {
        return SimulatedRoute {
            segments: Vec::new(),
            total_geo: 0.0,
            total_equiv: 0.0,
            total_energy: 0.0,
            remaining_energy: battery,
            feasible: false,
            warnings: vec![format!(
                "Total payload ({payload:.1}kg) exceeds drone capacity ({:.1}kg)",
                drone.payload_capacity
            )],
        };
    }

    // 构建完整路径: home → seq[0] → ... → seq[n-1] → home
    let mut full_path: Vec<String> = vec!["home".to_string()];
    full_path.extend(sequence.iter().cloned());
    full_path.push("home".to_string());

    let location_of = |id: &str| -> &GeoPointDto {
        if id == "home" {
            home
        } else {
            &targets[id].location
        }
    };

    for w in full_path.windows(2) {
        let from_id = &w[0];
        let to_id = &w[1];

        let geo_dist = geo_distance(location_of(from_id), location_of(to_id));

        // 等效距离 (基于出发时载重)
        let equiv_dist = equiv_distance(geo_dist, payload, drone.alpha, drone.beta);

        // 能耗
        let energy = equiv_dist * drone.alpha;

        let battery_before = battery;
        let battery_after = battery - energy;

        // 可行性检查 (仅记录 warning, 与 Python 一致)
        if energy > battery {
            warnings.push(format!(
                "Battery exhausted at segment {from_id}→{to_id}: \
                 need {energy:.1}Wh, have {battery:.1}Wh"
            ));
        }

        // 投递后载重减少 (home 不投递)
        let payload_after = if to_id == "home" {
            payload
        } else {
            payload - targets[to_id].demand
        };

        segments.push(SegmentDto {
            from_id: from_id.clone(),
            to_id: to_id.clone(),
            geo_distance: round2(geo_dist),
            equiv_distance: round2(equiv_dist),
            energy_consumed: round2(energy),
            payload_before: round2(payload),
            payload_after: round2(payload_after),
            battery_before: round2(battery_before),
            battery_after: round2(battery_after),
        });

        total_geo += geo_dist;
        total_equiv += equiv_dist;
        total_energy += energy;
        battery = battery_after;
        payload = payload_after;
    }

    let feasible = warnings.is_empty() && payload <= drone.payload_capacity;

    SimulatedRoute {
        segments,
        total_geo: round2(total_geo),
        total_equiv: round2(total_equiv),
        total_energy: round2(total_energy),
        remaining_energy: round2(battery),
        feasible,
        warnings,
    }
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

// ====================================================================
// energy 单测补充 — 1:1 移植 `a3_python/tests/test_energy_model.py` (21 例)
// ====================================================================

#[cfg(test)]
mod extended_tests {
    use super::*;

/// 水平线段距离
#[test]
fn euclidean_distance_horizontal() {
    let a = GeoPointDto { x: 0.0, y: 0.0 };
    let b = GeoPointDto { x: 300.0, y: 0.0 };
    assert_eq!(geo_distance(&a, &b), 300.0);
}

/// 等效距离公式精确性: 手工计算验证 (50 × (0.1+0.005×10)/0.1 = 75)
#[test]
fn equiv_distance_formula_accuracy() {
    let d = equiv_distance(50.0, 10.0, 0.1, 0.005);
    assert!((d - 75.0).abs() < 1e-12);
}

fn targets_map_of(pairs: &[(&str, f64, f64, f64)]) -> HashMap<String, TargetDto> {
    pairs
        .iter()
        .map(|(id, x, y, d)| {
            (
                id.to_string(),
                TargetDto {
                    id: id.to_string(),
                    location: GeoPointDto { x: *x, y: *y },
                    demand: *d,
                    ..Default::default()
                },
            )
        })
        .collect()
}

/// owned map → 引用 map (对齐 Python dict[str, Target] 的调用形态)
fn ref_map(owned: &HashMap<String, TargetDto>) -> HashMap<String, &TargetDto> {
    owned.iter().map(|(k, v)| (k.clone(), v)).collect()
}

/// 测试 9 (能量模型): 先送重货 vs 先送轻货 → 等效距离不同
/// 专利创新点 2: 载重-能耗耦合 → 访问顺序影响总等效距离
#[test]
fn simulate_heavy_first_vs_light_first() {
    let map = targets_map_of(&[("heavy", 100.0, 0.0, 10.0), ("light", 200.0, 0.0, 1.0)]);
    let drone = DroneSpecDto {
        payload_capacity: 20.0,
        battery_capacity: 10000.0,
        alpha: 0.1,
        beta: 0.005,
        ..Default::default()
    };
    let home = GeoPointDto { x: 0.0, y: 0.0 };
    let map = ref_map(&map);

    // 路线 1: 先重后轻 home→heavy→light→home
    let s1 = simulate_route_energy(&seq(&["heavy", "light"]), &map, &home, &drone);
    // 路线 2: 先轻后重 home→light→heavy→home
    let s2 = simulate_route_energy(&seq(&["light", "heavy"]), &map, &home, &drone);

    // 两种顺序都是可行的 (电量充裕)
    assert!(s1.feasible);
    assert!(s2.feasible);

    // 访问顺序不同 → 等效距离不同 (载重-能耗耦合的核心体现)
    assert_ne!(s1.total_equiv, s2.total_equiv);
}

/// 超载检测: total payload > capacity
#[test]
fn simulate_overload_detected() {
    let map = targets_map_of(&[("c1", 100.0, 0.0, 10.0)]);
    let drone = DroneSpecDto {
        payload_capacity: 5.0, // 容量只有 5kg
        battery_capacity: 5000.0,
        alpha: 0.1,
        beta: 0.005,
        ..Default::default()
    };
    let home = GeoPointDto { x: 0.0, y: 0.0 };
    let map = ref_map(&map);

    let sim = simulate_route_energy(&seq(&["c1"]), &map, &home, &drone);
    assert!(!sim.feasible);
    assert!(sim.segments.is_empty());
    assert!(sim.warnings.iter().any(|w| w.contains("exceeds drone capacity")));
}

/// 低电量检测: 电池不足以完成路线
#[test]
fn simulate_low_battery_detected() {
    let map = targets_map_of(&[("c1", 1000.0, 0.0, 1.0)]);
    let drone = DroneSpecDto {
        payload_capacity: 50.0,
        battery_capacity: 10.0, // 极少电量
        alpha: 0.1,
        beta: 0.005,
        ..Default::default()
    };
    let home = GeoPointDto { x: 0.0, y: 0.0 };
    let map = ref_map(&map);

    let sim = simulate_route_energy(&seq(&["c1"]), &map, &home, &drone);
    assert!(!sim.feasible);
    assert!(!sim.warnings.is_empty());
}

/// 3 点距离矩阵: 验证 3-4-5 三角形
#[test]
fn geo_matrix_3points() {
    let pts = [
        GeoPointDto { x: 0.0, y: 0.0 },
        GeoPointDto { x: 3.0, y: 0.0 },
        GeoPointDto { x: 0.0, y: 4.0 },
    ];
    let mut m = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            m[i * 3 + j] = geo_distance(&pts[i], &pts[j]);
        }
    }
    // 对角线为 0
    assert_eq!(m[0], 0.0);
    assert_eq!(m[4], 0.0);
    assert_eq!(m[8], 0.0);
    // 对称性
    assert_eq!(m[1], m[3]);
    assert_eq!(m[2], m[6]);
    assert_eq!(m[5], m[7]);
    // 具体值: (0,0)→(3,0)=3, (0,0)→(0,4)=4, (3,0)→(0,4)=5
    assert_eq!(m[1], 3.0);
    assert_eq!(m[2], 4.0);
    assert_eq!(m[5], 5.0);
}

/// 空点集 → 空矩阵 (n=0 时 compute_equiv_matrix 不 panic)
#[test]
fn equiv_matrix_empty_input() {
    let m = compute_equiv_matrix(&[], &[], 0.1, 0.005);
    assert!(m.is_empty());
}

/// 单点: 1×1 零矩阵
#[test]
fn geo_matrix_single_point() {
    let p = GeoPointDto { x: 5.0, y: 3.0 };
    assert_eq!(geo_distance(&p, &p), 0.0);
}

/// 矩阵对称性 + 非负性 (5 点确定性数据)
#[test]
fn geo_matrix_symmetry() {
    let pts = [
        GeoPointDto { x: -92.0, y: 41.0 },
        GeoPointDto { x: 13.0, y: -8.0 },
        GeoPointDto { x: 71.0, y: 65.0 },
        GeoPointDto { x: -32.0, y: -77.0 },
        GeoPointDto { x: 5.0, y: 22.0 },
    ];
    let n = pts.len();
    for i in 0..n {
        for j in 0..n {
            let d = geo_distance(&pts[i], &pts[j]);
            if i == j {
                assert_eq!(d, 0.0);
            } else {
                assert!(d > 0.0);
            }
            // 对称
            assert_eq!(d, geo_distance(&pts[j], &pts[i]));
        }
    }
}

/// 三角形不等式: d(i,j) ≤ d(i,k) + d(k,j)
#[test]
fn geo_matrix_triangle_inequality() {
    let pts = [
        GeoPointDto { x: 0.0, y: 0.0 },
        GeoPointDto { x: 10.0, y: 0.0 },
        GeoPointDto { x: 5.0, y: 8.0 },
    ];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                if i != j && i != k && j != k {
                    let dij = geo_distance(&pts[i], &pts[j]);
                    let dik = geo_distance(&pts[i], &pts[k]);
                    let dkj = geo_distance(&pts[k], &pts[j]);
                    assert!(dij <= dik + dkj + 1e-10);
                }
            }
        }
    }
}

/// 空载: 等效矩阵 = 几何矩阵
#[test]
fn equiv_matrix_empty_load() {
    let geo: Vec<f64> = vec![0.0, 3.0, 4.0, 3.0, 0.0, 5.0, 4.0, 5.0, 0.0];
    let m = compute_equiv_matrix(&geo, &[0.0, 0.0, 0.0], 0.1, 0.005);
    for (a, b) in m.iter().zip(geo.iter()) {
        assert!((a - b).abs() < 1e-12);
    }
}

/// 有载重: 等效矩阵 ≥ 几何矩阵 (逐元素)
#[test]
fn equiv_matrix_with_load() {
    let geo: Vec<f64> = vec![0.0, 10.0, 10.0, 10.0, 0.0, 14.14, 10.0, 14.14, 0.0];
    let m = compute_equiv_matrix(&geo, &[0.0, 5.0, 5.0], 0.1, 0.005);
    for (a, b) in m.iter().zip(geo.iter()) {
        assert!(*a >= *b - 1e-12, "equiv {a} < geo {b}");
    }
}

/// 等效矩阵公式精确性: 手工验算单个元素
/// home→c1: payload=10 → 100×1.5=150; c1→home: payload=0 → 100
#[test]
fn equiv_matrix_formula_accuracy() {
    let geo: Vec<f64> = vec![0.0, 100.0, 100.0, 0.0];
    let m = compute_equiv_matrix(&geo, &[0.0, 10.0], 0.1, 0.005);
    assert!((m[1] - 150.0).abs() < 1e-12); // equiv[0,1]
    assert!((m[2] - 100.0).abs() < 1e-12); // equiv[1,0]
}

/// alpha ≤ 0 → panic (对齐 Python ValueError)
#[test]
#[should_panic(expected = "alpha must be > 0")]
fn equiv_matrix_alpha_zero_panics() {
    compute_equiv_matrix(&[0.0, 1.0, 1.0, 0.0], &[0.0, 1.0], 0.0, 0.005);
}

/// beta 越大 → 等效距离越大 (载重敏感度高)
#[test]
fn equiv_matrix_higher_beta_larger_equiv() {
    let geo: Vec<f64> = vec![0.0, 100.0, 100.0, 0.0];
    let demands = [0.0, 20.0];
    let low = compute_equiv_matrix(&geo, &demands, 0.1, 0.001);
    let high = compute_equiv_matrix(&geo, &demands, 0.1, 0.01);
    assert!(high[1] > low[1]);
}

/// round2: 保留 2 位小数
#[test]
fn round2_basic() {
    assert_eq!(round2(123.4567), 123.46);
    assert_eq!(round2(0.0), 0.0);
    assert_eq!(round2(-0.0), 0.0); // -0.0 归一化
}

// helper: id 列表 → Vec<String>
fn seq(ids: &[&str]) -> Vec<String> {
    ids.iter().map(|s| s.to_string()).collect()
}
}
