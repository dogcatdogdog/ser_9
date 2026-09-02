//! 核心求解入口 — 对齐 Python `a3_python/solver.py` (W7 实现)
//!
//! 求解流程: 参数校验 (W8 硬化) → 电量感知 NN 构造 (N-start)
//! → 初始解不可行则直接返回 → VND 局部搜索 (2-opt + Or-opt, 增量评估)
//! 铁律: 纯函数 — 不 import HTTP/IO 层, 无网络、无全局状态。
//!
//! 注: `cfg.time_limit_secs > 0` 时 VND 检查运行时限 (读时钟),
//! 正常求解远快于时限 (n≤20 < 150ms) 不触发, 不影响确定性输出。

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::dto::{ApiError, Defaults, MultiStopReq, RoutePlanResp};
use crate::heuristic::{construct_nn, local_search_vnd};

/// MVP 目标点上限 (对齐 Python `MAX_TARGETS`)
const MAX_TARGETS: usize = 20;

/// 校验请求参数域 — 非法输入 → `BAD_REQUEST` (W8 硬化 P0)
///
/// 防御重点: `drone.alpha <= 0` 会触发 energy.rs 的 assert panic
/// (Python 语义为 ValueError 抛错); 在此统一拦截为可读的 400。
/// 比较用 `!(x > 0)` 形态 — 对 NaN 同样拦截。
fn validate_params(req: &MultiStopReq) -> Result<(), ApiError> {
    let d = &req.drone;
    // partial_cmp != Some(Greater): alpha <= 0 与 NaN 都拦截 (NaN-safe)
    if d.alpha.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return Err(ApiError::bad_request(format!(
            "drone.alpha must be > 0, got {}",
            d.alpha
        )));
    }
    if d.beta < 0.0 {
        return Err(ApiError::bad_request(format!(
            "drone.beta must be >= 0, got {}",
            d.beta
        )));
    }
    if d.payload_capacity < 0.0 {
        return Err(ApiError::bad_request(format!(
            "drone.payload_capacity must be >= 0, got {}",
            d.payload_capacity
        )));
    }
    if d.battery_capacity < 0.0 {
        return Err(ApiError::bad_request(format!(
            "drone.battery_capacity must be >= 0, got {}",
            d.battery_capacity
        )));
    }
    for t in &req.targets {
        if t.demand < 0.0 {
            return Err(ApiError::bad_request(format!(
                "target {} demand must be >= 0, got {}",
                t.id, t.demand
            )));
        }
    }
    Ok(())
}

/// 无人机多目标访问路线规划 — A3_SCHEMA.md §2.2
///
/// 求解流程:
///   1. 输入验证 (1 ≤ N ≤ 20 + 参数域), 非法输入 → `BAD_REQUEST`
///   2. 电量感知 NN 构造初始解 (N-start 变体)
///   3. 初始解不可行 (如载重超限) → 直接返回, 不做搜索
///   4. VND 局部搜索改进 (2-opt + Or-opt 交替, 增量评估, 受 time_limit 约束)
///
/// 返回的 RoutePlanResp.feasible=False 表示约束无法满足,
/// 查看 warnings 了解原因 (不返回 INFEASIBLE 错误 — 与 Python 一致,
/// 不可行是合法求解结果而非异常)。
pub fn plan_multistop(req: &MultiStopReq, cfg: &Defaults) -> Result<RoutePlanResp, ApiError> {
    // 输入验证: 结构 (数量) + 参数域 (W8 硬化: 防 panic + 语义清晰的 400)
    if req.targets.is_empty() {
        return Err(ApiError::bad_request("targets list cannot be empty"));
    }
    if req.targets.len() > MAX_TARGETS {
        return Err(ApiError::bad_request(format!(
            "target count {} exceeds MVP limit {MAX_TARGETS}",
            req.targets.len()
        )));
    }
    validate_params(req)?;

    // Phase 1: 电量感知 NN 构造初始解
    let initial = construct_nn(&req.targets, &req.home, &req.drone);

    // 如果初始解不可行 (如载重超限), 直接返回, 不做搜索
    if !initial.feasible {
        return Ok(initial);
    }

    // Phase 2: VND 局部搜索改进 (W8 硬化: time_limit_secs > 0 时受时限约束)
    let targets_map: HashMap<String, &crate::dto::TargetDto> =
        req.targets.iter().map(|t| (t.id.clone(), t)).collect();
    let deadline = if cfg.time_limit_secs > 0.0 {
        Some(
            Instant::now() + Duration::from_secs_f64(cfg.time_limit_secs),
        )
    } else {
        None
    };
    let improved = local_search_vnd(
        &initial,
        &targets_map,
        &req.home,
        &req.drone,
        cfg.max_iterations,
        3, // max_segment_size (对齐 Python 默认)
        deadline,
    );

    Ok(improved)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dto::{DroneSpecDto, GeoPointDto, TargetDto};

    fn drone(payload: f64, battery: f64) -> DroneSpecDto {
        DroneSpecDto {
            payload_capacity: payload,
            battery_capacity: battery,
            alpha: 0.1,
            beta: 0.005,
            ..Default::default()
        }
    }

    fn targets(coords_demands: &[(f64, f64, f64)]) -> Vec<TargetDto> {
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

    fn make_req(t: Vec<TargetDto>, drone: DroneSpecDto) -> MultiStopReq {
        MultiStopReq {
            targets: t,
            home: GeoPointDto { x: 0.0, y: 0.0 },
            drone,
        }
    }

    fn cfg(max_iterations: usize) -> Defaults {
        Defaults {
            max_iterations,
            time_limit_secs: 5.0,
            seed: 42,
        }
    }

    // ============ 正例 (happy path) ============

    /// 3 点、电量充裕 → feasible=True, 所有点被访问
    #[test]
    fn plan_3points_feasible() {
        let req = make_req(
            targets(&[(100.0, 0.0, 5.0), (0.0, 100.0, 5.0), (100.0, 100.0, 5.0)]),
            drone(30.0, 10000.0),
        );
        let plan = plan_multistop(&req, &cfg(20)).expect("ok");
        assert_eq!(plan.sequence.len(), 3);
        let visited: std::collections::HashSet<&str> =
            plan.sequence.iter().map(|s| s.as_str()).collect();
        assert_eq!(visited, ["c1", "c2", "c3"].into_iter().collect());
        assert!(plan.total_geo_distance > 0.0);
        assert!(plan.total_equiv_distance > 0.0);
        assert!(plan.total_energy_consumed > 0.0);
        assert!(plan.remaining_energy < req.drone.battery_capacity);
        assert_eq!(plan.total_payload_delivered, 15.0);
        assert!(plan.feasible);
        assert!(plan.warnings.is_empty());
    }

    /// 5 点、电量充裕 → feasible=True
    #[test]
    fn plan_5points_feasible() {
        let req = make_req(
            targets(&[
                (50.0, 0.0, 3.0), (0.0, 50.0, 3.0), (100.0, 50.0, 3.0),
                (50.0, 100.0, 3.0), (200.0, 200.0, 3.0),
            ]),
            drone(40.0, 20000.0),
        );
        let plan = plan_multistop(&req, &cfg(20)).expect("ok");
        assert_eq!(plan.sequence.len(), 5);
        assert!(plan.feasible);
        assert_eq!(plan.total_payload_delivered, 15.0);
    }

    // ============ 退化 ============

    /// 总载重 > capacity → feasible=False, warnings 含 capacity
    #[test]
    fn plan_overload_infeasible() {
        let req = make_req(
            targets(&[(100.0, 0.0, 8.0), (0.0, 100.0, 8.0)]),
            drone(10.0, 5000.0),
        );
        let plan = plan_multistop(&req, &cfg(20)).expect("ok");
        assert!(!plan.feasible);
        assert!(!plan.warnings.is_empty());
        assert!(plan.warnings[0].to_lowercase().contains("exceeds drone capacity"));
        assert_eq!(plan.total_geo_distance, 0.0);
    }

    /// 电量不足以支撑全路线 → feasible=False
    #[test]
    fn plan_low_battery_infeasible() {
        let req = make_req(
            targets(&[(500.0, 500.0, 1.0)]),
            drone(50.0, 5.0),
        );
        let plan = plan_multistop(&req, &cfg(20)).expect("ok");
        assert!(!plan.feasible);
    }

    // ============ 参数域校验 (W8 硬化: 防 panic) ============

    fn req_with_alpha(alpha: f64) -> MultiStopReq {
        let mut d = drone(30.0, 10000.0);
        d.alpha = alpha;
        make_req(targets(&[(100.0, 0.0, 5.0)]), d)
    }

    /// alpha=0 → BAD_REQUEST (否则会触发 energy.rs assert panic)
    #[test]
    fn plan_alpha_zero_rejected() {
        let err = plan_multistop(&req_with_alpha(0.0), &cfg(20)).expect_err("alpha=0 应报错");
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("alpha"));
    }

    /// alpha<0 → BAD_REQUEST
    #[test]
    fn plan_alpha_negative_rejected() {
        let err = plan_multistop(&req_with_alpha(-0.1), &cfg(20)).expect_err("alpha<0 应报错");
        assert_eq!(err.code, "BAD_REQUEST");
    }

    /// beta<0 → BAD_REQUEST
    #[test]
    fn plan_beta_negative_rejected() {
        let mut d = drone(30.0, 10000.0);
        d.beta = -0.001;
        let req = make_req(targets(&[(100.0, 0.0, 5.0)]), d);
        let err = plan_multistop(&req, &cfg(20)).expect_err("beta<0 应报错");
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("beta"));
    }

    /// capacity<0 → BAD_REQUEST
    #[test]
    fn plan_capacity_negative_rejected() {
        let mut d = drone(-1.0, 10000.0);
        d.alpha = 0.1;
        let req = make_req(targets(&[(100.0, 0.0, 5.0)]), d);
        let err = plan_multistop(&req, &cfg(20)).expect_err("capacity<0 应报错");
        assert_eq!(err.code, "BAD_REQUEST");
    }

    /// 负 demand → BAD_REQUEST
    #[test]
    fn plan_negative_demand_rejected() {
        let req = make_req(targets(&[(100.0, 0.0, -5.0)]), drone(30.0, 10000.0));
        let err = plan_multistop(&req, &cfg(20)).expect_err("demand<0 应报错");
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("demand"));
    }

    // ============ 边界 ============

    /// 0 点 → BAD_REQUEST
    #[test]
    fn plan_zero_targets_error() {
        let req = make_req(vec![], drone(30.0, 10000.0));
        let err = plan_multistop(&req, &cfg(20)).expect_err("空列表应报错");
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("empty"));
    }

    /// 1 点 → feasible=True, 直线往返
    #[test]
    fn plan_1point_boundary() {
        let req = make_req(targets(&[(100.0, 0.0, 5.0)]), drone(10.0, 5000.0));
        let plan = plan_multistop(&req, &cfg(20)).expect("ok");
        assert!(plan.feasible);
        assert_eq!(plan.sequence, vec!["c1"]);
        assert_eq!(plan.segments.len(), 2); // home→c1, c1→home
        assert!((plan.total_geo_distance - 200.0).abs() < 20.0);
    }

    /// 超过 20 点上限 → BAD_REQUEST
    #[test]
    fn plan_over_max_targets_error() {
        let t: Vec<TargetDto> = (0..21)
            .map(|i| TargetDto {
                id: format!("c{i}"),
                location: GeoPointDto { x: i as f64, y: 0.0 },
                demand: 0.1,
                ..Default::default()
            })
            .collect();
        let req = make_req(t, drone(50.0, 5000.0));
        let err = plan_multistop(&req, &cfg(20)).expect_err("超限应报错");
        assert_eq!(err.code, "BAD_REQUEST");
        assert!(err.message.contains("exceeds MVP limit"));
    }

    // ============ 一致性 ============

    /// 相同输入 → 相同输出 (确定性)
    #[test]
    fn plan_deterministic() {
        let req = make_req(
            targets(&[(100.0, 0.0, 5.0), (0.0, 100.0, 5.0)]),
            drone(50.0, 5000.0),
        );
        let r1 = plan_multistop(&req, &cfg(42)).expect("ok");
        let r2 = plan_multistop(&req, &cfg(42)).expect("ok");
        assert_eq!(r1.sequence, r2.sequence);
        assert_eq!(r1.total_equiv_distance, r2.total_equiv_distance);
        assert_eq!(r1.feasible, r2.feasible);
    }

    /// 相同 seed → 相同输出
    #[test]
    fn plan_same_seed_consistency() {
        let req = make_req(
            targets(&[(10.0, 0.0, 5.0), (50.0, 0.0, 5.0), (100.0, 0.0, 5.0)]),
            drone(50.0, 5000.0),
        );
        let r1 = plan_multistop(&req, &cfg(99)).expect("ok");
        let r2 = plan_multistop(&req, &cfg(99)).expect("ok");
        assert_eq!(r1.sequence, r2.sequence);
        assert_eq!(r1.total_geo_distance, r2.total_geo_distance);
    }
}
