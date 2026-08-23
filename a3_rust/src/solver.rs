//! 核心求解入口 — W6 空壳 (W7 实现, 对齐 Python `a3_python/solver.py`)
//!
//! 求解流程 (W7): NN 构造 → VND 搜索 (2-opt + Or-opt 交替改进)
//! 铁律: 纯函数 — 不 import HTTP/IO 层, 无网络、无全局状态。

use crate::dto::{ApiError, Defaults, MultiStopReq, RoutePlanResp};

/// 无人机多目标访问路线规划 — A3_SCHEMA.md §2.2
///
/// W6: 空壳, 返回 `INTERNAL` 未实现; W7 实现:
///   NN 构造 (载重感知等效距离) + VND 搜索 (2-opt/Or-opt) + 全量后验证
pub fn plan_multistop(req: &MultiStopReq, cfg: &Defaults) -> Result<RoutePlanResp, ApiError> {
    let _ = (req, cfg); // W6 空壳
    Err(ApiError::internal("plan_multistop not implemented until W7"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dto::{DroneSpecDto, GeoPointDto, TargetDto};

    fn sample_req() -> MultiStopReq {
        MultiStopReq {
            targets: vec![
                TargetDto {
                    id: "c1".into(),
                    location: GeoPointDto { x: 10.0, y: 5.0 },
                    demand: 5.0,
                    ..Default::default()
                },
                TargetDto {
                    id: "c2".into(),
                    location: GeoPointDto { x: 5.0, y: 12.0 },
                    demand: 8.0,
                    ..Default::default()
                },
            ],
            home: GeoPointDto { x: 0.0, y: 0.0 },
            drone: DroneSpecDto {
                payload_capacity: 20.0,
                battery_capacity: 5000.0,
                alpha: 0.1,
                beta: 0.005,
                ..Default::default()
            },
        }
    }

    /// W6 空壳: 合法输入返回 INTERNAL 未实现错误 (不 panic)
    #[test]
    fn skeleton_returns_internal_not_implemented() {
        let req = sample_req();
        let cfg = Defaults {
            max_iterations: 100,
            time_limit_secs: 5.0,
            seed: 42,
        };
        let err = plan_multistop(&req, &cfg).expect_err("W6 空壳应返回 Err");
        assert_eq!(err.code, "INTERNAL");
        assert!(err.message.contains("W7"));
    }
}
