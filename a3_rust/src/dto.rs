//! DTO 定义 — Python dataclass 的 1:1 翻译 (A3_SCHEMA.md §1 / §2.2)
//!
//! `TargetDto`、`GeoPointDto`、`DroneSpecDto` 分别对应 Python 的
//! `Target`、`GeoPoint`、`DroneSpec`，字段名与类型完全一致。

use serde::{Deserialize, Serialize};

/// 地理坐标点 (对齐 Python `GeoPoint`)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct GeoPointDto {
    /// 经度或平面 X (米)
    pub x: f64,
    /// 纬度或平面 Y (米)
    pub y: f64,
}

/// 目标点 (投递点 / 巡检点) — 对齐 Python `Target`
///
/// `#[serde(default)]`: JSON 缺失进阶字段时回落默认值 (时间窗 MVP 不做)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct TargetDto {
    /// 唯一标识, 如 "c1", "c2"
    pub id: String,
    /// 位置
    pub location: GeoPointDto,
    /// 货物需求量 (kg), 巡检点为 0
    pub demand: f64,
    // === 进阶 (W10+) ===
    /// 时间窗开始 (秒)
    pub tw_ready: Option<f64>,
    /// 时间窗结束 (秒)
    pub tw_due: Option<f64>,
    /// 停留时间 (秒)
    pub service_time: f64,
}

/// 无人机规格 — 对齐 Python `DroneSpec`
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DroneSpecDto {
    /// 最大载重 (kg)
    pub payload_capacity: f64,
    /// 电池总能量 (Wh)
    pub battery_capacity: f64,
    /// 空载能耗率 (Wh/m)
    pub alpha: f64,
    /// 载重敏感系数 (Wh/m/kg)
    pub beta: f64,
    /// 巡航速度 (m/s), 用于时间窗计算 (默认 10.0)
    pub cruise_speed: f64,
}

impl Default for DroneSpecDto {
    fn default() -> Self {
        Self {
            payload_capacity: 0.0,
            battery_capacity: 0.0,
            alpha: 0.0,
            beta: 0.0,
            cruise_speed: 10.0,
        }
    }
}

/// 输入请求 (JSON deserialized) — A3_SCHEMA.md §2.2
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MultiStopReq {
    /// N 个目标点 (1-20 个)
    pub targets: Vec<TargetDto>,
    /// 仓库位置
    pub home: GeoPointDto,
    /// 无人机规格
    pub drone: DroneSpecDto,
}

/// 求解器配置 — A3_SCHEMA.md §2.2
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Defaults {
    /// 最大迭代次数
    pub max_iterations: usize,
    /// 时间上限 (秒)
    pub time_limit_secs: f64,
    /// 随机种子 (固定可复现)
    pub seed: u64,
}

/// 路线中的一段 — 对齐 Python `Segment`
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SegmentDto {
    /// 出发点 id (home 或客户 id)
    pub from_id: String,
    /// 到达点 id
    pub to_id: String,
    /// 几何距离 (m)
    pub geo_distance: f64,
    /// 等效电量距离 (m)
    pub equiv_distance: f64,
    /// 本段耗电 (Wh)
    pub energy_consumed: f64,
    /// 出发时载重 (kg)
    pub payload_before: f64,
    /// 到达+投递后载重 (kg)
    pub payload_after: f64,
    /// 出发时电量 (Wh)
    pub battery_before: f64,
    /// 到达时电量 (Wh)
    pub battery_after: f64,
}

/// 路线规划结果 — 对齐 Python `RoutePlan`
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoutePlanResp {
    /// 访问顺序 (id 列表), 不含 home
    pub sequence: Vec<String>,
    /// 各段详情
    pub segments: Vec<SegmentDto>,
    /// 总几何距离 (m)
    pub total_geo_distance: f64,
    /// 总等效电量距离 (m)
    pub total_equiv_distance: f64,
    /// 总耗电 (Wh)
    pub total_energy_consumed: f64,
    /// 剩余电量 (Wh)
    pub remaining_energy: f64,
    /// 总送货量 (kg)
    pub total_payload_delivered: f64,
    /// 是否满足所有约束
    pub feasible: bool,
    /// 警告信息
    pub warnings: Vec<String>,
}

/// API 错误 — A3_SCHEMA.md §2.2: `{code, message}`
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ApiError {
    /// 错误码, 如 "INFEASIBLE", "BAD_REQUEST", "INTERNAL"
    pub code: String,
    /// 人类可读的错误信息
    pub message: String,
}

impl ApiError {
    /// 构造错误 (code 约定见 A3_SCHEMA.md §3)
    pub fn new(code: &str, message: impl Into<String>) -> Self {
        Self {
            code: code.to_string(),
            message: message.into(),
        }
    }

    /// 400 — 请求参数非法 (serde 解析失败 / 校验失败)
    pub fn bad_request(message: impl Into<String>) -> Self {
        Self::new("BAD_REQUEST", message)
    }

    /// 422 — 约束无法满足 (容量/电量)
    pub fn infeasible(message: impl Into<String>) -> Self {
        Self::new("INFEASIBLE", message)
    }

    /// 500 — 内部错误 (兜底)
    pub fn internal(message: impl Into<String>) -> Self {
        Self::new("INTERNAL", message)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DTO 1:1 对齐 — 反序列化 A3_SCHEMA.md §3 JSON 示例
    #[test]
    fn deserialize_request_matches_schema() {
        let json = r#"{
            "targets": [
                {"id": "c1", "location": {"x": 10.0, "y": 5.0}, "demand": 5.0},
                {"id": "c2", "location": {"x": 5.0, "y": 12.0}, "demand": 8.0}
            ],
            "home": {"x": 0.0, "y": 0.0},
            "drone": {
                "payload_capacity": 20.0,
                "battery_capacity": 5000.0,
                "alpha": 0.1,
                "beta": 0.005
            }
        }"#;
        let req: MultiStopReq = serde_json::from_str(json).expect("parse");
        assert_eq!(req.targets.len(), 2);
        assert_eq!(req.targets[0].id, "c1");
        assert_eq!(req.targets[1].demand, 8.0);
        assert_eq!(req.home, GeoPointDto { x: 0.0, y: 0.0 });
        assert_eq!(req.drone.beta, 0.005);
        // 进阶字段缺省回落
        assert_eq!(req.targets[0].tw_ready, None);
        assert_eq!(req.targets[0].service_time, 0.0);
        assert_eq!(req.drone.cruise_speed, 10.0);
    }

    /// 缺失可选字段 (targets 为空) 不 panic — serde default 路径
    #[test]
    fn deserialize_minimal_request_uses_defaults() {
        let json = r#"{"targets": [], "home": {"x": 1.0, "y": 2.0},
                        "drone": {"payload_capacity": 1.0, "battery_capacity": 1.0,
                                  "alpha": 0.1, "beta": 0.01}}"#;
        let req: MultiStopReq = serde_json::from_str(json).expect("parse");
        assert!(req.targets.is_empty());
    }

    /// ApiError 序列化形态 = {code, message} (schema §2.2)
    #[test]
    fn api_error_json_shape() {
        let err = ApiError::infeasible("overload");
        let s = serde_json::to_string(&err).expect("serialize");
        assert_eq!(s, r#"{"code":"INFEASIBLE","message":"overload"}"#);
    }

    /// 三个构造器对应三种错误码 (schema §3: BAD_REQUEST/INFEASIBLE/INTERNAL)
    #[test]
    fn api_error_constructors() {
        assert_eq!(ApiError::bad_request("bad json").code, "BAD_REQUEST");
        assert_eq!(ApiError::infeasible("overload").code, "INFEASIBLE");
        let internal = ApiError::internal("boom");
        assert_eq!(internal.code, "INTERNAL");
        assert_eq!(internal.message, "boom");
    }
}
