"""路线规划数据结构 — 对齐 A3_SCHEMA.md §1"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class GeoPoint:
    """地理坐标点"""
    x: float          # 经度或平面 X (米)
    y: float          # 纬度或平面 Y (米)


@dataclass
class Target:
    """目标点（投递点 / 巡检点）"""
    id: str           # 唯一标识, 如 "c1", "c2"
    location: GeoPoint
    demand: float     # 货物需求量 (kg), 巡检点为 0
    # === 进阶 (W10+) ===
    tw_ready: Optional[float] = None   # 时间窗开始 (秒)
    tw_due: Optional[float] = None     # 时间窗结束 (秒)
    service_time: float = 0.0          # 停留时间 (秒)


@dataclass
class DroneSpec:
    """无人机规格"""
    payload_capacity: float   # 最大载重 (kg)
    battery_capacity: float   # 电池总能量 (Wh)
    alpha: float              # 空载能耗率 (Wh/m)
    beta: float               # 载重敏感系数 (Wh/m/kg)
    cruise_speed: float = 10.0  # 巡航速度 (m/s), 用于时间窗计算


@dataclass
class Segment:
    """路线中的一段"""
    from_id: str        # 出发点 id (home 或客户 id)
    to_id: str          # 到达点 id
    geo_distance: float           # 几何距离 (m)
    equiv_distance: float         # 等效电量距离 (m)
    energy_consumed: float        # 本段耗电 (Wh)
    payload_before: float         # 出发时载重 (kg)
    payload_after: float          # 到达+投递后载重 (kg)
    battery_before: float         # 出发时电量 (Wh)
    battery_after: float          # 到达时电量 (Wh)


@dataclass
class RoutePlan:
    """路线规划结果"""
    sequence: list[str]       # 访问顺序 (id 列表), 不含 home
    segments: list[Segment]   # 各段详情
    total_geo_distance: float      # 总几何距离 (m)
    total_equiv_distance: float    # 总等效电量距离 (m)
    total_energy_consumed: float   # 总耗电 (Wh)
    remaining_energy: float        # 剩余电量 (Wh)
    total_payload_delivered: float # 总送货量 (kg)
    feasible: bool                 # 是否满足所有约束
    warnings: list[str] = field(default_factory=list)


# ====================================================================
# 无人机参数配置表 — 3 种标准机型 (对齐 A3_DEVPLAN.md W2 D5-D7)
# ====================================================================

DRONE_PRESETS: dict[str, DroneSpec] = {
    "light": DroneSpec(
        payload_capacity=5.0,       # 5kg 载重
        battery_capacity=1000.0,    # 1000Wh (1kWh)
        alpha=0.15,                 # 高能耗率 (小型机效率低)
        beta=0.01,                  # 高载重敏感 (小机身对载重敏感)
        cruise_speed=8.0,           # 8m/s
    ),
    "standard": DroneSpec(
        payload_capacity=50.0,      # 50kg 载重
        battery_capacity=5000.0,    # 5000Wh (5kWh)
        alpha=0.1,                  # 标准能耗率
        beta=0.005,                 # 标准载重敏感
        cruise_speed=10.0,          # 10m/s
    ),
    "heavy": DroneSpec(
        payload_capacity=500.0,     # 500kg 载重
        battery_capacity=100000.0,  # 100kWh
        alpha=0.08,                 # 低能耗率 (大型机效率高)
        beta=0.002,                 # 低载重敏感 (大机身对载重不敏感)
        cruise_speed=12.0,          # 12m/s
    ),
}
"""无人机预设配置表

三种机型覆盖不同使用场景:
  - light:    小型无人机, 适合短距离轻载配送
  - standard: 标准配送无人机, 基准对比机型
  - heavy:    大型无人机, 适合 Solomon 等标准 VRP 实例

使用:
    from a3_python.route import DRONE_PRESETS
    drone = DRONE_PRESETS["standard"]
"""
