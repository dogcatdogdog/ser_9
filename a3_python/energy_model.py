"""载重-电量耦合模型 + 等效距离变换 — 对齐 A3_REQUIREMENTS.md §3.2

核心公式:
  E(i→j) = geo_dist(i→j) × (α + β × load_before_departure(i)) / α

专利创新点 2: 载重-能耗耦合模型 — 能耗率 = α + β × 当前载重
"""

import math
from .route import GeoPoint, Target, DroneSpec, Segment


def euclidean_distance(a: GeoPoint, b: GeoPoint) -> float:
    """计算两点间的欧几里得距离 (米)"""
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def compute_equiv_distance(
    geo_dist: float,
    payload: float,
    alpha: float,
    beta: float,
) -> float:
    """等效电量距离变换 (专利创新点 1)

    将几何距离改造为载重感知的等效距离:
      equiv = geo_dist × (α + β × payload) / α

    Args:
        geo_dist: 几何距离 (m)
        payload: 出发时载重 (kg)
        alpha: 空载能耗率 (Wh/m)
        beta: 载重敏感系数 (Wh/m/kg)

    Returns:
        等效电量距离 (m)
    """
    if alpha <= 0:
        raise ValueError(f"alpha must be > 0, got {alpha}")
    return geo_dist * (alpha + beta * payload) / alpha


def compute_energy_for_segment(
    geo_dist: float,
    payload: float,
    alpha: float,
    beta: float,
) -> float:
    """计算一段路径的能耗 (Wh)

    energy = equiv_distance × alpha = geo_dist × (α + β × payload)
    """
    equiv = compute_equiv_distance(geo_dist, payload, alpha, beta)
    return equiv * alpha


def simulate_route_energy(
    sequence: list[str],
    targets: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> tuple[list[Segment], float, float, float, float, bool, list[str]]:
    """全量后验证: 逐段模拟 payload 变化 + 精确计算各段能耗

    Args:
        sequence: 访问顺序 (target id 列表, 不含 home)
        targets: id → Target 映射
        home: 仓库位置
        drone: 无人机规格

    Returns:
        (segments, total_geo, total_equiv, total_energy, remaining_energy, feasible, warnings)
    """
    segments: list[Segment] = []
    total_geo = 0.0
    total_equiv = 0.0
    total_energy = 0.0
    warnings: list[str] = []

    # 初始状态
    total_demand = sum(t.demand for t in targets.values())
    payload = total_demand
    battery = drone.battery_capacity

    if payload > drone.payload_capacity:
        return (
            [], 0.0, 0.0, 0.0, battery, False,
            [f"Total payload ({payload:.1f}kg) exceeds drone capacity ({drone.payload_capacity:.1f}kg)"]
        )

    # 构建完整路径: home → seq[0] → seq[1] → ... → seq[n-1] → home
    full_path = ["home"] + sequence + ["home"]

    for i in range(len(full_path) - 1):
        from_id = full_path[i]
        to_id = full_path[i + 1]

        # 获取坐标
        if from_id == "home":
            from_point = home
        else:
            from_point = targets[from_id].location

        if to_id == "home":
            to_point = home
        else:
            to_point = targets[to_id].location

        # 几何距离
        geo_dist = euclidean_distance(from_point, to_point)

        # 等效距离 (基于出发时载重)
        equiv_dist = compute_equiv_distance(
            geo_dist, payload, drone.alpha, drone.beta
        )

        # 能耗
        energy = equiv_dist * drone.alpha

        battery_before = battery
        battery_after = battery - energy

        # 可行性检查
        if energy > battery:
            warnings.append(
                f"Battery exhausted at segment {from_id}→{to_id}: "
                f"need {energy:.1f}Wh, have {battery:.1f}Wh"
            )

        # 投递后载重减少 (home 不投递)
        payload_after = payload
        if to_id != "home":
            payload_after = payload - targets[to_id].demand

        segment = Segment(
            from_id=from_id,
            to_id=to_id,
            geo_distance=round(geo_dist, 2),
            equiv_distance=round(equiv_dist, 2),
            energy_consumed=round(energy, 2),
            payload_before=round(payload, 2),
            payload_after=round(payload_after, 2),
            battery_before=round(battery_before, 2),
            battery_after=round(battery_after, 2),
        )
        segments.append(segment)

        total_geo += geo_dist
        total_equiv += equiv_dist
        total_energy += energy
        battery = battery_after
        payload = payload_after

    feasible = len(warnings) == 0 and payload <= drone.payload_capacity

    return (
        segments,
        round(total_geo, 2),
        round(total_equiv, 2),
        round(total_energy, 2),
        round(battery, 2),
        feasible,
        warnings,
    )
