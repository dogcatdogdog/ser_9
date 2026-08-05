"""载重-电量耦合模型 + 等效距离变换 — 对齐 A3_REQUIREMENTS.md §3.2

核心公式:
  E(i→j) = geo_dist(i→j) × (α + β × load_before_departure(i)) / α

专利创新点 2: 载重-能耗耦合模型 — 能耗率 = α + β × 当前载重
"""

import math
import numpy as np
from .route import GeoPoint, Target, DroneSpec, Segment


def euclidean_distance(a: GeoPoint, b: GeoPoint) -> float:
    """计算两点间的欧几里得距离 (米)"""
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def compute_geo_matrix(locations: list[GeoPoint]) -> np.ndarray:
    """批量计算所有点对间的几何距离矩阵 (N×N)

    使用 numpy 向量化计算, 复杂度 O(N²) 但常数极小。

    Args:
        locations: 坐标点列表 (包含 home 在内的 N 个点)

    Returns:
        np.ndarray: N×N 距离矩阵, mat[i][j] = |locations[i] - locations[j]|

    Example:
        >>> pts = [GeoPoint(0,0), GeoPoint(3,0), GeoPoint(0,4)]
        >>> mat = compute_geo_matrix(pts)
        >>> mat[0, 1]  # 3.0
        >>> mat[0, 2]  # 4.0
        >>> mat[1, 2]  # 5.0
    """
    n = len(locations)
    if n == 0:
        return np.empty((0, 0), dtype=np.float64)

    # 提取坐标数组: (N, 2)
    coords = np.array([[p.x, p.y] for p in locations], dtype=np.float64)

    # 向量化: mat[i,j] = sqrt((xi-xj)² + (yi-yj)²)
    # diff[i,j] = coords[i] - coords[j], shape (N, N, 2)
    diff = coords[:, np.newaxis, :] - coords[np.newaxis, :, :]
    geo_matrix = np.sqrt(np.sum(diff ** 2, axis=2))

    return geo_matrix


def compute_equiv_matrix(
    geo_matrix: np.ndarray,
    demands: list[float],
    alpha: float,
    beta: float,
) -> np.ndarray:
    """基于几何距离矩阵和需求量, 计算等效电量距离矩阵

    等效距离 = geo_dist × (α + β × payload_before) / α

    注意: 等效距离是状态依赖的 — payload_before 取决于访问顺序中
    目标点 i 出发时的剩余载重。本函数返回以「出发时载重 = sum(demands after i)」
    为近似的等效距离矩阵, 供构造启发式使用。

    Args:
        geo_matrix: N×N 几何距离矩阵 (第 0 行为 home, 后续为 targets)
        demands: 各点需求量列表 (长度 N, 第 0 项为 home=0)
        alpha: 空载能耗率 (Wh/m)
        beta: 载重敏感系数 (Wh/m/kg)

    Returns:
        np.ndarray: N×N 等效距离矩阵
    """
    if alpha <= 0:
        raise ValueError(f"alpha must be > 0, got {alpha}")

    n = len(demands)
    total_demand = sum(demands)
    equiv = np.zeros_like(geo_matrix)

    for i in range(n):
        # 从 i 出发时的剩余载重 (不含 i 自身, 因为 i 点已投递)
        # 近似: payload = total_demand - 已访问点的 demand
        # 这里用 i 点之后剩余的总 demand 作为近似
        payload = total_demand - demands[i]
        equiv[i, :] = geo_matrix[i, :] * (alpha + beta * payload) / alpha

    return equiv


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
