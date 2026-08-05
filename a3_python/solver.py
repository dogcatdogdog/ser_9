"""核心求解入口 — 对齐 A3_SCHEMA.md §2.1

W1 骨架: plan_multistop() 返回输入顺序的基本路线, 不做优化搜索。
优化算法 (NN 构造 / 2-opt / Or-opt) 将在 W3-W4 实现。
"""

import math
from .route import GeoPoint, Target, DroneSpec, RoutePlan, Segment
from .energy_model import euclidean_distance, simulate_route_energy

# 模块级常量
MAX_TARGETS = 20  # MVP 上限


def plan_multistop(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    seed: int = 42,
) -> RoutePlan:
    """无人机多目标访问路线规划。

    W1 骨架: 按输入顺序构造路线, 逐段模拟能耗, 判定可行性。
    不做启发式构造和局部搜索优化。

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格
        seed: 随机种子, 保证确定性 (默认 42)

    Returns:
        RoutePlan: 路线规划结果，包含访问序列、各段详情、可行性判定

    Raises:
        ValueError: 如果 target 数量为 0 或超过上限
    """
    # 输入验证
    if len(targets) == 0:
        raise ValueError("targets list cannot be empty")
    if len(targets) > MAX_TARGETS:
        raise ValueError(
            f"target count {len(targets)} exceeds MVP limit {MAX_TARGETS}"
        )

    # 构建 id → Target 映射
    targets_map = {t.id: t for t in targets}

    # W1: 按输入顺序排列 (不做优化)
    sequence = [t.id for t in targets]

    # 模拟路线能耗
    segments, total_geo, total_equiv, total_energy, remaining, feasible, warnings = (
        simulate_route_energy(sequence, targets_map, home, drone)
    )

    total_payload = sum(t.demand for t in targets)

    # 不可行时仍保留 sequence + warnings 供诊断
    # simulate_route_energy 已通过 warnings 说明了不可行原因 (超载/电量不足)

    return RoutePlan(
        sequence=sequence,
        segments=segments,
        total_geo_distance=total_geo,
        total_equiv_distance=total_equiv,
        total_energy_consumed=total_energy,
        remaining_energy=remaining,
        total_payload_delivered=total_payload,
        feasible=feasible,
        warnings=warnings,
    )
