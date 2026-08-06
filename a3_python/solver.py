"""核心求解入口 — 对齐 A3_SCHEMA.md §2.1

W1 骨架: plan_multistop() 返回输入顺序的基本路线, 不做优化搜索。
W3 升级: 使用电量感知 NN (N-start) 构造初始解。
W4 升级: VND 局部搜索 (2-opt + Or-opt, 增量评估) 改进初始解。
"""

import math
from .route import GeoPoint, Target, DroneSpec, RoutePlan, Segment
from .energy_model import euclidean_distance, simulate_route_energy
from .heuristic import construct_nn, local_search_vnd

# 模块级常量
MAX_TARGETS = 20  # MVP 上限
DEFAULT_MAX_ITERATIONS = 20  # VND 外层迭代上限


def plan_multistop(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    seed: int = 42,
) -> RoutePlan:
    """无人机多目标访问路线规划。

    求解流程:
      1. 输入验证 (1 ≤ N ≤ 20)
      2. 电量感知 NN 构造初始解 (N-start 变体, W3)
      3. VND 局部搜索改进 (2-opt + Or-opt 交替, 增量评估, W4)
      4. 返回最终路线 (可能不可行, 检查 RoutePlan.feasible)

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格
        seed: 随机种子, 保证确定性 (默认 42, 当前算法均为确定性)

    Returns:
        RoutePlan: 路线规划结果, 包含访问序列、各段详情、可行性判定

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

    # Phase 1: 电量感知 NN 构造初始解 (W3)
    initial = construct_nn(targets, home, drone)

    # 如果初始解不可行 (如载重超限), 直接返回, 不做搜索
    if not initial.feasible:
        return initial

    # Phase 2: VND 局部搜索改进 (W4)
    targets_map = {t.id: t for t in targets}
    improved = local_search_vnd(
        initial,
        targets_map,
        home,
        drone,
        max_iterations=DEFAULT_MAX_ITERATIONS,
    )

    return improved
