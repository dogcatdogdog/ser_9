"""核心求解入口 — 对齐 A3_SCHEMA.md §2.1

W1 骨架: plan_multistop() 返回输入顺序的基本路线, 不做优化搜索。
W3 升级: 使用电量感知 NN (N-start) 构造初始解。
  局部搜索 (2-opt / Or-opt) 将在 W4 加入。
"""

import math
from .route import GeoPoint, Target, DroneSpec, RoutePlan, Segment
from .energy_model import euclidean_distance, simulate_route_energy
from .heuristic import construct_nn

# 模块级常量
MAX_TARGETS = 20  # MVP 上限


def plan_multistop(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    seed: int = 42,
) -> RoutePlan:
    """无人机多目标访问路线规划。

    W3: 使用电量感知 NN 构造初始解 (N-start 变体)。
    W4 将加入局部搜索进一步改进。

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格
        seed: 随机种子, 保证确定性 (默认 42, 当前 NN 为确定性算法)

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

    # W3: 使用电量感知 NN 构造初始解
    # W4 将在此之后加入 local_search_2opt + local_search_or_opt 改进
    return construct_nn(targets, home, drone)
