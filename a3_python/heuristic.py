"""构造启发式 — NN (最近邻) + Savings (W3 实现)

W1 骨架: 函数签名已定义, 实现待 W3 完成。
"""

from .route import Target, GeoPoint, DroneSpec, RoutePlan


def construct_nn(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """电量感知最近邻构造 (W3 实现)

    从 home 出发, 每次选 equiv_dist 最小且满足剩余电量约束的未访问点。
    不可行时返回部分路线 + 不可行标记。

    Args:
        targets: 目标点列表
        home: 仓库位置
        drone: 无人机规格

    Returns:
        RoutePlan: 构造的路线 (W1 返回空壳)
    """
    raise NotImplementedError("NN construction — W3 实现")


def construct_savings(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """Clarke-Wright Savings 改造版 (W3 实现)

    saving(i,j) = equiv_dist(home,i) + equiv_dist(home,j) - equiv_dist(i,j)
    按 saving 从大到小合并路线, 每次合并检查电量可行性。

    Args:
        targets: 目标点列表
        home: 仓库位置
        drone: 无人机规格

    Returns:
        RoutePlan: 构造的路线 (W1 返回空壳)
    """
    raise NotImplementedError("Savings construction — W3 实现")


def local_search_2opt(
    route: RoutePlan,
    targets: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_iterations: int = 100,
) -> RoutePlan:
    """2-opt 局部搜索改进 (W4 实现)

    翻转一段子路径, 增量评估电量可行性。

    Args:
        route: 初始路线
        targets: id → Target 映射
        home: 仓库位置
        drone: 无人机规格
        max_iterations: 最大迭代次数

    Returns:
        RoutePlan: 改进后的路线
    """
    raise NotImplementedError("2-opt local search — W4 实现")


def local_search_or_opt(
    route: RoutePlan,
    targets: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_segment_size: int = 3,
    max_iterations: int = 100,
) -> RoutePlan:
    """Or-opt 局部搜索改进 (W4 实现)

    移动 1-3 个连续点到新位置, 增量评估。

    Args:
        route: 初始路线
        targets: id → Target 映射
        home: 仓库位置
        drone: 无人机规格
        max_segment_size: 最大移动段长度 (默认 3)
        max_iterations: 最大迭代次数

    Returns:
        RoutePlan: 改进后的路线
    """
    raise NotImplementedError("Or-opt local search — W4 实现")
