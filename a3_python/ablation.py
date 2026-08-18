"""消融实验 — 量化每个创新点的贡献 (W5)

R5.2 调研结论 (A3_RESEARCH_PLAN.md §R5.2):
  变体 6 个, 通过组合现有纯函数实现, 不修改 plan_multistop 签名:

    full          = 完整方法 (NN 构造 + VND 搜索)           — 基准
    nn_only       = 仅 NN 构造, 不搜索                      — 量化局部搜索贡献
    no_oropt      = NN + 仅 2-opt 搜索                      — 量化 Or-opt 贡献
    no_2opt       = NN + 仅 Or-opt 搜索                     — 量化 2-opt 贡献
    fixed_payload = β=0 构造+搜索 (忽略载重耦合), 真实 β 评估 — 量化创新点 2
    no_energy     = 无电池约束构造+搜索 (超大电池), 真实电池评估 — 量化电量约束

指标: 每 variant × 每实例 × runs → 总等效距离 mean±std, 求解时间 mean±std, 可行率。
注意: 算法完全确定性, 同一实例多次运行结果相同 — runs 语义为
"不同随机种子生成的实例" (跨实例统计 mean±std 才有意义)。
"""

import time
from dataclasses import dataclass
from statistics import fmean, pstdev

from .route import GeoPoint, Target, DroneSpec, RoutePlan
from .energy_model import simulate_route_energy
from .heuristic import construct_nn, local_search_2opt, local_search_or_opt
from .solver import plan_multistop, DEFAULT_MAX_ITERATIONS

# === 模块级常量 ===

ABLATION_VARIANTS = [
    "full",
    "nn_only",
    "no_oropt",
    "no_2opt",
    "fixed_payload",
    "no_energy",
]
"""消融变体列表 (顺序即报告顺序)"""


# === 数据结构 ===

@dataclass
class AblationStats:
    """消融变体在单个实例上的统计"""
    variant: str
    instance: str
    n_runs: int                  # 运行次数 (实例数)
    n_feasible: int              # 可行运行数
    feasible_rate: float         # 可行率 (0-1)
    cost_mean: float             # 总等效距离 mean (仅可行运行)
    cost_std: float              # 总等效距离 std (仅可行运行)
    time_ms_mean: float          # 求解时间 mean (ms)
    time_ms_std: float           # 求解时间 std (ms)


# === 变体求解 ===

def solve_variant(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    variant: str,
    max_iterations: int = DEFAULT_MAX_ITERATIONS,
) -> RoutePlan:
    """按消融变体求解一个实例.

    Args:
        targets: 目标点列表
        home: 仓库位置
        drone: 无人机规格 (真实模型, 用于最终评估)
        variant: 变体名, 见 ABLATION_VARIANTS
        max_iterations: 搜索外层迭代上限

    Returns:
        RoutePlan: 该变体的路线. 注意: fixed_payload / no_energy 的
                   构造与搜索在简化模型下进行, 返回前用真实 drone 重新评估.

    Raises:
        ValueError: 未知变体名
    """
    targets_map = {t.id: t for t in targets}

    if variant == "full":
        return plan_multistop(targets, home, drone, seed=42)

    if variant == "nn_only":
        return construct_nn(targets, home, drone)

    if variant == "no_oropt":
        plan = construct_nn(targets, home, drone)
        if not plan.feasible:
            return plan
        return local_search_2opt(plan, targets_map, home, drone,
                                 max_iterations=max_iterations)

    if variant == "no_2opt":
        plan = construct_nn(targets, home, drone)
        if not plan.feasible:
            return plan
        return local_search_or_opt(plan, targets_map, home, drone,
                                   max_iterations=max_iterations)

    if variant == "fixed_payload":
        # 构造/搜索忽略载重耦合 (β=0 → equiv = geo); 评估用真实 drone
        flat_drone = DroneSpec(
            payload_capacity=drone.payload_capacity,
            battery_capacity=drone.battery_capacity,
            alpha=drone.alpha,
            beta=0.0,  # 固定载重: 不随载荷变化
        )
        plan = _search_pipeline(targets, home, flat_drone, max_iterations)
        return _re_evaluate(plan, targets, home, drone)

    if variant == "no_energy":
        # 构造/搜索无电池约束 (超大电池); 评估用真实 drone
        no_energy_drone = DroneSpec(
            payload_capacity=drone.payload_capacity,
            battery_capacity=1e12,  # 相当于无电池约束
            alpha=drone.alpha,
            beta=drone.beta,
        )
        plan = _search_pipeline(targets, home, no_energy_drone, max_iterations)
        return _re_evaluate(plan, targets, home, drone)

    raise ValueError(
        f"Unknown ablation variant '{variant}', expected one of {ABLATION_VARIANTS}"
    )


def _search_pipeline(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_iterations: int,
) -> RoutePlan:
    """NN 构造 + VND 搜索 (使用给定 drone 的简化模型)."""
    targets_map = {t.id: t for t in targets}
    plan = construct_nn(targets, home, drone)
    if not plan.feasible:
        return plan
    from .heuristic import local_search_vnd
    return local_search_vnd(plan, targets_map, home, drone,
                            max_iterations=max_iterations)


def _re_evaluate(
    plan: RoutePlan,
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """用真实无人机重新评估一个序列 (真实 α/β + 真实电池)."""
    targets_map = {t.id: t for t in targets}
    segments, geo, equiv, energy, remaining, feasible, warnings = (
        simulate_route_energy(plan.sequence, targets_map, home, drone)
    )
    return RoutePlan(
        sequence=plan.sequence,
        segments=segments,
        total_geo_distance=geo,
        total_equiv_distance=equiv,
        total_energy_consumed=energy,
        remaining_energy=remaining,
        total_payload_delivered=sum(t.demand for t in targets),
        feasible=feasible,
        warnings=warnings,
    )


# === 批量消融 ===

def run_ablation(
    instances: list[tuple[str, list[Target], GeoPoint]],
    drone: DroneSpec,
    max_iterations: int = DEFAULT_MAX_ITERATIONS,
) -> dict[str, dict[str, AblationStats]]:
    """在多个实例上跑完整消融矩阵 (每个变体 × 每个实例 1 次).

    算法确定性: 同实例多次运行结果相同; 跨实例统计由调用方
    (benchmark.py) 用不同种子生成实例后聚合.

    Args:
        instances: [(instance_name, targets, home), ...]
        drone: 无人机规格 (真实模型)
        max_iterations: 搜索外层迭代上限

    Returns:
        {variant: {instance: AblationStats}}
    """
    results: dict[str, dict[str, AblationStats]] = {}
    for variant in ABLATION_VARIANTS:
        results[variant] = {}
        for name, targets, home in instances:
            t0 = time.perf_counter()
            plan = solve_variant(targets, home, drone, variant, max_iterations)
            elapsed_ms = (time.perf_counter() - t0) * 1000

            results[variant][name] = AblationStats(
                variant=variant,
                instance=name,
                n_runs=1,
                n_feasible=1 if plan.feasible else 0,
                feasible_rate=1.0 if plan.feasible else 0.0,
                cost_mean=plan.total_equiv_distance if plan.feasible else 0.0,
                cost_std=0.0,
                time_ms_mean=round(elapsed_ms, 2),
                time_ms_std=0.0,
            )
    return results


def aggregate_stats(
    results: dict[str, dict[str, AblationStats]],
    group_key: callable,
) -> dict[str, dict[str, AblationStats]]:
    """按分组键聚合逐实例消融统计 (跨实例 mean±std, 如按规模 n 分组).

    算法确定性 → 统计方差来自实例间差异而非运行噪声:
    "runs" 语义 = 不同种子生成的实例 (R5.2 调研结论).

    Args:
        results: run_ablation 的输出 {variant: {instance: AblationStats}}
        group_key: 实例名 → 分组键的函数, 如 lambda name: name.split("_")[0]

    Returns:
        {variant: {group: AblationStats}} — 组内可行实例的 cost mean±std
    """
    from collections import defaultdict

    grouped: dict[str, dict[str, list[AblationStats]]] = defaultdict(
        lambda: defaultdict(list))
    for variant, inst_map in results.items():
        for instance, stats in inst_map.items():
            grouped[variant][group_key(instance)].append(stats)

    out: dict[str, dict[str, AblationStats]] = {}
    for variant, groups in grouped.items():
        out[variant] = {}
        for group, stats_list in groups.items():
            feasible = [s for s in stats_list if s.n_feasible > 0]
            n_runs = sum(s.n_runs for s in stats_list)
            n_feasible = sum(s.n_feasible for s in stats_list)
            costs = [s.cost_mean for s in feasible]
            times = [s.time_ms_mean for s in stats_list]
            out[variant][group] = AblationStats(
                variant=variant,
                instance=group,
                n_runs=n_runs,
                n_feasible=n_feasible,
                feasible_rate=n_feasible / n_runs if n_runs else 0.0,
                cost_mean=fmean(costs) if costs else 0.0,
                cost_std=pstdev(costs) if len(costs) > 1 else 0.0,
                time_ms_mean=fmean(times) if times else 0.0,
                time_ms_std=pstdev(times) if len(times) > 1 else 0.0,
            )
    return out
