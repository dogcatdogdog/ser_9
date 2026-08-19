"""精确解基线 — OR-Tools CP-SAT + 能量感知 DP (仅评测用, 不进 solver 核心路径)

W5 (R5.1 调研结论, A3_RESEARCH_PLAN.md):
  - solve_tsp_exact_cpsat():  CP-SAT 电路约束, n≤20 几何 TSP 精确解
  - solve_tsp_exact_dp():     Held-Karp DP, n≤15, 用于验证 CP-SAT 正确性
  - solve_energy_exact_dp():  能量感知 DP (状态相关边权), n≤15,
                              能量目标 (总等效距离) 下的精确最优 (无电池约束)
  - evaluate_sequence_cost(): 任意序列在真实电量模型下的后验证

用法 (benchmark 层):
    opt_geo = solve_tsp_exact_cpsat(targets, home, drone=drone)   # 几何精确
    opt_eng = solve_energy_exact_dp(targets, home, drone)          # 能量精确
    gap = compute_gap(plan.total_geo_distance, opt_geo.objective)
"""

import math
import time
from dataclasses import dataclass, field

from .route import GeoPoint, Target, DroneSpec
from .energy_model import (
    euclidean_distance,
    compute_equiv_distance,
    simulate_route_energy,
)

# === 模块级常量 ===

MAX_DP_POINTS = 15            # DP 精确解点数上限 (2^n × n 状态, n=15 时 ~2s)
CP_SAT_SCALE = 100            # 距离 ×100 取整 (CP-SAT 整数模型)
DEFAULT_CP_SAT_LIMIT = 60.0   # CP-SAT 求解时限 (秒)
CP_SAT_EPS = 0.5              # 取整误差容差 (×100 后 ±0.5)


# === 数据结构 ===

@dataclass
class ExactResult:
    """精确求解结果 (评测基线, 不进 solver 核心路径)"""
    instance_name: str
    objective: float               # 目标值 (几何距离或等效距离, 米)
    sequence: list[str]            # 访问顺序 (target ids, 不含 home)
    solve_time_ms: float           # 求解耗时 (ms)
    status: str                    # "optimal" / "feasible" / "timeout" / "error"
    energy_feasible: bool          # 该序列在真实电量模型下是否可行 (后验证)
    energy_warnings: list[str] = field(default_factory=list)

    @property
    def n_points(self) -> int:
        """序列中的目标点数"""
        return len(self.sequence)


# === 通用评估 ===

def evaluate_sequence_cost(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    sequence: list[str],
) -> tuple[float, bool, list[str]]:
    """用真实电量模型评估任意访问序列.

    Args:
        targets: 目标点列表
        home: 仓库位置
        drone: 无人机规格
        sequence: 访问顺序 (target id 列表)

    Returns:
        (total_equiv, feasible, warnings): 总等效距离, 是否可行, 警告列表
    """
    targets_map = {t.id: t for t in targets}
    _, _, total_equiv, _, _, feasible, warnings = simulate_route_energy(
        sequence, targets_map, home, drone
    )
    return total_equiv, feasible, warnings


def compute_gap(our_cost: float, opt_cost: float) -> float:
    """gap 计算公式: (our − opt) / opt × 100%

    Args:
        our_cost: 我们的解的目标值
        opt_cost: 精确最优解的目标值 (> 0)

    Returns:
        gap 百分比 (越小越好, 可为负)

    Raises:
        ValueError: 如果 opt_cost ≤ 0
    """
    if opt_cost <= 0:
        raise ValueError(f"opt_cost must be > 0, got {opt_cost}")
    return (our_cost - opt_cost) / opt_cost * 100.0


# === 几何精确解 ===

def solve_tsp_exact_cpsat(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec | None = None,
    time_limit: float = DEFAULT_CP_SAT_LIMIT,
    instance_name: str = "unnamed",
    seed: int = 42,
) -> ExactResult:
    """OR-Tools CP-SAT 电路约束 — 几何 TSP 精确解 (n ≤ 20)

    距离矩阵 ×100 取整后求解整数模型; AddCircuit 强制哈密顿环。
    实测 n=5/10/15/20 均 OPTIMAL, 与 Held-Karp DP 完全一致 (R5.1 调研验证)。

    Args:
        targets: 目标点列表 (0-20 个)
        home: 仓库位置
        drone: 可选 — 提供时对最优序列做真实电量模型后验证 (energy_feasible)
        time_limit: CP-SAT 求解时限 (秒)
        instance_name: 实例名称 (报告用)
        seed: CP-SAT 随机种子 (确定性)

    Returns:
        ExactResult: objective = 几何最优总距离
    """
    from ortools.sat.python import cp_model

    n = len(targets)
    if n == 0:
        return ExactResult(instance_name, 0.0, [], 0.0, "optimal", True)

    pts = [home] + [t.location for t in targets]
    dist = [[round(euclidean_distance(pts[i], pts[j]) * CP_SAT_SCALE)
             for j in range(n + 1)] for i in range(n + 1)]

    model = cp_model.CpModel()
    x: dict[tuple[int, int], object] = {}
    for i in range(n + 1):
        for j in range(n + 1):
            if i != j:
                x[i, j] = model.NewBoolVar(f"x_{i}_{j}")

    # 每个节点恰好一条出边和一条入边
    for i in range(n + 1):
        model.AddExactlyOne(x[i, j] for j in range(n + 1) if i != j)
        model.AddExactlyOne(x[j, i] for j in range(n + 1) if i != j)

    # 哈密顿环 (单环覆盖所有节点)
    arcs = [(i, j, x[i, j]) for i in range(n + 1) for j in range(n + 1) if i != j]
    model.AddCircuit(arcs)

    model.Minimize(sum(x[i, j] * dist[i][j]
                       for i in range(n + 1) for j in range(n + 1) if i != j))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    solver.parameters.num_workers = 1  # 单线程保证确定性 (n≤20 TSP 实测 <60ms)
    solver.parameters.random_seed = seed

    t0 = time.perf_counter()
    status = solver.Solve(model)
    elapsed_ms = (time.perf_counter() - t0) * 1000

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return ExactResult(instance_name, 0.0, [], round(elapsed_ms, 2),
                           solver.StatusName(status), True)

    # 提取环: 从 home (节点 0) 出发跟随出边
    seq_ids: list[str] = []
    cur = 0
    while True:
        nxt = None
        for j in range(n + 1):
            if j != cur and solver.Value(x[cur, j]) == 1:
                nxt = j
                break
        if nxt is None or nxt == 0:
            break
        seq_ids.append(targets[nxt - 1].id)
        cur = nxt

    obj = solver.ObjectiveValue() / CP_SAT_SCALE
    status_name = ("optimal" if status == cp_model.OPTIMAL else "feasible")

    # 后验证: 真实电量模型下的可行性
    energy_feasible = True
    warnings: list[str] = []
    if drone is not None and seq_ids:
        _, energy_feasible, warnings = evaluate_sequence_cost(
            targets, home, drone, seq_ids
        )

    return ExactResult(instance_name, float(obj), seq_ids,
                       round(elapsed_ms, 2), status_name,
                       energy_feasible, warnings)


def solve_tsp_exact_dp(
    targets: list[Target],
    home: GeoPoint,
    instance_name: str = "unnamed",
) -> ExactResult:
    """Held-Karp DP — 几何 TSP 精确解 (n ≤ MAX_DP_POINTS)

    经典 bitmask DP: dp[mask][j] = 访问 mask (不含 home) 后停在 j 的最小距离。
    用于验证 CP-SAT 的正确性 (R5.1: n=5/10/15 完全一致)。

    Args:
        targets: 目标点列表 (1-15 个)
        home: 仓库位置
        instance_name: 实例名称

    Returns:
        ExactResult: objective = 几何最优总距离

    Raises:
        ValueError: 如果目标点数超过 MAX_DP_POINTS
    """
    n = len(targets)
    if n == 0:
        return ExactResult(instance_name, 0.0, [], 0.0, "optimal", True)
    if n > MAX_DP_POINTS:
        raise ValueError(
            f"target count {n} exceeds DP limit {MAX_DP_POINTS} "
            f"(use CP-SAT for larger instances)"
        )

    pts = [home] + [t.location for t in targets]
    dist = [[euclidean_distance(pts[i], pts[j]) for j in range(n + 1)]
            for i in range(n + 1)]

    t0 = time.perf_counter()
    seq_idx, best = _held_karp(dist)
    elapsed_ms = (time.perf_counter() - t0) * 1000

    seq_ids = [targets[i - 1].id for i in seq_idx]
    return ExactResult(instance_name, best, seq_ids, round(elapsed_ms, 2),
                       "optimal", True)


def _held_karp(dist: list[list[float]]) -> tuple[list[int], float]:
    """Held-Karp DP 核心 — 返回 (客户访问顺序 [1..n-1 索引], 最优环长)

    节点 0 为 home (仅起点/终点), 客户节点 1..n-1。
    mask 的 bit k (k≥1) 表示客户 k 已访问; home 的 bit 0 不使用。
    """
    n = len(dist)
    INF = float("inf")
    N = 1 << n
    dp = [[INF] * n for _ in range(N)]
    parent = [[-1] * n for _ in range(N)]

    for j in range(1, n):
        dp[1 << j][j] = dist[0][j]

    for mask in range(N):
        for j in range(n):
            if not (mask >> j) & 1 or dp[mask][j] == INF:
                continue
            for k in range(1, n):  # k != 0: home 只出现在起点/终点
                if (mask >> k) & 1:
                    continue
                nm = mask | (1 << k)
                c = dp[mask][j] + dist[j][k]
                if c < dp[nm][k]:
                    dp[nm][k] = c
                    parent[nm][k] = j

    full = N - 2  # bits 1..n-1 全置位 (不含 bit 0)
    best, last = min((dp[full][j] + dist[j][0], j) for j in range(1, n))

    # 回溯访问顺序 (客户索引)
    seq: list[int] = []
    mask = full
    cur = last
    while cur != -1:
        seq.append(cur)
        nxt = parent[mask][cur]
        mask ^= 1 << cur
        cur = nxt
    seq.reverse()
    return seq, best


# === 能量感知精确解 ===

def solve_energy_exact_dp(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    instance_name: str = "unnamed",
) -> ExactResult:
    """能量感知精确 DP — 等效距离目标下的精确最优 (n ≤ MAX_DP_POINTS)

    状态相关边权: 边 i→j 的等效距离取决于出发时载重,
    而载重 = total_demand − Σdemand(mask) 由已访问集合唯一确定,
    因此 dp[mask][j] 与几何 TSP 形式完全一致, 仅边权改为状态相关 (R5.1 验证)。

    注意: 这是"无电池约束"的精确最优。电池约束下的精确最优需分支定界,
    留待论文期 (W10+); 当前以最优序列的电池可行性 (evaluate_sequence_cost) 补充。

    Args:
        targets: 目标点列表 (1-15 个)
        home: 仓库位置
        drone: 无人机规格
        instance_name: 实例名称

    Returns:
        ExactResult: objective = 等效距离精确最优, energy_feasible = 最优序列
                     在真实电量模型下是否可行

    Raises:
        ValueError: 如果目标点数超过 MAX_DP_POINTS
    """
    n = len(targets)
    if n == 0:
        return ExactResult(instance_name, 0.0, [], 0.0, "optimal", True)
    if n > MAX_DP_POINTS:
        raise ValueError(
            f"target count {n} exceeds DP limit {MAX_DP_POINTS} "
            f"(use CP-SAT for larger instances)"
        )

    pts = [home] + [t.location for t in targets]
    demands = [0.0] + [t.demand for t in targets]
    total_demand = sum(demands)

    def equiv(i: int, j: int, payload: float) -> float:
        return compute_equiv_distance(
            euclidean_distance(pts[i], pts[j]), payload, drone.alpha, drone.beta
        )

    t0 = time.perf_counter()
    seq_idx, best = _energy_aware_dp(pts, demands, total_demand, equiv)
    elapsed_ms = (time.perf_counter() - t0) * 1000

    seq_ids = [targets[i - 1].id for i in seq_idx]

    # 最优序列的电池可行性 (补充指标)
    energy_feasible, warnings = True, []
    if seq_ids:
        _, energy_feasible, warnings = evaluate_sequence_cost(
            targets, home, drone, seq_ids
        )

    return ExactResult(instance_name, best, seq_ids, round(elapsed_ms, 2),
                       "optimal", energy_feasible, warnings)


def _energy_aware_dp(
    pts: list[GeoPoint],
    demands: list[float],
    total_demand: float,
    equiv,
) -> tuple[list[int], float]:
    """能量感知 DP 核心 — 返回 (客户访问顺序 [1..n-1 索引], 最优等效距离)

    dp[mask][j] = 访问 mask 后停在 j 的最小等效距离;
    边 j→k 的载重 = total_demand − Σdemand(mask) (mask 已投递, k 未投递)。
    """
    n = len(pts)
    INF = float("inf")
    N = 1 << n
    dp = [[INF] * n for _ in range(N)]
    parent = [[-1] * n for _ in range(N)]

    for j in range(1, n):
        dp[1 << j][j] = equiv(0, j, total_demand)

    # 预计算每个 mask 的 demand 和
    mask_demand = [0.0] * N
    for mask in range(1, N):
        lb = mask & (-mask)
        idx = lb.bit_length() - 1
        mask_demand[mask] = mask_demand[mask ^ lb] + demands[idx]

    for mask in range(N):
        sm = mask_demand[mask]
        for j in range(n):
            if not (mask >> j) & 1 or dp[mask][j] == INF:
                continue
            payload_at_j = total_demand - sm  # 已投递 mask 全部后, 从 j 出发
            for k in range(1, n):  # k != 0: home 只出现在起点/终点
                if (mask >> k) & 1:
                    continue
                nm = mask | (1 << k)
                c = dp[mask][j] + equiv(j, k, payload_at_j)
                if c < dp[nm][k]:
                    dp[nm][k] = c
                    parent[nm][k] = j

    full = N - 2  # bits 1..n-1 全置位 (不含 bit 0)
    best, last = min((dp[full][j] + equiv(j, 0, 0.0), j) for j in range(1, n))

    seq: list[int] = []
    mask = full
    cur = last
    while cur != -1:
        seq.append(cur)
        nxt = parent[mask][cur]
        mask ^= 1 << cur
        cur = nxt
    seq.reverse()
    return seq, best
