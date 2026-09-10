"""带电量约束的能量最优精确求解 (主干模块, W9)

定位
----
补上 MVP 的一个空白: `a3_python/exact.py::solve_energy_exact_dp` 求的是
**无电池约束**的精确最优; 本模块在其上加入"任意中间点剩余电量 >= 0"的硬约束,
并把可在线求解的规模从 n <= 15 (纯 Python 三重循环, n=20 需 96.5 s) 扩到
**n <= 20** (分层向量化 + 上界剪枝, n=20 实测中位 1.77 s)。
本模块**不修改** `exact.py` 的任何既有函数 (向后兼容), 只新增能力。

目标函数 (等效距离 / 能耗, 见 A3_REQUIREMENTS.md §3.2)
-----------------------------------------------------
    总能耗 = alpha * D_total + beta * SUM_i q_i * D_i
其中 D_i = 到达点 i 的累计距离。该形式可归约为"加权最小延迟 + 总距离项",
故状态相关的边权只取决于 (已访问集合 mask) —— 出发载重 = 总需求 - 已投递需求。

电量约束只是**可行性闸门** (不是"另一个最优解")
------------------------------------------------
目标函数就是全程总能耗, 而沿访问顺序的累计能耗**单调不减**, 故
"任意时刻 <= cap" 等价于 "总能耗 <= cap"。设无约束最优为 OPT:
  * cap >= OPT  => 最优解本身可行 => 本模块的最优序列与 `solve_energy_exact_dp`
    **完全相同**;
  * cap <  OPT  => 任何解的总能耗 >= OPT > cap => **无可行解** (本模块能判定)。
即本模块相对无约束精确 DP 的增量是"**可行性判定** + **性能**", 而非"另一个最优解"。
该性质由 `a3_python/tests/test_exact_battery_fast.py` 钉死。

状态压缩依据 (支配性)
--------------------
到达同一 (已访问集合 mask, 当前点 j) 状态时, **耗能最少**的子路径剩余电量最多,
而未来代价只取决于 (mask, j) —— 故只需保留"最小耗能"这一条标签,
无需把电量作为独立状态维。因此状态数仍为 O(2^n · n)。

两条实现路径 (语义完全一致, 前者是后者的正确性参照)
--------------------------------------------------
  1. `solve_energy_exact_battery_reference` —— 朴素三重循环 (O(2^n·n²) Python 级循环)。
     逐位可复现, 供改造后的实现做等价性断言。
  2. `solve_energy_exact_battery` —— 分层向量化 + 上界剪枝的加速版 (默认走这条):
       (a) **按 popcount 分层推进**: 第 k 层只保留"恰好访问 k 个点"的状态,
           并用"压实" (只保留该层有有限值的行) 把内存从 O(2^n·n) 降到
           O(max_k C(n,k) · k);
       (b) **numpy 向量化转移**: 同一层内所有状态向未访问点的转移批量计算,
           内层不再有 Python 三重循环;
       (c) **可采纳上界剪枝**: 先用启发式得到可行上界 UB, 再用指派松弛下界 LB 剪掉
           `cost + LB > UB` 的状态 (LB 的可采纳性证明见 `_layer_lower_bound`)。

规模量级 (n 上限的来源)
----------------------
`MAX_EXACT_BATTERY_POINTS = 20` (= `solver.MAX_TARGETS`, MVP 上限)。
实测 (Python 3.12 / numpy 2.5.1, 见 `a3_python/adaptive.py` 的估算常量):

    n  耗时中位   峰值内存
    15   0.071 s    5.2 MB
    18   0.421 s   36.7 MB
    20   1.774 s  155.8 MB

耗时约按 2×/点 增长, 峰值内存约按 2×/点 增长 (n=21 外推 ~300 MB), 故 n > 20 时
本模块不再适用 —— 生产侧由 `a3_python.adaptive` 按预算自动降级到启发式。

设计约束:
  - 纯函数, 无 I/O / 无全局状态 (仅模块级常量)
  - 不修改 a3_python 下任何现有模块; 复用其数据类型
  - 硬编码数值一律声明为模块级常量

术语说明 (浮点口径):
  本模块定义的"出发载重" = `total_demand - mask_demand[mask]` (子集和递推),
  而 `energy_model.simulate_route_energy` 用"载重逐点累减"。两者数学等价,
  浮点上可能差 ~1 ULP (1e-13), 故跨模块断言一律留 1e-9 容差。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .route import DroneSpec, GeoPoint, Target
from .energy_model import euclidean_distance
from .solver import plan_multistop

# === 模块级常量 ===

MAX_EXACT_BATTERY_POINTS = 20  # MVP 上限 (对齐 solver.MAX_TARGETS); n=20 实测 1.77 s / 156 MB
_EPS = 1e-9                    # 浮点比较容差 (Wh)
_INF = float("inf")

# 父指针 dtype 上限: int8 可存 0..127, n <= 127 时够用 (2^n 内存早已不可行)
_PARENT_INT8_MAX_N = 127


@dataclass
class ExactBatteryResult:
    """带电池约束的精确求解结果"""
    objective: float          # 最优总能耗 (Wh)
    sequence: list[str]       # 访问顺序 (target id)
    feasible: bool            # 是否存在可行解
    states_explored: int      # 扩展过的 (mask, j) 状态数 (统计用)
    pruned_states: int = 0    # 被上界剪枝掉的状态数 (仅加速版统计)
    ub_initial: float = _INF  # 剪枝所用初始上界 (启发式能耗; 无则为 +inf)


def solve_energy_exact_battery_reference(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_points: int = MAX_EXACT_BATTERY_POINTS,
) -> ExactBatteryResult:
    """带电池约束的能量最优精确 DP (Held-Karp 变体, n <= max_points) —— **参照实现**

    朴素三重循环, 逻辑自 STEP1 改造起保持不变, 仅被重命名。
    与无电池版 (`exact.solve_energy_exact_dp`) 的差别: 每段累计耗能不得超过电池容量,
    因此需要"部分状态不可达"的剪枝, 而不是简单地对全排列求最小。

    支配性论证见模块 docstring: 只需保留到达 (mask, j) 的最小耗能标签。

    Args:
        targets: 目标点列表 (1 ~ max_points 个)
        home: 仓库位置
        drone: 无人机规格 (alpha/beta/battery_capacity/payload_capacity)
        max_points: 允许的最大目标点数 (默认 MAX_EXACT_BATTERY_POINTS);
            调大仅用于探索/基准测试 —— 状态数 O(2^n·n) 会指数增长

    Returns:
        ExactBatteryResult: 最优能耗与序列; 无可行解时 feasible=False

    Raises:
        ValueError: 目标点数超过 max_points
    """
    n = len(targets)
    if n == 0:
        return ExactBatteryResult(0.0, [], True, 0)
    if n > max_points:
        raise ValueError(
            f"target count {n} exceeds exact-battery limit {max_points}"
        )

    pts = [home] + [t.location for t in targets]
    demand = [0.0] + [t.demand for t in targets]
    total_demand = sum(demand)

    # 载重预检: 总需求超过载重上限则任何顺序均不可行
    if total_demand > drone.payload_capacity + _EPS:
        return ExactBatteryResult(float("inf"), [], False, 0)

    size = 1 << n
    INF = float("inf")
    # dp[mask][j]: 已访问 mask (不含 home), 停在 j (1..n) 的最小累计能耗
    dp = [[INF] * (n + 1) for _ in range(size)]
    parent = [[-1] * (n + 1) for _ in range(size)]

    # 初始化: home -> j
    for j in range(1, n + 1):
        energy = euclidean_distance(pts[0], pts[j]) * (
            drone.alpha + drone.beta * total_demand
        )
        if energy <= drone.battery_capacity + _EPS:
            dp[1 << (j - 1)][j] = energy

    # 每个 mask 的已投递需求量 (用于求出发载重)
    mask_demand = [0.0] * size
    for mask in range(1, size):
        low = mask & (-mask)
        mask_demand[mask] = mask_demand[mask ^ low] + demand[low.bit_length()]

    states = 0
    for mask in range(size):
        remaining_load = total_demand - mask_demand[mask]
        for j in range(1, n + 1):
            cur = dp[mask][j]
            if cur == INF or not (mask >> (j - 1)) & 1:
                continue
            states += 1
            for k in range(1, n + 1):
                if (mask >> (k - 1)) & 1:
                    continue
                energy = cur + euclidean_distance(pts[j], pts[k]) * (
                    drone.alpha + drone.beta * remaining_load
                )
                if energy > drone.battery_capacity + _EPS:
                    continue  # 中间点电量不足 -> 该转移不可行
                nxt = mask | (1 << (k - 1))
                if energy < dp[nxt][k]:
                    dp[nxt][k] = energy
                    parent[nxt][k] = j

    # 收尾: 返回 home (载重为 0, 仅 alpha 项)
    full = size - 1
    best, last = INF, -1
    for j in range(1, n + 1):
        if dp[full][j] == INF:
            continue
        total = dp[full][j] + euclidean_distance(pts[j], pts[0]) * drone.alpha
        if total <= drone.battery_capacity + _EPS and total < best:
            best, last = total, j

    if last < 0:
        return ExactBatteryResult(float("inf"), [], False, states)

    seq: list[int] = []
    mask, cur = full, last
    while cur != -1:
        seq.append(cur)
        nxt = parent[mask][cur]
        mask ^= 1 << (cur - 1)
        cur = nxt
    seq.reverse()

    return ExactBatteryResult(best, [targets[i - 1].id for i in seq], True, states)


# ======================================================================
# 加速版内部工具
# ======================================================================

def _distance_matrix(pts: list[GeoPoint]) -> np.ndarray:
    """构造 (n+1)×(n+1) 距离矩阵, 索引 0 = home

    刻意用 `energy_model.euclidean_distance` (math.sqrt) 逐元素填充而非 numpy 广播,
    以保证与参照实现 / 暴力枚举**逐位相同**的浮点结果 (n <= 20 时开销可忽略)。
    """
    m = len(pts)
    mat = np.empty((m, m), dtype=np.float64)
    for i in range(m):
        for j in range(m):
            mat[i, j] = euclidean_distance(pts[i], pts[j])
    return mat


def _subset_sums(size: int, n: int, values: np.ndarray) -> np.ndarray:
    """子集和 DP: `sums[mask] = Σ_{i ∈ mask} values[i]` (位 i-1 对应 values[i])

    向量化写法: 对第 i 位, 把数组 reshape 成若干长度 2^i 的块;
    块内下半 (不含位 i) 与上半 (含位 i) 逐元素对应, 直接相加即可, 无 gather。
    复杂度 O(n·2^n) 但全部在 numpy 内。

    **位序必须从高到低** (i = n → 1): 这样每个 mask 的求和顺序与参照实现
    `mask_demand[mask] = mask_demand[mask ^ low] + demand[low]` (low = 最低位)
    完全一致 —— 两者都是"按位从高到低累加"。顺序一致才能保证浮点结果
    **逐位相同** (否则会有 ~1 ULP 的差异, 足以让 1e-9 级断言失败)。
    """
    sums = np.zeros(size, dtype=np.float64)
    for i in range(n, 0, -1):
        bit = 1 << (i - 1)
        blocks = sums.reshape(-1, 2 * bit)
        blocks[:, bit:] = blocks[:, :bit] + values[i]
    return sums


def _route_energy_exact(
    order: list[int],
    dmat: np.ndarray,
    demand: np.ndarray,
    total_demand: float,
    drone: DroneSpec,
) -> float | None:
    """按给定访问顺序精确模拟能耗; 任一中间点电量不足则返回 None

    与参照实现的累加顺序/公式逐位一致 (逐段 `+= d*(alpha+beta*load)`)。
    """
    load = total_demand
    cur = 0
    energy = 0.0
    for k in order:
        energy += dmat[cur, k] * (drone.alpha + drone.beta * load)
        if energy > drone.battery_capacity + _EPS:
            return None
        load -= demand[k]
        cur = k
    energy += dmat[cur, 0] * drone.alpha
    if energy > drone.battery_capacity + _EPS:
        return None
    return float(energy)


def route_energy_upper_bound(
    sequence: list[str],
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> float:
    """给定访问顺序的**精确能耗** (供剪枝上界 UB 使用)

    与 `solve_energy_exact_battery` 用同一算术口径 (`total_demand - mask_demand[mask]`),
    故可作为合法上界参与剪枝而不会因 ~1 ULP 偏差误剪最优解。

    **不要**改用 `RoutePlan.total_energy_consumed`: 它保留 2 位小数, 可能比真值小
    0.005, 而 UB 一旦低于真实最优就会被误剪。

    Args:
        sequence: 访问顺序 (target id 列表, 不含 home)
        targets: 目标点列表
        home: 仓库位置
        drone: 无人机规格

    Returns:
        float: 该顺序的总能耗 (Wh); 顺序不是 targets 的完整排列、或违反电量约束时
            返回 `math.inf` (= "没有可用上界", 等价于不剪枝)
    """
    n = len(targets)
    order = _order_from_ids(sequence, targets)
    if order is None:
        return _INF
    if n == 0:
        return 0.0
    pts = [home] + [t.location for t in targets]
    demand = np.array([0.0] + [t.demand for t in targets], dtype=np.float64)
    total_demand = sum([0.0] + [t.demand for t in targets])
    if total_demand > drone.payload_capacity + _EPS:
        return _INF
    value = _route_energy_exact(
        order, _distance_matrix(pts), demand, total_demand, drone
    )
    return value if value is not None else _INF


def _order_from_ids(sequence: list[str], targets: list[Target]) -> list[int] | None:
    """把 id 序列翻译成 1..n 的索引序列; 不是完整排列时返回 None"""
    pos = {t.id: i + 1 for i, t in enumerate(targets)}
    out: list[int] = []
    for tid in sequence:
        idx = pos.get(tid)
        if idx is None:
            return None
        out.append(idx)
    if sorted(out) != list(range(1, len(targets) + 1)):
        return None
    return out


def _greedy_nn_upper_bound(
    dmat: np.ndarray,
    demand: np.ndarray,
    total_demand: float,
    drone: DroneSpec,
    n: int,
) -> float | None:
    """贪心 NN 多起点兜底上界 (O(n³), n<=20 开销可忽略)

    仅用于 `plan_multistop` 不可行时给剪枝提供一个可行 UB; 找不到可行解返回 None。
    """
    alpha, beta = drone.alpha, drone.beta
    cap = drone.battery_capacity
    best: float | None = None
    for start in range(1, n + 1):
        unvis = set(range(1, n + 1))
        cur, load, energy = 0, total_demand, 0.0
        ok = True
        while unvis:
            picked = -1
            for k in sorted(unvis, key=lambda x: dmat[cur, x]):
                inc = dmat[cur, k] * (alpha + beta * load)
                if energy + inc <= cap + _EPS:
                    picked = k
                    break
            if picked < 0:
                ok = False
                break
            energy += dmat[cur, picked] * (alpha + beta * load)
            load -= demand[picked]
            cur = picked
            unvis.discard(picked)
        if not ok:
            continue
        energy += dmat[cur, 0] * alpha
        if energy <= cap + _EPS and (best is None or energy < best):
            best = float(energy)
    return best


def _initial_upper_bound(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    dmat: np.ndarray,
    demand: np.ndarray,
    total_demand: float,
) -> float:
    """初始上界 UB: 启发式可行解的**精确**能耗 (不是 RoutePlan 里四舍五入的两位小数)

    UB 必须是"某个真实可行解的能耗", 否则剪枝可能丢掉最优解。
    故此处取序列后用 `_route_energy_exact` 重算, 而不是直接采信
    `RoutePlan.total_energy_consumed` (其保留 2 位小数, 可能比真值小 0.005)。
    """
    n = len(targets)
    try:
        plan = plan_multistop(targets, home, drone)
    except Exception:  # 启发式对输入敏感, 任何异常都退化为"无 UB"
        plan = None
    if plan is not None and plan.feasible:
        order = _order_from_ids(plan.sequence, targets)
        if order is not None:
            value = _route_energy_exact(order, dmat, demand, total_demand, drone)
            if value is not None:
                return value
    fallback = _greedy_nn_upper_bound(dmat, demand, total_demand, drone, n)
    return fallback if fallback is not None else _INF


def _layer_lower_bound(
    masks: np.ndarray,
    sub_w: np.ndarray,
    w_total: float,
    d_home: np.ndarray,
    alpha: float,
    n: int,
) -> np.ndarray:
    """某一层所有 mask 的可采纳下界 LB(mask) —— 剩余未完成部分的能耗下界

    定义 (S = 未访问点集合 = complement(mask)):

        LB(mask) = Σ_{i∈S} (α + β·q_i)·minin(i)  +  α · min_{i∈S} d(i, home)

    其中 minin(i) = min_{i'≠i} d(i', i) (对全部点取最小, 含已访问点与 home)。

    **可采纳性证明** (即 LB <= 真实剩余能耗, 故不会剪掉最优解):

    设剩余航段为 j → v₁ → … → v_m → home, 其中 S = {v₁..v_m}, 共 m+1 段。

    1. α 部分: 真实剩余位移 α·Σ_ℓ d_ℓ。m 个"到达某个 i∈S"的航段与 S 一一对应,
       第 i 个航段长度 >= minin(i); 返航段起点 v_m ∈ S, 长度 >= min_{i∈S} d(i,home)。
       故 α·Σd_ℓ >= α·[Σ_{i∈S} minin(i) + min_{i∈S} d(i,home)]。
    2. β 部分: 真实为 β·Σ_ℓ d_ℓ·L_ℓ (L_ℓ = 该航段出发时载重)。记 ℓ_i 为抵达 i 的航段,
       出发时 i 尚未投递, 故 L_{ℓ_i} >= q_i, 于是 β·d_{ℓ_i}·L_{ℓ_i} >= β·q_i·minin(i)。
       各 ℓ_i (i∈S) 互不相同, 且返航段载重为 0 (全部已投递) 对 β 部分贡献 0。
       故 β 部分 >= β·Σ_{i∈S} q_i·minin(i)。

    两者相加即得 LB, 且 (α + β·q_i)·minin(i) 的求和已由子集和数组 `sub_w` 预计算。
    注: 该下界严格强于"一律用 α 乘距离"的版本 (后者是本式令 β=0 的特例)。

    Args:
        masks: 该层的 mask 数组 (int32/int64)
        sub_w: 子集和数组, sub_w[mask] = Σ_{i∈mask} (α+β·q_i)·minin(i)
        w_total: Σ_{i=1..n} (α+β·q_i)·minin(i)
        d_home: 长度 n+1 的数组, d_home[i] = d(i, home)
        alpha: 空载能耗率
        n: 目标点数

    Returns:
        np.ndarray: 长度 = len(masks) 的下界数组; S 为空 (满 mask) 时返航项取 0
    """
    lb = w_total - sub_w[masks]
    best_home = np.full(masks.shape[0], _INF, dtype=np.float64)
    for i in range(1, n + 1):
        unvis = ((masks >> (i - 1)) & 1) == 0
        np.minimum(best_home, np.where(unvis, d_home[i], _INF), out=best_home)
    # 满 mask (S 为空) 时 best_home 仍为 +inf, 但满 mask 层不做剪枝, 不会用到
    return lb + alpha * best_home


def _solve_layered(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_points: int,
    use_pruning: bool,
    ub_override: float | None,
) -> ExactBatteryResult:
    """分层向量化 + 上界剪枝的精确求解内核 (语义与参照实现完全一致)"""
    n = len(targets)
    if n == 0:
        return ExactBatteryResult(0.0, [], True, 0)
    if n > max_points:
        raise ValueError(
            f"target count {n} exceeds exact-battery limit {max_points}"
        )

    pts = [home] + [t.location for t in targets]
    demand_list = [0.0] + [t.demand for t in targets]
    demand = np.array(demand_list, dtype=np.float64)
    # 与参照实现同序求和 (Python 顺序累加, 不用 numpy 的分段求和):
    # total_demand 参与 `total_demand - mask_demand` 与能耗率计算, 顺序不同会有 ~1 ULP 偏差
    total_demand = sum(demand_list)
    if total_demand > drone.payload_capacity + _EPS:
        return ExactBatteryResult(float("inf"), [], False, 0)

    alpha = float(drone.alpha)
    beta = float(drone.beta)
    cap = float(drone.battery_capacity)
    dmat = _distance_matrix(pts)
    d_home = dmat[:, 0].copy()

    # --- 上界 UB (剪枝用) ---
    if ub_override is not None:
        ub = float(ub_override)
    elif use_pruning:
        ub = _initial_upper_bound(targets, home, drone, dmat, demand, total_demand)
    else:
        ub = _INF

    size = 1 << n
    # --- 按 popcount 分层: 第 k 层 = 恰好访问 k 个点的 mask (升序 = colex 序) ---
    masks_all = np.arange(size, dtype=np.int32)
    popcnt = np.bitwise_count(masks_all)
    layers = [masks_all[popcnt == k] for k in range(n + 1)]
    del masks_all, popcnt
    # mask -> 该 mask 在所属层数组中的行号 (层内压实后会就地覆盖)
    row_of_mask = np.full(size, -1, dtype=np.int32)
    for k in range(n + 1):
        row_of_mask[layers[k]] = np.arange(layers[k].shape[0], dtype=np.int32)

    # --- 剪枝用预计算 ---
    off_diag = dmat.copy()
    np.fill_diagonal(off_diag, _INF)
    min_in = off_diag.min(axis=0)                       # minin(i) = min_{i'≠i} d(i',i)
    w = (alpha + beta * demand) * min_in                # 每点的"入边下界"权重
    w[0] = 0.0                                          # home 不是投递点
    sub_w = _subset_sums(size, n, w)
    w_total = float(w[1:].sum())
    sub_q = _subset_sums(size, n, demand)

    par_dtype = np.int8 if n <= _PARENT_INT8_MAX_N else np.int32

    dp_cur = np.full((1, n + 1), _INF, dtype=np.float64)
    dp_cur[0, 0] = 0.0                                  # (空集, 停在 home)
    masks_cur = layers[0]
    parents: list[np.ndarray | None] = [None] * (n + 1)

    explored = 0
    pruned = 0
    cols_idx_cache: dict[int, np.ndarray] = {}

    for k_cur in range(n):
        masks_next = layers[k_cur + 1]
        m_next = masks_next.shape[0]
        rem_load = total_demand - sub_q[masks_cur]      # (M,) 出发载重
        factor = alpha + beta * rem_load                # (M,) 该层每状态的能耗率

        dp_next = np.full((m_next, n + 1), _INF, dtype=np.float64)
        par_next = np.full((m_next, n + 1), -1, dtype=par_dtype)

        for kk in range(1, n + 1):
            bit = 1 << (kk - 1)
            sel = (masks_cur & bit) == 0
            if not sel.any():
                continue
            subm = masks_cur[sel]
            subdp = dp_cur[sel]
            subf = factor[sel]
            # (m, n+1) 批量: 每个状态 j 转移到 kk 的累计能耗
            cols = dmat[:, kk][None, :] * subf[:, None]
            cols += subdp
            cols[cols > cap + _EPS] = _INF              # 电量不足 -> 不可行
            best_j = np.argmin(cols, axis=1)            # 并列时取最小 j (与参照一致)
            m = cols.shape[0]
            idx = cols_idx_cache.get(m)
            if idx is None:
                idx = np.arange(m)
                cols_idx_cache[m] = idx
            best_v = cols[idx, best_j]
            r = row_of_mask[subm | bit]
            dp_next[r, kk] = best_v
            par_next[r, kk] = best_j.astype(par_dtype)

        finite = np.isfinite(dp_next)
        if use_pruning and k_cur + 1 < n:
            lb = _layer_lower_bound(masks_next, sub_w, w_total, d_home, alpha, n)
            keep = (dp_next + lb[:, None]) <= ub + _EPS
        else:
            keep = finite
        pruned += int(np.count_nonzero(finite & ~keep))
        alive = np.any(finite & keep, axis=1)           # 整行压实 (省内存 + 省算力)
        dp_cur = dp_next[alive]
        par_next = par_next[alive]
        masks_cur = masks_next[alive]
        row_of_mask[masks_cur] = np.arange(masks_cur.shape[0], dtype=np.int32)
        parents[k_cur + 1] = par_next
        explored += int(np.count_nonzero(np.isfinite(dp_cur)))

    if masks_cur.shape[0] == 0:
        return ExactBatteryResult(_INF, [], False, explored, pruned, ub)

    # --- 收尾: 返回 home (载重 0, 仅 alpha 项) ---
    close = dp_cur[0, :] + d_home * alpha
    close[close > cap + _EPS] = _INF
    best_j = int(np.argmin(close))
    best = float(close[best_j])
    if not np.isfinite(best):
        return ExactBatteryResult(_INF, [], False, explored, pruned, ub)

    # --- 回溯 (父指针只存"上一个点", 上一个 mask = mask ^ bit_j) ---
    seq_idx: list[int] = []
    mask = size - 1
    cur = best_j
    for _ in range(n):
        seq_idx.append(cur)
        prev = int(parents[mask.bit_count()][row_of_mask[mask], cur])
        mask ^= 1 << (cur - 1)
        cur = prev
    seq_idx.reverse()

    return ExactBatteryResult(
        best, [targets[i - 1].id for i in seq_idx], True, explored, pruned, ub
    )


def solve_energy_exact_battery(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_points: int = MAX_EXACT_BATTERY_POINTS,
    *,
    use_pruning: bool = True,
    vectorized: bool = True,
    ub_override: float | None = None,
) -> ExactBatteryResult:
    """带电池约束的能量最优精确 DP —— 分层向量化 + 上界剪枝加速版

    语义 = 参照实现 (同样的输入 -> 同样的最优能耗与访问序列),
    仅追加了三个**可选**开关 (默认启用加速路径):

      - `vectorized=True` (默认): 走分层向量化实现;
        置 False 时直接调用参照实现 `solve_energy_exact_battery_reference`
        (朴素三重循环), 用于等价性对照。
      - `use_pruning=True` (默认): 启用"启发式上界 + 可采纳下界"剪枝;
        置 False 则只做向量化、不剪枝 (用于隔离剪枝的贡献)。
        剪枝不改变最优解: 详见 `_layer_lower_bound` 的可采纳性证明。
      - `ub_override`: **协同/诊断钩子**, 直接指定剪枝用的初始上界
        (生产侧由 `a3_python.adaptive` 传入降级解或启发式解的精确能耗,
        见 `route_energy_upper_bound`)。调用方必须保证它是某个真实可行解的能耗
        (即 >= 真实最优), 否则可能丢最优解; 传 None (默认) 时由启发式自动求取。
        该参数亦用于构造"上界恰好等于最优值"的剪枝边界用例。

    Args:
        targets: 目标点列表 (1 ~ max_points 个)
        home: 仓库位置
        drone: 无人机规格 (alpha/beta/battery_capacity/payload_capacity)
        max_points: 允许的最大目标点数 (默认 MAX_EXACT_BATTERY_POINTS = 20)

    Returns:
        ExactBatteryResult: 最优能耗与序列; 无可行解时 feasible=False

    Raises:
        ValueError: 目标点数超过 max_points
    """
    if not vectorized:
        return solve_energy_exact_battery_reference(
            targets, home, drone, max_points
        )
    return _solve_layered(targets, home, drone, max_points, use_pruning, ub_override)
