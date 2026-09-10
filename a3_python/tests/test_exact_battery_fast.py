"""带电池约束的精确 DP — 分层向量化 + 上界剪枝 等价性单测

覆盖:
  正例 / 一致性 (与暴力枚举 + 参照实现逐位一致) / 剪枝边界 (最优解贴近上界) /
  剪枝下界可采纳性直接验证 / 开关等价性 / 大 n 自洽 / 退化 / 边界 / 回归

**浮点口径说明 (很重要)**:
  本模块的"出发载重"按参照实现的写法 = `total_demand - mask_demand[mask]`,
  其中 `mask_demand` 是子集和递推 `md[mask] = md[mask ^ low] + q[low]` (low = 最低位)。
  这是本模块**定义的**算术口径 —— 改造后的实现刻意复刻它, 故与参照实现逐位相同。

  另有一种数学上等价但浮点路径不同的口径: 载重逐点累减 (`load -= q`)。
  它被 `energy_model.simulate_route_energy` 使用, 与前者可能差 ~1 ULP (1e-13 量级)。
  因此本文件: 逐位断言一律用子集和口径; 对逐点累减口径另做 1e-9 容差断言,
  两种口径都成立才说明实现是对的。
"""
import itertools
import math

import numpy as np
import pytest

from a3_python.route import DroneSpec, GeoPoint, Target
from a3_python.energy_model import euclidean_distance
from a3_python.data_generator import generate_scenario
from a3_python.solver import plan_multistop
from a3_python.exact_battery import (
    MAX_EXACT_BATTERY_POINTS,
    _distance_matrix,
    _layer_lower_bound,
    _subset_sums,
    route_energy_upper_bound,
    solve_energy_exact_battery,
    solve_energy_exact_battery_reference,
)

DRONE = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0,
                  alpha=0.1, beta=0.005)
DRONE_TIGHT = DroneSpec(payload_capacity=50.0, battery_capacity=400.0,
                        alpha=0.1, beta=0.005)
DRONE_MID = DroneSpec(payload_capacity=50.0, battery_capacity=2000.0,
                      alpha=0.1, beta=0.005)
DRONE_HUGE = DroneSpec(payload_capacity=50.0, battery_capacity=10_000_000.0,
                       alpha=0.1, beta=0.005)


# ==================== 测试用独立实现 ====================

def _mask_demands(targets: list[Target]) -> list[float]:
    """mask -> Σ_{i∈mask} q_i, 复刻参照实现的递推 (按最低位展开), 保证同序同值"""
    dem = [0.0] + [t.demand for t in targets]
    md = [0.0] * (1 << len(targets))
    for mask in range(1, 1 << len(targets)):
        low = mask & (-mask)
        md[mask] = md[mask ^ low] + dem[low.bit_length()]
    return md


def _route_energy(ids, targets, home, drone, md) -> float:
    """按 id 序列逐段累加能耗 (子集和口径: 出发载重 = total - md[mask])"""
    tlist = [t.id for t in targets]
    pts = [home] + [t.location for t in targets]
    total = sum([0.0] + [t.demand for t in targets])
    energy, mask, cur = 0.0, 0, 0
    for tid in ids:
        k = 1 + tlist.index(tid)
        energy += euclidean_distance(pts[cur], pts[k]) * (
            drone.alpha + drone.beta * (total - md[mask]))
        mask |= 1 << (k - 1)
        cur = k
    energy += euclidean_distance(pts[cur], pts[0]) * drone.alpha
    return energy


def _route_energy_seq_load(ids, targets, home, drone) -> float:
    """另一种等价浮点口径: 载重逐点累减 (可能差 ~1 ULP), 仅做 1e-9 容差断言"""
    tlist = [t.id for t in targets]
    tmap = {t.id: t for t in targets}
    pts = [home] + [t.location for t in targets]
    energy, load, cur = 0.0, sum(t.demand for t in targets), 0
    for tid in ids:
        k = 1 + tlist.index(tid)
        energy += euclidean_distance(pts[cur], pts[k]) * (
            drone.alpha + drone.beta * load)
        load -= tmap[tid].demand
        cur = k
    energy += euclidean_distance(pts[cur], pts[0]) * drone.alpha
    return energy


def _brute_force(targets, home, drone, md):
    """暴力枚举所有排列 (子集和载重口径); 无可行解返回 (None, None)"""
    n = len(targets)
    pts = [home] + [t.location for t in targets]
    dem = [0.0] + [t.demand for t in targets]
    total = sum(dem)
    if total > drone.payload_capacity:
        return None, None
    best, best_perm = math.inf, None
    for perm in itertools.permutations(range(1, n + 1)):
        energy, mask, cur, ok = 0.0, 0, 0, True
        for k in perm:
            energy += euclidean_distance(pts[cur], pts[k]) * (
                drone.alpha + drone.beta * (total - md[mask]))
            if energy > drone.battery_capacity + 1e-9:
                ok = False
                break
            mask |= 1 << (k - 1)
            cur = k
        if not ok:
            continue
        energy += euclidean_distance(pts[cur], pts[0]) * drone.alpha
        if energy > drone.battery_capacity + 1e-9:
            continue
        if energy < best:
            best, best_perm = energy, list(perm)
    if best_perm is None:
        return None, None
    return best, [targets[i - 1].id for i in best_perm]


def _assert_triple_agreement(targets, home, drone, tag=""):
    """改造后 vs 参照实现 vs 暴力枚举: 可行性 + 最优能耗 + 序列三重逐位一致"""
    md = _mask_demands(targets)
    fast = solve_energy_exact_battery(targets, home, drone)
    ref = solve_energy_exact_battery_reference(targets, home, drone)
    bf, bf_ids = _brute_force(targets, home, drone, md)

    assert fast.feasible == ref.feasible, f"可行性不一致 {tag}"
    assert fast.feasible == (bf is not None), f"暴力枚举可行性不一致 {tag}"
    if not fast.feasible:
        return
    # 逐位 (bitwise) 一致 —— 不是 approx
    assert fast.objective == ref.objective, (
        f"目标值非逐位一致 {tag}: fast={fast.objective!r} ref={ref.objective!r}")
    assert fast.objective == bf, (
        f"目标值与暴力枚举非逐位一致 {tag}: fast={fast.objective!r} bf={bf!r}")
    assert fast.sequence == ref.sequence, f"序列与参照实现不一致 {tag}"
    assert fast.sequence == bf_ids, f"序列与暴力枚举不一致 {tag}"
    assert sorted(fast.sequence) == sorted(t.id for t in targets)
    # 独立复算 (子集和口径 -> 逐位; 逐点累减口径 -> 1e-9)
    assert _route_energy(fast.sequence, targets, home, drone, md) == fast.objective
    assert _route_energy_seq_load(fast.sequence, targets, home, drone) == \
        pytest.approx(fast.objective, abs=1e-9)
    assert fast.objective <= drone.battery_capacity + 1e-9


# ==================== 一致性: 与暴力枚举 + 参照实现逐位一致 ====================

@pytest.mark.parametrize("n", [5, 6, 7])
@pytest.mark.parametrize("distribution", ["random", "circle", "cluster"])
@pytest.mark.parametrize("seed", [0, 1])
def test_fast_matches_brute_and_reference(n, distribution, seed):
    """宽松电量: 三者逐位一致"""
    home, tg = generate_scenario(n=n, distribution=distribution, seed=seed,
                                 scale=800.0, demand_range=(1.0, 3.0))
    _assert_triple_agreement(tg, home, DRONE, f"n={n}/{distribution}/s{seed}/wide")


def test_fast_matches_brute_n8():
    """n=8 上限档: 与暴力枚举逐位一致"""
    home, tg = generate_scenario(n=8, distribution="random", seed=0,
                                 scale=800.0, demand_range=(1.0, 3.0))
    _assert_triple_agreement(tg, home, DRONE, "n=8")


@pytest.mark.parametrize("n", [9, 10])
def test_fast_matches_reference_larger_n(n):
    """较大 n: 与参照实现逐位一致 (暴力枚举已不可行)"""
    home, tg = generate_scenario(n=n, distribution="random", seed=1,
                                 scale=1000.0, demand_range=(1.0, 3.0))
    fast = solve_energy_exact_battery(tg, home, DRONE, max_points=n)
    ref = solve_energy_exact_battery_reference(tg, home, DRONE, max_points=n)
    assert fast.feasible == ref.feasible
    assert fast.objective == ref.objective
    assert fast.sequence == ref.sequence


# ==================== route_energy_upper_bound (剪枝上界公共接口) ====================

def test_upper_bound_matches_objective_for_optimal_sequence():
    """最优序列的精确能耗 == 精确解目标值 (1e-9 容差, 见文件头的浮点口径说明)

    此处刻意不用 <= 逐位断言: `route_energy_upper_bound` 走"载重逐点累减"口径
    (`_route_energy_exact`), 而分层 DP 走"子集和"口径, 两者可差 ~1 ULP (1e-13)。
    该偏差对剪枝是安全的: 判据含 `+ _EPS = 1e-9` 容差, 且 UB 偏 1 ULP 高于真值
    只会削弱剪枝、不会误剪最优解。
    """
    home, tg = generate_scenario(n=6, distribution="random", seed=0,
                                 scale=800.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert r.feasible
    ub = route_energy_upper_bound(r.sequence, tg, home, DRONE)
    assert ub == pytest.approx(r.objective, abs=1e-9)
    # 上界绝不低于最优值 (否则剪枝会误剪最优解)
    assert ub >= r.objective - 1e-9


def test_upper_bound_is_valid_for_heuristic_sequence():
    """启发式序列的精确能耗 >= 最优值 (合法上界, 剪枝不会误剪最优)"""
    home, tg = generate_scenario(n=7, distribution="random", seed=2,
                                 scale=1000.0, demand_range=(1.0, 3.0))
    plan = plan_multistop(tg, home, DRONE)
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert plan.feasible and r.feasible
    ub = route_energy_upper_bound(plan.sequence, tg, home, DRONE)
    assert math.isfinite(ub)
    assert ub >= r.objective - 1e-9


def test_upper_bound_rejects_incomplete_sequence():
    """非完整排列 / 未知 id: 返回 +inf (= 无可用上界)"""
    home, tg = generate_scenario(n=4, distribution="random", seed=0,
                                 scale=800.0, demand_range=(1.0, 3.0))
    ids = [t.id for t in tg]
    assert route_energy_upper_bound(ids[:-1], tg, home, DRONE) == math.inf
    assert route_energy_upper_bound(ids + ["nope"], tg, home, DRONE) == math.inf


def test_upper_bound_infeasible_sequence_returns_inf():
    """电量不足的顺序: 返回 +inf (不可行解不能当上界)"""
    home, tg = generate_scenario(n=5, distribution="circle", seed=0,
                                 scale=2000.0, demand_range=(1.0, 3.0))
    ids = [t.id for t in tg]
    assert route_energy_upper_bound(ids, tg, home, DRONE_TIGHT) == math.inf


def test_upper_bound_empty_targets():
    """空目标集: 能耗为 0"""
    assert route_energy_upper_bound([], [], GeoPoint(x=0.0, y=0.0), DRONE) == 0.0


# ==================== 电量约束的性质 (结构性事实) ====================

def _cap_drone(cap: float) -> DroneSpec:
    """同机型但把电池容量设为 cap"""
    return DroneSpec(payload_capacity=DRONE.payload_capacity,
                     battery_capacity=cap, alpha=DRONE.alpha, beta=DRONE.beta)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_battery_constraint_is_feasibility_gate_only(seed):
    """电量约束在本问题里只是**可行性闸门**, 不改变最优解

    证明 (三行): 目标 = 全程总能耗; 累计能耗单调不减, 故"任意时刻 ≤ cap"
    ⟺ "总能耗 ≤ cap"。于是设无约束最优为 OPT:
      * cap ≥ OPT ⇒ 最优解本身可行 ⇒ 约束解 = 无约束解;
      * cap < OPT ⇒ 任何解的总能耗 ≥ OPT > cap ⇒ 无可行解。
    本用例把这条性质钉死: cap 恰等于 OPT 时可解且解不变, cap 略低即不可行。
    """
    home, tg = generate_scenario(n=6, distribution="random", seed=seed,
                                 scale=800.0, demand_range=(1.0, 3.0))
    unconstrained = solve_energy_exact_battery_reference(tg, home, DRONE_HUGE)
    assert unconstrained.feasible
    opt = unconstrained.objective

    at_opt = solve_energy_exact_battery(tg, home, _cap_drone(opt))
    assert at_opt.feasible
    assert at_opt.objective == opt
    assert at_opt.sequence == unconstrained.sequence

    assert not solve_energy_exact_battery(tg, home, _cap_drone(opt - 1e-6)).feasible
    assert not solve_energy_exact_battery(tg, home, _cap_drone(opt * 0.5)).feasible
    assert not solve_energy_exact_battery(
        tg, home, _cap_drone(opt * 0.5), use_pruning=False).feasible


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_triple_agreement_at_exact_optimum_cap(seed):
    """电量上限恰好 = 最优能耗: 三路 (含暴力枚举) 逐位一致

    这是"约束最紧但仍可行"的极端点 —— 只有最优路线能过关。
    """
    home, tg = generate_scenario(n=6, distribution="random", seed=seed,
                                 scale=800.0, demand_range=(1.0, 3.0))
    opt = solve_energy_exact_battery_reference(tg, home, DRONE_HUGE).objective
    _assert_triple_agreement(tg, home, _cap_drone(opt), f"cap=OPT/s{seed}")


def test_matches_unconstrained_exact_dp():
    """与 MVP 无约束精确 DP (`exact.solve_energy_exact_dp`) 的最优序列一致

    这条断言把上面的结构性事实钉进回归: 既然电量约束只是可行性闸门,
    "带电池能量最优"的最优序列必须与 MVP 已有的无约束精确 DP 完全相同
    (差异仅在: 本模块还能判定 cap < OPT 时**不可行**)。
    """
    from a3_python.exact import solve_energy_exact_dp

    for n in (5, 8, 10):
        for seed in range(2):
            home, tg = generate_scenario(n=n, distribution="random", seed=seed,
                                         scale=1000.0, demand_range=(1.0, 3.0))
            dp = solve_energy_exact_dp(tg, home, DRONE_HUGE)
            bat = solve_energy_exact_battery(tg, home, DRONE_HUGE, max_points=n)
            assert bat.feasible
            assert bat.sequence == dp.sequence
            # 目标值 (能耗) 与按该序列独立复算的结果逐位相同
            assert bat.objective == _route_energy(
                dp.sequence, tg, home, DRONE_HUGE, _mask_demands(tg))


def test_infeasible_agreement():
    """电量远小于最优能耗导致完全不可行: 三路判定一致"""
    home, tg = generate_scenario(n=6, distribution="circle", seed=0,
                                 scale=1500.0, demand_range=(1.0, 3.0))
    md = _mask_demands(tg)
    fast = solve_energy_exact_battery(tg, home, DRONE_TIGHT)
    ref = solve_energy_exact_battery_reference(tg, home, DRONE_TIGHT)
    bf, _ = _brute_force(tg, home, DRONE_TIGHT, md)
    assert not fast.feasible and not ref.feasible and bf is None
    assert fast.objective == ref.objective == float("inf")


# ==================== 剪枝边界: 最优解恰好贴近上界 ====================

@pytest.mark.parametrize("n", [5, 6, 7, 8])
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_pruning_boundary_ub_exactly_optimal(n, seed):
    """剪枝边界: 强行令初始上界 UB == 真实最优值 (最紧的合法上界)

    此时最优路径的每个状态都恰好取到 `cost + LB <= UB` 的边界,
    若剪枝判据写成 `>= UB` 就会把最优解剪掉 —— 本用例专门守住这条线。
    """
    home, tg = generate_scenario(n=n, distribution="random", seed=seed,
                                 scale=800.0, demand_range=(1.0, 3.0))
    ref = solve_energy_exact_battery_reference(tg, home, DRONE)
    assert ref.feasible
    boundary = solve_energy_exact_battery(tg, home, DRONE, ub_override=ref.objective)
    assert boundary.feasible
    assert boundary.objective == ref.objective
    assert boundary.sequence == ref.sequence
    assert boundary.ub_initial == ref.objective


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_pruning_boundary_battery_equals_optimum(seed):
    """最紧的边界组合: 电量上限 = 最优能耗 **且** 剪枝上界 = 最优能耗

    此时每个中间状态的累计能耗都恰好顶到电量上限, 且剪枝判据取到等号边界 ——
    任何一边写错 (如把 `<= ub + eps` 写成 `< ub`) 都会丢最优解。
    """
    home, tg = generate_scenario(n=6, distribution="random", seed=seed,
                                 scale=800.0, demand_range=(1.0, 3.0))
    opt = solve_energy_exact_battery_reference(tg, home, DRONE_HUGE).objective
    ref = solve_energy_exact_battery_reference(tg, home, _cap_drone(opt))
    assert ref.feasible
    boundary = solve_energy_exact_battery(tg, home, _cap_drone(opt),
                                          ub_override=opt)
    assert boundary.objective == ref.objective
    assert boundary.sequence == ref.sequence
    assert boundary.pruned_states > 0, "边界用例应确实发生了剪枝"


def test_pruning_boundary_at_larger_n():
    """n=10 上界取最优值: 剪枝生效且不误剪"""
    home, tg = generate_scenario(n=10, distribution="random", seed=3,
                                 scale=1000.0, demand_range=(1.0, 3.0))
    ref = solve_energy_exact_battery_reference(tg, home, DRONE, max_points=10)
    assert ref.feasible
    boundary = solve_energy_exact_battery(tg, home, DRONE, max_points=10,
                                          ub_override=ref.objective)
    assert boundary.objective == ref.objective
    assert boundary.sequence == ref.sequence
    assert boundary.pruned_states > 0


# ==================== 下界可采纳性 (admissibility) 直接验证 ====================

@pytest.mark.parametrize("n", [5, 6])
def test_lower_bound_is_admissible_for_every_mask(n):
    """对每个 (mask, j) 直接验证: LB(mask) <= 真实剩余最优能耗

    真实剩余最优 = 枚举 S 的全部排列 (从 j 出发, 回到 home), 取最小。
    这是剪枝正确性的根 —— 只要 LB 可采纳, 剪枝就不会丢最优解。
    """
    home, tg = generate_scenario(n=n, distribution="random", seed=2,
                                 scale=800.0, demand_range=(1.0, 3.0))
    drone = DRONE
    pts = [home] + [t.location for t in tg]
    demand = np.array([0.0] + [t.demand for t in tg])
    total = sum([0.0] + [t.demand for t in tg])
    dmat = _distance_matrix(pts)
    dist = dmat.tolist()     # 纯 Python 副本: 内层枚举用, 避免 numpy 标量索引开销
    dem = demand.tolist()
    size = 1 << n

    alpha, beta = drone.alpha, drone.beta
    off = dmat.copy()
    np.fill_diagonal(off, np.inf)
    w = (alpha + beta * demand) * off.min(axis=0)
    w[0] = 0.0
    sub_w = _subset_sums(size, n, w)
    w_total = float(w[1:].sum())

    for mask in range(size):
        unvis = [i for i in range(1, n + 1) if not (mask >> (i - 1)) & 1]
        if not unvis:
            continue  # 满 mask 不做剪枝
        lb = float(_layer_lower_bound(
            np.array([mask], dtype=np.int32), sub_w, w_total,
            dmat[:, 0], alpha, n)[0])
        depart_load = total - float(sum(dem[i] for i in range(1, n + 1)
                                        if (mask >> (i - 1)) & 1))
        for j in ([0] if mask == 0 else [i for i in range(1, n + 1)
                                         if (mask >> (i - 1)) & 1]):
            best_rem = math.inf
            for perm in itertools.permutations(unvis):
                e, load, cur = 0.0, depart_load, j
                for k in perm:
                    e += dist[cur][k] * (alpha + beta * load)
                    load -= dem[k]
                    cur = k
                e += dist[cur][0] * alpha
                best_rem = min(best_rem, e)
            assert lb <= best_rem + 1e-12, (
                f"下界不可采纳! mask={mask} j={j} LB={lb} > 真实剩余最优={best_rem}")


# ==================== 开关等价性 ====================

@pytest.mark.parametrize("n", [6, 8, 10, 12])
@pytest.mark.parametrize("seed", [0, 1])
def test_pruning_on_off_identical(n, seed):
    """剪枝开/关结果逐位一致 (剪枝只影响速度, 不影响解)"""
    home, tg = generate_scenario(n=n, distribution="random", seed=seed,
                                 scale=1000.0, demand_range=(1.0, 3.0))
    on = solve_energy_exact_battery(tg, home, DRONE, max_points=n, use_pruning=True)
    off = solve_energy_exact_battery(tg, home, DRONE, max_points=n, use_pruning=False)
    assert on.feasible == off.feasible
    assert on.objective == off.objective
    assert on.sequence == off.sequence
    assert off.pruned_states == 0


def test_vectorized_false_dispatches_to_reference():
    """vectorized=False 必须等价于参照实现 (含统计量)"""
    home, tg = generate_scenario(n=7, distribution="random", seed=4,
                                 scale=800.0, demand_range=(1.0, 3.0))
    off = solve_energy_exact_battery(tg, home, DRONE, vectorized=False)
    ref = solve_energy_exact_battery_reference(tg, home, DRONE)
    assert (off.objective, off.sequence, off.feasible, off.states_explored) == \
           (ref.objective, ref.sequence, ref.feasible, ref.states_explored)


def test_pruning_is_actually_active():
    """n=12 上剪枝必须真的发生 (否则该特性形同虚设)"""
    home, tg = generate_scenario(n=12, distribution="random", seed=0,
                                 scale=1000.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE, max_points=12)
    assert r.pruned_states > 0
    # 与不剪枝相比确实少扩展了状态
    off = solve_energy_exact_battery(tg, home, DRONE, max_points=12, use_pruning=False)
    assert r.states_explored < off.states_explored
    # 剪枝有成本: UB 必须是有限值才有意义
    assert math.isfinite(r.ub_initial)


# ==================== 大 n 自洽性 ====================

@pytest.mark.parametrize("n", [13, 15])
def test_large_n_self_consistent(n):
    """n=13/15: 剪枝开/关一致 + 独立复算目标值 + 电量约束成立"""
    home, tg = generate_scenario(n=n, distribution="random", seed=0,
                                 scale=1000.0, demand_range=(1.0, 3.0))
    md = _mask_demands(tg)
    on = solve_energy_exact_battery(tg, home, DRONE, max_points=n)
    off = solve_energy_exact_battery(tg, home, DRONE, max_points=n, use_pruning=False)
    assert on.feasible and off.feasible
    assert on.objective == off.objective
    assert on.sequence == off.sequence
    assert _route_energy(on.sequence, tg, home, DRONE, md) == on.objective
    assert on.objective <= DRONE.battery_capacity + 1e-9
    assert sorted(on.sequence) == sorted(t.id for t in tg)
    # 精确解不劣于启发式
    plan = plan_multistop(tg, home, DRONE)
    if plan.feasible:
        assert on.objective <= plan.total_energy_consumed + 5e-3


# ==================== 退化 / 边界 ====================

def test_empty_targets():
    """空目标集: 能耗 0, 无状态"""
    r = solve_energy_exact_battery([], GeoPoint(x=0.0, y=0.0), DRONE)
    assert r.feasible and r.sequence == [] and r.objective == 0.0
    assert r.states_explored == 0 and r.pruned_states == 0


def test_single_target_feasible():
    """单点: 与参照实现逐位一致"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id="t1", location=GeoPoint(x=100.0, y=0.0), demand=5.0)]
    fast = solve_energy_exact_battery(tg, home, DRONE)
    ref = solve_energy_exact_battery_reference(tg, home, DRONE)
    assert fast.feasible and fast.sequence == ["t1"]
    assert fast.objective == ref.objective


def test_payload_over_capacity_is_infeasible():
    """总需求超载重上限: 判定不可行 (剪枝路径也不得漏判)"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id=f"t{i}", location=GeoPoint(x=float(i * 10), y=0.0), demand=30.0)
          for i in range(1, 3)]
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert not r.feasible
    assert not solve_energy_exact_battery(tg, home, DRONE, use_pruning=False).feasible


def test_battery_too_small_is_infeasible():
    """电量过小: 无可行解 (剪枝开/关一致)"""
    home = GeoPoint(x=0.0, y=0.0)
    tiny = DroneSpec(payload_capacity=50.0, battery_capacity=1.0,
                     alpha=0.1, beta=0.005)
    tg = [Target(id="t1", location=GeoPoint(x=5000.0, y=0.0), demand=1.0)]
    assert not solve_energy_exact_battery(tg, home, tiny).feasible
    assert not solve_energy_exact_battery(tg, home, tiny, use_pruning=False).feasible


def test_exceeds_point_limit_raises():
    """超过点数上限: 两条路径都抛 ValueError"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id=f"t{i}", location=GeoPoint(x=float(i), y=0.0), demand=0.1)
          for i in range(MAX_EXACT_BATTERY_POINTS + 1)]
    with pytest.raises(ValueError):
        solve_energy_exact_battery(tg, home, DRONE)
    with pytest.raises(ValueError):
        solve_energy_exact_battery(tg, home, DRONE, use_pruning=False)


# ==================== 回归 ====================

def test_regression_heuristic_failure_case():
    """回归: tight_10p_s9 (启发式失败) —— 加速版仍能找到可行解且与参照一致"""
    home, tg = generate_scenario(n=10, distribution="circle", seed=9,
                                 scale=3000.0, demand_range=(1.0, 3.0))
    plan = plan_multistop(tg, home, DRONE)
    fast = solve_energy_exact_battery(tg, home, DRONE, max_points=10)
    ref = solve_energy_exact_battery_reference(tg, home, DRONE, max_points=10)
    assert not plan.feasible, "前提: 该案例启发式应失败"
    assert fast.feasible and ref.feasible
    assert fast.objective == ref.objective
    assert fast.sequence == ref.sequence


def test_regression_mid_battery_sweep():
    """回归: 中度电量下多种子扫描, 三路逐位一致"""
    for seed in range(5):
        home, tg = generate_scenario(n=7, distribution="random", seed=seed,
                                     scale=1200.0, demand_range=(1.0, 4.0))
        _assert_triple_agreement(tg, home, DRONE_MID, f"mid/s{seed}")


def test_regression_ub_override_from_heuristic_is_consistent():
    """回归 (自适应协同): 用启发式解的精确能耗作 UB, 最优解与自动 UB 完全相同

    这是 `a3_python.adaptive` 精确模式的协同基础 —— 上界只影响剪枝强度,
    不影响最优解。
    """
    for seed in range(3):
        home, tg = generate_scenario(n=8, distribution="random", seed=seed,
                                     scale=1000.0, demand_range=(1.0, 3.0))
        auto = solve_energy_exact_battery(tg, home, DRONE)
        plan = plan_multistop(tg, home, DRONE)
        assert plan.feasible
        ub = route_energy_upper_bound(plan.sequence, tg, home, DRONE)
        forced = solve_energy_exact_battery(tg, home, DRONE, ub_override=ub)
        assert forced.feasible
        assert forced.objective == auto.objective
        assert forced.sequence == auto.sequence
        assert forced.ub_initial == ub
