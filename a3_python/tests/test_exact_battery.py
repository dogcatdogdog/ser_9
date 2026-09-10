"""带电池约束的能量最优精确 DP — 单测

覆盖: 正例 / 退化 / 边界 / 一致性(与暴力枚举对照) / 与无电池版对照。
共享 fixtures 定义在 conftest.py。
"""
import itertools
import math

import pytest

from a3_python.route import DroneSpec, GeoPoint, Target
from a3_python.energy_model import euclidean_distance
from a3_python.data_generator import generate_scenario
from a3_python.solver import plan_multistop
from a3_python.exact_battery import (
    MAX_EXACT_BATTERY_POINTS,
    solve_energy_exact_battery,
)

DRONE = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0,
                  alpha=0.1, beta=0.005)
DRONE_TIGHT = DroneSpec(payload_capacity=50.0, battery_capacity=400.0,
                        alpha=0.1, beta=0.005)


def _brute_force(targets, home, drone):
    """暴力枚举所有排列, 返回 (最优能耗, 序列); 无可行解返回 (None, None)"""
    n = len(targets)
    pts = [home] + [t.location for t in targets]
    dem = [0.0] + [t.demand for t in targets]
    total = sum(dem)
    if total > drone.payload_capacity:
        return None, None
    best, best_seq = math.inf, None
    for perm in itertools.permutations(range(1, n + 1)):
        energy, load, cur, ok = 0.0, total, 0, True
        for k in perm:
            energy += euclidean_distance(pts[cur], pts[k]) * (
                drone.alpha + drone.beta * load)
            if energy > drone.battery_capacity + 1e-9:
                ok = False
                break
            load -= dem[k]
            cur = k
        if not ok:
            continue
        energy += euclidean_distance(pts[cur], pts[0]) * drone.alpha
        if energy > drone.battery_capacity + 1e-9:
            continue
        if energy < best:
            best, best_seq = energy, list(perm)
    if best_seq is None:
        return None, None
    return best, [targets[i - 1].id for i in best_seq]


# === 正例 ===

def test_single_target_feasible():
    """单点: 有解"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id="t1", location=GeoPoint(x=100.0, y=0.0), demand=5.0)]
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert r.feasible
    assert r.sequence == ["t1"]


def test_multiple_targets_returns_all():
    """多点: 序列包含全部目标点且无重复"""
    home, tg = generate_scenario(n=6, distribution="random", seed=0,
                                 scale=800.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert r.feasible
    assert sorted(r.sequence) == sorted(t.id for t in tg)
    assert len(r.sequence) == len(set(r.sequence))


def test_objective_matches_reported_sequence():
    """目标值与所报序列的实际能耗一致"""
    home, tg = generate_scenario(n=5, distribution="circle", seed=1,
                                 scale=800.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE)
    tmap = {t.id: t for t in tg}
    pts = [home] + [t.location for t in tg]
    energy, load, cur = 0.0, sum(t.demand for t in tg), 0
    for tid in r.sequence:
        k = 1 + [t.id for t in tg].index(tid)
        energy += euclidean_distance(pts[cur], pts[k]) * (DRONE.alpha + DRONE.beta * load)
        load -= tmap[tid].demand
        cur = k
    energy += euclidean_distance(pts[cur], pts[0]) * DRONE.alpha
    assert r.objective == pytest.approx(energy, abs=1e-9)


# === 一致性: 与暴力枚举对照 ===

@pytest.mark.parametrize("n", [5, 6, 7])
@pytest.mark.parametrize("seed", [0, 1])
def test_matches_brute_force(n, seed):
    """宽松电量下, DP 与暴力枚举的最优值精确一致"""
    home, tg = generate_scenario(n=n, distribution="random", seed=seed,
                                 scale=800.0, demand_range=(1.0, 3.0))
    dp = solve_energy_exact_battery(tg, home, DRONE)
    bf, _ = _brute_force(tg, home, DRONE)
    assert dp.feasible == (bf is not None)
    if bf is not None:
        assert dp.objective == pytest.approx(bf, abs=1e-9)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_matches_brute_force_under_tight_battery(seed):
    """电量绑定时仍与暴力枚举一致 (此时存在被剪枝的不可行排列)"""
    home, tg = generate_scenario(n=7, distribution="circle", seed=seed,
                                 scale=1500.0, demand_range=(1.0, 3.0))
    dp = solve_energy_exact_battery(tg, home, DRONE_TIGHT)
    bf, _ = _brute_force(tg, home, DRONE_TIGHT)
    assert dp.feasible == (bf is not None)
    if bf is not None:
        assert dp.objective == pytest.approx(bf, abs=1e-9)


# === 退化 ===

def test_empty_targets():
    """空目标集: 能耗为 0"""
    r = solve_energy_exact_battery([], GeoPoint(x=0.0, y=0.0), DRONE)
    assert r.feasible and r.sequence == [] and r.objective == 0.0


def test_payload_over_capacity_is_infeasible():
    """总需求超过载重上限: 判定不可行"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id=f"t{i}", location=GeoPoint(x=float(i * 10), y=0.0), demand=30.0)
          for i in range(1, 3)]
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert not r.feasible


def test_battery_too_small_is_infeasible():
    """电量过小: 无可行解"""
    home = GeoPoint(x=0.0, y=0.0)
    tiny = DroneSpec(payload_capacity=50.0, battery_capacity=1.0,
                     alpha=0.1, beta=0.005)
    tg = [Target(id="t1", location=GeoPoint(x=5000.0, y=0.0), demand=1.0)]
    r = solve_energy_exact_battery(tg, home, tiny)
    assert not r.feasible


# === 边界 ===

def test_exceeds_point_limit_raises():
    """超过点数上限: 抛 ValueError"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id=f"t{i}", location=GeoPoint(x=float(i), y=0.0), demand=0.1)
          for i in range(MAX_EXACT_BATTERY_POINTS + 1)]
    with pytest.raises(ValueError):
        solve_energy_exact_battery(tg, home, DRONE)


def test_point_limit_is_mvp_upper_bound():
    """上限常量 == MVP 目标点数上限 (20), 保证 n<=20 全部可精确求解"""
    from a3_python.solver import MAX_TARGETS

    assert MAX_EXACT_BATTERY_POINTS == 20 == MAX_TARGETS


def test_exact_handles_full_mvp_size():
    """边界: n = 20 (MVP 上限) 可精确求解且结果自洽"""
    home, tg = generate_scenario(n=MAX_EXACT_BATTERY_POINTS, distribution="random",
                                 seed=0, scale=1000.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert r.feasible
    assert sorted(r.sequence) == sorted(t.id for t in tg)
    assert r.objective <= DRONE.battery_capacity + 1e-9
    # 剪枝上界来自真实可行解, 故必然 >= 最优值 (否则会误剪最优解)
    assert r.ub_initial >= r.objective - 1e-9
    assert r.states_explored > 0 and r.pruned_states > 0


def test_feasible_result_respects_battery():
    """返回的可行解在能量模型下确实不超电量"""
    home, tg = generate_scenario(n=7, distribution="circle", seed=3,
                                 scale=1500.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE_TIGHT)
    if r.feasible:
        assert r.objective <= DRONE_TIGHT.battery_capacity + 1e-9


def test_dp_beats_or_equals_heuristic_when_both_feasible():
    """两者都可行时, 精确解不劣于启发式"""
    home, tg = generate_scenario(n=8, distribution="random", seed=4,
                                 scale=800.0, demand_range=(1.0, 3.0))
    r = solve_energy_exact_battery(tg, home, DRONE)
    plan = plan_multistop(tg, home, DRONE)
    if r.feasible and plan.feasible:
        # RoutePlan 的能耗保留 2 位小数, 故容差取半位舍入 + 余量
        assert r.objective <= plan.total_energy_consumed + 5e-3


def test_exact_finds_solution_where_heuristic_fails():
    """回归: tight_10p_s9 (启发式失败) 上精确解能找到可行解

    该实例是我们已记录的唯一启发式失败案例, 此处作为回归基准。
    """
    home, tg = generate_scenario(n=10, distribution="circle", seed=9,
                                 scale=3000.0, demand_range=(1.0, 3.0))
    plan = plan_multistop(tg, home, DRONE)
    r = solve_energy_exact_battery(tg, home, DRONE)
    assert not plan.feasible, "前提: 该案例启发式应失败"
    assert r.feasible, "精确解应能找到可行解"
