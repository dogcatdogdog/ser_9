"""test_exact.py — 精确解基线 (OR-Tools CP-SAT + 能量感知 DP) 单测

W5 (R5.1 调研结论的实现验证):
  - CP-SAT 电路约束在 n≤20 得到几何 TSP 精确解 (与 Held-Karp DP 一致)
  - 能量感知 DP (状态相关边权) 是能量目标的精确最优
  - 几何最优在真实电量模型下的可行性 (专利论据: 会"坠机")
"""

import math

import pytest

from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.data_generator import generate_targets
from a3_python.solver import plan_multistop
from a3_python.exact import (
    MAX_DP_POINTS,
    compute_gap,
    evaluate_sequence_cost,
    solve_energy_exact_dp,
    solve_tsp_exact_cpsat,
    solve_tsp_exact_dp,
)
from a3_python.route import DRONE_PRESETS


def _triangle_3pts() -> list[Target]:
    """3 点等边三角形: home 在原点, 半径 100, 相邻点距离 100√3 ≈ 173.2"""
    pts = []
    for i in range(3):
        ang = 2 * math.pi * i / 3
        pts.append(Target(
            id=f"c{i+1}",
            location=GeoPoint(x=100 * math.cos(ang), y=100 * math.sin(ang)),
            demand=1.0,
        ))
    return pts


# ====================================================================
# 正例: CP-SAT 与 Held-Karp DP 一致
# ====================================================================

def test_cpsat_matches_dp_5pts():
    """5 点圆形分布: CP-SAT == Held-Karp (几何精确解一致)"""
    targets = generate_targets(5, distribution="circle", scale=1000.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)

    cp = solve_tsp_exact_cpsat(targets, home, instance_name="test_5")
    hk = solve_tsp_exact_dp(targets, home, instance_name="test_5")

    assert cp.status == "optimal"
    assert hk.status == "optimal"
    assert abs(cp.objective - hk.objective) < 0.5, (
        f"CP-SAT {cp.objective} vs Held-Karp {hk.objective}"
    )
    assert set(cp.sequence) == {t.id for t in targets}


def test_cpsat_matches_dp_10pts():
    """10 点圆形分布: CP-SAT == Held-Karp"""
    targets = generate_targets(10, distribution="circle", scale=1000.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)

    cp = solve_tsp_exact_cpsat(targets, home, instance_name="test_10")
    hk = solve_tsp_exact_dp(targets, home, instance_name="test_10")

    assert cp.status == "optimal"
    assert abs(cp.objective - hk.objective) < 0.5


def test_cpsat_deterministic():
    """相同输入 → 相同结果 (CP-SAT 确定性)"""
    targets = generate_targets(6, distribution="random", scale=800.0, seed=7)
    home = GeoPoint(x=0.0, y=0.0)

    r1 = solve_tsp_exact_cpsat(targets, home, instance_name="t")
    r2 = solve_tsp_exact_cpsat(targets, home, instance_name="t")

    assert r1.sequence == r2.sequence
    assert r1.objective == r2.objective


# ====================================================================
# 正例: 能量感知 DP
# ====================================================================

def test_energy_dp_triangle_known_opt():
    """3 点等边三角形: 能量感知精确最优 = 587.4 (解析推导)"""
    drone = DRONE_PRESETS["standard"]  # α=0.1, β=0.005
    targets = _triangle_3pts()
    home = GeoPoint(x=0.0, y=0.0)

    result = solve_energy_exact_dp(targets, home, drone, instance_name="tri")

    # 解析最优: home→p1 (payload=3) → p2 (2) → p3 (1) → home (0)
    # 100×1.15 + 173.2×1.10 + 173.2×1.05 + 100 = 587.4
    expected = (100 * 1.15 + 100 * math.sqrt(3) * 1.10
                + 100 * math.sqrt(3) * 1.05 + 100)
    assert result.status == "optimal"
    assert abs(result.objective - expected) < 0.01
    assert set(result.sequence) == {t.id for t in targets}


def test_energy_dp_never_worse_than_heuristic():
    """一致性: 能量精确最优 ≤ plan_multistop (启发式不优于精确解)"""
    # demand_range 收窄保证总载重 < 50kg (standard 机型容量内, 路线可行)
    targets = generate_targets(10, distribution="circle", scale=1000.0,
                               seed=42, demand_range=(1.0, 3.0))
    home = GeoPoint(x=0.0, y=0.0)
    drone = DRONE_PRESETS["standard"]

    opt = solve_energy_exact_dp(targets, home, drone, instance_name="c10")
    plan = plan_multistop(targets, home, drone)

    assert opt.status == "optimal"
    assert plan.feasible
    assert opt.objective <= plan.total_equiv_distance + 1e-6


def test_energy_dp_matches_brute_force_4pts():
    """4 点: 能量 DP 与暴力枚举完全一致"""
    drone = DRONE_PRESETS["standard"]
    targets = generate_targets(4, distribution="circle", scale=100.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)

    opt = solve_energy_exact_dp(targets, home, drone, instance_name="b4")

    # 暴力枚举
    import itertools
    pts = [home] + [t.location for t in targets]
    from a3_python.energy_model import compute_equiv_distance, euclidean_distance
    best = None
    for perm in itertools.permutations(range(1, len(pts))):
        total = sum(t.demand for t in targets)
        c = 0.0
        cur = 0
        for p in perm:
            c += compute_equiv_distance(
                euclidean_distance(pts[cur], pts[p]), total, drone.alpha, drone.beta)
            total -= targets[p - 1].demand
            cur = p
        c += compute_equiv_distance(
            euclidean_distance(pts[cur], pts[0]), 0.0, drone.alpha, drone.beta)
        best = c if best is None else min(best, c)

    assert abs(opt.objective - best) < 1e-9


# ====================================================================
# 关键论据: 几何最优在真实电量模型下不可行
# ====================================================================

def test_geometric_opt_battery_infeasible_n10():
    """专利论据 (电量): 10 点 circle scale=3000 + 3500Wh 电池 (seed=42)
    → 几何最优序列电池耗尽, 而我们的约束感知解可行 (实测验证)"""
    drone = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0,
                      alpha=0.1, beta=0.005)
    targets = generate_targets(10, distribution="circle", scale=3000.0,
                               seed=42, demand_range=(1.0, 3.0))
    home = GeoPoint(x=0.0, y=0.0)

    opt = solve_tsp_exact_cpsat(targets, home, drone=drone, instance_name="tight")

    assert opt.status == "optimal"
    assert opt.energy_feasible is False, (
        "几何最优应电量不可行 — 专利论据 (标准求解器会坠机)"
    )
    assert any("Battery" in w for w in opt.energy_warnings), (
        f"警告应为电池耗尽: {opt.energy_warnings}"
    )

    # 对比: 我们的约束感知解在此实例上可行
    plan = plan_multistop(targets, home, drone)
    assert plan.feasible, f"我们的解应可行: {plan.warnings}"


def test_geometric_opt_capacity_infeasible():
    """专利论据 (载重): 默认 demand (1,10) 总需求超容量 → 几何最优载重超限"""
    drone = DRONE_PRESETS["standard"]  # 50kg 容量
    targets = generate_targets(10, distribution="circle", scale=1000.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)

    assert sum(t.demand for t in targets) > drone.payload_capacity
    opt = solve_tsp_exact_cpsat(targets, home, drone=drone, instance_name="c10")

    assert opt.status == "optimal"
    assert opt.energy_feasible is False
    assert any("exceeds drone capacity" in w for w in opt.energy_warnings)


def test_geometric_opt_battery_feasible_n5():
    """对比: 5 点 circle + standard 机型 → 几何最优电量可行"""
    targets = generate_targets(5, distribution="circle", scale=1000.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)
    drone = DRONE_PRESETS["standard"]

    opt = solve_tsp_exact_cpsat(targets, home, drone=drone, instance_name="c5")

    assert opt.status == "optimal"
    assert opt.energy_feasible is True


# ====================================================================
# 边界/退化
# ====================================================================

def test_1point_roundtrip():
    """1 点: 最优 = 2 × dist(home, target)"""
    targets = [Target(id="c1", location=GeoPoint(x=100.0, y=0.0), demand=1.0)]
    home = GeoPoint(x=0.0, y=0.0)

    cp = solve_tsp_exact_cpsat(targets, home, instance_name="one")
    hk = solve_tsp_exact_dp(targets, home, instance_name="one")

    assert cp.objective == pytest.approx(200.0, abs=0.5)  # ×100 取整误差
    assert hk.objective == pytest.approx(200.0, abs=1e-9)


def test_empty_targets():
    """0 点: 返回空结果, 不抛异常"""
    home = GeoPoint(x=0.0, y=0.0)

    cp = solve_tsp_exact_cpsat([], home)
    hk = solve_tsp_exact_dp([], home)
    ed = solve_energy_exact_dp([], home, DRONE_PRESETS["standard"])

    for r in (cp, hk, ed):
        assert r.objective == 0.0
        assert r.sequence == []
        assert r.status == "optimal"


def test_dp_limit_exceeded():
    """边界: 超过 DP 上限 (16 点) → ValueError"""
    targets = generate_targets(16, distribution="circle", scale=1000.0)
    home = GeoPoint(x=0.0, y=0.0)

    assert MAX_DP_POINTS == 15
    with pytest.raises(ValueError, match="exceeds DP limit"):
        solve_tsp_exact_dp(targets, home)
    with pytest.raises(ValueError, match="exceeds DP limit"):
        solve_energy_exact_dp(targets, home, DRONE_PRESETS["standard"])


# ====================================================================
# 工具函数 + 回归
# ====================================================================

def test_compute_gap_formula():
    """gap 公式: (our − opt) / opt × 100%"""
    assert compute_gap(110.0, 100.0) == pytest.approx(10.0)
    assert compute_gap(100.0, 100.0) == pytest.approx(0.0)
    assert compute_gap(90.0, 100.0) == pytest.approx(-10.0)
    with pytest.raises(ValueError):
        compute_gap(100.0, 0.0)


def test_evaluate_sequence_cost_consistency():
    """一致性: evaluate_sequence_cost(plan 序列) == plan.total_equiv_distance"""
    targets = generate_targets(5, distribution="circle", scale=1000.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)
    drone = DRONE_PRESETS["standard"]

    plan = plan_multistop(targets, home, drone)
    cost, feasible, warnings = evaluate_sequence_cost(targets, home, drone, plan.sequence)

    assert feasible == plan.feasible
    assert cost == pytest.approx(plan.total_equiv_distance)


def test_5points_gap_under_10pct():
    """回归: 5 点实例, 我们的几何距离 vs 最优 gap < 10% (对齐 A3_SCHEMA §5.2 #10)"""
    targets = generate_targets(5, distribution="circle", scale=1000.0, seed=42)
    home = GeoPoint(x=0.0, y=0.0)
    drone = DRONE_PRESETS["standard"]

    plan = plan_multistop(targets, home, drone)
    opt = solve_tsp_exact_cpsat(targets, home, drone=drone, instance_name="r5")

    assert plan.feasible
    assert opt.status == "optimal"
    gap = compute_gap(plan.total_geo_distance, opt.objective)
    assert gap < 10.0, f"gap={gap:.2f}% 超出 10%"
