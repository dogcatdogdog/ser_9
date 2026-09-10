"""`a3_python.adaptive` 自适应求解入口 — 单测

覆盖: 正例 (预算充足走精确 / 预算不足走降级) / 退化 (不可行实例) /
边界 (n=1 / 极小预算 / 越小内存 / 超上限) / 一致性 (两模式对比, 降级 == plan_multistop) /
回归 (协同: 启发式 UB 不改变最优解)。
共享 fixtures 定义在 conftest.py。
"""
import math
import time

import pytest

from a3_python.route import DroneSpec, GeoPoint, Target
from a3_python.data_generator import generate_scenario
from a3_python.solver import MAX_TARGETS, plan_multistop
from a3_python.exact_battery import (
    MAX_EXACT_BATTERY_POINTS,
    solve_energy_exact_battery,
)
from a3_python.adaptive import (
    DEFAULT_MEMORY_LIMIT_MB,
    DEFAULT_TIME_LIMIT_SECS,
    EST_SAFETY_FACTOR,
    EXACT_EST_GROWTH,
    EXACT_MEM_AT_N20_MB,
    EXACT_MEM_OVERHEAD_MB,
    EXACT_TIME_AT_N20_SECS,
    EXACT_TIME_OVERHEAD_SECS,
    MODE_EXACT,
    MODE_HEURISTIC,
    MODE_WARNING_PREFIX,
    choose_mode,
    estimate_exact_resources,
    plan_multistop_adaptive,
    solve_mode_of,
)

DRONE = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0,
                  alpha=0.1, beta=0.005)

# 参考数据: 精确模式实测 (中位耗时 / 峰值内存), 见 adaptive.py 模块常量注释
MEASURED_TIME_S = {14: 0.053, 15: 0.071, 16: 0.112, 17: 0.206,
                   18: 0.421, 19: 0.743, 20: 1.774}
MEASURED_TIME_WORST_S = {14: 0.055, 15: 0.074, 16: 0.122, 17: 0.232,
                         18: 0.442, 19: 0.816, 20: 1.857}
MEASURED_MEM_MB = {14: 2.5, 15: 5.2, 16: 9.5, 17: 19.1,
                   18: 36.7, 19: 75.4, 20: 155.8}


def _scenario(n, seed=0, scale=1000.0):
    """统一场景构造 (与 adaptive.py 估算口径一致)"""
    return generate_scenario(n=n, distribution="random", seed=seed,
                             scale=scale, demand_range=(1.0, 3.0))


def _mode_warnings(plan):
    """warnings 中的模式说明条目"""
    return [w for w in plan.warnings if w.startswith(MODE_WARNING_PREFIX)]


# ==================== 正例 ====================

def test_generous_budget_uses_exact_mode():
    """预算充足: 走精确模式, 结果等于 solve_energy_exact_battery 的最优序列"""
    home, tg = _scenario(10)
    plan = plan_multistop_adaptive(tg, home, DRONE,
                                   time_limit_secs=5.0, memory_limit_mb=250.0)
    exact = solve_energy_exact_battery(tg, home, DRONE)
    assert solve_mode_of(plan) == MODE_EXACT
    assert plan.feasible
    assert plan.sequence == exact.sequence
    assert sorted(plan.sequence) == sorted(t.id for t in tg)


def test_exact_mode_energy_matches_exact_objective():
    """精确模式的 RoutePlan 能耗与精确解目标值一致 (2 位小数舍入容差)"""
    home, tg = _scenario(9, seed=1)
    plan = plan_multistop_adaptive(tg, home, DRONE)
    exact = solve_energy_exact_battery(tg, home, DRONE)
    assert solve_mode_of(plan) == MODE_EXACT
    assert plan.total_energy_consumed == pytest.approx(exact.objective, abs=6e-3)
    assert plan.total_payload_delivered == pytest.approx(
        sum(t.demand for t in tg), abs=1e-9)


def test_exact_mode_segments_are_complete_chain():
    """精确模式的 segments 覆盖 home→...→home 全链, 与 sequence 长度一致"""
    home, tg = _scenario(8, seed=2)
    plan = plan_multistop_adaptive(tg, home, DRONE)
    assert solve_mode_of(plan) == MODE_EXACT
    assert len(plan.segments) == len(tg) + 1
    assert plan.segments[0].from_id == "home"
    assert plan.segments[-1].to_id == "home"
    assert [s.to_id for s in plan.segments[:-1]] == plan.sequence
    assert plan.segments[-1].battery_after >= 0.0


@pytest.mark.parametrize("n", [5, 10, 13])
def test_exact_mode_not_worse_than_heuristic(n):
    """一致性: 同一实例上精确模式的能耗 <= 降级(启发式)模式的能耗"""
    home, tg = _scenario(n, seed=3)
    exact_plan = plan_multistop_adaptive(tg, home, DRONE,
                                         time_limit_secs=5.0, memory_limit_mb=250.0)
    heur_plan = plan_multistop_adaptive(tg, home, DRONE,
                                        time_limit_secs=0.0, memory_limit_mb=250.0)
    assert solve_mode_of(exact_plan) == MODE_EXACT
    assert solve_mode_of(heur_plan) == MODE_HEURISTIC
    assert exact_plan.feasible and heur_plan.feasible
    assert exact_plan.total_energy_consumed <= heur_plan.total_energy_consumed + 5e-3


def test_full_mvp_size_exact_within_budget():
    """端到端: n = 20 (MVP 上限) 在默认预算下走精确模式, 且实际耗时在估算之内"""
    home, tg = _scenario(MAX_EXACT_BATTERY_POINTS)
    est_time, est_mem = estimate_exact_resources(MAX_EXACT_BATTERY_POINTS)
    t0 = time.perf_counter()
    plan = plan_multistop_adaptive(tg, home, DRONE)
    elapsed = time.perf_counter() - t0
    assert solve_mode_of(plan) == MODE_EXACT
    assert plan.feasible
    assert sorted(plan.sequence) == sorted(t.id for t in tg)
    # 估算刻意保守: 实际耗时不应超过估算 (估算本身含 1.5 倍安全系数)
    assert elapsed <= est_time, f"实际耗时 {elapsed:.2f}s 超出估算 {est_time:.2f}s"
    assert est_mem <= DEFAULT_MEMORY_LIMIT_MB


# ==================== 降级 (预算不足) ====================

@pytest.mark.parametrize("kwargs, expect_reason", [
    ({"time_limit_secs": 0.0}, "time"),
    ({"memory_limit_mb": 1.0}, "memory"),
])
def test_starved_budget_falls_back_to_heuristic(kwargs, expect_reason):
    """预算不足: 降级为 plan_multistop, 且与直接调用 plan_multistop 完全一致"""
    home, tg = _scenario(10, seed=4)
    plan = plan_multistop_adaptive(tg, home, DRONE, **kwargs)
    baseline = plan_multistop(tg, home, DRONE)
    assert solve_mode_of(plan) == MODE_HEURISTIC
    assert plan.sequence == baseline.sequence
    assert plan.total_energy_consumed == baseline.total_energy_consumed
    assert plan.total_equiv_distance == baseline.total_equiv_distance
    assert plan.feasible == baseline.feasible

    decision = choose_mode(len(tg), **{
        "time_limit_secs": kwargs.get("time_limit_secs", DEFAULT_TIME_LIMIT_SECS),
        "memory_limit_mb": kwargs.get("memory_limit_mb", DEFAULT_MEMORY_LIMIT_MB),
    })
    assert decision.mode == MODE_HEURISTIC
    assert expect_reason in decision.reason


def test_heuristic_mode_degenerates_for_large_n():
    """n 超出精确上限(但仍在 MVP 内不可能) — 用估算接口验证降级决策"""
    decision = choose_mode(MAX_EXACT_BATTERY_POINTS + 1)
    assert decision.mode == MODE_HEURISTIC
    assert "exceeds exact limit" in decision.reason


def test_degrade_path_preserves_original_warnings():
    """降级路径: 原 plan_multistop 的 warnings 保留, 模式说明追加在末尾"""
    home = GeoPoint(x=0.0, y=0.0)
    tiny = DroneSpec(payload_capacity=50.0, battery_capacity=1.0,
                     alpha=0.1, beta=0.005)
    tg = [Target(id="t1", location=GeoPoint(x=5000.0, y=0.0), demand=1.0)]
    plan = plan_multistop_adaptive(tg, home, tiny, time_limit_secs=0.0)
    baseline = plan_multistop(tg, home, tiny)
    assert plan.warnings[:len(baseline.warnings)] == baseline.warnings
    assert len(plan.warnings) == len(baseline.warnings) + 1
    assert plan.warnings[-1].startswith(MODE_WARNING_PREFIX)


# ==================== 退化 (不可行实例) ====================

def test_infeasible_battery_returns_infeasible_with_exact_verdict():
    """电量过小: 精确模式确认不可行 → 返回降级解 + exact=infeasible 标注"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id="t1", location=GeoPoint(x=5000.0, y=0.0), demand=1.0)]
    tiny = DroneSpec(payload_capacity=50.0, battery_capacity=1.0,
                     alpha=0.1, beta=0.005)
    plan = plan_multistop_adaptive(tg, home, tiny,
                                   time_limit_secs=5.0, memory_limit_mb=250.0)
    assert not plan.feasible
    assert solve_mode_of(plan) == MODE_HEURISTIC
    assert any("exact=infeasible" in w for w in plan.warnings)
    assert not solve_energy_exact_battery(tg, home, tiny).feasible


def test_infeasible_payload_returns_infeasible():
    """总载重超上限: 两模式都不可行, 返回降级解且 feasible=False"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id=f"t{i}", location=GeoPoint(x=float(i * 10), y=0.0), demand=30.0)
          for i in range(1, 3)]
    plan = plan_multistop_adaptive(tg, home, DRONE)
    assert not plan.feasible
    assert plan.sequence == []
    assert len(plan.warnings) >= 1


# ==================== 边界 ====================

def test_single_target_uses_exact_mode():
    """边界 n=1: 预算充足 → 精确模式, 序列即该点"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id="t1", location=GeoPoint(x=100.0, y=0.0), demand=5.0)]
    plan = plan_multistop_adaptive(tg, home, DRONE)
    assert solve_mode_of(plan) == MODE_EXACT
    assert plan.sequence == ["t1"]
    assert plan.feasible
    assert len(plan.segments) == 2


def test_zero_targets_raises():
    """边界 n=0: 与 plan_multistop 同契约 → ValueError"""
    with pytest.raises(ValueError, match="cannot be empty"):
        plan_multistop_adaptive([], GeoPoint(x=0.0, y=0.0), DRONE)


def test_over_max_targets_raises():
    """边界 n > MVP 上限: ValueError (两种模式都不支持)"""
    home = GeoPoint(x=0.0, y=0.0)
    tg = [Target(id=f"t{i}", location=GeoPoint(x=float(i), y=0.0), demand=0.1)
          for i in range(MAX_TARGETS + 1)]
    with pytest.raises(ValueError, match="exceeds MVP limit"):
        plan_multistop_adaptive(tg, home, DRONE)


def test_negative_time_budget_never_runs_exact():
    """边界: 时间为负 → 降级 (不得尝试精确模式)"""
    home, tg = _scenario(6, seed=5)
    plan = plan_multistop_adaptive(tg, home, DRONE, time_limit_secs=-1.0)
    assert solve_mode_of(plan) == MODE_HEURISTIC


def test_choose_mode_boundary_is_conditional_not_strict():
    """边界: 预算恰等于估算值 → 走精确 (判据取 > 而非 >=)"""
    est_time, est_mem = estimate_exact_resources(10)
    decision = choose_mode(10, time_limit_secs=est_time, memory_limit_mb=est_mem)
    assert decision.mode == MODE_EXACT
    assert decision.est_time_secs == est_time
    assert decision.est_memory_mb == est_mem

    below = choose_mode(10, time_limit_secs=est_time * 0.99)
    assert below.mode == MODE_HEURISTIC


def test_choose_mode_reports_both_estimates():
    """决策对象始终带完整估算值 (便于日志与容量规划)"""
    decision = choose_mode(20)
    assert decision.mode == MODE_EXACT
    assert decision.est_time_secs > 0.0
    assert decision.est_memory_mb > 0.0
    assert decision.reason


# ==================== 估算公式 (保守性) ====================

def test_estimate_is_conservative_against_measured_table():
    """估算必须 >= 实测 (耗时对最差值, 内存对实测值) —— 保守性, 宁可高估"""
    for n in range(14, MAX_EXACT_BATTERY_POINTS + 1):
        est_time, est_mem = estimate_exact_resources(n)
        assert est_time >= MEASURED_TIME_WORST_S[n], (
            f"n={n} 耗时估算 {est_time:.3f}s < 实测最差 {MEASURED_TIME_WORST_S[n]}s")
        assert est_mem >= MEASURED_MEM_MB[n], (
            f"n={n} 内存估算 {est_mem:.1f}MB < 实测 {MEASURED_MEM_MB[n]}MB")


def test_estimate_matches_formula():
    """估算公式可复算 (常量集中, 换机器只需重测 n=20 两点)"""
    for n in (1, 10, 20):
        scale = EXACT_EST_GROWTH ** (n - MAX_EXACT_BATTERY_POINTS)
        expected_time = (EXACT_TIME_OVERHEAD_SECS + EXACT_TIME_AT_N20_SECS * scale) \
            * EST_SAFETY_FACTOR
        expected_mem = (EXACT_MEM_OVERHEAD_MB + EXACT_MEM_AT_N20_MB * scale) \
            * EST_SAFETY_FACTOR
        assert estimate_exact_resources(n) == (expected_time, expected_mem)


def test_estimate_has_safety_margin_over_median():
    """对 n=20 锚点: 估算不低于实测中位数的 1.5 倍 (额外安全裕度确实存在)"""
    est_time, est_mem = estimate_exact_resources(MAX_EXACT_BATTERY_POINTS)
    assert est_time >= MEASURED_TIME_S[20] * EST_SAFETY_FACTOR
    assert est_mem >= MEASURED_MEM_MB[20] * EST_SAFETY_FACTOR


def test_estimate_grows_with_n():
    """估算随 n 单调递增, 且相邻比值趋近 EXACT_EST_GROWTH (大 n 时)"""
    values = [estimate_exact_resources(n)[1] for n in range(14, 21)]
    assert values == sorted(values)
    assert values[-1] > values[-2]
    # 大 n 时主导项是指数项, 比值 -> EXACT_EST_GROWTH
    overhead = EXACT_MEM_OVERHEAD_MB * EST_SAFETY_FACTOR
    ratio = (values[-1] - overhead) / (values[-2] - overhead)
    assert ratio == pytest.approx(EXACT_EST_GROWTH, rel=1e-9)


def test_estimate_is_deterministic():
    """纯函数: 相同输入 -> 相同输出"""
    assert estimate_exact_resources(17) == estimate_exact_resources(17)


def test_estimate_below_exact_limit_is_cheap():
    """小 n 的估算远小于默认预算 (n < 14 一律走精确)"""
    est_time, est_mem = estimate_exact_resources(5)
    assert est_time < DEFAULT_TIME_LIMIT_SECS
    assert est_mem < DEFAULT_MEMORY_LIMIT_MB
    assert choose_mode(5).mode == MODE_EXACT


# ==================== 一致性 / 回归 ====================

def test_mode_note_is_readable_and_absent_for_plain_solver():
    """模式说明可读回; 直接来自 plan_multistop 的结果无模式说明"""
    home, tg = _scenario(6, seed=6)
    plan = plan_multistop_adaptive(tg, home, DRONE)
    assert solve_mode_of(plan) == MODE_EXACT
    assert solve_mode_of(plan_multistop(tg, home, DRONE)) is None


def test_adaptive_is_deterministic():
    """一致性: 相同输入 + 相同 seed → 相同序列与模式"""
    home, tg = _scenario(7, seed=7)
    r1 = plan_multistop_adaptive(tg, home, DRONE, seed=42)
    r2 = plan_multistop_adaptive(tg, home, DRONE, seed=42)
    assert r1.sequence == r2.sequence
    assert r1.total_energy_consumed == r2.total_energy_consumed
    assert r1.warnings == r2.warnings


def test_collaboration_ub_does_not_change_optimum():
    """回归 (协同): 精确模式的上界来自降级解, 但最优解与自动 UB 完全相同"""
    home, tg = _scenario(8, seed=8)
    adaptive_plan = plan_multistop_adaptive(tg, home, DRONE)
    auto = solve_energy_exact_battery(tg, home, DRONE)
    assert solve_mode_of(adaptive_plan) == MODE_EXACT
    assert adaptive_plan.sequence == auto.sequence
    assert auto.feasible
    assert math.isfinite(auto.ub_initial)


def test_warning_entry_is_informative_not_an_error():
    """模式说明是说明性条目: 不影响 feasible, 且带 n/估算值信息"""
    home, tg = _scenario(9, seed=9)
    plan = plan_multistop_adaptive(tg, home, DRONE)
    notes = _mode_warnings(plan)
    assert len(notes) == 1
    assert "mode=exact" in notes[0]
    assert f"n={len(tg)}" in notes[0]
    assert "est_time=" in notes[0] and "est_memory=" in notes[0]
    assert plan.feasible is True
