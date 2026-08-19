"""test_ablation.py — 消融实验 单测

W5 (R5.2 调研结论):
  - 6 变体通过组合现有纯函数实现, 不改 plan_multistop 签名
  - 搜索类变体不劣化解 (VND 只接受改进)
  - fixed_payload / no_energy 用真实 drone 重新评估
"""

import pytest

from a3_python.route import GeoPoint, DroneSpec, RoutePlan
from a3_python.data_generator import generate_targets
from a3_python.solver import plan_multistop
from a3_python.exact import evaluate_sequence_cost
from a3_python.ablation import (
    ABLATION_VARIANTS,
    AblationStats,
    aggregate_stats,
    run_ablation,
    solve_variant,
)

from a3_python.route import DRONE_PRESETS


def _circle_instance(n: int, seed: int = 42, scale: float = 1000.0):
    """生成 circle 实例 (demand 收窄保证 standard 机型可行)"""
    targets = generate_targets(n, distribution="circle", scale=scale,
                               seed=seed, demand_range=(1.0, 3.0))
    return targets, GeoPoint(x=0.0, y=0.0)


# ====================================================================
# 正例: 变体一致性
# ====================================================================

def test_full_equals_plan_multistop():
    """full 变体 == plan_multistop (一致性)"""
    targets, home = _circle_instance(5)
    drone = DRONE_PRESETS["standard"]

    v = solve_variant(targets, home, drone, "full")
    p = plan_multistop(targets, home, drone)

    assert v.sequence == p.sequence
    assert v.total_equiv_distance == p.total_equiv_distance
    assert v.feasible == p.feasible


def test_search_variants_never_worse_than_nn_only():
    """正例: 搜索类变体 (full/no_oropt/no_2opt) 不劣于 nn_only"""
    targets, home = _circle_instance(10)
    drone = DRONE_PRESETS["standard"]

    base = solve_variant(targets, home, drone, "nn_only")
    assert base.feasible

    for variant in ["full", "no_oropt", "no_2opt"]:
        v = solve_variant(targets, home, drone, variant)
        assert v.feasible
        assert v.total_equiv_distance <= base.total_equiv_distance + 1e-6, (
            f"{variant} 不应劣于 nn_only"
        )


def test_full_not_worse_than_partial_search():
    """正例: full (VND) ≤ no_oropt / no_2opt (更多邻域不劣化)"""
    targets, home = _circle_instance(10, seed=7)
    drone = DRONE_PRESETS["standard"]

    full = solve_variant(targets, home, drone, "full")
    for variant in ["no_oropt", "no_2opt"]:
        v = solve_variant(targets, home, drone, variant)
        assert full.total_equiv_distance <= v.total_equiv_distance + 1e-6


def test_fixed_payload_re_evaluated_with_real_drone():
    """fixed_payload: 构造用 β=0, 评估用真实 β (一致性)"""
    targets, home = _circle_instance(5)
    drone = DRONE_PRESETS["standard"]

    v = solve_variant(targets, home, drone, "fixed_payload")

    # 返回的 RoutePlan 必须与真实能量模型评估一致
    cost, feasible, _ = evaluate_sequence_cost(targets, home, drone, v.sequence)
    assert v.total_equiv_distance == pytest.approx(cost)
    assert v.feasible == feasible


def test_no_energy_re_evaluated_with_real_drone():
    """no_energy: 构造用超大电池, 评估用真实电池 (一致性)"""
    targets, home = _circle_instance(5)
    drone = DRONE_PRESETS["standard"]

    v = solve_variant(targets, home, drone, "no_energy")

    cost, feasible, _ = evaluate_sequence_cost(targets, home, drone, v.sequence)
    assert v.total_equiv_distance == pytest.approx(cost)
    assert v.feasible == feasible


# ====================================================================
# 退化: 约束忽略的代价
# ====================================================================

def test_no_energy_battery_warning_on_tight_instance():
    """退化: 电量绑定实例 (3000Wh/scale=3000), no_energy 的真实电池
    后验证应给出电池耗尽警告 (而非载重警告)"""
    from a3_python.route import DroneSpec

    drone = DroneSpec(payload_capacity=50.0, battery_capacity=3000.0,
                      alpha=0.1, beta=0.005)
    targets, home = _circle_instance(10, scale=3000.0)

    no_energy = solve_variant(targets, home, drone, "no_energy")

    # 构造阶段无电池约束, 评估阶段用真实电池 → 若不可行, 原因必须是电池
    if not no_energy.feasible:
        assert any("Battery" in w for w in no_energy.warnings), (
            f"不可行原因应为电池耗尽: {no_energy.warnings}"
        )


def test_fixed_payload_can_produce_different_route():
    """退化: 固定载重忽略耦合 → 路线可能不同于 full (或同成本)"""
    targets, home = _circle_instance(10, seed=3)
    drone = DRONE_PRESETS["standard"]

    full = solve_variant(targets, home, drone, "full")
    fixed = solve_variant(targets, home, drone, "fixed_payload")

    # 两种构造目标不同 (equiv vs geo), 解可能不同; 至少不抛异常且结构合法
    assert isinstance(fixed, RoutePlan)
    assert len(fixed.sequence) == len(targets) or not fixed.feasible
    assert full.feasible


# ====================================================================
# 一致性 + 边界
# ====================================================================

def test_deterministic():
    """一致性: 相同输入 → 相同输出"""
    targets, home = _circle_instance(5)
    drone = DRONE_PRESETS["standard"]

    for variant in ABLATION_VARIANTS:
        r1 = solve_variant(targets, home, drone, variant)
        r2 = solve_variant(targets, home, drone, variant)
        assert r1.sequence == r2.sequence
        assert r1.total_equiv_distance == r2.total_equiv_distance


def test_unknown_variant_raises():
    """边界: 未知变体 → ValueError"""
    targets, home = _circle_instance(5)
    drone = DRONE_PRESETS["standard"]

    with pytest.raises(ValueError, match="Unknown ablation variant"):
        solve_variant(targets, home, drone, "no_such_variant")


def test_run_ablation_structure():
    """run_ablation: 返回 {variant: {instance: AblationStats}}, 所有变体覆盖"""
    inst = [("c5", *_circle_instance(5)), ("c10", *_circle_instance(10))]
    drone = DRONE_PRESETS["standard"]

    results = run_ablation(inst, drone)

    assert set(results.keys()) == set(ABLATION_VARIANTS)
    for variant, inst_map in results.items():
        assert set(inst_map.keys()) == {"c5", "c10"}
        for stats in inst_map.values():
            assert isinstance(stats, AblationStats)
            assert stats.n_runs == 1
            assert stats.feasible_rate in (0.0, 1.0)


def test_aggregate_stats_by_group():
    """aggregate_stats: 按规模分组聚合, mean±std + 可行率"""
    inst = [
        ("c5_s1", *_circle_instance(5, seed=1)),
        ("c5_s2", *_circle_instance(5, seed=2)),
        ("c10_s1", *_circle_instance(10, seed=1)),
    ]
    drone = DRONE_PRESETS["standard"]
    results = run_ablation(inst, drone)

    def group_fn(name: str) -> str:
        return name.split("_")[0]  # "c5"/"c10"

    agg = aggregate_stats(results, group_fn)

    assert set(agg["full"].keys()) == {"c5", "c10"}
    c5 = agg["full"]["c5"]
    assert c5.n_runs == 2
    assert c5.feasible_rate == 1.0
    assert c5.cost_mean > 0
    assert c5.cost_std > 0  # 两个不同种子实例 → 有方差
    assert c5.cost_std <= c5.cost_mean  # 合理性检查
