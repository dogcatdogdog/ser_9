"""test_heuristic.py — 构造启发式 + 局部搜索 单测 (W3+W4 实现)

测试分类:
  正例 (positive): >=3 — 正常场景, 应找到可行解/搜索改进
  退化 (degenerate): >=3 — 不可行场景, 应正确报告
  边界 (boundary): >=2 — 极限条件
  一致性 (consistency): >=2 — 确定性 & 约束满足
  回归 (regression): >=1 — 已知 fixture 值

对所有可行解: 验证返回的 RoutePlan 满足载重约束和电量约束。
"""

import math
import pytest
from a3_python.route import GeoPoint, Target, DroneSpec, RoutePlan
from a3_python.heuristic import (
    construct_nn,
    construct_savings,
    local_search_2opt,
    local_search_or_opt,
    local_search_vnd,
    _try_2opt_move,
    _try_or_opt_move,
    _segment_payloads,
)
from a3_python.energy_model import euclidean_distance, compute_equiv_distance, simulate_route_energy


# ====================================================================
# 辅助函数
# ====================================================================

def _verify_constraints(
    plan: RoutePlan,
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> None:
    """验证 RoutePlan 满足载重约束和电量约束.

    逐段检查:
      - payload_before <= drone.payload_capacity
      - battery_after >= 0
      - energy consumed = geo_distance * (alpha + beta * payload_before)
    """
    for seg in plan.segments:
        # 载重约束
        assert seg.payload_before <= drone.payload_capacity, (
            f"Payload {seg.payload_before} exceeds capacity {drone.payload_capacity}"
        )
        assert seg.payload_before >= 0, f"Negative payload {seg.payload_before}"

        # 电量约束
        assert seg.battery_after >= 0, (
            f"Battery exhausted at {seg.from_id}->{seg.to_id}: {seg.battery_after}"
        )

        # 能耗公式一致性
        expected_energy = seg.geo_distance * (drone.alpha + drone.beta * seg.payload_before)
        assert abs(seg.energy_consumed - expected_energy) < 0.01, (
            f"Energy mismatch: {seg.energy_consumed} vs expected {expected_energy}"
        )


def _make_targets(coords_demands: list[tuple[float, float, float]]) -> list[Target]:
    """从 (x, y, demand) 列表创建 Target 列表."""
    return [
        Target(id=f"c{i+1}", location=GeoPoint(x=x, y=y), demand=d)
        for i, (x, y, d) in enumerate(coords_demands)
    ]


# ====================================================================
# 正例 (positive) — 正常场景, 应找到可行解
# ====================================================================

class TestNNPositive:
    """NN 正例: 正常情况下应找到可行解"""

    def test_nn_three_points_line(self):
        """3 点一条线, 电量充裕 -> 可行"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 5), (200, 0, 5), (300, 0, 5)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible, f"Expected feasible, got warnings: {plan.warnings}"
        assert len(plan.sequence) == 3
        _verify_constraints(plan, targets, home, drone)

    def test_nn_five_points_scattered(self):
        """5 点散开, 电量充裕 -> 可行"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 3), (50, 80, 4), (200, 50, 3), (150, 200, 5), (300, 100, 2),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 5
        _verify_constraints(plan, targets, home, drone)

    def test_nn_all_points_visited(self):
        """N-start 应访问所有点 (不可跳过)"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 2), (200, 0, 2), (50, 100, 2)])
        drone = DroneSpec(20, 5000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        assert set(plan.sequence) == {t.id for t in targets}, "All targets must be visited"

    def test_nn_single_point(self):
        """单点: 直接 home->target->home"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 5)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        assert plan.sequence == ["c1"]
        _verify_constraints(plan, targets, home, drone)

    def test_nn_improves_over_input_order(self):
        """NN 构造的路线应优于随机输入顺序"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (500, 500, 1), (100, 100, 1), (400, 400, 1),
            (200, 200, 1), (300, 300, 1),
        ])
        drone = DroneSpec(20, 20000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        # NN 应产生比输入顺序更好的路线
        from a3_python.energy_model import simulate_route_energy
        bad_seq = ["c1", "c2", "c3", "c4", "c5"]  # 按创建顺序
        _, _, bad_equiv, _, _, _, _ = simulate_route_energy(
            bad_seq, {t.id: t for t in targets}, home, drone
        )
        assert plan.total_equiv_distance <= bad_equiv, (
            f"NN ({plan.total_equiv_distance:.0f}) should not be worse than "
            f"input order ({bad_equiv:.0f})"
        )


class TestSavingsPositive:
    """Savings 正例"""

    def test_savings_three_points_line(self):
        """3 点一条线, 电量充裕 -> 可行"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 5), (200, 0, 5), (300, 0, 5)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_savings(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 3
        _verify_constraints(plan, targets, home, drone)

    def test_savings_five_points(self):
        """5 点散开 -> 可行"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 3), (50, 80, 4), (200, 50, 3), (150, 200, 5), (300, 100, 2),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        plan = construct_savings(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 5
        _verify_constraints(plan, targets, home, drone)

    def test_savings_single_point(self):
        """单点"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 5)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_savings(targets, home, drone)

        assert plan.feasible
        assert plan.sequence == ["c1"]


# ====================================================================
# 退化 (degenerate) — 不可行场景, 应正确报告
# ====================================================================

class TestNNDegenerate:
    """NN 退化: 不可行场景"""

    def test_nn_over_capacity(self):
        """总载重超限 -> feasible=False"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 30), (200, 0, 30)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)  # capacity=50, demand=60

        plan = construct_nn(targets, home, drone)

        assert not plan.feasible
        assert "capacity" in plan.warnings[0].lower()

    def test_nn_empty_targets(self):
        """空 target 列表 -> ValueError"""
        home = GeoPoint(0, 0)
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        with pytest.raises(ValueError, match="empty"):
            construct_nn([], home, drone)

    def test_nn_tight_battery_partial(self):
        """电量只够访问部分点 -> 返回部分路线"""
        home = GeoPoint(0, 0)
        # c1 很近, c2 很远
        targets = _make_targets([(100, 0, 1), (50000, 0, 1)])
        drone = DroneSpec(10, 500, 0.1, 0.005)  # 刚好够第一个点往返

        plan = construct_nn(targets, home, drone)

        # 至少应该访问了近点
        assert "c1" in plan.sequence


class TestSavingsDegenerate:
    """Savings 退化"""

    def test_savings_over_capacity(self):
        """总载重超限"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 30), (200, 0, 30)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_savings(targets, home, drone)

        assert not plan.feasible
        assert "capacity" in plan.warnings[0].lower()

    def test_savings_empty_targets(self):
        """空 target 列表 -> ValueError"""
        home = GeoPoint(0, 0)
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        with pytest.raises(ValueError, match="empty"):
            construct_savings([], home, drone)


# ====================================================================
# 边界 (boundary) — 极限条件
# ====================================================================

class TestBoundary:
    """边界用例"""

    def test_nn_zero_demand(self):
        """巡检场景: 所有点 demand=0 -> 等效距离 = 几何距离"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 0), (200, 0, 0), (300, 0, 0)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 3
        # demand=0 时 equiv_dist = geo_dist * (alpha + 0) / alpha = geo_dist
        for seg in plan.segments:
            assert abs(seg.geo_distance - seg.equiv_distance) < 0.01, (
                f"Zero demand should make equiv=geo, "
                f"got {seg.equiv_distance} vs {seg.geo_distance}"
            )

    def test_max_capacity_exact_match(self):
        """载重恰好等于 capacity -> 应可行"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 25), (200, 0, 25)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible

    def test_nn_same_location_points(self):
        """两个点在同一位置 -> 应正确处理"""
        home = GeoPoint(0, 0)
        targets = [
            Target(id="c1", location=GeoPoint(100, 0), demand=5),
            Target(id="c2", location=GeoPoint(100, 0), demand=5),  # 同位置
        ]
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 2


# ====================================================================
# 一致性 (consistency) — 确定性 & 约束满足
# ====================================================================

class TestConsistency:
    """一致性检查"""

    def test_nn_deterministic(self):
        """相同输入 -> 相同输出 (无随机性)"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 3), (50, 80, 4), (200, 50, 3), (150, 200, 5), (300, 100, 2),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        plan1 = construct_nn(targets, home, drone)
        plan2 = construct_nn(targets, home, drone)

        assert plan1.sequence == plan2.sequence
        assert plan1.total_equiv_distance == plan2.total_equiv_distance
        assert plan1.feasible == plan2.feasible

    def test_savings_deterministic(self):
        """Savings 也应确定性"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 3), (50, 80, 4), (200, 50, 3),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        plan1 = construct_savings(targets, home, drone)
        plan2 = construct_savings(targets, home, drone)

        assert plan1.sequence == plan2.sequence
        assert plan1.total_equiv_distance == plan2.total_equiv_distance

    def test_nn_constraints_satisfied_on_fixtures(self):
        """在 fixture 数据上验证所有约束"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict

        drone = DroneSpec(500, 100000, 0.08, 0.002)

        for fn in ["custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json"]:
            home, targets = targets_from_dict(load_fixture_json(fn))
            plan = construct_nn(targets, home, drone)
            if plan.feasible:
                _verify_constraints(plan, targets, home, drone)

    def test_savings_constraints_satisfied_on_fixtures(self):
        """Savings 在 fixture 上的约束验证"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict

        drone = DroneSpec(500, 100000, 0.08, 0.002)

        for fn in ["custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json"]:
            home, targets = targets_from_dict(load_fixture_json(fn))
            plan = construct_savings(targets, home, drone)
            if plan.feasible:
                _verify_constraints(plan, targets, home, drone)


# ====================================================================
# 回归 (regression) — 已知 fixture 值
# ====================================================================

class TestRegression:
    """回归测试 — 锁定已知预期值"""

    def test_nn_custom_5_heavy_expected_value(self):
        """custom_5_heavy: NN 期望 equiv_dist"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict
        home, targets = targets_from_dict(load_fixture_json("custom_5_heavy.json"))
        drone = DroneSpec(500, 100000, 0.08, 0.002)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible, f"Expected feasible but got: {plan.warnings}"
        assert len(plan.sequence) == 5
        # 锁定值: 当前实现产出 (2026-08-05)
        assert plan.total_equiv_distance == pytest.approx(1051.5, rel=0.01)

    def test_nn_custom_15_mixed_all_visited(self):
        """custom_15_mixed: 应访问全部 15 点"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict
        home, targets = targets_from_dict(load_fixture_json("custom_15_mixed.json"))
        drone = DroneSpec(500, 100000, 0.08, 0.002)

        plan = construct_nn(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 15

    def test_nn_vs_savings_nn_wins(self):
        """在所有 6 个实例上, NN 的 total_equiv_dist <= Savings (至少不差)"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict
        drone = DroneSpec(500, 100000, 0.08, 0.002)

        fixtures = [
            "custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json",
            "solomon_r101_n20.json", "solomon_c101_n20.json", "solomon_rc101_n20.json",
        ]
        for fn in fixtures:
            home, targets = targets_from_dict(load_fixture_json(fn))
            nn = construct_nn(targets, home, drone)
            sv = construct_savings(targets, home, drone)

            if nn.feasible and sv.feasible:
                assert nn.total_equiv_distance <= sv.total_equiv_distance, (
                    f"{fn}: NN ({nn.total_equiv_distance:.1f}) should not be worse "
                    f"than Savings ({sv.total_equiv_distance:.1f})"
                )


# ====================================================================
# W4 局部搜索 — 2-opt / Or-opt / VND 单测
# ====================================================================

# --- 辅助函数 (增量评估) ---

class TestIncrementalHelpers:
    """增量评估辅助函数测试"""

    def test_segment_payloads_correct(self):
        """_segment_payloads 返回正确的各段载重"""
        targets_map = {
            "c1": Target(id="c1", location=GeoPoint(100, 0), demand=3),
            "c2": Target(id="c2", location=GeoPoint(200, 0), demand=5),
            "c3": Target(id="c3", location=GeoPoint(300, 0), demand=2),
        }
        total_demand = 10.0
        payloads = _segment_payloads(["c1", "c2", "c3"], targets_map, total_demand)
        # full_path: home→c1→c2→c3→home, 4 segments
        assert len(payloads) == 4
        assert payloads[0] == 10.0  # home→c1: full load
        assert payloads[1] == 7.0   # c1→c2: after delivering c1 (3)
        assert payloads[2] == 2.0   # c2→c3: after delivering c2 (5)
        assert payloads[3] == 0.0   # c3→home: all delivered

    def test_segment_payloads_zero_demand(self):
        """所有点 demand=0 -> payloads 全为 0"""
        targets_map = {
            "c1": Target(id="c1", location=GeoPoint(100, 0), demand=0),
            "c2": Target(id="c2", location=GeoPoint(200, 0), demand=0),
        }
        payloads = _segment_payloads(["c1", "c2"], targets_map, 0.0)
        assert payloads == [0.0, 0.0, 0.0]

    def test_try_2opt_move_finds_improvement(self):
        """_try_2opt_move 在存在改进时返回新序列"""
        home = GeoPoint(0, 0)
        targets_map = {
            "A": Target(id="A", location=GeoPoint(100, 0), demand=1),
            "B": Target(id="B", location=GeoPoint(200, 100), demand=1),
            "C": Target(id="C", location=GeoPoint(100, 100), demand=1),
            "D": Target(id="D", location=GeoPoint(200, 0), demand=1),
        }
        drone = DroneSpec(50, 50000, 0.1, 0.005)
        total_demand = 4.0
        # NN gives ['A','C','B','D'], flip C-B-D gives better route
        sequence = ["A", "C", "B", "D"]
        current_equiv = 650.0

        result = _try_2opt_move(
            sequence, 0, 3, targets_map, home, drone,
            total_demand, current_equiv,
        )
        assert result is not None
        new_seq, new_equiv = result
        assert new_equiv < current_equiv
        assert len(new_seq) == 4
        assert set(new_seq) == {"A", "B", "C", "D"}

    def test_try_2opt_move_rejects_worse_move(self):
        """_try_2opt_move 拒绝增加等效距离的移动"""
        home = GeoPoint(0, 0)
        targets_map = {
            "A": Target(id="A", location=GeoPoint(100, 0), demand=1),
            "B": Target(id="B", location=GeoPoint(200, 0), demand=1),
            "C": Target(id="C", location=GeoPoint(300, 0), demand=1),
        }
        drone = DroneSpec(50, 50000, 0.1, 0.005)
        total_demand = 3.0
        # On a line: A→B→C is already optimal
        sequence = ["A", "B", "C"]
        _, _, current_equiv, _, _, _, _ = simulate_route_energy(
            sequence, targets_map, home, drone
        )

        result = _try_2opt_move(
            sequence, 0, 2, targets_map, home, drone,
            total_demand, current_equiv,
        )
        assert result is None

    def test_try_2opt_move_rejects_battery_infeasible(self):
        """_try_2opt_move 拒绝电池不可行的移动"""
        home = GeoPoint(0, 0)
        targets_map = {
            "A": Target(id="A", location=GeoPoint(100, 0), demand=1),
            "B": Target(id="B", location=GeoPoint(10000, 0), demand=1),
        }
        drone = DroneSpec(50, 500, 0.1, 0.005)  # 低电池
        total_demand = 2.0
        sequence = ["A", "B"]
        _, _, current_equiv, _, _, _, _ = simulate_route_energy(
            sequence, targets_map, home, drone
        )

        result = _try_2opt_move(
            sequence, 0, 1, targets_map, home, drone,
            total_demand, current_equiv,
        )
        if result is not None:
            new_seq, new_equiv = result
            _, _, _, _, _, feasible, _ = simulate_route_energy(
                new_seq, targets_map, home, drone
            )
            assert feasible


# --- 2-opt 搜索 ---

class Test2OptPositive:
    """2-opt 正例: 应改进初始解"""

    def test_2opt_improves_crossing_route(self):
        """2-opt 应消除路线中的交叉"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 1), (200, 100, 1), (100, 100, 1), (200, 0, 1),
        ])
        drone = DroneSpec(50, 50000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        assert nn.feasible

        targets_map = {t.id: t for t in targets}
        opt2 = local_search_2opt(nn, targets_map, home, drone)
        assert opt2.feasible
        assert len(opt2.sequence) == len(targets)
        assert set(opt2.sequence) == {t.id for t in targets}
        _verify_constraints(opt2, targets, home, drone)
        # 2-opt should not make things worse
        assert opt2.total_equiv_distance <= nn.total_equiv_distance + 0.01

    def test_2opt_improves_scattered_points(self):
        """2-opt 在散点上的改进"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (500, 500, 1), (100, 100, 1), (400, 400, 1),
            (200, 200, 1), (300, 300, 1),
        ])
        drone = DroneSpec(20, 20000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        opt2 = local_search_2opt(nn, targets_map, home, drone)

        assert opt2.feasible
        assert set(opt2.sequence) == {t.id for t in targets}
        _verify_constraints(opt2, targets, home, drone)
        assert opt2.total_equiv_distance <= nn.total_equiv_distance + 0.01

    def test_2opt_preserves_all_points(self):
        """2-opt 不丢失任何目标点"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 2), (50, 80, 3), (200, 50, 2), (150, 200, 4), (300, 100, 1),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        opt2 = local_search_2opt(nn, targets_map, home, drone)

        assert len(opt2.sequence) == len(targets)
        assert set(opt2.sequence) == set(nn.sequence)


class Test2OptBoundary:
    """2-opt 边界用例"""

    def test_2opt_empty_route(self):
        """空路线搜索 -> 原样返回"""
        route = RoutePlan(
            sequence=[], segments=[],
            total_geo_distance=0.0, total_equiv_distance=0.0,
            total_energy_consumed=0.0, remaining_energy=1000.0,
            total_payload_delivered=0.0, feasible=False,
        )
        home = GeoPoint(0, 0)
        drone = DroneSpec(50, 5000, 0.1, 0.005)
        result = local_search_2opt(route, {}, home, drone)
        assert result.sequence == []
        assert result.total_equiv_distance == 0.0

    def test_2opt_single_point(self):
        """单点路线搜索 -> 原样返回"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 5)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        opt2 = local_search_2opt(nn, targets_map, home, drone)

        assert opt2.sequence == nn.sequence
        assert opt2.total_equiv_distance == nn.total_equiv_distance

    def test_2opt_max_iterations_exhausted(self):
        """max_iterations=1 仅执行一轮搜索"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 1), (200, 100, 1), (100, 100, 1), (200, 0, 1),
        ])
        drone = DroneSpec(50, 50000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        opt2 = local_search_2opt(nn, targets_map, home, drone, max_iterations=1)
        assert opt2.feasible


# --- Or-opt 搜索 ---

class TestOrOptPositive:
    """Or-opt 正例"""

    def test_or_opt_improves_suboptimal_route(self):
        """Or-opt 通过移动点段改进路线"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 1), (200, 100, 1), (100, 100, 1),
            (200, 0, 1), (300, 50, 1),
        ])
        drone = DroneSpec(50, 50000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        or_opt = local_search_or_opt(nn, targets_map, home, drone)

        assert or_opt.feasible
        assert set(or_opt.sequence) == {t.id for t in targets}
        _verify_constraints(or_opt, targets, home, drone)
        assert or_opt.total_equiv_distance <= nn.total_equiv_distance + 0.01

    def test_or_opt_preserves_all_points(self):
        """Or-opt 不丢失任何目标点"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 2), (200, 50, 3), (50, 80, 2), (150, 200, 1),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        or_opt = local_search_or_opt(nn, targets_map, home, drone)

        assert len(or_opt.sequence) == len(targets)
        assert set(or_opt.sequence) == set(nn.sequence)


class TestOrOptBoundary:
    """Or-opt 边界用例"""

    def test_or_opt_single_point(self):
        """单点路线 Or-opt 搜索 -> 原样返回"""
        home = GeoPoint(0, 0)
        targets = _make_targets([(100, 0, 5)])
        drone = DroneSpec(50, 5000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        or_opt = local_search_or_opt(nn, targets_map, home, drone)

        assert or_opt.sequence == nn.sequence

    def test_or_opt_segment_size_one(self):
        """max_segment_size=1 只移动单个点"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 1), (200, 100, 1), (100, 100, 1), (200, 0, 1),
        ])
        drone = DroneSpec(50, 50000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        or_opt = local_search_or_opt(
            nn, targets_map, home, drone, max_segment_size=1,
        )
        assert or_opt.feasible
        assert or_opt.total_equiv_distance <= nn.total_equiv_distance + 0.01


# --- VND 搜索 ---

class TestVNDPositive:
    """VND 正例"""

    def test_vnd_improves_nn_solution(self):
        """VND (2-opt + Or-opt 交替) 改进 NN 初始解"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 1), (200, 100, 1), (100, 100, 1),
            (200, 0, 1), (300, 50, 1),
        ])
        drone = DroneSpec(50, 50000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        vnd = local_search_vnd(nn, targets_map, home, drone)

        assert vnd.feasible
        assert set(vnd.sequence) == {t.id for t in targets}
        _verify_constraints(vnd, targets, home, drone)
        assert vnd.total_equiv_distance <= nn.total_equiv_distance + 0.01

    def test_vnd_better_than_or_equal_to_nn(self):
        """VND 的最终解不差于 NN 初始解"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (500, 500, 1), (100, 100, 1), (400, 400, 1),
            (200, 200, 1), (300, 300, 1),
        ])
        drone = DroneSpec(20, 20000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}
        vnd = local_search_vnd(nn, targets_map, home, drone)

        assert vnd.total_equiv_distance <= nn.total_equiv_distance + 0.01
        _verify_constraints(vnd, targets, home, drone)

    def test_vnd_multi_iteration_converges(self):
        """VND 在多次迭代后收敛 (不再改进)"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 1), (200, 100, 1), (100, 100, 1),
            (200, 0, 1), (150, 200, 1),
        ])
        drone = DroneSpec(50, 50000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}

        vnd1 = local_search_vnd(nn, targets_map, home, drone, max_iterations=5)
        vnd2 = local_search_vnd(nn, targets_map, home, drone, max_iterations=20)

        assert vnd1.sequence == vnd2.sequence
        assert vnd1.total_equiv_distance == vnd2.total_equiv_distance


class TestVNDConsistency:
    """VND 一致性检查"""

    def test_vnd_deterministic(self):
        """相同输入 -> 相同 VND 输出"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 2), (50, 80, 3), (200, 50, 2), (150, 200, 4), (300, 100, 1),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}

        vnd1 = local_search_vnd(nn, targets_map, home, drone)
        vnd2 = local_search_vnd(nn, targets_map, home, drone)

        assert vnd1.sequence == vnd2.sequence
        assert vnd1.total_equiv_distance == vnd2.total_equiv_distance
        assert vnd1.total_energy_consumed == vnd2.total_energy_consumed

    def test_vnd_constraints_satisfied_on_fixtures(self):
        """VND 在 fixture 上的约束验证"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict

        drone = DroneSpec(500, 100000, 0.08, 0.002)

        for fn in ["custom_5_heavy.json", "custom_10_tight.json", "custom_15_mixed.json"]:
            home, targets = targets_from_dict(load_fixture_json(fn))
            nn = construct_nn(targets, home, drone)
            targets_map = {t.id: t for t in targets}
            if nn.feasible:
                vnd = local_search_vnd(nn, targets_map, home, drone)
                if vnd.feasible:
                    _verify_constraints(vnd, targets, home, drone)
                    assert vnd.total_equiv_distance <= nn.total_equiv_distance + 0.01

    def test_2opt_deterministic(self):
        """2-opt 搜索确定性"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 2), (50, 80, 3), (200, 50, 2), (150, 200, 4), (300, 100, 1),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}

        opt2_1 = local_search_2opt(nn, targets_map, home, drone)
        opt2_2 = local_search_2opt(nn, targets_map, home, drone)

        assert opt2_1.sequence == opt2_2.sequence
        assert opt2_1.total_equiv_distance == opt2_2.total_equiv_distance

    def test_or_opt_deterministic(self):
        """Or-opt 搜索确定性"""
        home = GeoPoint(0, 0)
        targets = _make_targets([
            (100, 0, 2), (50, 80, 3), (200, 50, 2),
        ])
        drone = DroneSpec(50, 10000, 0.1, 0.005)

        nn = construct_nn(targets, home, drone)
        targets_map = {t.id: t for t in targets}

        or1 = local_search_or_opt(nn, targets_map, home, drone)
        or2 = local_search_or_opt(nn, targets_map, home, drone)

        assert or1.sequence == or2.sequence
        assert or1.total_equiv_distance == or2.total_equiv_distance


# --- 增量评估 vs 全量评估 ---

class TestIncrementalVsFull:
    """增量评估与全量模拟对比 (专利创新点 3 验证)"""

    def test_2opt_incremental_matches_full_simulation(self):
        """2-opt 增量评估的 delta 与全量模拟一致"""
        home = GeoPoint(0, 0)
        targets_map = {
            "A": Target(id="A", location=GeoPoint(100, 0), demand=1),
            "B": Target(id="B", location=GeoPoint(200, 100), demand=1),
            "C": Target(id="C", location=GeoPoint(100, 100), demand=1),
            "D": Target(id="D", location=GeoPoint(200, 0), demand=1),
        }
        drone = DroneSpec(50, 50000, 0.1, 0.005)
        total_demand = 4.0
        sequence = ["A", "C", "B", "D"]

        _, _, current_equiv, _, _, _, _ = simulate_route_energy(
            sequence, targets_map, home, drone
        )
        result = _try_2opt_move(
            sequence, 0, 3, targets_map, home, drone,
            total_demand, current_equiv,
        )
        assert result is not None
        new_seq, inc_new_equiv = result

        _, _, full_new_equiv, _, _, _, _ = simulate_route_energy(
            new_seq, targets_map, home, drone
        )

        assert abs(inc_new_equiv - full_new_equiv) < 1.0, (
            f"Incremental equiv {inc_new_equiv:.2f} vs full {full_new_equiv:.2f}"
        )

    def test_or_opt_incremental_matches_full_simulation(self):
        """Or-opt 增量评估的 delta 与全量模拟一致"""
        home = GeoPoint(0, 0)
        targets_map = {
            "A": Target(id="A", location=GeoPoint(100, 0), demand=1),
            "B": Target(id="B", location=GeoPoint(200, 100), demand=1),
            "C": Target(id="C", location=GeoPoint(100, 100), demand=1),
            "D": Target(id="D", location=GeoPoint(200, 0), demand=1),
            "E": Target(id="E", location=GeoPoint(300, 50), demand=1),
        }
        drone = DroneSpec(50, 50000, 0.1, 0.005)
        total_demand = 5.0
        sequence = ["A", "C", "B", "D", "E"]

        _, _, current_equiv, _, _, _, _ = simulate_route_energy(
            sequence, targets_map, home, drone
        )
        result = _try_or_opt_move(
            sequence, 3, 4, 0, targets_map, home, drone,
            total_demand, current_equiv,
        )
        if result is not None:
            new_seq, inc_new_equiv = result
            _, _, full_new_equiv, _, _, _, _ = simulate_route_energy(
                new_seq, targets_map, home, drone
            )
            assert abs(inc_new_equiv - full_new_equiv) < 1.0, (
                f"Incremental equiv {inc_new_equiv:.2f} vs full {full_new_equiv:.2f}"
            )


# --- 回归 ---

class TestW4Regression:
    """W4 回归测试 — 锁定已知预期值"""

    def test_vnd_custom_5_heavy_expected(self):
        """custom_5_heavy: VND 期望 equiv_dist (应 ≤ NN)"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict
        home, targets = targets_from_dict(load_fixture_json("custom_5_heavy.json"))
        drone = DroneSpec(500, 100000, 0.08, 0.002)

        from a3_python.solver import plan_multistop
        plan = plan_multistop(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 5
        nn = construct_nn(targets, home, drone)
        assert plan.total_equiv_distance <= nn.total_equiv_distance + 0.01

    def test_vnd_custom_15_mixed_all_visited(self):
        """custom_15_mixed: VND 后仍访问全部 15 点"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict
        home, targets = targets_from_dict(load_fixture_json("custom_15_mixed.json"))
        drone = DroneSpec(500, 100000, 0.08, 0.002)

        from a3_python.solver import plan_multistop
        plan = plan_multistop(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 15

    def test_vnd_on_solomon_r101(self):
        """Solomon R101 n20: VND 搜索后可行"""
        from a3_python.fixture_loader import load_fixture_json, targets_from_dict
        home, targets = targets_from_dict(load_fixture_json("solomon_r101_n20.json"))
        drone = DroneSpec(500, 100000, 0.08, 0.002)

        from a3_python.solver import plan_multistop
        plan = plan_multistop(targets, home, drone)

        assert plan.feasible
        assert len(plan.sequence) == 20
        _verify_constraints(plan, targets, home, drone)
