"""集成测试 — 标准 VRP 实例上验证完整求解流程

对齐 A3_SCHEMA.md §5.3: 5-8 例, Solomon + 自建场景

W1-W4: 验证可行性和解结构
W5: 加入 OR-Tools gap 断言 (用例 11/12/13)
"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec, RoutePlan
from a3_python.solver import plan_multistop


# === 辅助函数 ===

def _count_visited(sequence: list[str], targets: list[Target]) -> int:
    """统计实际访问的点数"""
    target_ids = {t.id for t in targets}
    return sum(1 for sid in sequence if sid in target_ids)


# ============================================================
# 用例 11-13: Solomon 标准实例 (gap vs OR-Tools — W5 引入)
# ============================================================

class TestSolomonInstances:
    """Solomon VRPTW 前 20 点子集集成测试"""

    def test_r101_n20_feasible(self, solomon_r101_n20, drone_heavy_lift):
        """用例 11: Solomon R101 20 点 → feasible=True, 所有点被访问"""
        home, targets = solomon_r101_n20
        result = plan_multistop(targets, home, drone_heavy_lift)

        assert result.feasible is True, f"Should be feasible, warnings: {result.warnings}"
        assert len(result.sequence) == 20, f"All 20 points visited, got {len(result.sequence)}"
        assert _count_visited(result.sequence, targets) == 20

        # 解结构验证
        assert result.total_geo_distance > 0
        assert result.total_equiv_distance > 0
        assert result.total_energy_consumed > 0
        assert result.total_payload_delivered == sum(t.demand for t in targets)

        # W5: assert gap vs OR-Tools < 10%

    def test_c101_n20_feasible(self, solomon_c101_n20, drone_heavy_lift):
        """用例 12: Solomon C101 20 点 (聚类分布) → feasible=True"""
        home, targets = solomon_c101_n20
        result = plan_multistop(targets, home, drone_heavy_lift)

        assert result.feasible is True, f"Should be feasible, warnings: {result.warnings}"
        assert len(result.sequence) == 20
        assert _count_visited(result.sequence, targets) == 20

        # W5: assert gap vs OR-Tools < 10%

    def test_rc101_n20_feasible(self, solomon_rc101_n20, drone_heavy_lift):
        """用例 13: Solomon RC101 20 点 (混合分布) → feasible=True"""
        home, targets = solomon_rc101_n20
        result = plan_multistop(targets, home, drone_heavy_lift)

        assert result.feasible is True, f"Should be feasible, warnings: {result.warnings}"
        assert len(result.sequence) == 20
        assert _count_visited(result.sequence, targets) == 20

        # W5: assert gap vs OR-Tools < 10%


# ============================================================
# 用例 14-16: 自建无人机配送场景
# ============================================================

class TestCustomScenarios:
    """自建配送场景集成测试 (含载重+电量约束)"""

    def test_5_heavy_feasible(self, custom_5_heavy, drone_default):
        """用例 14: 自建 5 点重载 → feasible=True, 载重约束满足"""
        home, targets = custom_5_heavy
        drone = drone_default
        total_demand = sum(t.demand for t in targets)

        # 确认总载重未超容量
        assert total_demand <= drone.payload_capacity, (
            f"Total demand {total_demand} should fit in {drone.payload_capacity}kg"
        )

        result = plan_multistop(targets, home, drone)

        assert result.feasible is True, f"Should be feasible, warnings: {result.warnings}"
        assert len(result.sequence) == 5
        assert result.total_payload_delivered == total_demand

        # 每段载重都不超 capacity
        for seg in result.segments:
            assert seg.payload_before <= drone.payload_capacity

    def test_10_tight_infeasible(self, custom_10_tight, drone_small):
        """用例 15: 自建 10 点电量紧张 → feasible=False, 原因明确"""
        home, targets = custom_10_tight
        # drone_small: 1000Wh 电池不足以覆盖 10 个远点
        result = plan_multistop(targets, home, drone_small)

        # 电量不足以走完全程 → 应不可行
        assert result.feasible is False, "Should be infeasible due to battery"
        assert len(result.warnings) > 0, "Should have at least one warning"

    def test_15_mixed_feasible(self, custom_15_mixed, drone_big):
        """用例 16: 自建 15 点联合约束 → feasible=True, 载重+电量联合满足"""
        home, targets = custom_15_mixed
        drone = drone_big
        result = plan_multistop(targets, home, drone)

        assert result.feasible is True, f"Should be feasible, warnings: {result.warnings}"
        assert len(result.sequence) == 15
        assert _count_visited(result.sequence, targets) == 15

        # 载重约束: 每段出发载重 ≤ capacity
        for seg in result.segments:
            assert seg.payload_before <= drone.payload_capacity, (
                f"Segment {seg.from_id}→{seg.to_id}: payload {seg.payload_before} > capacity {drone.payload_capacity}"
            )

        # 电量约束: 每段电池不耗尽
        for seg in result.segments:
            assert seg.battery_after >= 0, (
                f"Segment {seg.from_id}→{seg.to_id}: battery went negative ({seg.battery_after})"
            )


# ============================================================
# 用例 17-18: 对比测试 (电量约束改变了解)
# ============================================================

class TestConstraintComparison:
    """对比: 有无电量约束时解的差异"""

    def test_energy_constraint_changes_solution(self, custom_5_heavy):
        """验证电量约束从无到有会影响解的结构"""
        home, targets = custom_5_heavy

        # 1) 电量充裕 → feasible
        drone_large = DroneSpec(
            payload_capacity=50.0,
            battery_capacity=50000.0,  # 很大
            alpha=0.1,
            beta=0.005,
        )
        r_large = plan_multistop(targets, home, drone_large)
        assert r_large.feasible is True

        # 2) 电量紧张 → 可能 infeasible 或至少路线不同
        drone_tight = DroneSpec(
            payload_capacity=50.0,
            battery_capacity=50.0,  # 极小
            alpha=0.1,
            beta=0.005,
        )
        r_tight = plan_multistop(targets, home, drone_tight)

        # 电量极少时必定不可行或至少剩余电量远小于充裕情况
        if r_tight.feasible:
            assert r_tight.remaining_energy < r_large.remaining_energy
        else:
            assert len(r_tight.warnings) > 0

    def test_payload_affects_equiv_distance(self, drone_default):
        """载重变化影响等效距离: 满载路线 vs 空载路线的等效距离不同"""
        home = GeoPoint(x=0.0, y=0.0)

        # 重载 targets
        targets_heavy = [
            Target(id="h1", location=GeoPoint(x=100, y=0), demand=20.0),
            Target(id="h2", location=GeoPoint(x=0, y=100), demand=20.0),
        ]
        # 轻载 targets
        targets_light = [
            Target(id="l1", location=GeoPoint(x=100, y=0), demand=0.1),
            Target(id="l2", location=GeoPoint(x=0, y=100), demand=0.1),
        ]

        drone = drone_default
        r_heavy = plan_multistop(targets_heavy, home, drone)
        r_light = plan_multistop(targets_light, home, drone)

        # 几何距离相同 (因为坐标相同) — W1 按输入顺序
        assert r_heavy.total_geo_distance == pytest.approx(r_light.total_geo_distance, rel=0.001)

        # 等效距离: 重载 > 轻载 (载重-能耗耦合)
        assert r_heavy.total_equiv_distance > r_light.total_equiv_distance
