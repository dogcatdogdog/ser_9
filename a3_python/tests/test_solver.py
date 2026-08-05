"""test_solver.py — plan_multistop() 单元测试

对齐 A3_SCHEMA.md §5.2: 正例 / 退化 / 边界 / 一致性
共享 fixtures 定义在 conftest.py。
"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.solver import plan_multistop


# ====================================================================
# 正例 (Happy Path) — 电量充裕, 所有点被访问, feasible=True
# ====================================================================

def test_plan_3points_feasible(home, drone_default):
    """测试 1 (正例): 3 点、电量充裕 → feasible=True, 所有点被访问"""
    drone = DroneSpec(payload_capacity=30.0, battery_capacity=10000.0,
                      alpha=0.1, beta=0.005)
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
        Target(id="c2", location=GeoPoint(x=0, y=100), demand=5.0),
        Target(id="c3", location=GeoPoint(x=100, y=100), demand=5.0),
    ]

    result = plan_multistop(targets, home, drone)

    assert len(result.sequence) == 3
    assert set(result.sequence) == {"c1", "c2", "c3"}
    assert result.total_geo_distance > 0
    assert result.total_equiv_distance > 0
    assert result.total_energy_consumed > 0
    assert result.remaining_energy < drone.battery_capacity
    assert result.total_payload_delivered == 15.0
    assert result.feasible is True
    assert len(result.warnings) == 0


def test_plan_5points_feasible(home, drone_default):
    """测试 (正例): 5 点、电量充裕 → feasible=True"""
    drone = DroneSpec(payload_capacity=40.0, battery_capacity=20000.0,
                      alpha=0.1, beta=0.005)
    targets = [
        Target(id="c1", location=GeoPoint(x=50, y=0), demand=3.0),
        Target(id="c2", location=GeoPoint(x=0, y=50), demand=3.0),
        Target(id="c3", location=GeoPoint(x=100, y=50), demand=3.0),
        Target(id="c4", location=GeoPoint(x=50, y=100), demand=3.0),
        Target(id="c5", location=GeoPoint(x=200, y=200), demand=3.0),
    ]

    result = plan_multistop(targets, home, drone)

    assert len(result.sequence) == 5
    assert result.feasible is True
    assert result.total_payload_delivered == 15.0


# ====================================================================
# 退化 (Degraded) — 超载 / 电量不足 → feasible=False + 明确原因
# ====================================================================

def test_plan_overload_infeasible(home, drone_default):
    """测试 2 (退化): 总载重 > capacity → feasible=False, warnings 包含 'overload'"""
    drone = DroneSpec(payload_capacity=10.0, battery_capacity=5000.0,
                      alpha=0.1, beta=0.005)
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=8.0),
        Target(id="c2", location=GeoPoint(x=0, y=100), demand=8.0),  # total 16 > 10
    ]

    result = plan_multistop(targets, home, drone)

    assert result.feasible is False
    assert len(result.warnings) > 0
    assert any("exceeds drone capacity" in w.lower() for w in result.warnings)
    # W3: NN 检测到超载立即返回空路线 (比强行访问全部点更安全)
    assert result.total_geo_distance == 0.0


def test_plan_low_battery_infeasible(home):
    """测试 (退化): 电量不足以支撑全路线 → feasible=False"""
    drone = DroneSpec(payload_capacity=50.0, battery_capacity=5.0,
                      alpha=0.1, beta=0.005)
    targets = [
        Target(id="c1", location=GeoPoint(x=500, y=500), demand=1.0),
    ]

    result = plan_multistop(targets, home, drone)

    assert result.feasible is False
    assert len(result.warnings) > 0 or result.feasible is False


# ====================================================================
# 边界 (Boundary) — 0 点 / 1 点 / 超上限
# ====================================================================

def test_plan_zero_targets_error(home, drone_default):
    """测试 3 (边界): 0 点 → ValueError"""
    with pytest.raises(ValueError, match="cannot be empty"):
        plan_multistop([], home, drone_default)


def test_plan_1point_boundary(home, drone_default):
    """测试 (边界): 1 点 → feasible=True, 直线往返"""
    drone = DroneSpec(payload_capacity=10.0, battery_capacity=5000.0,
                      alpha=0.1, beta=0.005)
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
    ]

    result = plan_multistop(targets, home, drone)

    assert result.feasible is True
    assert result.sequence == ["c1"]
    assert len(result.segments) == 2  # home→c1, c1→home
    assert result.total_geo_distance == pytest.approx(200.0, rel=0.1)


def test_plan_over_max_targets_error(home, drone_default):
    """测试 (边界): 超过 20 点上限定 → ValueError"""
    targets = [
        Target(id=f"c{i}", location=GeoPoint(x=float(i), y=0.0), demand=0.1)
        for i in range(21)
    ]

    with pytest.raises(ValueError, match="exceeds MVP limit"):
        plan_multistop(targets, home, drone_default)


# ====================================================================
# 一致性 (Consistency) — 确定性 / 可复现
# ====================================================================

def test_plan_deterministic(home, drone_default):
    """测试 (一致性): 相同输入 + 相同 seed → 相同输出"""
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
        Target(id="c2", location=GeoPoint(x=0, y=100), demand=5.0),
    ]

    r1 = plan_multistop(targets, home, drone_default, seed=42)
    r2 = plan_multistop(targets, home, drone_default, seed=42)

    assert r1.sequence == r2.sequence
    assert r1.total_equiv_distance == r2.total_equiv_distance
    assert r1.feasible == r2.feasible


def test_plan_same_seed_consistency(home, drone_default):
    """测试 (一致性): 相同 seed → 相同输出, 不同 seed 可能不同 (W1 按输入顺序所以相同)"""
    targets = [
        Target(id="c1", location=GeoPoint(x=10, y=0), demand=5.0),
        Target(id="c2", location=GeoPoint(x=50, y=0), demand=5.0),
        Target(id="c3", location=GeoPoint(x=100, y=0), demand=5.0),
    ]

    r1 = plan_multistop(targets, home, drone_default, seed=99)
    r2 = plan_multistop(targets, home, drone_default, seed=99)
    assert r1.sequence == r2.sequence
    assert r1.total_geo_distance == r2.total_geo_distance
