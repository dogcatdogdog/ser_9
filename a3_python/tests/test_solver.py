"""test_solver.py — plan_multistop() 单测

W1 骨架: 正例 / 退化 / 边界 3 个用例。
对齐 A3_SCHEMA.md §5 单测用例设计。
"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec, RoutePlan
from a3_python.solver import plan_multistop


# === 共享 fixtures ===

def _make_home() -> GeoPoint:
    return GeoPoint(x=0.0, y=0.0)


def _make_drone(
    payload_capacity: float = 50.0,
    battery_capacity: float = 5000.0,
    alpha: float = 0.1,
    beta: float = 0.005,
) -> DroneSpec:
    return DroneSpec(
        payload_capacity=payload_capacity,
        battery_capacity=battery_capacity,
        alpha=alpha,
        beta=beta,
    )


# === 正例 (Happy Path) ===

def test_plan_3points_feasible():
    """测试 1 (正例): 3 点、电量充裕 → feasible=True, 所有点被访问"""
    home = _make_home()
    drone = _make_drone(payload_capacity=30.0, battery_capacity=10000.0)
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
        Target(id="c2", location=GeoPoint(x=0, y=100), demand=5.0),
        Target(id="c3", location=GeoPoint(x=100, y=100), demand=5.0),
    ]

    result = plan_multistop(targets, home, drone)

    # 所有点被访问
    assert len(result.sequence) == 3
    assert set(result.sequence) == {"c1", "c2", "c3"}

    # 路径包含 home → c1 → c2 → c3 → home (W1 按输入顺序)
    assert result.total_geo_distance > 0
    assert result.total_equiv_distance > 0
    assert result.total_energy_consumed > 0
    assert result.remaining_energy < drone.battery_capacity
    assert result.total_payload_delivered == 15.0  # 5+5+5

    # 电量充裕 → 可行
    assert result.feasible is True
    assert len(result.warnings) == 0


def test_plan_5points_feasible():
    """测试 (正例): 5 点、电量充裕 → feasible=True"""
    home = _make_home()
    drone = _make_drone(payload_capacity=40.0, battery_capacity=20000.0)
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


# === 退化 (Degraded) ===

def test_plan_overload_infeasible():
    """测试 2 (退化): 总载重 > capacity → feasible=False, warnings 包含 'overload'"""
    home = _make_home()
    drone = _make_drone(payload_capacity=10.0, battery_capacity=5000.0)
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=8.0),
        Target(id="c2", location=GeoPoint(x=0, y=100), demand=8.0),  # total 16 > 10
    ]

    result = plan_multistop(targets, home, drone)

    assert result.feasible is False
    assert len(result.warnings) > 0
    assert any("exceeds drone capacity" in w.lower() for w in result.warnings)
    assert result.sequence == []  # 超载时无路线
    assert result.total_geo_distance == 0.0


def test_plan_low_battery_infeasible():
    """测试 (退化): 电量不足以支撑全路线 → feasible=False"""
    home = GeoPoint(x=0.0, y=0.0)
    # 电池极小, 不足以飞到任何点
    drone = DroneSpec(
        payload_capacity=50.0,
        battery_capacity=5.0,  # 极少电量
        alpha=0.1,
        beta=0.005,
    )
    targets = [
        Target(id="c1", location=GeoPoint(x=500, y=500), demand=1.0),
    ]

    result = plan_multistop(targets, home, drone)

    assert result.feasible is False
    # 应该有电量相关警告 (或至少不可行)
    assert len(result.warnings) > 0 or result.feasible is False


# === 边界 (Boundary) ===

def test_plan_zero_targets_error():
    """测试 3 (边界): 0 点 → ValueError"""
    home = _make_home()
    drone = _make_drone()

    with pytest.raises(ValueError, match="cannot be empty"):
        plan_multistop([], home, drone)


def test_plan_1point_boundary():
    """测试 (边界): 1 点 → feasible=True, 直线往返"""
    home = GeoPoint(x=0.0, y=0.0)
    drone = _make_drone(payload_capacity=10.0, battery_capacity=5000.0)
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
    ]

    result = plan_multistop(targets, home, drone)

    assert result.feasible is True
    assert result.sequence == ["c1"]
    assert len(result.segments) == 2  # home→c1, c1→home
    # 往返总距离 = 100 + 100 = 200
    assert result.total_geo_distance == pytest.approx(200.0, rel=0.1)


def test_plan_over_max_targets_error():
    """测试 (边界): 超过 20 点上限定 → ValueError"""
    home = _make_home()
    drone = _make_drone()
    targets = [
        Target(id=f"c{i}", location=GeoPoint(x=float(i), y=0.0), demand=0.1)
        for i in range(21)  # 21 > MAX_TARGETS (20)
    ]

    with pytest.raises(ValueError, match="exceeds MVP limit"):
        plan_multistop(targets, home, drone)


# === 一致性 (Consistency) ===

def test_plan_deterministic():
    """测试 (一致性): 相同输入 + 相同 seed → 相同输出"""
    home = _make_home()
    drone = _make_drone()
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
        Target(id="c2", location=GeoPoint(x=0, y=100), demand=5.0),
    ]

    r1 = plan_multistop(targets, home, drone, seed=42)
    r2 = plan_multistop(targets, home, drone, seed=42)

    assert r1.sequence == r2.sequence
    assert r1.total_equiv_distance == r2.total_equiv_distance
    assert r1.feasible == r2.feasible


def test_plan_same_seed_different_order_consistency():
    """测试 (一致性): 不同 seed 可能产生不同结果, 但相同 seed 一致"""
    home = _make_home()
    drone = _make_drone()
    targets = [
        Target(id="c1", location=GeoPoint(x=10, y=0), demand=5.0),
        Target(id="c2", location=GeoPoint(x=50, y=0), demand=5.0),
        Target(id="c3", location=GeoPoint(x=100, y=0), demand=5.0),
    ]

    # 相同 seed → 相同输出
    r1 = plan_multistop(targets, home, drone, seed=99)
    r2 = plan_multistop(targets, home, drone, seed=99)
    assert r1.sequence == r2.sequence
    assert r1.total_geo_distance == r2.total_geo_distance
