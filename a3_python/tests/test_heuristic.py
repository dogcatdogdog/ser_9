"""test_heuristic.py — 构造启发式 + 局部搜索 单测 (W3-W4 实现)

W1 骨架: 所有测试预期跳过 (NotImplementedError).
"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.heuristic import (
    construct_nn,
    construct_savings,
    local_search_2opt,
    local_search_or_opt,
)


# === W1 骨架: 预期 NotImplementedError ===

def test_construct_nn_not_implemented():
    """W1: NN 构造尚未实现"""
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=50.0,
        battery_capacity=5000.0,
        alpha=0.1,
        beta=0.005,
    )
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
    ]
    with pytest.raises(NotImplementedError):
        construct_nn(targets, home, drone)


def test_construct_savings_not_implemented():
    """W1: Savings 构造尚未实现"""
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=50.0,
        battery_capacity=5000.0,
        alpha=0.1,
        beta=0.005,
    )
    targets = [
        Target(id="c1", location=GeoPoint(x=100, y=0), demand=5.0),
    ]
    with pytest.raises(NotImplementedError):
        construct_savings(targets, home, drone)


def test_local_search_2opt_not_implemented():
    """W1: 2-opt 尚未实现"""
    from a3_python.route import RoutePlan
    dummy_route = RoutePlan(
        sequence=[], segments=[],
        total_geo_distance=0.0, total_equiv_distance=0.0,
        total_energy_consumed=0.0, remaining_energy=0.0,
        total_payload_delivered=0.0, feasible=False,
    )
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=50.0, battery_capacity=5000.0,
        alpha=0.1, beta=0.005,
    )
    with pytest.raises(NotImplementedError):
        local_search_2opt(dummy_route, {}, home, drone)


def test_local_search_or_opt_not_implemented():
    """W1: Or-opt 尚未实现"""
    from a3_python.route import RoutePlan
    dummy_route = RoutePlan(
        sequence=[], segments=[],
        total_geo_distance=0.0, total_equiv_distance=0.0,
        total_energy_consumed=0.0, remaining_energy=0.0,
        total_payload_delivered=0.0, feasible=False,
    )
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=50.0, battery_capacity=5000.0,
        alpha=0.1, beta=0.005,
    )
    with pytest.raises(NotImplementedError):
        local_search_or_opt(dummy_route, {}, home, drone)
