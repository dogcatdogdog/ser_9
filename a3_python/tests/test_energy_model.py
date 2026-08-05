"""test_energy_model.py — 等效距离变换 + 载重-能耗耦合模型 单测

对齐 A3_SCHEMA.md §5 单测用例设计: 用例 7, 9
"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.energy_model import (
    euclidean_distance,
    compute_equiv_distance,
    compute_energy_for_segment,
    simulate_route_energy,
)


# === 几何距离 ===

def test_euclidean_distance_same_point():
    """同一点距离为 0"""
    a = GeoPoint(x=5.0, y=3.0)
    assert euclidean_distance(a, a) == 0.0


def test_euclidean_distance_horizontal():
    """水平线段"""
    a = GeoPoint(x=0.0, y=0.0)
    b = GeoPoint(x=300.0, y=0.0)
    assert euclidean_distance(a, b) == 300.0


def test_euclidean_distance_3_4_5():
    """勾股数 3-4-5"""
    a = GeoPoint(x=0.0, y=0.0)
    b = GeoPoint(x=3.0, y=4.0)
    assert euclidean_distance(a, b) == 5.0


# === 等效距离变换 (专利创新点 1) ===

def test_equiv_distance_empty_load():
    """空载: equiv = geo (α + β×0) / α = geo"""
    geo = 100.0
    result = compute_equiv_distance(geo, payload=0.0, alpha=0.1, beta=0.005)
    assert result == pytest.approx(100.0)


def test_equiv_distance_loaded():
    """测试 7 (边界): 满载 vs 空载 同段 — equiv_dist(满载) > equiv_dist(空载)"""
    geo = 100.0
    alpha = 0.1
    beta = 0.005

    empty = compute_equiv_distance(geo, payload=0.0, alpha=alpha, beta=beta)
    loaded = compute_equiv_distance(geo, payload=20.0, alpha=alpha, beta=beta)

    # 满载等效距离大于空载 (载重-能耗耦合)
    assert loaded > empty
    # 满载: geo × (0.1 + 0.005×20) / 0.1 = geo × 0.2 / 0.1 = 2×geo
    assert loaded == pytest.approx(200.0)


def test_equiv_distance_formula_accuracy():
    """等效距离公式精确性: 手工计算验证"""
    geo = 50.0
    alpha = 0.1
    beta = 0.005
    payload = 10.0

    # equiv = geo × (α + β × payload) / α
    #       = 50 × (0.1 + 0.005×10) / 0.1
    #       = 50 × (0.1 + 0.05) / 0.1
    #       = 50 × 0.15 / 0.1
    #       = 50 × 1.5 = 75.0
    expected = 75.0
    result = compute_equiv_distance(geo, payload, alpha, beta)
    assert result == pytest.approx(expected)


def test_equiv_distance_alpha_zero_raises():
    """alpha ≤ 0 应抛出 ValueError"""
    with pytest.raises(ValueError):
        compute_equiv_distance(100.0, 0.0, alpha=0.0, beta=0.005)


# === 能耗计算 ===

def test_compute_energy_for_segment():
    """能耗 = equiv_dist × alpha"""
    geo = 100.0
    alpha = 0.1
    beta = 0.005
    payload = 10.0

    energy = compute_energy_for_segment(geo, payload, alpha, beta)
    # equiv = 100 × (0.1 + 0.05) / 0.1 = 150
    # energy = 150 × 0.1 = 15.0
    assert energy == pytest.approx(15.0)


# === 路线模拟 ===

def test_simulate_route_energy_heavy_first_vs_light_first():
    """测试 9 (能量模型): 先送重货 vs 先送轻货 → 等效距离不同

    专利创新点 2: 载重-能耗耦合 → 访问顺序影响总等效距离
    """
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=20.0,
        battery_capacity=10000.0,
        alpha=0.1,
        beta=0.005,
    )
    targets = {
        "heavy": Target(id="heavy", location=GeoPoint(x=100, y=0), demand=10.0),
        "light": Target(id="light", location=GeoPoint(x=200, y=0), demand=1.0),
    }

    # 路线 1: 先重后轻 home→heavy→light→home
    _, _, equiv1, _, _, feasible1, _ = simulate_route_energy(
        ["heavy", "light"], targets, home, drone
    )

    # 路线 2: 先轻后重 home→light→heavy→home
    _, _, equiv2, _, _, feasible2, _ = simulate_route_energy(
        ["light", "heavy"], targets, home, drone
    )

    # 两种顺序都是可行的 (电量充裕)
    assert feasible1 is True
    assert feasible2 is True

    # 访问顺序不同 → 等效距离不同 (载重-能耗耦合的核心体现)
    assert equiv1 != equiv2


def test_simulate_route_overload_detected():
    """超载检测: total payload > capacity"""
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=5.0,  # 容量只有 5kg
        battery_capacity=5000.0,
        alpha=0.1,
        beta=0.005,
    )
    targets = {
        "c1": Target(id="c1", location=GeoPoint(x=100, y=0), demand=10.0),  # > 5
    }

    segments, geo, equiv, energy, remaining, feasible, warnings = simulate_route_energy(
        ["c1"], targets, home, drone
    )

    assert feasible is False
    assert len(segments) == 0
    assert any("exceeds drone capacity" in w for w in warnings)


def test_simulate_route_low_battery_detected():
    """低电量检测: 电池不足以完成路线"""
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=50.0,
        battery_capacity=10.0,  # 极少电量
        alpha=0.1,
        beta=0.005,
    )
    targets = {
        "c1": Target(id="c1", location=GeoPoint(x=1000, y=0), demand=1.0),
    }

    _, _, _, _, _, feasible, warnings = simulate_route_energy(
        ["c1"], targets, home, drone
    )

    assert feasible is False
    assert len(warnings) > 0
