"""test_energy_model.py — 等效距离变换 + 载重-能耗耦合模型 单测

对齐 A3_SCHEMA.md §5 单测用例设计: 用例 7, 9
W2 新增: compute_geo_matrix + compute_equiv_matrix 单测
"""

import pytest
import numpy as np
from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.energy_model import (
    euclidean_distance,
    compute_geo_matrix,
    compute_equiv_matrix,
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


# ====================================================================
# W2 新增: 距离矩阵 (compute_geo_matrix)
# ====================================================================

def test_geo_matrix_3points():
    """3 点距离矩阵: 验证 3-4-5 三角形"""
    pts = [
        GeoPoint(x=0.0, y=0.0),
        GeoPoint(x=3.0, y=0.0),
        GeoPoint(x=0.0, y=4.0),
    ]
    mat = compute_geo_matrix(pts)

    assert mat.shape == (3, 3)

    # 对角线为 0
    assert mat[0, 0] == pytest.approx(0.0)
    assert mat[1, 1] == pytest.approx(0.0)
    assert mat[2, 2] == pytest.approx(0.0)

    # 对称性
    assert mat[0, 1] == pytest.approx(mat[1, 0])
    assert mat[0, 2] == pytest.approx(mat[2, 0])
    assert mat[1, 2] == pytest.approx(mat[2, 1])

    # 具体值: (0,0)→(3,0)=3, (0,0)→(0,4)=4, (3,0)→(0,4)=5
    assert mat[0, 1] == pytest.approx(3.0)
    assert mat[0, 2] == pytest.approx(4.0)
    assert mat[1, 2] == pytest.approx(5.0)


def test_geo_matrix_empty():
    """空点集: 返回 (0,0) 矩阵"""
    mat = compute_geo_matrix([])
    assert mat.shape == (0, 0)


def test_geo_matrix_single_point():
    """单点: 返回 1×1 零矩阵"""
    pts = [GeoPoint(x=5.0, y=3.0)]
    mat = compute_geo_matrix(pts)
    assert mat.shape == (1, 1)
    assert mat[0, 0] == pytest.approx(0.0)


def test_geo_matrix_symmetry():
    """5 点随机测试: 验证对称性和非负性"""
    import numpy as np
    rng = np.random.default_rng(42)
    pts = [GeoPoint(x=rng.uniform(-100, 100), y=rng.uniform(-100, 100))
           for _ in range(5)]
    mat = compute_geo_matrix(pts)

    assert mat.shape == (5, 5)
    # 对称
    assert np.allclose(mat, mat.T)
    # 对角线为 0
    assert np.allclose(np.diag(mat), 0.0)
    # 非对角 > 0 (不同点)
    for i in range(5):
        for j in range(5):
            if i != j:
                assert mat[i, j] > 0


def test_geo_matrix_triangle_inequality():
    """三角形不等式: d(i,j) ≤ d(i,k) + d(k,j)"""
    pts = [
        GeoPoint(x=0.0, y=0.0),
        GeoPoint(x=10.0, y=0.0),
        GeoPoint(x=5.0, y=8.0),
    ]
    mat = compute_geo_matrix(pts)

    # 对任意 i,j,k: mat[i,j] <= mat[i,k] + mat[k,j]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                if i != j and i != k and j != k:
                    assert mat[i, j] <= mat[i, k] + mat[k, j] + 1e-10


# ====================================================================
# W2 新增: 等效距离矩阵 (compute_equiv_matrix)
# ====================================================================

def test_equiv_matrix_empty_load():
    """空载: 等效矩阵 = 几何矩阵"""
    pts = [
        GeoPoint(x=0.0, y=0.0),
        GeoPoint(x=3.0, y=0.0),
        GeoPoint(x=0.0, y=4.0),
    ]
    geo = compute_geo_matrix(pts)
    demands = [0.0, 0.0, 0.0]  # 全空载

    equiv = compute_equiv_matrix(geo, demands, alpha=0.1, beta=0.005)

    # 空载时 equiv = geo (因为 α+β×0 / α = 1)
    assert np.allclose(equiv, geo)


def test_equiv_matrix_with_load():
    """有载重: 等效矩阵 ≥ 几何矩阵 (逐元素)"""
    pts = [
        GeoPoint(x=0.0, y=0.0),   # home
        GeoPoint(x=10.0, y=0.0),  # c1, demand=5
        GeoPoint(x=0.0, y=10.0),  # c2, demand=5
    ]
    geo = compute_geo_matrix(pts)
    demands = [0.0, 5.0, 5.0]  # home=0, c1=5, c2=5
    alpha = 0.1
    beta = 0.005

    equiv = compute_equiv_matrix(geo, demands, alpha, beta)

    # 每个元素 equiv[i,j] >= geo[i,j]
    for i in range(3):
        for j in range(3):
            if i != j:
                assert equiv[i, j] >= geo[i, j], f"equiv[{i},{j}] < geo[{i},{j}]"


def test_equiv_matrix_formula_accuracy():
    """等效矩阵公式精确性: 手工验算单个元素"""
    pts = [
        GeoPoint(x=0.0, y=0.0),   # home, demand=0
        GeoPoint(x=100.0, y=0.0),  # c1, demand=10
    ]
    geo = compute_geo_matrix(pts)  # geo[0,1] = 100.0
    demands = [0.0, 10.0]
    alpha = 0.1
    beta = 0.005

    equiv = compute_equiv_matrix(geo, demands, alpha, beta)

    # 从 home(0) 出发: payload = total_demand - demand[0] = 10
    # equiv = 100 × (0.1 + 0.005×10) / 0.1 = 100 × 1.5 = 150.0
    expected_home_to_c1 = 150.0
    assert equiv[0, 1] == pytest.approx(expected_home_to_c1)

    # 从 c1(1) 出发: payload = total_demand - demand[1] = 10 - 10 = 0
    # equiv = 100 × (0.1 + 0.005×0) / 0.1 = 100.0
    assert equiv[1, 0] == pytest.approx(100.0)


def test_equiv_matrix_alpha_zero_raises():
    """alpha ≤ 0 抛出 ValueError"""
    pts = [GeoPoint(x=0.0, y=0.0), GeoPoint(x=1.0, y=0.0)]
    geo = compute_geo_matrix(pts)
    with pytest.raises(ValueError):
        compute_equiv_matrix(geo, [0.0, 1.0], alpha=0.0, beta=0.005)


def test_equiv_matrix_higher_beta_larger_equiv():
    """beta 越大 → 等效距离越大 (载重敏感度高)"""
    pts = [
        GeoPoint(x=0.0, y=0.0),
        GeoPoint(x=100.0, y=0.0),
    ]
    geo = compute_geo_matrix(pts)
    demands = [0.0, 20.0]
    alpha = 0.1

    equiv_low_beta = compute_equiv_matrix(geo.copy(), demands, alpha, beta=0.001)
    equiv_high_beta = compute_equiv_matrix(geo.copy(), demands, alpha, beta=0.01)

    # high beta 的等效距离更大
    assert equiv_high_beta[0, 1] > equiv_low_beta[0, 1]
