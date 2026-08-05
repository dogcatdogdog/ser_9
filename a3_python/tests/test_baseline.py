"""test_baseline.py — PyVRP 基线求解 单测

W2: 验证 PyVRP 能稳定求解 5/10/20 点 TSP，记录 baseline 指标。
数据生成器测试已移至 test_data_generator.py。
"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.baseline import solve_tsp_pyvrp, BaselineResult
from a3_python.data_generator import generate_targets


# ====================================================================
# 正例: PyVRP 求解
# ====================================================================

def test_solve_tsp_pyvrp_5points():
    """5 点圆形分布: PyVRP 应在 5s 内找到可行解"""
    targets = generate_targets(5, distribution="circle", scale=1000.0)
    home = GeoPoint(x=0.0, y=0.0)

    result = solve_tsp_pyvrp(targets, home, time_limit=5, instance_name="test_5")

    assert isinstance(result, BaselineResult)
    assert result.n_points == 5
    assert result.feasible is True
    assert result.total_distance > 0
    assert result.solve_time_ms > 0
    assert len(result.route) == 5
    assert set(result.route) == {t.id for t in targets}


def test_solve_tsp_pyvrp_10points():
    """10 点圆形分布: PyVRP 应在 5s 内找到可行解"""
    targets = generate_targets(10, distribution="circle", scale=1000.0)
    home = GeoPoint(x=0.0, y=0.0)

    result = solve_tsp_pyvrp(targets, home, time_limit=5, instance_name="test_10")

    assert result.feasible is True
    assert result.n_points == 10
    assert len(result.route) == 10
    assert result.total_distance > 0


def test_solve_tsp_pyvrp_deterministic():
    """相同 seed + 相同输入 → 相同结果 (确定性)"""
    targets = generate_targets(5, distribution="circle", scale=1000.0)
    home = GeoPoint(x=0.0, y=0.0)

    r1 = solve_tsp_pyvrp(targets, home, seed=42, instance_name="test")
    r2 = solve_tsp_pyvrp(targets, home, seed=42, instance_name="test")

    assert r1.route == r2.route
    assert r1.total_distance == r2.total_distance
    assert r1.optimal_cost == r2.optimal_cost


# ====================================================================
# 退化/边界
# ====================================================================

def test_solve_tsp_pyvrp_empty():
    """0 点: 空实例 → 返回空结果"""
    result = solve_tsp_pyvrp([], GeoPoint(x=0.0, y=0.0))
    assert result.n_points == 0
    assert result.feasible is True
    assert result.total_distance == 0.0
    assert result.route == []


def test_solve_tsp_pyvrp_1point():
    """1 点: 往返 home"""
    targets = [Target(id="c1", location=GeoPoint(x=100, y=0), demand=1.0)]
    home = GeoPoint(x=0.0, y=0.0)

    result = solve_tsp_pyvrp(targets, home, time_limit=5)

    assert result.feasible is True
    assert result.n_points == 1
    assert len(result.route) == 1
    assert result.total_distance > 0


# ====================================================================
# 集成: PyVRP 在 Solomon 真实数据上
# ====================================================================

def test_solve_tsp_solomon_r101():
    """Solomon R101 n20: PyVRP 应找到可行解"""
    from a3_python.tests.utils import load_fixture_json, targets_from_dict

    try:
        data = load_fixture_json("solomon_r101_n20.json")
        home, targets = targets_from_dict(data)
    except FileNotFoundError:
        pytest.skip("Solomon R101 fixture not found")

    result = solve_tsp_pyvrp(targets, home, time_limit=10, instance_name="r101")

    assert result.feasible is True
    assert result.n_points == 20
    assert len(result.route) == 20
    assert result.total_distance > 10
    assert result.total_distance < 100000
