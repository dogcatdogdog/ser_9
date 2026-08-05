"""共享 fixtures — 所有测试文件可用 (pytest 自动发现)"""

import pytest
from a3_python.route import GeoPoint, Target, DroneSpec
from a3_python.fixture_loader import load_fixture_json, targets_from_dict


# === 基础 fixtures ===

@pytest.fixture
def home() -> GeoPoint:
    """默认仓库: 原点"""
    return GeoPoint(x=0.0, y=0.0)


@pytest.fixture
def drone_default() -> DroneSpec:
    """默认无人机: 50kg 载重, 5000Wh 电池, α=0.1, β=0.005"""
    return DroneSpec(
        payload_capacity=50.0,
        battery_capacity=5000.0,
        alpha=0.1,
        beta=0.005,
    )


@pytest.fixture
def drone_big() -> DroneSpec:
    """大型无人机: 100kg 载重, 20000Wh 电池"""
    return DroneSpec(
        payload_capacity=100.0,
        battery_capacity=20000.0,
        alpha=0.08,
        beta=0.003,
    )


@pytest.fixture
def drone_small() -> DroneSpec:
    """小型无人机: 5kg 载重, 1000Wh 电池 (电量紧张场景)"""
    return DroneSpec(
        payload_capacity=5.0,
        battery_capacity=1000.0,
        alpha=0.15,
        beta=0.01,
    )


@pytest.fixture
def drone_heavy_lift() -> DroneSpec:
    """重型无人机: 500kg 载重, 100000Wh 电池 (Solomon 标准实例用)"""
    return DroneSpec(
        payload_capacity=500.0,
        battery_capacity=100000.0,
        alpha=0.08,
        beta=0.002,
    )


# === Solomon 数据集 fixtures ===

@pytest.fixture
def solomon_r101_n20():
    """Solomon R101 前 20 点 (Random 分布)"""
    data = load_fixture_json("solomon_r101_n20.json")
    return targets_from_dict(data)


@pytest.fixture
def solomon_c101_n20():
    """Solomon C101 前 20 点 (Clustered 分布)"""
    data = load_fixture_json("solomon_c101_n20.json")
    return targets_from_dict(data)


@pytest.fixture
def solomon_rc101_n20():
    """Solomon RC101 前 20 点 (Mixed 分布)"""
    data = load_fixture_json("solomon_rc101_n20.json")
    return targets_from_dict(data)


# === 自建场景 fixtures ===

@pytest.fixture
def custom_5_heavy():
    """自建: 5 点, 重载场景"""
    data = load_fixture_json("custom_5_heavy.json")
    return targets_from_dict(data)


@pytest.fixture
def custom_10_tight():
    """自建: 10 点, 电量紧张场景"""
    data = load_fixture_json("custom_10_tight.json")
    return targets_from_dict(data)


@pytest.fixture
def custom_15_mixed():
    """自建: 15 点, 载重+电量联合约束"""
    data = load_fixture_json("custom_15_mixed.json")
    return targets_from_dict(data)
