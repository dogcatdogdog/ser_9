"""test_data_generator.py — 数据生成器 单测

W2: 验证各分布类型的生成器输出正确的点数和结构。
"""

import pytest
from a3_python.route import GeoPoint, Target, DRONE_PRESETS, DroneSpec
from a3_python.data_generator import (
    generate_targets,
    generate_scenario,
    DEFAULT_DISTRIBUTIONS,
)


# ====================================================================
# 数据生成器: generate_targets
# ====================================================================

class TestGenerateTargets:
    """数据生成器: 各分布类型"""

    @pytest.mark.parametrize("dist", DEFAULT_DISTRIBUTIONS)
    def test_all_distributions(self, dist):
        """每种分布都能生成正确数量的点"""
        for n in [3, 5, 10]:
            targets = generate_targets(n, distribution=dist, seed=42)
            assert len(targets) == n
            # 所有 id 唯一
            ids = [t.id for t in targets]
            assert len(ids) == len(set(ids)), f"Duplicate ids in {dist}"
            # 所有点有坐标
            for t in targets:
                assert isinstance(t.location, GeoPoint)
                assert isinstance(t.location.x, float)
                assert isinstance(t.location.y, float)
            # 所有 demand > 0
            for t in targets:
                assert t.demand > 0

    def test_deterministic_seed(self):
        """相同 seed → 相同输出"""
        t1 = generate_targets(10, distribution="random", seed=42)
        t2 = generate_targets(10, distribution="random", seed=42)
        for a, b in zip(t1, t2):
            assert a.id == b.id
            assert a.location.x == b.location.x
            assert a.location.y == b.location.y
            assert a.demand == b.demand

    def test_different_seed_different_output(self):
        """不同 seed → 随机分布输出不同"""
        t1 = generate_targets(10, distribution="random", seed=42)
        t2 = generate_targets(10, distribution="random", seed=99)
        # 不同 seed 的随机分布应产生不同的点
        coords1 = [(t.location.x, t.location.y) for t in t1]
        coords2 = [(t.location.x, t.location.y) for t in t2]
        assert coords1 != coords2

    def test_circle_same_seed_same_circle(self):
        """圆形分布: 相同 seed → 相同 (圆形是确定性的, 不受 seed 影响)"""
        t1 = generate_targets(5, distribution="circle", seed=42)
        t2 = generate_targets(5, distribution="circle", seed=99)
        # 圆形分布不受 seed 影响 (纯几何)
        for a, b in zip(t1, t2):
            assert a.location.x == b.location.x
            assert a.location.y == b.location.y

    def test_invalid_distribution(self):
        """未知分布 → ValueError"""
        with pytest.raises(ValueError, match="Unknown distribution"):
            generate_targets(5, distribution="not_a_distribution")

    def test_demand_range(self):
        """需求在指定范围内"""
        targets = generate_targets(20, distribution="random", seed=42,
                                   demand_range=(5.0, 15.0))
        for t in targets:
            assert 5.0 <= t.demand <= 15.0

    def test_scale_affects_coordinates(self):
        """scale 参数影响坐标范围"""
        targets_small = generate_targets(5, distribution="random", seed=42, scale=100.0)
        targets_large = generate_targets(5, distribution="random", seed=42, scale=1000.0)
        # 大 scale 下的坐标范围更大
        for ts, tl in zip(targets_small, targets_large):
            assert abs(tl.location.x) >= abs(ts.location.x) * 0.9


# ====================================================================
# 数据生成器: generate_scenario
# ====================================================================

class TestGenerateScenario:
    """场景生成: home + targets"""

    def test_default_home(self):
        """默认 home 在原点"""
        home, targets = generate_scenario(5, distribution="circle")
        assert home.x == 0.0
        assert home.y == 0.0
        assert len(targets) == 5

    def test_custom_home(self):
        """自定义 home"""
        home, targets = generate_scenario(
            5, distribution="circle",
            home=GeoPoint(x=100.0, y=200.0),
        )
        assert home.x == 100.0
        assert home.y == 200.0


# ====================================================================
# 无人机预设配置表
# ====================================================================

class TestDronePresets:
    """DRONE_PRESETS: 3 种标准机型"""

    def test_three_presets_exist(self):
        """存在 3 种预设机型"""
        assert set(DRONE_PRESETS.keys()) == {"light", "standard", "heavy"}

    def test_all_presets_are_valid(self):
        """所有预设都是合法的 DroneSpec"""
        for name, drone in DRONE_PRESETS.items():
            assert isinstance(drone, DroneSpec), f"{name} is not DroneSpec"
            assert drone.payload_capacity > 0, f"{name}: payload_capacity <= 0"
            assert drone.battery_capacity > 0, f"{name}: battery_capacity <= 0"
            assert drone.alpha > 0, f"{name}: alpha <= 0"
            assert drone.beta > 0, f"{name}: beta <= 0"
            assert drone.cruise_speed > 0, f"{name}: cruise_speed <= 0"

    def test_light_smallest_payload(self):
        """light 机型载重最小"""
        light = DRONE_PRESETS["light"]
        standard = DRONE_PRESETS["standard"]
        heavy = DRONE_PRESETS["heavy"]
        assert light.payload_capacity < standard.payload_capacity < heavy.payload_capacity

    def test_heavy_most_efficient(self):
        """heavy 机型空载能耗率最低 (效率最高)"""
        light = DRONE_PRESETS["light"]
        standard = DRONE_PRESETS["standard"]
        heavy = DRONE_PRESETS["heavy"]
        assert heavy.alpha < standard.alpha < light.alpha

    def test_light_most_sensitive_to_load(self):
        """light 机型载重敏感系数最高"""
        light = DRONE_PRESETS["light"]
        heavy = DRONE_PRESETS["heavy"]
        assert light.beta > heavy.beta

    def test_presets_immutable_access(self):
        """预设可通过 key 访问并复制"""
        drone = DRONE_PRESETS["standard"]
        # 确认可以创建副本
        import copy
        drone_copy = copy.deepcopy(drone)
        assert drone_copy.payload_capacity == drone.payload_capacity
        assert drone_copy.alpha == drone.alpha
        assert drone_copy.beta == drone.beta
