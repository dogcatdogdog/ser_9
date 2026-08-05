"""数据生成器 — 自建测试场景 + 导出 fixture JSON

用法:
    from a3_python.data_generator import generate_targets, save_fixture

    targets = generate_targets(n=10, distribution="circle")
    save_fixture("my_10_circle.json", home, targets, "10 点圆形分布")

支持分布:
  - circle:  圆形均匀分布 (标准测试)
  - random:  随机均匀分布
  - grid:    网格分布
  - line:    直线分布 (退化场景)
  - cluster: 聚类分布 (模拟 Solomon C 类)

W2 D5-D7: 自建 5/10/20 点测试场景, 配合无人机参数配置表使用。
"""

import json
import math
import os
from .route import GeoPoint, Target

# === 模块级常量 ===

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "tests", "fixtures")

DEFAULT_DISTRIBUTIONS = ["circle", "random", "grid", "line", "cluster"]


# === 核心生成函数 ===

def generate_targets(
    n: int,
    distribution: str = "circle",
    seed: int = 42,
    scale: float = 1000.0,
    demand_range: tuple[float, float] = (1.0, 10.0),
) -> list[Target]:
    """生成 n 个目标点

    Args:
        n: 目标点数
        distribution: 分布类型
            - 'circle':  圆形均匀分布
            - 'random':  随机均匀分布在 [-scale, scale]²
            - 'grid':    近似正方形网格
            - 'line':    直线分布 (x 轴沿线)
            - 'cluster': 3 个聚类中心, 各含 n//3 个点
        seed: 随机种子
        scale: 分布范围 (米)
        demand_range: 需求范围 (min, max) kg

    Returns:
        list[Target]: 目标点列表
    """
    import numpy as np
    rng = np.random.default_rng(seed)

    targets = []
    for i in range(n):
        if distribution == "circle":
            angle = 2 * math.pi * i / n
            x = scale * math.cos(angle)
            y = scale * math.sin(angle)
        elif distribution == "random":
            x = rng.uniform(-scale, scale)
            y = rng.uniform(-scale, scale)
        elif distribution == "grid":
            side = max(1, math.ceil(math.sqrt(n)))
            row = i // side
            col = i % side
            spacing = 2 * scale / side
            x = col * spacing - scale
            y = row * spacing - scale
        elif distribution == "line":
            # 沿 x 轴分布, 加少量 y 噪声
            x = -scale + 2 * scale * i / max(1, n - 1)
            y = rng.uniform(-scale * 0.05, scale * 0.05)
        elif distribution == "cluster":
            # 3 个聚类中心
            centers = [
                (scale * 0.7, scale * 0.7),
                (-scale * 0.7, -scale * 0.7),
                (scale * 0.7, -scale * 0.7),
            ]
            c_idx = i % 3
            cx, cy = centers[c_idx]
            x = cx + rng.normal(0, scale * 0.1)
            y = cy + rng.normal(0, scale * 0.1)
        else:
            raise ValueError(f"Unknown distribution: {distribution}")

        demand = round(rng.uniform(*demand_range), 1)

        targets.append(Target(
            id=f"t{i+1}",
            location=GeoPoint(x=round(x, 1), y=round(y, 1)),
            demand=demand,
        ))
    return targets


def generate_scenario(
    n: int,
    distribution: str = "circle",
    seed: int = 42,
    scale: float = 1000.0,
    demand_range: tuple[float, float] = (1.0, 10.0),
    home: GeoPoint | None = None,
) -> tuple[GeoPoint, list[Target]]:
    """生成一个完整场景 (home + targets)

    Args:
        n: 目标点数
        distribution: 分布类型
        seed: 随机种子
        scale: 分布范围 (米)
        demand_range: 需求范围 (min, max) kg
        home: 仓库位置, 默认原点

    Returns:
        (home, targets) 元组
    """
    if home is None:
        home = GeoPoint(x=0.0, y=0.0)
    targets = generate_targets(n, distribution, seed, scale, demand_range)
    return home, targets


# === Fixture 导出 ===

def save_fixture(
    filename: str,
    home: GeoPoint,
    targets: list[Target],
    description: str = "",
) -> str:
    """将场景导出为 fixture JSON 文件

    Args:
        filename: 文件名 (如 "custom_20_grid.json")
        home: 仓库位置
        targets: 目标点列表
        description: 场景描述

    Returns:
        str: 写入的文件路径
    """
    data = {
        "description": description,
        "home": {"x": home.x, "y": home.y},
        "targets": [
            {"id": t.id, "x": t.location.x, "y": t.location.y, "demand": t.demand}
            for t in targets
        ],
    }
    path = os.path.join(FIXTURES_DIR, filename)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return path


def generate_all_fixtures() -> list[str]:
    """生成 W2 标准测试集: 5/10/20 点 × 3 种分布 + 退化场景

    Returns:
        list[str]: 生成的文件路径列表
    """
    paths: list[str] = []
    home = GeoPoint(x=0.0, y=0.0)

    scenarios = [
        (5, "circle", "5 点圆形分布 (标准)"),
        (10, "circle", "10 点圆形分布 (标准)"),
        (20, "circle", "20 点圆形分布 (标准)"),
        (20, "random", "20 点随机分布"),
        (10, "cluster", "10 点聚类分布 (类似 Solomon C)"),
        (5, "line", "5 点直线分布 (退化)"),
    ]

    for n, dist, desc in scenarios:
        targets = generate_targets(n, distribution=dist, seed=42)
        filename = f"generated_{n}p_{dist}.json"
        path = save_fixture(filename, home, targets, desc)
        paths.append(path)

    return paths


if __name__ == "__main__":
    print("=== 生成 W2 标准测试集 ===\n")
    paths = generate_all_fixtures()
    for p in paths:
        print(f"  ✓ {p}")
    print(f"\n[DONE] {len(paths)} fixtures generated.")
