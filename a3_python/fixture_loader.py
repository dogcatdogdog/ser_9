"""共享 fixture 加载工具 — 供 conftest + benchmark + baseline 使用

从 tests/utils.py 提取，解决"生产代码导入测试包"的架构问题。
"""

import json
import os
from .route import GeoPoint, Target

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "tests", "fixtures")


def load_fixture_json(filename: str) -> dict:
    """加载 fixtures/ 目录下的 JSON 文件"""
    path = os.path.join(FIXTURES_DIR, filename)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def targets_from_dict(data: dict) -> tuple[GeoPoint, list[Target]]:
    """从 fixture dict 解析 home + targets 列表

    注意: 支持两种 fixture 格式:
      - 嵌套格式 (A3_SCHEMA.md 标准): {"location": {"x":..., "y":...}}
      - 扁平格式 (fixture 简化):      {"x":..., "y":...}
    坐标和 demand 统一 cast 为 float (Solomon 数据用整数)。
    """
    h = data["home"]
    home = GeoPoint(x=float(h["x"]), y=float(h["y"]))
    targets = []
    for t in data["targets"]:
        # 支持嵌套 location 和扁平两种格式
        if "location" in t:
            loc = t["location"]
            x, y = float(loc["x"]), float(loc["y"])
        else:
            x, y = float(t["x"]), float(t["y"])
        targets.append(Target(
            id=t["id"],
            location=GeoPoint(x=x, y=y),
            demand=float(t["demand"]),
        ))
    return home, targets
