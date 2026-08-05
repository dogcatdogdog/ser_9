"""共享工具函数 — 供 conftest + baseline + 测试文件使用

包含 fixture JSON 加载/解析, 避免从 conftest 导入 (conftest 是 pytest 特殊文件)。
"""

import json
import os
from a3_python.route import GeoPoint, Target

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")


def load_fixture_json(filename: str) -> dict:
    """加载 fixtures/ 目录下的 JSON 文件"""
    path = os.path.join(FIXTURES_DIR, filename)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def targets_from_dict(data: dict) -> tuple[GeoPoint, list[Target]]:
    """从 fixture dict 解析 home + targets 列表"""
    h = data["home"]
    home = GeoPoint(x=h["x"], y=h["y"])
    targets = [
        Target(
            id=t["id"],
            location=GeoPoint(x=t["x"], y=t["y"]),
            demand=t["demand"],
        )
        for t in data["targets"]
    ]
    return home, targets
