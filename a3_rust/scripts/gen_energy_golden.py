"""生成 Rust 交叉验证 golden — Python (numpy) vs Rust (std f64::sqrt) 等效距离矩阵

用法: & "D:\\ser_9\\env312\\python.exe" -X utf8 a3_rust/scripts/gen_energy_golden.py
输出: a3_rust/tests/fixtures/energy_golden.json

W6 强制对齐 (DEVPLAN): 同输入喂 Python 和 Rust, 等效距离矩阵误差 < 1e-6,
误差入单测断言 (tests/cross_check.rs)。
"""

import json
import os
import sys

# repo root (a3_rust/scripts/ → 上溯两级)
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO_ROOT)

import numpy as np

from a3_python.energy_model import compute_equiv_matrix
from a3_python.fixture_loader import FIXTURES_DIR, load_fixture_json, targets_from_dict

# 与 a3_python/tests/conftest.py 的 drone_default 保持一致
ALPHA = 0.1
BETA = 0.005

FIXTURES = [
    "custom_5_heavy.json",
    "custom_10_tight.json",
    "custom_15_mixed.json",
    "solomon_r101_n20.json",
    "solomon_c101_n20.json",
    "solomon_rc101_n20.json",
]


def compute_geo_matrix(locations: list) -> np.ndarray:
    """与 a3_python.energy_model.compute_geo_matrix 相同的向量化实现"""
    coords = np.array([[p.x, p.y] for p in locations], dtype=np.float64)
    diff = coords[:, np.newaxis, :] - coords[np.newaxis, :, :]
    return np.sqrt(np.sum(diff ** 2, axis=2))


def main() -> None:
    instances = []
    for fname in FIXTURES:
        data = load_fixture_json(fname)
        home, targets = targets_from_dict(data)
        locations = [home] + [t.location for t in targets]
        demands = [0.0] + [t.demand for t in targets]

        geo = compute_geo_matrix(locations)
        equiv = compute_equiv_matrix(geo, demands, ALPHA, BETA)

        instances.append({
            "name": fname.removesuffix(".json"),
            "n": len(locations),
            "locations": [[float(p.x), float(p.y)] for p in locations],
            "demands": demands,
            "geo_matrix": geo.reshape(-1).tolist(),
            "equiv_matrix": equiv.reshape(-1).tolist(),
        })

    out = {"alpha": ALPHA, "beta": BETA, "instances": instances}
    out_path = os.path.join(
        os.path.dirname(__file__), "..", "tests", "fixtures", "energy_golden.json"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)

    total = sum(len(i["geo_matrix"]) for i in instances)
    print(f"wrote {out_path} ({len(instances)} instances, {total} matrix elements)")


if __name__ == "__main__":
    main()
