"""生成 solver golden — Python `plan_multistop` 固定种子输出 (W7 HTTP golden 测试)

用法: & "D:\\ser_9\\env312\\python.exe" -X utf8 a3_rust/scripts/gen_solver_golden.py
输出: a3_rust/tests/fixtures/solver_golden.json

W7 强制对齐 (DEVPLAN): Python 固定种子输出 JSON → Rust 服务响应逐字段比对。
"""

import json
import os
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO_ROOT)

from dataclasses import asdict

from a3_python.fixture_loader import load_fixture_json, targets_from_dict
from a3_python.route import DroneSpec
from a3_python.solver import plan_multistop

# 与测试约定一致 (Python test_heuristic 同款)
DRONE = DroneSpec(payload_capacity=500.0, battery_capacity=100000.0,
                  alpha=0.08, beta=0.002)

FIXTURES = [
    "custom_5_heavy.json",
    "custom_10_tight.json",
    "custom_15_mixed.json",
    "solomon_r101_n20.json",
    "solomon_c101_n20.json",
    "solomon_rc101_n20.json",
]


def main() -> None:
    instances = []
    for fname in FIXTURES:
        home, targets = targets_from_dict(load_fixture_json(fname))
        plan = plan_multistop(targets, home, DRONE, seed=42)
        instances.append({
            "fixture": fname,
            "home": {"x": home.x, "y": home.y},
            "drone": {"payload_capacity": DRONE.payload_capacity,
                      "battery_capacity": DRONE.battery_capacity,
                      "alpha": DRONE.alpha, "beta": DRONE.beta},
            # 嵌套格式: 对齐 A3_SCHEMA.md §3.1 HTTP 请求契约
            "targets": [{"id": t.id,
                         "location": {"x": t.location.x, "y": t.location.y},
                         "demand": t.demand} for t in targets],
            "expected": {
                "sequence": plan.sequence,
                "segments": [asdict(s) for s in plan.segments],
                "total_geo_distance": plan.total_geo_distance,
                "total_equiv_distance": plan.total_equiv_distance,
                "total_energy_consumed": plan.total_energy_consumed,
                "remaining_energy": plan.remaining_energy,
                "total_payload_delivered": plan.total_payload_delivered,
                "feasible": plan.feasible,
                "warnings": plan.warnings,
            },
        })

    out_path = os.path.join(os.path.dirname(__file__), "..", "tests", "fixtures",
                            "solver_golden.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"instances": instances}, f, ensure_ascii=False, indent=1)
    print(f"wrote {out_path} ({len(instances)} instances)")


if __name__ == "__main__":
    main()
