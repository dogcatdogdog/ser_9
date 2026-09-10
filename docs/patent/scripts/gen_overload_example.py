"""3.3 节载重超限实例（63.8 kg）的可复现锚点

作用: 交底书 §3.3 其一引用的"默认需求量 10 点实例总需求 63.8 kg > 50 kg 载重上限"
此前仅见于 A3_RESEARCH_PLAN.md 调研笔记，无可复现入口。本脚本固化该实例并输出
总需求、几何最优序列及其载重可行性，便于核对。

复现: python -X utf8 docs/patent/scripts/gen_overload_example.py
"""
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)

from a3_python.data_generator import generate_targets  # noqa: E402
from a3_python.route import GeoPoint, DRONE_PRESETS  # noqa: E402
from a3_python.exact import solve_tsp_exact_dp, evaluate_sequence_cost  # noqa: E402

N, SCALE, SEED = 10, 1000.0, 42
OUT = os.path.join(os.path.dirname(__file__), "overload_example.json")


def main():
    home = GeoPoint(x=0.0, y=0.0)
    drone = DRONE_PRESETS["standard"]
    targets = generate_targets(n=N, distribution="circle", seed=SEED,
                               scale=SCALE, demand_range=(1.0, 10.0))
    total_demand = sum(t.demand for t in targets)
    geo = solve_tsp_exact_dp(targets, home)
    equiv, feasible, warnings = evaluate_sequence_cost(targets, home, drone, geo.sequence)

    data = {
        "n": N, "distribution": "circle", "scale": SCALE, "seed": SEED,
        "demand_range": [1.0, 10.0],
        "home": {"x": home.x, "y": home.y},
        "drone": {"payload_capacity": drone.payload_capacity,
                  "battery_capacity": drone.battery_capacity,
                  "alpha": drone.alpha, "beta": drone.beta},
        "total_demand": round(total_demand, 1),
        "capacity": drone.payload_capacity,
        "overload": total_demand > drone.payload_capacity,
        "geo_opt_seq": geo.sequence,
        "geo_opt_total_geo": round(geo.objective, 1),
        "geo_opt_feasible_under_constraints": feasible,
        "geo_opt_warnings": warnings,
        "targets": [{"id": t.id, "x": round(t.location.x, 1), "y": round(t.location.y, 1),
                     "demand": round(t.demand, 1)} for t in targets],
    }
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    print(f"[OK] circle n={N} scale={SCALE} seed={SEED} demand∈[1,10]")
    print(f"  总需求 = {data['total_demand']} kg  (载重上限 {data['capacity']} kg)")
    print(f"  超限 = {data['overload']}")
    print(f"  几何最优序列可行性 = {feasible}; warnings = {warnings}")


if __name__ == "__main__":
    main()
