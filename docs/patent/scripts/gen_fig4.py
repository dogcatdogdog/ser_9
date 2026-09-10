"""fig4 实例数据生成 — "访问顺序影响能耗" 演示实例

目标: 找一个 n=8 实例, 满足:
  - 几何精确最优(闭环 TSP)路线在真实电量模型下可行;
  - 我们的能量感知解 (plan_multistop) 可行;
  - 两条访问顺序不同;
  - 能量感知解总等效电量距离比几何最优路线省电 3%~25% (图上有可读差异)。

输出: 写入 fig4_data.json (坐标/需求/两序列/各段能耗/对比指标), 供 fig4.tex 与
交底书 §八 走通实例使用。复现: python -X utf8 docs/patent/scripts/gen_fig4.py
"""
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)

from a3_python.data_generator import generate_scenario  # noqa: E402
from a3_python.route import DRONE_PRESETS  # noqa: E402
from a3_python.solver import plan_multistop  # noqa: E402
from a3_python.exact import (  # noqa: E402
    solve_tsp_exact_dp,
    solve_energy_exact_dp,
    evaluate_sequence_cost,
)
from a3_python.energy_model import simulate_route_energy  # noqa: E402

N = 8
SCALE = 800.0
OUT = os.path.join(os.path.dirname(__file__), "fig4_data.json")


def dump_segments(targets, home, drone, seq):
    tmap = {t.id: t for t in targets}
    segs, _geo, total_equiv, total_energy, _rem, feasible, warns = (
        simulate_route_energy(seq, tmap, home, drone)
    )
    rows = []
    for s in segs:
        rows.append({
            "from": s.from_id, "to": s.to_id,
            "geo": round(s.geo_distance, 1),
            "equiv": round(s.equiv_distance, 1),
            "energy": round(s.energy_consumed, 2),
            "payload_before": round(s.payload_before, 2),
            "payload_after": round(s.payload_after, 2),
        })
    return {
        "total_equiv": round(total_equiv, 1),
        "total_energy_wh": round(total_energy, 2),
        "feasible": feasible,
        "warnings": warns,
        "segments": rows,
    }


def main():
    drone = DRONE_PRESETS["standard"]
    for seed in range(120):
        home, targets = generate_scenario(
            n=N, distribution="random", seed=seed, scale=SCALE,
        )
        total_demand = sum(t.demand for t in targets)
        if total_demand > drone.payload_capacity:
            continue  # 载重超限, 跳过

        geo = solve_tsp_exact_dp(targets, home)
        our = plan_multistop(targets, home, drone)
        if not our.feasible:
            continue

        equiv_geo, feas_geo, _ = evaluate_sequence_cost(
            targets, home, drone, geo.sequence
        )
        if not feas_geo:
            continue  # 需要两条都可行, 对比"更省电"而非"谁不可行"
        if geo.sequence == list(our.sequence):
            continue

        # 口径与主表 Gap_eng% 一致: 以基线(几何最优序列的能量成本)为分母
        saved = (equiv_geo - our.total_equiv_distance) / equiv_geo
        if 0.03 <= saved <= 0.30:
            eng = solve_energy_exact_dp(targets, home, drone)  # 能量精确最优(参考)
            data = {
                "n": N,
                "distribution": "random",
                "seed": seed,
                "scale": SCALE,
                "home": {"x": home.x, "y": home.y},
                "drone": {
                    "alpha": drone.alpha, "beta": drone.beta,
                    "payload_capacity": drone.payload_capacity,
                    "battery_capacity": drone.battery_capacity,
                },
                "total_demand": round(total_demand, 1),
                "targets": [
                    {"id": t.id, "x": round(t.location.x, 1), "y": round(t.location.y, 1),
                     "demand": round(t.demand, 1)}
                    for t in targets
                ],
                "geo_opt_seq": geo.sequence,
                "geo_opt": dump_segments(targets, home, drone, geo.sequence),
                "geo_geo_dist": round(geo.objective, 1),
                "our_seq": list(our.sequence),
                "our": dump_segments(targets, home, drone, list(our.sequence)),
                "our_geo_dist": round(our.total_geo_distance, 1),
                "energy_dp_seq": eng.sequence,
                "energy_dp": {
                    "total_equiv": round(eng.objective, 1),
                    "feasible": eng.energy_feasible,
                },
                "saved_pct": round(saved * 100.0, 1),
            }
            with open(OUT, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            print(f"[OK] seed={seed}  saved={saved*100:.1f}%")
            print("geo_opt:", geo.sequence)
            print("our    :", list(our.sequence))
            print("energyDP:", eng.sequence)
            print("total_equiv geo/our:", equiv_geo, our.total_equiv_distance)
            return

    print("[FAIL] no suitable instance found in seed range")


if __name__ == "__main__":
    main()
