"""电量绑定实施例数据 — 用于交底书 §8.4（补 D4：原 §8.3 实施例电池未绑定）

场景: tight 组（circle n=10, scale=3000, demand 1~3kg, 电池 3500Wh）
特点: 等效电量距离预算 = 3500/0.1 = 35000 m，实际用量达 ~96% → 电量约束绑定。
输出: constrained_example.json（坐标/需求/两序列/逐段载重电量/可行性对照）
复现: python -X utf8 docs/patent/scripts/gen_constrained_example.py
"""
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)

from a3_python.data_generator import generate_scenario  # noqa: E402
from a3_python.route import GeoPoint, DroneSpec  # noqa: E402
from a3_python.solver import plan_multistop  # noqa: E402
from a3_python.exact import (  # noqa: E402
    solve_tsp_exact_dp,
    solve_energy_exact_dp,
    evaluate_sequence_cost,
)
from a3_python.energy_model import simulate_route_energy  # noqa: E402

N, SCALE, SEED = 10, 3000.0, 0
OUT = os.path.join(os.path.dirname(__file__), "constrained_example.json")

# 与 benchmark.py::_build_tight_instances 一致
DRONE = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0, alpha=0.1, beta=0.005)


def dump(targets, home, seq):
    tmap = {t.id: t for t in targets}
    segs, geo, eq, en, rem, feas, warns = simulate_route_energy(seq, tmap, home, DRONE)
    return {
        "total_geo": round(geo, 1), "total_equiv": round(eq, 1),
        "total_energy_wh": round(en, 2), "remaining_wh": round(rem, 2),
        "budget_equiv": round(DRONE.battery_capacity / DRONE.alpha, 1),
        "equiv_usage_pct": round(eq / (DRONE.battery_capacity / DRONE.alpha) * 100, 1),
        "feasible": feas, "warnings": warns,
        "segments": [{
            "from": s.from_id, "to": s.to_id,
            "geo": round(s.geo_distance, 1), "equiv": round(s.equiv_distance, 1),
            "energy": round(s.energy_consumed, 2),
            "payload_before": round(s.payload_before, 2),
            "battery_before": round(s.battery_before, 1),
            "battery_after": round(s.battery_after, 1),
        } for s in segs],
    }


def main():
    home, targets = generate_scenario(n=N, distribution="circle", seed=SEED,
                                      scale=SCALE, demand_range=(1.0, 3.0))
    our = plan_multistop(targets, home, DRONE)
    geo = solve_tsp_exact_dp(targets, home)
    eng = solve_energy_exact_dp(targets, home, DRONE)

    # 几何最优环的两个方向能耗不同: 取能耗更低者作为公平基线
    geo_fwd = dump(targets, home, geo.sequence)
    geo_rev = dump(targets, home, list(reversed(geo.sequence)))
    geo_best, geo_best_dir = ((geo_fwd, "forward") if geo_fwd["total_equiv"] <= geo_rev["total_equiv"]
                              else (geo_rev, "reverse"))
    geo_eval = evaluate_sequence_cost(targets, home, DRONE, geo.sequence)

    data = {
        "n": N, "distribution": "circle", "scale": SCALE, "seed": SEED,
        "home": {"x": home.x, "y": home.y},
        "drone": {"alpha": DRONE.alpha, "beta": DRONE.beta,
                  "payload_capacity": DRONE.payload_capacity,
                  "battery_capacity": DRONE.battery_capacity},
        "total_demand": round(sum(t.demand for t in targets), 1),
        "budget_equiv_m": round(DRONE.battery_capacity / DRONE.alpha, 1),
        "targets": [{"id": t.id, "x": round(t.location.x, 1), "y": round(t.location.y, 1),
                     "demand": round(t.demand, 1)} for t in targets],
        "our_seq": list(our.sequence), "our": dump(targets, home, list(our.sequence)),
        "geo_opt_seq_fwd": geo.sequence,
        "geo_opt_fwd": geo_fwd,
        "geo_opt_seq_rev": list(reversed(geo.sequence)),
        "geo_opt_rev": geo_rev,
        # 公平基线 = 几何最优环两方向中能耗更低者
        "geo_opt_best_dir": geo_best_dir,
        "geo_opt_best": geo_best,
        "geo_opt_feasible_under_battery": geo_eval[1],
        "energy_opt_seq": eng.sequence,
        "energy_opt": dump(targets, home, eng.sequence),
    }
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    print(f"[OK] tight n={N} seed={SEED}")
    print(f"  预算(等效距离) = {data['budget_equiv_m']} m")
    print(f"  本方法: 等效={data['our']['total_equiv']} m "
          f"({data['our']['equiv_usage_pct']}% 预算) 可行={data['our']['feasible']}")
    print(f"  几何最优 正向: 等效={geo_fwd['total_equiv']} m 可行={geo_fwd['feasible']}")
    print(f"  几何最优 反向: 等效={geo_rev['total_equiv']} m 可行={geo_rev['feasible']}")
    print(f"  公平基线(较优方向={geo_best_dir}): {geo_best['total_equiv']} m")
    saved = (geo_best["total_equiv"] - data["our"]["total_equiv"]) / geo_best["total_equiv"] * 100
    print(f"  → 本方法相对公平基线省电 {saved:.2f}%")
    print(f"  能量精确最优: 等效={data['energy_opt']['total_equiv']} m 可行={data['energy_opt']['feasible']}")


if __name__ == "__main__":
    main()
