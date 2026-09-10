"""基准重算: 我们的解 vs **带电池的真实最优**

背景 (见 docs/exploration/WMLP_INSIGHT.md):
  MVP 的 benchmark 以"几何最优序列在能量模型下的能耗"作基线 (gap_eng),
  但那**不是**能量目标下的最优解 —— 因此"省 X%"衡量的是与一个错误基线的差距。
  本脚本用 `solve_energy_exact_battery` 得到真实最优, 重算三个量:
    1. gap_true   = (我们的能耗 - 真实最优) / 真实最优     —— 质量上界
    2. gap_vs_geo = (我们的能耗 - 几何最优序列能耗) / 该值  —— MVP 口径(作对照)
    3. 真实最优相对几何基线的改善 = (几何基线 - 真实最优) / 真实最优

输出: results/gap_rebaseline.json + markdown 表
复现: python -X utf8 a3_python/tw/run_gap_rebaseline.py
"""
from __future__ import annotations

import json
import os
import statistics
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)

from a3_python.route import DRONE_PRESETS, DroneSpec, GeoPoint  # noqa: E402
from a3_python.solver import plan_multistop  # noqa: E402
from a3_python.exact import solve_tsp_exact_dp, evaluate_sequence_cost  # noqa: E402
from a3_python.data_generator import generate_scenario  # noqa: E402
from a3_python.exact_battery import solve_energy_exact_battery  # noqa: E402

OUT_JSON = os.path.join(REPO, "results", "gap_rebaseline.json")
OUT_MD = os.path.join(REPO, "results", "gap_rebaseline.md")

# 与 benchmark.py 一致的实例配置
SELF_DISTRIBUTIONS = ["circle", "random"]
POINT_COUNTS = [5, 10, 15]


def build_instances(num_runs: int = 10):
    """三组实例: 自建(circle/random) + 电量紧张; 全部 n<=15 以便精确求解"""
    out = []
    for n in POINT_COUNTS:
        for dist in SELF_DISTRIBUTIONS:
            for seed in range(num_runs):
                home, tg = generate_scenario(n=n, distribution=dist, seed=seed,
                                             scale=1000.0, demand_range=(1.0, 3.0))
                out.append((f"{dist}_{n}p_s{seed}", tg, home,
                            DRONE_PRESETS["standard"]))
    # 电量紧张组 (n=10)
    tight = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0,
                      alpha=0.1, beta=0.005)
    for seed in range(num_runs):
        home, tg = generate_scenario(n=10, distribution="circle", seed=seed,
                                     scale=3000.0, demand_range=(1.0, 3.0))
        out.append((f"tight_10p_s{seed}", tg, home, tight))
    return out


def main() -> None:
    rows = []
    t_exact = 0.0
    for name, targets, home, drone in build_instances():
        plan = plan_multistop(targets, home, drone)
        t0 = time.perf_counter()
        exact = solve_energy_exact_battery(targets, home, drone)
        t_exact += time.perf_counter() - t0

        geo = solve_tsp_exact_dp(targets, home)
        geo_equiv, _, _ = evaluate_sequence_cost(targets, home, drone, geo.sequence)

        row = {
            "instance": name,
            "n": len(targets),
            "our_feasible": plan.feasible,
            "exact_feasible": exact.feasible,
            "our_energy": round(plan.total_energy_consumed, 4),
            "exact_energy": round(exact.objective, 4) if exact.feasible else None,
            "geo_baseline_energy": round(geo_equiv * drone.alpha, 4),
            "same_sequence": plan.sequence == exact.sequence if exact.feasible else None,
        }
        if exact.feasible and plan.feasible:
            row["gap_true_pct"] = round(
                (plan.total_energy_consumed - exact.objective) / exact.objective * 100, 3)
            row["gap_vs_geo_pct"] = round(
                (plan.total_energy_consumed - geo_equiv * drone.alpha)
                / (geo_equiv * drone.alpha) * 100, 3)
            row["opt_improve_vs_geo_pct"] = round(
                (geo_equiv * drone.alpha - exact.objective) / exact.objective * 100, 3)
        else:
            row["gap_true_pct"] = None
            row["gap_vs_geo_pct"] = None
            row["opt_improve_vs_geo_pct"] = None
        rows.append(row)

    feasible_rows = [r for r in rows if r["gap_true_pct"] is not None]

    def stats(key):
        vals = [r[key] for r in feasible_rows]
        return (round(statistics.mean(vals), 3),
                round(max(vals), 3), round(min(vals), 3)) if vals else (None, None, None)

    summary = {
        "total_instances": len(rows),
        "both_feasible": len(feasible_rows),
        "our_infeasible": sum(1 for r in rows if not r["our_feasible"]),
        "exact_infeasible": sum(1 for r in rows if not r["exact_feasible"]),
        "sequence_identical": sum(1 for r in feasible_rows if r["same_sequence"]),
        "gap_true_pct_mean_max_min": stats("gap_true_pct"),
        "gap_vs_geo_pct_mean_max_min": stats("gap_vs_geo_pct"),
        "opt_improve_vs_geo_pct_mean_max_min": stats("opt_improve_vs_geo_pct"),
        "exact_total_time_s": round(t_exact, 2),
    }

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "rows": rows}, f, ensure_ascii=False, indent=2)

    lines = ["# 基准重算: 我们的解 vs 带电池的真实最优\n",
             f"- 实例总数: {summary['total_instances']} (两者均可行 {summary['both_feasible']})",
             f"- 本方法不可行: {summary['our_infeasible']}  精确解不可行: {summary['exact_infeasible']}",
             f"- 序列与精确最优完全相同: {summary['sequence_identical']}/{summary['both_feasible']}",
             f"- **gap_true** (我们 vs 真实最优): 均值 {summary['gap_true_pct_mean_max_min'][0]}%"
             f" 最大 {summary['gap_true_pct_mean_max_min'][1]}%",
             f"- gap_vs_geo (MVP 口径): 均值 {summary['gap_vs_geo_pct_mean_max_min'][0]}%",
             f"- 真实最优相对几何基线的改善: 均值 {summary['opt_improve_vs_geo_pct_mean_max_min'][0]}%",
             "", "| instance | n | our | exact | gap_true% | gap_vs_geo% | 同序 |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(
            f"| {r['instance']} | {r['n']} | {r['our_energy']} | {r['exact_energy']} | "
            f"{r['gap_true_pct']} | {r['gap_vs_geo_pct']} | {r['same_sequence']} |")
    with open(OUT_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"\n[OK] -> {OUT_JSON}\n[OK] -> {OUT_MD}")


if __name__ == "__main__":
    main()
