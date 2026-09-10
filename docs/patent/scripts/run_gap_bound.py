"""真实可计算的差距上界 — 用可采纳下界 LB(∅) 给出降级解的质量保证

背景: 交底书原主张"降级模式给出可量化的质量界", 但此前只有"实测统计"(非保证)。
本脚本用精确模式内部已使用的**可采纳下界** LB(∅) 直接算出**真实上界**:

    opt >= LB(空集)                       (可采纳性, 已由 exact_battery 证明并逐状态验证)
    => cost_降级 - opt <= cost_降级 - LB(空集)
    => 相对差距 = (cost_降级 - opt)/opt <= (cost_降级 - LB(空集))/LB(空集)

该上界**对任意实例成立**(非统计), 且计算代价为 O(n^2), 与启发式同量级。

输出: results/gap_bound.json + md, 含每实例的 上界/实测 gap/紧度
复现: python -X utf8 docs/patent/scripts/run_gap_bound.py
"""
from __future__ import annotations

import json
import os
import statistics
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)

from a3_python.route import DRONE_PRESETS, DroneSpec  # noqa: E402
from a3_python.solver import plan_multistop  # noqa: E402
from a3_python.energy_model import euclidean_distance  # noqa: E402
from a3_python.data_generator import generate_scenario  # noqa: E402
from a3_python.exact_battery import solve_energy_exact_battery  # noqa: E402

OUT_JSON = os.path.join(REPO, "results", "gap_bound.json")
OUT_MD = os.path.join(REPO, "results", "gap_bound.md")


def lb_empty(targets, home, drone) -> float:
    """可采纳下界 LB(空集): 全部目标点均未访问时的剩余能耗下界

    与 exact_battery._layer_lower_bound 同式:
      LB = Σ_{i∈S} (α + β·q_i)·minin(i) + α·min_{i∈S} d(i, home)
    S = 全部目标点; minin(i) = min_{i'≠i} d(i', i) (含已访问点与 home)
    """
    pts = [home] + [t.location for t in targets]
    n = len(targets)
    total = 0.0
    for k in range(1, n + 1):
        # 入边最短: 对全部点(含 home)取最小
        min_in = min(euclidean_distance(pts[m], pts[k])
                     for m in range(len(pts)) if m != k)
        total += (drone.alpha + drone.beta * targets[k - 1].demand) * min_in
    min_home = min(euclidean_distance(pts[k], home) for k in range(1, n + 1))
    total += drone.alpha * min_home
    return total


def build_instances(num_runs: int = 10):
    out = []
    for n in (5, 10, 15):
        for dist in ("circle", "random"):
            for seed in range(num_runs):
                home, tg = generate_scenario(n=n, distribution=dist, seed=seed,
                                             scale=1000.0, demand_range=(1.0, 3.0))
                out.append((f"{dist}_{n}p_s{seed}", tg, home,
                            DRONE_PRESETS["standard"]))
    tight = DroneSpec(payload_capacity=50.0, battery_capacity=3500.0,
                      alpha=0.1, beta=0.005)
    for seed in range(num_runs):
        home, tg = generate_scenario(n=10, distribution="circle", seed=seed,
                                     scale=3000.0, demand_range=(1.0, 3.0))
        out.append((f"tight_10p_s{seed}", tg, home, tight))
    return out


def main() -> None:
    rows = []
    for name, targets, home, drone in build_instances():
        plan = plan_multistop(targets, home, drone)
        exact = solve_energy_exact_battery(targets, home, drone)
        if not (plan.feasible and exact.feasible):
            continue
        lb = lb_empty(targets, home, drone)
        cost = plan.total_energy_consumed
        bound = (cost - lb) / lb if cost > lb else 0.0
        real = (cost - exact.objective) / exact.objective
        rows.append({
            "instance": name,
            "n": len(targets),
            "lb_empty_wh": round(lb, 4),
            "opt_wh": round(exact.objective, 4),
            "heur_wh": round(cost, 4),
            "bound_pct": round(bound * 100, 2),   # 真实上界
            "real_gap_pct": round(real * 100, 3),  # 实测差距
        })

    bounds = [r["bound_pct"] for r in rows]
    reals = [r["real_gap_pct"] for r in rows]
    valid = all(r["bound_pct"] >= r["real_gap_pct"] - 1e-6 for r in rows)
    summary = {
        "instances": len(rows),
        "bound_pct_mean": round(statistics.mean(bounds), 2),
        "bound_pct_max": round(max(bounds), 2),
        "real_gap_pct_mean": round(statistics.mean(reals), 3),
        "real_gap_pct_max": round(max(reals), 3),
        "bound_holds_for_all": valid,
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "rows": rows}, f, ensure_ascii=False, indent=2)

    lines = ["# 降级解的真实差距上界（用可采纳下界 LB(∅) 计算）\n",
             f"- 实例数：{summary['instances']}",
             f"- **上界均值 {summary['bound_pct_mean']}%，最大 {summary['bound_pct_max']}%**",
             f"- 实测差距均值 {summary['real_gap_pct_mean']}%，最大 {summary['real_gap_pct_max']}%",
             f"- 上界对全部实例成立（≥ 实测差距）：{summary['bound_holds_for_all']}",
             "", "| 实例 | n | 下界 LB(Wh) | 精确最优(Wh) | 降级解(Wh) | **上界%** | 实测gap% |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['instance']} | {r['n']} | {r['lb_empty_wh']} | {r['opt_wh']} | "
                     f"{r['heur_wh']} | **{r['bound_pct']}** | {r['real_gap_pct']} |")
    with open(OUT_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"[OK] -> {OUT_JSON}")


if __name__ == "__main__":
    main()
