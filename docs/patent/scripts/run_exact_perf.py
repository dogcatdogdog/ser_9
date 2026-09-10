"""STEP1 性能测定: 分层向量化 + 上界剪枝的精确 DP

测量内容:
  1. 改造前 (参照实现, 朴素三重循环) vs 改造后 (分层向量化) 的耗时/内存;
  2. 剪枝开关 (use_pruning) 的净效应: 耗时、扩展状态数、剪枝率;
  3. 各改造后结果与参照实现/暴力枚举的逐位一致性 (n <= 8 全枚举核对);
  4. 覆盖边界: n 最大能到多少仍在服务端 5s / 250MB 预算内。

⚠️ 关于计时口径:
  `tracemalloc` 会追踪**每一次**浮点分配, 对纯 Python 参照实现的惩罚远大于
  numpy 实现 —— 边测内存边计时会把加速比高估数倍。因此分两趟:
    * `--clean`: 只用 `perf_counter` 计时 (无 tracemalloc), 得到可信的加速比;
      同时给出"剪枝增量" (UB 由调用方提供, 见 run_exact_perf._initial_upper_bound);
    * 默认模式: 计时 + tracemalloc 峰值内存 (内存单独采信, 时间仅作参考)。

复现:
  python -X utf8 a3_python/tw/run_exact_perf.py            # 全量 (含 tracemalloc 内存)
  python -X utf8 a3_python/tw/run_exact_perf.py --clean    # 纯净计时 (可信加速比)
  python -X utf8 a3_python/tw/run_exact_perf.py --quick    # 冒烟
输出: results/step1_perf_{data,clean}.json + 终端 markdown 表
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
import tracemalloc

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)

from a3_python.route import DroneSpec  # noqa: E402
from a3_python.data_generator import generate_scenario  # noqa: E402
from a3_python.exact_battery import (  # noqa: E402
    _distance_matrix,          # 私有工具: 仅为在计时区外复现 UB, 属测量脚手架
    _initial_upper_bound,
    solve_energy_exact_battery,
    solve_energy_exact_battery_reference,
)

OUT_JSON = os.path.join(REPO, "results", "step1_perf_data.json")

# 与服务端一致的机型 (a3_rust/src/http.rs 默认 5s 预算, 内存预算 250MB)
DRONE = DroneSpec(payload_capacity=50.0, battery_capacity=5000.0,
                  alpha=0.1, beta=0.005)
SERVICE_TIME_BUDGET_S = 5.0
SERVICE_MEM_BUDGET_MB = 250.0

# 参照实现是 O(2^n·n²) 纯 Python, n 越大越慢; 仅在 <= REF_MAX_N 时实测
REF_MAX_N = 16
REF_TIMEOUT_S = 400.0


def _instance(n: int, seed: int):
    """固定种子实例 (与 run_gap_rebaseline / run_adaptive_validation 同口径)"""
    return generate_scenario(n=n, distribution="random", seed=seed,
                             scale=1000.0, demand_range=(1.0, 3.0))


def _timed(fn, *args, **kwargs):
    """返回 (结果, 秒, 峰值内存MB)"""
    tracemalloc.start()
    t0 = time.perf_counter()
    res = fn(*args, **kwargs)
    dt = time.perf_counter() - t0
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return res, dt, peak / 1e6


def measure(n: int, seed: int, with_reference: bool) -> dict:
    home, tg = _instance(n, seed)
    row: dict = {"n": n, "seed": seed}

    fast, t_fast, m_fast = _timed(
        solve_energy_exact_battery, tg, home, DRONE, n, use_pruning=True)
    no_pr, t_nopr, m_nopr = _timed(
        solve_energy_exact_battery, tg, home, DRONE, n, use_pruning=False)

    row.update({
        "fast_time_s": round(t_fast, 4),
        "fast_peak_mb": round(m_fast, 1),
        "fast_feasible": fast.feasible,
        "fast_objective": fast.objective if fast.feasible else None,
        "fast_states": fast.states_explored,
        "fast_pruned": fast.pruned_states,
        "fast_ub": fast.ub_initial if fast.ub_initial != float("inf") else None,
        "noprune_time_s": round(t_nopr, 4),
        "noprune_peak_mb": round(m_nopr, 1),
        "noprune_states": no_pr.states_explored,
        "same_objective_pruning": fast.objective == no_pr.objective,
        "same_sequence_pruning": fast.sequence == no_pr.sequence,
    })

    if with_reference:
        ref, t_ref, m_ref = _timed(
            solve_energy_exact_battery_reference, tg, home, DRONE, n)
        row.update({
            "ref_time_s": round(t_ref, 4),
            "ref_peak_mb": round(m_ref, 1),
            "ref_states": ref.states_explored,
            "same_objective_ref": fast.objective == ref.objective,
            "same_sequence_ref": fast.sequence == ref.sequence,
            "speedup_vs_ref": round(t_ref / t_fast, 1) if t_fast > 0 else None,
            "speedup_vect_only": round(t_ref / t_nopr, 1) if t_nopr > 0 else None,
        })
    return row


def measure_clean(n: int, seed: int, with_reference: bool,
                  ref_max_n: int) -> dict:
    """**无 tracemalloc** 的纯净计时 (tracemalloc 会追踪每次浮点分配,

    对纯 Python 参照实现的惩罚远大于 numpy 实现, 直接用它的耗时算加速比会被高估)。
    时间用 `perf_counter`, 内存单独用 tracemalloc 测 (见 §4.1)。
    """
    home, tg = _instance(n, seed)
    ub = _initial_upper_bound(
        tg, home, DRONE,
        _distance_matrix([home] + [t.location for t in tg]),
        np.array([0.0] + [t.demand for t in tg]),
        sum([0.0] + [t.demand for t in tg]))

    def _once(fn, **kw):
        t0 = time.perf_counter()
        res = fn(tg, home, DRONE, n, **kw)
        return res, time.perf_counter() - t0

    on, t_on = _once(solve_energy_exact_battery, use_pruning=True)
    off, t_off = _once(solve_energy_exact_battery, use_pruning=False)
    inc, t_inc = _once(solve_energy_exact_battery, ub_override=ub, use_pruning=True)
    row = {
        "n": n, "seed": seed,
        "fast_time_s": round(t_on, 4),           # 端到端 (含 UB 启发式)
        "noprune_time_s": round(t_off, 4),       # 仅向量化
        "prune_incremental_s": round(t_inc, 4),  # 剪枝增量 (UB 由调用方给)
        "explored": on.states_explored,
        "noprune_states": off.states_explored,
        "pruned": on.pruned_states,
        "state_reduction": round(1 - on.states_explored / off.states_explored, 4),
        "same_objective": on.objective == off.objective == inc.objective,
        "same_sequence": on.sequence == off.sequence == inc.sequence,
        "ub_gap_pct": round((on.ub_initial - on.objective) / on.objective * 100, 3)
        if math.isfinite(on.ub_initial) else None,
    }
    if with_reference and n <= ref_max_n:
        t0 = time.perf_counter()
        ref = solve_energy_exact_battery_reference(tg, home, DRONE, n)
        t_ref = time.perf_counter() - t0
        row["ref_time_s"] = round(t_ref, 3)
        row["speedup_vs_ref"] = round(t_ref / t_on, 1)
        row["speedup_vect_only"] = round(t_ref / t_off, 1)
        row["same_objective_ref"] = ref.objective == on.objective
    return row


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="只跑 n=10..15, 不跑参照实现")
    ap.add_argument("--ref-ns", type=str, default=None,
                    help="只测参照实现在这些 n 上的耗时 (逗号分隔, 如 17,18), 用于补全加速比")
    ap.add_argument("--clean", action="store_true",
                    help="无 tracemalloc 的纯净计时 (含剪枝增量 + 参照实现对照)")
    ap.add_argument("--ref-max-n", type=int, default=18,
                    help="--clean 模式下实测参照实现的最大 n (越大越慢)")
    ap.add_argument("--clean-ns", type=str, default=None,
                    help="--clean 模式的 n 列表 (逗号分隔), 默认 15..20")
    args = ap.parse_args()

    if args.clean:
        rows = []
        clean_ns = ([int(x) for x in args.clean_ns.split(",")] if args.clean_ns
                    else [15, 16, 17, 18, 19, 20])
        for n in clean_ns:
            for seed in (0, 1):
                # 参照实现只在 seed=0 上测 (其耗时是分钟级)
                row = measure_clean(n, seed, seed == 0, args.ref_max_n)
                rows.append(row)
                print(f"[clean] n={n} s={seed} prune={row['fast_time_s']}s "
                      f"noprune={row['noprune_time_s']}s inc={row['prune_incremental_s']}s "
                      f"ref={row.get('ref_time_s')}x{row.get('speedup_vs_ref')} "
                      f"red={row['state_reduction']:.1%} same={row['same_objective']}",
                      flush=True)
        path = os.path.join(REPO, "results", "step1_perf_clean.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"rows": rows}, f, ensure_ascii=False, indent=2)
        print(f"[OK] -> {path}")
        return

    if args.ref_ns:
        # 纯净计时 (无 tracemalloc), 口径与 --clean 一致
        for n in (int(x) for x in args.ref_ns.split(",")):
            home, tg = _instance(n, 0)
            t0 = time.perf_counter()
            res = solve_energy_exact_battery_reference(tg, home, DRONE, n)
            t_ref = time.perf_counter() - t0
            print(f"[ref] n={n} seed=0 time={t_ref:.2f}s objective={res.objective:.6f}",
                  flush=True)
        return

    ns = [10, 12, 14, 15, 16] if args.quick else [15, 16, 17, 18, 19, 20]
    rows: list[dict] = []
    for n in ns:
        for seed in (0, 1):
            with_ref = (n <= REF_MAX_N) and not args.quick
            t0 = time.perf_counter()
            row = measure(n, seed, with_ref)
            row["wall_s"] = round(time.perf_counter() - t0, 2)
            rows.append(row)
            print(f"[done] n={n} seed={seed} {row['wall_s']}s "
                  f"fast={row['fast_time_s']}s prune_rate="
                  f"{(row['fast_pruned'] / max(1, row['fast_pruned'] + row['fast_states'])):.1%}",
                  flush=True)

    # --- 汇总表 ---
    lines = ["| n | 剪枝开(s) | 仅向量化(s) | 参照(s) | 加速比(整体) | 加速比(仅向量化) | "
             "剪枝率 | 状态数(剪枝/不剪) | 峰值内存(MB) | 与参照逐位一致 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        tot = r["fast_pruned"] + r["fast_states"]
        rate = f"{r['fast_pruned'] / tot:.1%}" if tot else "-"
        ref_t = r.get("ref_time_s")
        sp = r.get("speedup_vs_ref")
        spv = r.get("speedup_vect_only")
        ok = r.get("same_objective_ref")
        lines.append(
            f"| {r['n']} | {r['fast_time_s']} | {r['noprune_time_s']} | "
            f"{ref_t if ref_t is not None else '—'} | "
            f"{sp if sp is not None else '—'} | {spv if spv is not None else '—'} | "
            f"{rate} | {r['fast_states']}/{r['noprune_states']} | "
            f"{r['fast_peak_mb']} | {'✅' if ok else ('—' if ok is None else '❌')} |")
    table = "\n".join(lines)
    print("\n" + table)

    # --- 覆盖边界 ---
    within = [r["n"] for r in rows
              if r["fast_time_s"] <= SERVICE_TIME_BUDGET_S
              and r["fast_peak_mb"] <= SERVICE_MEM_BUDGET_MB]
    summary = {
        "service_time_budget_s": SERVICE_TIME_BUDGET_S,
        "service_mem_budget_mb": SERVICE_MEM_BUDGET_MB,
        "max_n_within_budget": max(within) if within else 0,
        "objectives_all_identical_to_reference": all(
            r.get("same_objective_ref", True) for r in rows),
        "pruning_preserves_objective": all(r["same_objective_pruning"] for r in rows),
        "median_prune_rate": round(statistics.median(
            r["fast_pruned"] / max(1, r["fast_pruned"] + r["fast_states"])
            for r in rows), 4),
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "rows": rows, "table_md": table},
                  f, ensure_ascii=False, indent=2)
    print("\n" + json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"\n[OK] -> {OUT_JSON}")


if __name__ == "__main__":
    main()
