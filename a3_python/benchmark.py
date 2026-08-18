"""评测脚本 — vs OR-Tools / PyVRP / 消融实验 (W5 完整实现)

用法:
    python a3_python/benchmark.py --points 5,10,20 --runs 10 --output results/
    python a3_python/benchmark.py --quick

W5 评测内容 (A3_DEVPLAN.md W5 D1-D3):
  1. vs OR-Tools 精确解: 自建 5/10/15 点, 双基线 gap
     - gap_geo: 我们的几何距离 vs CP-SAT 几何最优
     - gap_energy: 我们的等效距离 vs 几何最优序列在能量模型下的成本
     - gap_energy_opt: vs 能量感知 DP 精确最优 (≤15 点)
  2. vs PyVRP (无电量约束): 自建 + Solomon R101/C101/RC101, 对比解差异
  3. Solomon 标准实例验收: BKS = CP-SAT 几何最优 (R5.1 结论)
  4. 消融矩阵: full / nn_only / no_oropt / no_2opt / fixed_payload / no_energy
  5. 规模扩展: 求解时间曲线 (5→10→15→20 + Solomon n20)

输出 (论文-ready):
  results/benchmark_<ts>.json — 全量数据
  results/table_<ts>.md       — 指标表 (markdown)

关键口径 (R5.1 调研结论):
  - 几何最优 ≠ 能量最优: 几何最优序列可能在真实电量模型下不可行 ("坠机")
    → 每个基线同时记录 battery_feasible
  - "runs" 语义 = 不同随机种子生成的实例 (算法确定性, 同实例多次运行无意义)
"""

import datetime
import json
import os
import time

from .route import GeoPoint, DroneSpec
from .solver import plan_multistop
from .fixture_loader import load_fixture_json, targets_from_dict
from .data_generator import generate_targets
from .route import DRONE_PRESETS

# === 模块级常量 ===

PYVRP_TIME_LIMIT = 2.0      # PyVRP 求解时限 (秒) — n≤20 TSP 收敛快
CP_SAT_TIME_LIMIT = 60.0    # CP-SAT 精确求解时限 (秒)
SOLOMON_NAMES = ["solomon_r101_n20", "solomon_c101_n20", "solomon_rc101_n20"]
SELF_DISTRIBUTIONS = ["circle", "random"]


# === 快速冒烟 ===

def quick_smoke_test() -> dict:
    """快速冒烟测试: 5 个场景, 每个 1 次 (W1 保留)

    Returns:
        dict: {场景名: {feasible, cost, time_ms}}
    """
    home = GeoPoint(x=0.0, y=0.0)
    drone = DroneSpec(
        payload_capacity=50.0,
        battery_capacity=20000.0,
        alpha=0.1,
        beta=0.005,
    )
    drone_large = DroneSpec(
        payload_capacity=500.0,
        battery_capacity=100000.0,
        alpha=0.08,
        beta=0.002,
    )

    scenarios = {
        "5p_circle": (generate_targets(5, distribution="circle", scale=100.0, seed=42, demand_range=(1.0, 1.0)), home, drone),
        "10p_circle": (generate_targets(10, distribution="circle", scale=100.0, seed=42, demand_range=(1.0, 1.0)), home, drone),
        "20p_circle": (generate_targets(20, distribution="circle", scale=100.0, seed=42, demand_range=(1.0, 1.0)), home, drone),
    }

    # 加载 Solomon 场景 (用大型无人机, 因为 Solomon 实例 demand 较大)
    for name in SOLOMON_NAMES[:2]:
        try:
            data = load_fixture_json(f"{name}.json")
            h, t = targets_from_dict(data)
            scenarios[name] = (t, h, drone_large)
        except FileNotFoundError:
            pass

    results = {}
    print(f"{'Scenario':<25} {'Feasible':<10} {'Cost (equiv_m)':<16} {'Time (ms)':<10}")
    print("-" * 61)

    for name, (targets, h, d) in scenarios.items():
        t0 = time.perf_counter()
        result = plan_multistop(targets, h, d)
        elapsed_ms = (time.perf_counter() - t0) * 1000

        results[name] = {
            "feasible": result.feasible,
            "cost": result.total_equiv_distance,
            "time_ms": round(elapsed_ms, 2),
            "warnings": result.warnings,
        }

        print(
            f"{name:<25} "
            f"{str(result.feasible):<10} "
            f"{result.total_equiv_distance:<16.1f} "
            f"{elapsed_ms:<10.2f}"
        )

    return results


# === 实例构建 ===

def _build_self_built_instances(
    point_counts: list[int],
    num_runs: int,
    scale: float = 1000.0,
) -> list[tuple[str, list, GeoPoint, DroneSpec]]:
    """自建实例: 每种分布 × 每个规模 × num_runs 个种子.

    demand_range 收窄到 (1, 3)kg 保证 standard 机型 (50kg) 可装下
    20 点的总需求 (~40kg); 电量绑定场景由 _build_tight_instances 单独覆盖.
    """
    instances = []
    for n in point_counts:
        for dist in SELF_DISTRIBUTIONS:
            for seed in range(num_runs):
                targets = generate_targets(
                    n, distribution=dist, scale=scale, seed=seed,
                    demand_range=(1.0, 3.0),
                )
                home = GeoPoint(x=0.0, y=0.0)
                name = f"{dist}_{n}p_s{seed}"
                instances.append((name, targets, home, DRONE_PRESETS["standard"]))
    return instances


def _build_tight_instances(num_runs: int) -> list[tuple[str, list, GeoPoint, DroneSpec]]:
    """电量绑定实例组: 演示"几何最优坠机, 我们的解可行" (R5.1 专利论据).

    circle n=10, scale=3000, 3500Wh 电池 — 实测部分种子 (如 seed=42) 的
    几何最优序列电池耗尽, 而约束感知 NN+VND 找到可行解.
    """
    tight_drone = DroneSpec(
        payload_capacity=50.0,
        battery_capacity=3500.0,
        alpha=0.1,
        beta=0.005,
    )
    instances = []
    for seed in range(num_runs):
        targets = generate_targets(
            10, distribution="circle", scale=3000.0, seed=seed,
            demand_range=(1.0, 3.0),
        )
        instances.append((f"tight_10p_s{seed}", targets,
                          GeoPoint(x=0.0, y=0.0), tight_drone))
    return instances


def _build_solomon_instances() -> list[tuple[str, list, GeoPoint, DroneSpec]]:
    """Solomon 标准实例: R101/C101/RC101 前 20 点, heavy 机型."""
    instances = []
    for name in SOLOMON_NAMES:
        try:
            data = load_fixture_json(f"{name}.json")
            home, targets = targets_from_dict(data)
            instances.append((name, targets, home, DRONE_PRESETS["heavy"]))
        except FileNotFoundError:
            pass
    return instances


# === 单实例对比 ===

def _compare_instance(
    name: str,
    targets: list,
    home: GeoPoint,
    drone: DroneSpec,
) -> dict:
    """单个实例的完整对比: 我们的方法 vs OR-Tools / 能量 DP / PyVRP.

    Returns:
        dict: 一行指标 (详见 run_benchmark 的列说明)
    """
    from .exact import (
        MAX_DP_POINTS,
        compute_gap,
        evaluate_sequence_cost,
        solve_energy_exact_dp,
        solve_tsp_exact_cpsat,
    )
    from .baseline import solve_tsp_pyvrp

    n = len(targets)
    row: dict = {
        "instance": name,
        "n": n,
    }

    # 1) 我们的方法
    t0 = time.perf_counter()
    plan = plan_multistop(targets, home, drone)
    row["our_time_ms"] = round((time.perf_counter() - t0) * 1000, 2)
    row["our_equiv"] = round(plan.total_equiv_distance, 2)
    row["our_geo"] = round(plan.total_geo_distance, 2)
    row["our_feasible"] = plan.feasible

    # 2) OR-Tools 几何精确解 (BKS) — 同时记录其电池可行性
    opt = solve_tsp_exact_cpsat(targets, home, drone=drone,
                                time_limit=CP_SAT_TIME_LIMIT, instance_name=name)
    row["opt_geo"] = round(opt.objective, 2)
    row["opt_time_ms"] = opt.solve_time_ms
    row["opt_geo_battery_feasible"] = opt.energy_feasible
    row["gap_geo_pct"] = round(
        compute_gap(plan.total_geo_distance, opt.objective), 2)

    # 3) 几何最优序列在能量模型下的成本 (同目标公平比较)
    if opt.sequence:
        opt_equiv, _, _ = evaluate_sequence_cost(targets, home, drone, opt.sequence)
        row["opt_geo_equiv"] = round(opt_equiv, 2)
        row["gap_energy_pct"] = round(
            compute_gap(plan.total_equiv_distance, opt_equiv), 2)
    else:
        row["opt_geo_equiv"] = None
        row["gap_energy_pct"] = None

    # 4) 能量感知 DP 精确最优 (≤15 点) — 能量目标的真实最优
    if n <= MAX_DP_POINTS:
        opt_e = solve_energy_exact_dp(targets, home, drone, instance_name=name)
        row["opt_energy"] = round(opt_e.objective, 2)
        row["opt_energy_battery_feasible"] = opt_e.energy_feasible
        # 仅当最优序列电池可行时, gap 才是同约束下的公平比较
        if opt_e.energy_feasible:
            row["gap_energy_opt_pct"] = round(
                compute_gap(plan.total_equiv_distance, opt_e.objective), 2)
        else:
            row["gap_energy_opt_pct"] = None  # 最优不可行 → 无公平 gap
    else:
        row["opt_energy"] = None
        row["opt_energy_battery_feasible"] = None
        row["gap_energy_opt_pct"] = None

    # 5) PyVRP 无电量基线 (几何最优近似) + 其路线在能量模型下的表现
    pyvrp = solve_tsp_pyvrp(targets, home, time_limit=PYVRP_TIME_LIMIT,
                            instance_name=name, bks_cost=opt.objective)
    row["pyvrp_geo"] = round(pyvrp.total_distance, 2)
    row["pyvrp_time_ms"] = pyvrp.solve_time_ms
    row["pyvrp_gap_pct"] = round(pyvrp.gap_vs_best, 2) if pyvrp.gap_vs_best is not None else None
    if pyvrp.route:
        p_equiv, p_feas, _ = evaluate_sequence_cost(targets, home, drone, pyvrp.route)
        row["pyvrp_equiv"] = round(p_equiv, 2)
        row["pyvrp_battery_feasible"] = p_feas
    else:
        row["pyvrp_equiv"] = None
        row["pyvrp_battery_feasible"] = None

    return row


# === 消融 ===

def _run_ablation_on_instances(
    instances: list[tuple[str, list, GeoPoint, DroneSpec]],
) -> dict:
    """在基准实例集上跑消融矩阵, 按规模分组聚合."""
    from .ablation import aggregate_stats, run_ablation
    from .exact import MAX_DP_POINTS

    # 消融用 ≤15 点自建实例 (standard 机型; Solomon n20 超出 DP 范围单独验收)
    small = [(name, targets, home) for name, targets, home, _ in instances
             if name.split("_")[0] in SELF_DISTRIBUTIONS
             and len(targets) <= MAX_DP_POINTS]
    results = run_ablation(small, DRONE_PRESETS["standard"])

    def group_fn(name: str) -> str:
        # "circle_10p_s3" → "circle_10p" (按 分布+规模 分组)
        parts = name.split("_")
        return f"{parts[0]}_{parts[1]}"

    return aggregate_stats(results, group_fn)


# === 表格输出 ===

def _format_md_table(rows: list[dict]) -> str:
    """论文-ready 指标表 (markdown)"""
    headers = [
        "Instance", "N", "Our_equiv", "Our_geo", "Our_time(ms)", "Feas",
        "Opt_geo", "Opt_bat", "Gap_geo%", "Gap_eng%",
        "Opt_eng", "Opt_eng_bat", "Gap_eng_opt%",
        "PyVRP_geo", "PyVRP_gap%", "PyVRP_bat",
    ]
    lines = ["| " + " | ".join(headers) + " |",
             "|" + "|".join(["---"] * len(headers)) + "|"]
    for r in rows:
        def fmt(v, nd=1):
            if v is None:
                return "—"
            if isinstance(v, bool):
                return "✓" if v else "✗"
            if isinstance(v, (int, float)):
                return f"{v:.{nd}f}" if v != int(v) else str(int(v))
            return str(v)
        cells = [
            r["instance"], fmt(r["n"], 0),
            fmt(r["our_equiv"]), fmt(r["our_geo"]),
            fmt(r["our_time_ms"]), fmt(r["our_feasible"]),
            fmt(r["opt_geo"]), fmt(r["opt_geo_battery_feasible"]),
            fmt(r["gap_geo_pct"]), fmt(r["gap_energy_pct"]),
            fmt(r["opt_energy"]), fmt(r["opt_energy_battery_feasible"]),
            fmt(r["gap_energy_opt_pct"]),
            fmt(r["pyvrp_geo"]), fmt(r["pyvrp_gap_pct"]),
            fmt(r["pyvrp_battery_feasible"]),
        ]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _format_ablation_md(ablation: dict) -> str:
    """消融矩阵表 (markdown): 每变体 × 每组 相对成本 + 可行率"""
    from .ablation import ABLATION_VARIANTS

    # 组列表 (按 full 变体的实例顺序)
    groups = list(ablation["full"].keys())

    lines = []
    for group in groups:
        full_cost = ablation["full"][group].cost_mean
        lines.append(f"\n**{group}** (full 成本 = {full_cost:.1f}):\n")
        lines.append("| Variant | Cost(mean±std) | Rel% | Time(ms) | Feasible% |")
        lines.append("|---|---|---|---|---|")
        for variant in ABLATION_VARIANTS:
            s = ablation[variant][group]
            rel = (s.cost_mean / full_cost * 100.0) if full_cost > 0 and s.n_feasible else None
            rel_str = f"{rel:.1f}%" if rel is not None else "—"
            cost_str = (f"{s.cost_mean:.1f}±{s.cost_std:.1f}"
                        if s.n_feasible else "—")
            lines.append(
                f"| {variant:<14} | {cost_str:<16} | {rel_str:<6} "
                f"| {s.time_ms_mean:.1f} | {s.feasible_rate * 100:.0f}% |"
            )
    return "\n".join(lines)


def _format_scaling_md(rows: list[dict]) -> str:
    """规模扩展时间曲线 (markdown): 按 N 聚合"""
    by_n: dict[int, list[dict]] = {}
    for r in rows:
        by_n.setdefault(r["n"], []).append(r)

    lines = ["| N | 实例数 | Our_time(ms) | Opt_geo_time(ms) | PyVRP_time(ms) |",
             "|---|---|---|---|---|"]
    for n in sorted(by_n):
        rs = by_n[n]
        our = sum(r["our_time_ms"] for r in rs) / len(rs)
        opt_t = sum(r["opt_time_ms"] for r in rs) / len(rs)
        pyvrp = sum(r["pyvrp_time_ms"] for r in rs) / len(rs)
        lines.append(f"| {n} | {len(rs)} | {our:.2f} | {opt_t:.2f} | {pyvrp:.2f} |")
    return "\n".join(lines)


# === 主入口 ===

def run_benchmark(
    point_counts: list[int],
    num_runs: int = 10,
    output_dir: str = "results/",
) -> dict:
    """运行完整评测 (W5 实现)

    对比:
      - vs OR-Tools 精确解 (CP-SAT, ≤20 点) + 能量感知 DP 精确最优 (≤15 点)
      - vs PyVRP (无电量约束)
      - 消融矩阵 (6 变体)
      - 规模扩展时间曲线

    Args:
        point_counts: 测试规模列表, 如 [5, 10, 20]
        num_runs: 每个规模的实例数 (不同随机种子)
        output_dir: 结果输出目录

    Returns:
        dict: 全量评测数据 {meta, instances, ablation, scaling}
    """
    from .ablation import ABLATION_VARIANTS

    t_start = time.perf_counter()

    # 构建实例
    instances = _build_self_built_instances(point_counts, num_runs)
    instances += _build_tight_instances(num_runs)
    instances += _build_solomon_instances()
    print(f"[benchmark] {len(instances)} 实例 (自建 {len(instances) - len(SOLOMON_NAMES)} + "
          f"Solomon {len(SOLOMON_NAMES)}), 消融变体 {len(ABLATION_VARIANTS)} 个\n")

    # 逐实例对比
    print(f"{'Instance':<22} {'N':<4} {'Our_equiv':<10} {'Opt_geo':<9} "
          f"{'Gap_geo%':<9} {'Gap_eng%':<9} {'Feas':<6} {'Opt_bat':<8}")
    print("-" * 82)
    rows = []
    for name, targets, home, drone in instances:
        row = _compare_instance(name, targets, home, drone)
        rows.append(row)
        print(f"{name:<22} {row['n']:<4} {row['our_equiv']:<10.1f} "
              f"{row['opt_geo']:<9.1f} {row['gap_geo_pct']:<9.2f} "
              f"{row['gap_energy_pct'] if row['gap_energy_pct'] is not None else float('nan'):<9.2f} "
              f"{'✓' if row['our_feasible'] else '✗':<6} "
              f"{'✓' if row['opt_geo_battery_feasible'] else '✗':<8}")

    # 消融
    print(f"\n[benchmark] 消融实验 ({len(ABLATION_VARIANTS)} 变体)...")
    ablation = _run_ablation_on_instances(instances)

    # 聚合
    meta = {
        "generated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "point_counts": point_counts,
        "num_runs": num_runs,
        "total_instances": len(instances),
        "elapsed_secs": round(time.perf_counter() - t_start, 2),
        "notes": [
            "gap_geo% = (our_geo − opt_geo)/opt_geo — 几何距离视角",
            "gap_eng% = (our_equiv − opt_geo序列在能量模型下成本)/该成本 — 同目标公平比较 (可为负: 我们更优)",
            "gap_eng_opt% = vs 能量感知 DP 精确最优 (≤15 点, 仅当最优序列电池可行)",
            "Opt_bat/PyVRP_bat = 基线路线在真实电量模型下的可行性 (✗ = 会坠机)",
        ],
    }
    scaling = {"point_counts": point_counts, "num_runs": num_runs}

    # AblationStats → dict (JSON 序列化)
    ablation_json = {
        variant: {inst: stats.__dict__ for inst, stats in inst_map.items()}
        for variant, inst_map in ablation.items()
    }
    payload = {"meta": meta, "instances": rows,
               "ablation": ablation_json, "scaling": scaling}

    # 输出文件
    os.makedirs(output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = os.path.join(output_dir, f"benchmark_{ts}.json")
    md_path = os.path.join(output_dir, f"table_{ts}.md")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    with open(md_path, "w", encoding="utf-8") as f:
        f.write(f"# A3 评测指标表 (W5 月1中检)\n\n")
        f.write(f"生成时间: {meta['generated_at']} | 实例数: {len(rows)} | "
                f"总耗时: {meta['elapsed_secs']}s\n\n")
        f.write("## 1. 完整对比指标表\n\n")
        f.write(_format_md_table(rows))
        f.write("\n\n## 2. 消融矩阵\n\n")
        f.write(_format_ablation_md(ablation))
        f.write("\n\n## 3. 规模扩展时间曲线\n\n")
        f.write("> 注: CP-SAT/PyVRP 单实例耗时见 JSON 全量数据\n\n")
        f.write(_format_scaling_md(rows))
        f.write("\n\n## 口径说明\n\n")
        for note in meta["notes"]:
            f.write(f"- {note}\n")

    print(f"\n[benchmark] 完成, 总耗时 {meta['elapsed_secs']}s")
    print(f"  JSON: {json_path}")
    print(f"  MD  : {md_path}")
    return payload


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="A3 多目标路径规划评测")
    parser.add_argument("--points", type=str, default="5,10,20",
                        help="测试规模, 逗号分隔 (如 5,10,20)")
    parser.add_argument("--runs", type=int, default=10,
                        help="每个规模的实例数 (不同随机种子)")
    parser.add_argument("--output", type=str, default="results/",
                        help="结果输出目录")
    parser.add_argument("--quick", action="store_true",
                        help="快速冒烟测试 (W1 可用)")
    args = parser.parse_args()

    if args.quick:
        quick_smoke_test()
    else:
        point_counts = [int(x) for x in args.points.split(",")]
        run_benchmark(point_counts, args.runs, args.output)
