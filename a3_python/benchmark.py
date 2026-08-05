"""评测脚本 — vs OR-Tools / PyVRP / 消融实验

用法:
    python a3_python/benchmark.py --points 5,10,20 --runs 10 --output results/
    python a3_python/benchmark.py --quick

W1: quick_smoke_test 已可用, 完整 benchmark 待 W5。
"""

import json
import os
import time
from .route import GeoPoint, Target, DroneSpec
from .solver import plan_multistop
from .fixture_loader import load_fixture_json, targets_from_dict
from .data_generator import generate_targets


def quick_smoke_test() -> dict:
    """快速冒烟测试: 5 个场景, 每个 1 次

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

    # 加载 Solomon 场景 (用大型无人机, 因为 Solom 实例 demand 较大)
    for name in ["solomon_r101_n20", "solomon_c101_n20"]:
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


def run_benchmark(
    point_counts: list[int],
    num_runs: int = 10,
    output_dir: str = "results/",
) -> None:
    """运行完整评测 (W5 实现)

    对比:
      - vs OR-Tools 精确解 (≤15 点)
      - vs PyVRP (无电量约束)
      - vs VeRyPy (15 种经典启发式)
      - 消融: 去掉载重耦合 / 只用最近邻

    Args:
        point_counts: 测试规模列表, 如 [5, 10, 20]
        num_runs: 每个规模的重复次数
        output_dir: 结果输出目录
    """
    raise NotImplementedError("Full benchmark — W5 实现 (quick_smoke_test 可用)")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="A3 多目标路径规划评测")
    parser.add_argument("--points", type=str, default="5,10,20",
                        help="测试规模, 逗号分隔 (如 5,10,20)")
    parser.add_argument("--runs", type=int, default=10,
                        help="每个规模的重复次数")
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
