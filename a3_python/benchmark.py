"""评测脚本 — vs OR-Tools / PyVRP / 消融实验 (W5 实现)

W1 骨架: 入口函数已定义, 实现待 W5 完成。

用法:
    python a3_python/benchmark.py --points 5,10,20 --runs 10 --output results/
    python a3_python/benchmark.py --quick
"""


def run_benchmark(
    point_counts: list[int],
    num_runs: int = 10,
    output_dir: str = "results/",
) -> None:
    """运行完整评测 (W5 实现)

    对比:
      - vs OR-Tools 精确解 (≤15 点)
      - vs PyVRP (无电量约束)
      - 消融: 去掉载重耦合 / 只用最近邻

    Args:
        point_counts: 测试规模列表, 如 [5, 10, 20]
        num_runs: 每个规模的重复次数
        output_dir: 结果输出目录
    """
    raise NotImplementedError("Benchmark — W5 实现")


def quick_smoke_test() -> None:
    """快速冒烟测试: 只跑 5 点, 1 次"""
    raise NotImplementedError("Quick smoke test — W5 实现")


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
                        help="快速冒烟测试")
    args = parser.parse_args()

    if args.quick:
        quick_smoke_test()
    else:
        point_counts = [int(x) for x in args.points.split(",")]
        run_benchmark(point_counts, args.runs, args.output)
