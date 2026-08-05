"""PyVRP 基线求解 — 对比无电量约束的 TSP 最优解

用法:
    from a3_python.baseline import solve_tsp_pyvrp, run_baseline_suite

    # 单个实例
    result = solve_tsp_pyvrp(targets, home)

    # 批量评测
    suite = run_baseline_suite([5, 10, 20])

W2: 用 PyVRP 在 5/10/20 点数据集上求解（无电量约束），记录 baseline 指标。
W5: 加入 vs OR-Tools 精确解对比。
"""

import math
import time
import os
from dataclasses import dataclass, field
from typing import Optional

from pyvrp import Model
from pyvrp.stop import MaxRuntime

from .route import GeoPoint, Target, DroneSpec
from .data_generator import generate_targets

# === 模块级常量 ===

DEFAULT_TIME_LIMIT = 5          # 秒 — PyVRP 求解时限
DISTANCE_SCALE = 100            # 距离 × 100 取整 (PyVRP 用整数距离)
FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "tests", "fixtures")


# === 数据结构 ===

@dataclass
class BaselineResult:
    """PyVRP 基线求解结果"""
    instance_name: str                        # 实例名称
    n_points: int                             # 目标点数
    route: list[str]                          # 访问顺序 (target ids)
    total_distance: float                     # 总几何距离 (最优解)
    optimal_cost: int                         # PyVRP cost 值
    solve_time_ms: float                      # 求解耗时 (ms)
    feasible: bool                            # PyVRP 是否找到可行解
    num_routes: int                           # 使用的路线数
    gap_vs_best: Optional[float] = None       # vs 已知最优解的 gap (W5)


# === 核心函数 ===

def solve_tsp_pyvrp(
    targets: list[Target],
    home: GeoPoint,
    time_limit: float = DEFAULT_TIME_LIMIT,
    instance_name: str = "unnamed",
    seed: int = 42,
) -> BaselineResult:
    """用 PyVRP 求解纯距离 TSP (无电量约束)

    构造完整距离矩阵, 用 PyVRP 单车辆求解 TSP。
    目标: 最小化总距离 (忽略电量/载重耦合)。

    Args:
        targets: 目标点列表
        home: 仓库位置
        time_limit: PyVRP 求解时限 (秒)
        instance_name: 实例名称 (用于报告)
        seed: 随机种子

    Returns:
        BaselineResult: 包含最优解路线、总距离、求解时间等
    """
    n = len(targets)
    if n == 0:
        return BaselineResult(
            instance_name=instance_name, n_points=0,
            route=[], total_distance=0.0, optimal_cost=0,
            solve_time_ms=0.0, feasible=True, num_routes=0,
        )

    # 1) 构建 PyVRP Model
    model = Model()

    # 添加仓库
    depot = model.add_depot(x=int(home.x * DISTANCE_SCALE),
                            y=int(home.y * DISTANCE_SCALE),
                            name="depot")

    # 添加客户
    clients = []
    for t in targets:
        c = model.add_client(
            x=int(t.location.x * DISTANCE_SCALE),
            y=int(t.location.y * DISTANCE_SCALE),
            delivery=int(t.demand),
            name=t.id,
        )
        clients.append(c)

    # 添加边: 所有地点两两之间
    all_nodes = [depot] + clients
    max_dist = 0
    for frm in all_nodes:
        for to in all_nodes:
            if frm is to:
                continue
            dx = frm.x - to.x  # type: ignore[attr-defined]
            dy = frm.y - to.y  # type: ignore[attr-defined]
            dist = int(math.sqrt(dx * dx + dy * dy))
            max_dist = max(max_dist, dist)
            model.add_edge(frm, to, distance=dist, duration=dist)

    # 添加无人机 (单架, 容量足够)
    total_demand = sum(t.demand for t in targets)
    model.add_vehicle_type(
        num_available=1,
        capacity=[int(total_demand) + 10],     # 容量留有裕量
        max_distance=max_dist * (n + 2) * 2,   # 航程足够覆盖任意路线
    )

    # 2) 求解
    t0 = time.perf_counter()
    result = model.solve(stop=MaxRuntime(time_limit), seed=seed, display=False)
    elapsed_ms = (time.perf_counter() - t0) * 1000

    solution = result.best

    # 3) 提取路线
    route: list[str] = []
    total_distance = 0.0
    if solution.is_complete() and solution.num_routes() > 0:
        for r_idx, route_obj in enumerate(solution.routes()):
            if r_idx > 0:
                break  # 单车辆, 只取第一条路线
            # 收集访问顺序
            depot_visits = 0
            for visit_idx in route_obj:
                if visit_idx == 0:  # depot
                    depot_visits += 1
                else:
                    # client index is 1-based (0 = depot)
                    client_idx = visit_idx - 1
                    if 0 <= client_idx < len(targets):
                        route.append(targets[client_idx].id)
            total_distance = route_obj.distance() / DISTANCE_SCALE

    return BaselineResult(
        instance_name=instance_name,
        n_points=n,
        route=route,
        total_distance=round(total_distance, 2),
        optimal_cost=int(result.cost()),
        solve_time_ms=round(elapsed_ms, 2),
        feasible=solution.is_feasible() and solution.is_complete(),
        num_routes=solution.num_routes(),
    )


def run_baseline_suite(
    point_counts: list[int] | None = None,
    time_limit: float = DEFAULT_TIME_LIMIT,
) -> dict[str, BaselineResult]:
    """对自建数据集运行 PyVRP 基线评测

    对每个规模 n, 生成圆形分布测试点, 用 PyVRP 求解。
    同时加载 Solomon fixtures 求解。

    Args:
        point_counts: 测试规模列表, 默认 [5, 10, 15, 20]
        time_limit: 每个实例的求解时限 (秒)

    Returns:
        dict: {instance_name: BaselineResult}
    """
    if point_counts is None:
        point_counts = [5, 10, 15, 20]

    results: dict[str, BaselineResult] = {}
    home = GeoPoint(x=0.0, y=0.0)

    # 自建数据集
    for n in point_counts:
        targets = generate_targets(n, distribution="circle", scale=1000.0)
        name = f"circle_{n}p"
        results[name] = solve_tsp_pyvrp(targets, home, time_limit, name)

    # Solomon fixtures (如果可用)
    from .fixture_loader import load_fixture_json, targets_from_dict
    for fixture_name in ["solomon_r101_n20", "solomon_c101_n20", "solomon_rc101_n20"]:
        try:
            data = load_fixture_json(f"{fixture_name}.json")
            h, t = targets_from_dict(data)
            results[fixture_name] = solve_tsp_pyvrp(t, h, time_limit, fixture_name)
        except FileNotFoundError:
            pass

    return results


def print_baseline_report(results: dict[str, BaselineResult]) -> None:
    """打印基线评测报告 (表格形式)"""
    print(f"{'Instance':<25} {'N':<5} {'Distance':<12} {'Time (ms)':<12} {'Feasible':<10} {'Route':<30}")
    print("-" * 100)
    for name, r in results.items():
        route_str = " → ".join(r.route[:5])
        if len(r.route) > 5:
            route_str += " → ..."
        print(
            f"{name:<25} {r.n_points:<5} {r.total_distance:<12.1f} "
            f"{r.solve_time_ms:<12.1f} {str(r.feasible):<10} {route_str:<30}"
        )


if __name__ == "__main__":
    print("=== PyVRP Baseline Suite ===\n")
    results = run_baseline_suite([5, 10, 15, 20])
    print_baseline_report(results)
    print("\n[DONE] Baseline suite complete.")
