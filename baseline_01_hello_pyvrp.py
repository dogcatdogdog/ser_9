"""
Baseline 01: PyVRP Hello World (v0.13.4 API)
最小可运行示例 — 5 个送货点 + 1 个仓库 + 1 架无人机
"""
import math
from pyvrp import Model
from pyvrp.stop import MaxRuntime

# === 1. 建模 ===
model = Model()

# 添加仓库 (0.13.4: x, y 直接传)
depot = model.add_depot(x=0, y=0, name="depot")

# 添加客户
clients = [
    model.add_client(x=10, y=5,  delivery=5,  name="c1"),
    model.add_client(x=5,  y=12, delivery=8,  name="c2"),
    model.add_client(x=15, y=8,  delivery=3,  name="c3"),
    model.add_client(x=20, y=3,  delivery=6,  name="c4"),
    model.add_client(x=8,  y=15, delivery=4,  name="c5"),
]

# 添加边: 所有地点两两之间 (depot + 5 clients)
all_nodes = [depot] + clients
for frm in all_nodes:
    for to in all_nodes:
        if frm is to:
            continue
        dx = frm.x - to.x  # type: ignore[attr-defined]
        dy = frm.y - to.y  # type: ignore[attr-defined]
        dist = int(math.sqrt(dx * dx + dy * dy) * 100)
        model.add_edge(frm, to, distance=dist, duration=dist)

# 添加无人机
model.add_vehicle_type(
    num_available=1,
    capacity=[30],        # 最大载重 (送货总量=5+8+3+6+4=26)
    max_distance=20000,   # 最大航程 (要给够)
)

# === 2. 求解 ===
result = model.solve(stop=MaxRuntime(5), seed=42, display=True)

# === 3. 输出结果 ===
solution = result.best
print("\n" + "=" * 60)
print("求解完成!")
print(f"  目标函数值 (cost): {result.cost()}")
print(f"  路线数: {solution.num_routes()}")
print(f"  总距离: {solution.distance()}")
print(f"  是否可行: {solution.is_feasible()}")
print(f"  是否完整: {solution.is_complete()}")

for idx, route in enumerate(solution.routes()):
    print(f"\n  路线 {idx+1}:")
    # 收集访问顺序
    # Route 在 0.13.4 迭代返回 int (client/depot index)
    visits = []
    for idx in route:
        if idx == 0:
            visits.append("depot")
        else:
            visits.append(f"c{idx}")
    print(f"    访问顺序: {' → '.join(visits)}")
    print(f"    载重: delivery={route.delivery()} pickup={route.pickup()}")
    print(f"    距离: {route.distance()}")
    print(f"    可行: {route.is_feasible()}")

print("\n[OK] Baseline 01 PASSED -- PyVRP is working!")
