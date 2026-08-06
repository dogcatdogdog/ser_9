"""构造启发式 — NN (最近邻) + Savings (W3 实现)

W3 实现: 电量感知 NN (N-start) + Clarke-Wright Savings (equiv_dist 改造版)

调研结论 (A3_RESEARCH_PLAN.md §R3):
  R3.1: Solomon-style 约束 NN — cost=equiv_dist, 容量+电量双重检查
  R3.2: C-W Savings — s(i,j) = equiv(i→home,P-d_i) + equiv(home→j,P) − equiv(i→j,P-d_i)
         n≤20 时全量 feasibility check, 无需增量
  R3.3: N-start NN — n≤20 时 O(N×n²) 完全可承受
"""

from .route import Target, GeoPoint, DroneSpec, RoutePlan
from .energy_model import (
    euclidean_distance,
    compute_equiv_distance,
    simulate_route_energy,
)


def _build_route_plan(
    sequence: list[str],
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """从访问序列构建 RoutePlan (封装 simulate_route_energy)"""
    segments, total_geo, total_equiv, total_energy, remaining, feasible, warnings = (
        simulate_route_energy(sequence, targets_map, home, drone)
    )
    total_payload = sum(t.demand for t in targets_map.values())
    return RoutePlan(
        sequence=sequence,
        segments=segments,
        total_geo_distance=total_geo,
        total_equiv_distance=total_equiv,
        total_energy_consumed=total_energy,
        remaining_energy=remaining,
        total_payload_delivered=total_payload,
        feasible=feasible,
        warnings=warnings,
    )


def _check_roundtrip_feasibility(
    from_loc: GeoPoint,
    to_target: Target,
    current_payload: float,
    current_battery: float,
    home: GeoPoint,
    drone: DroneSpec,
) -> tuple[bool, float, float, float]:
    """检查从 from_loc 到 to_target 并返航 home 的可行性.

    返回 (feasible, equiv_dist_to_target, energy_to_target, payload_after_target).
    """
    geo_to = euclidean_distance(from_loc, to_target.location)
    equiv_to = compute_equiv_distance(
        geo_to, current_payload, drone.alpha, drone.beta
    )
    energy_to = equiv_to * drone.alpha

    payload_after = current_payload - to_target.demand
    geo_home = euclidean_distance(to_target.location, home)
    equiv_home = compute_equiv_distance(
        geo_home, payload_after, drone.alpha, drone.beta
    )
    energy_home = equiv_home * drone.alpha

    feasible = current_battery >= energy_to + energy_home
    return feasible, equiv_to, energy_to, payload_after


def _nn_single_start(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    total_demand: float,
    first_target: Target,
) -> RoutePlan:
    """单起点 NN: 从 home 出发, 先访问 first_target, 然后贪心扩展.

    Args:
        targets: 完整目标点列表
        home: 仓库位置
        drone: 无人机规格
        total_demand: 总需求量 (预计算)
        first_target: 第一个访问的目标点

    Returns:
        RoutePlan: 构造的路线 (可能部分可行)
    """
    targets_map = {t.id: t for t in targets}
    n = len(targets)

    # 从 home 到 first_target
    feasible, equiv_to, energy_to, payload_after = _check_roundtrip_feasibility(
        home, first_target, total_demand, drone.battery_capacity, home, drone
    )

    if not feasible:
        # 第一个点就不可行, 返回空路线
        targets_map = {t.id: t for t in targets}
        plan = _build_route_plan([], targets_map, home, drone)
        plan.warnings.append(
            f"Cannot reach first target {first_target.id} from home: "
            f"need {energy_to:.1f}Wh roundtrip, battery={drone.battery_capacity:.1f}Wh"
        )
        plan.feasible = False
        return plan

    sequence = [first_target.id]
    unvisited = set(t.id for t in targets)
    unvisited.remove(first_target.id)
    current_loc = first_target.location
    current_payload = payload_after
    current_battery = drone.battery_capacity - energy_to

    # 贪心扩展
    while unvisited:
        best_id = None
        best_cost = float('inf')
        best_payload_after = current_payload
        best_energy = 0.0

        for t in targets:
            if t.id in unvisited:
                feasible_t, equiv_t, energy_t, payload_t = (
                    _check_roundtrip_feasibility(
                        current_loc, t, current_payload, current_battery,
                        home, drone
                    )
                )
                if feasible_t and equiv_t < best_cost:
                    best_cost = equiv_t
                    best_id = t.id
                    best_energy = energy_t
                    best_payload_after = payload_t

        if best_id is None:
            # 无可行下一跳: 返回部分路线
            plan = _build_route_plan(sequence, targets_map, home, drone)
            # 标记不可行
            plan.warnings.append(
                f"No feasible next stop after {sequence[-1]}: "
                f"visited {len(sequence)}/{n} targets"
            )
            # 强制设为不可行
            plan.feasible = False
            return plan

        sequence.append(best_id)
        unvisited.remove(best_id)
        current_loc = targets_map[best_id].location
        current_payload = best_payload_after
        current_battery -= best_energy

    return _build_route_plan(sequence, targets_map, home, drone)


def construct_nn(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """电量感知最近邻构造 — N-start 变体.

    从每个目标点作为第一个访问点各跑一次 NN, 取最优可行解。
    如果所有起点都不可行, 返回访问点数最多的部分路线。

    算法:
      1. 对每个 target 作为 first: 从 home→first 开始, 贪心扩展
      2. 每次选 equiv_dist 最小且满足 round-trip 电量约束的未访问点
      3. 无可行下一跳时返回部分路线
      4. 从所有 N 次尝试中选 feasible=True 且 total_equiv_distance 最小的

    复杂度: O(N × n²) — n≤20 时 ~8000 步, <1ms

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格

    Returns:
        RoutePlan: 构造的路线。feasible=True 表示所有点都已访问且满足约束;
                   feasible=False 时查看 warnings 了解原因。
    """
    if not targets:
        raise ValueError("targets list cannot be empty")

    total_demand = sum(t.demand for t in targets)

    # 快速载重检查
    if total_demand > drone.payload_capacity:
        targets_map = {t.id: t for t in targets}
        return RoutePlan(
            sequence=[],
            segments=[],
            total_geo_distance=0.0,
            total_equiv_distance=0.0,
            total_energy_consumed=0.0,
            remaining_energy=drone.battery_capacity,
            total_payload_delivered=0.0,
            feasible=False,
            warnings=[
                f"Total payload ({total_demand:.1f}kg) exceeds drone capacity "
                f"({drone.payload_capacity:.1f}kg)"
            ],
        )

    # 单点特例: 只需检查 home→target→home
    if len(targets) == 1:
        return _nn_single_start(targets, home, drone, total_demand, targets[0])

    # N-start: 尝试每个 target 作为第一个访问点
    best_plan: RoutePlan | None = None
    best_visited = 0

    for first in targets:
        plan = _nn_single_start(targets, home, drone, total_demand, first)

        if plan.feasible:
            # 可行解: 选 total_equiv_distance 最小的
            if (best_plan is None or
                    plan.total_equiv_distance < best_plan.total_equiv_distance):
                best_plan = plan
        else:
            # 不可行: 记录访问点数最多的 (用于降级返回)
            if len(plan.sequence) > best_visited:
                best_visited = len(plan.sequence)
                if best_plan is None or not best_plan.feasible:
                    best_plan = plan

    if best_plan is None:
        # 不应该到这里, 但安全起见返回空计划
        targets_map = {t.id: t for t in targets}
        return _build_route_plan([], targets_map, home, drone)

    return best_plan


def construct_savings(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """Clarke-Wright Savings 改造版 — 载重感知的节约算法.

    算法:
      1. 每个目标点独立成路线: home → t → home
      2. 计算所有点对 (i,j) 的 savings:
         s(i,j) = equiv(i→home, P−dᵢ) + equiv(home→j, P) − equiv(i→j, P−dᵢ)
      3. 按 savings 降序排列
      4. 贪心合并: i 在路线末尾、j 在路线开头时尝试合并
      5. 每次合并后用 simulate_route_energy 做全量可行性检查
      6. 返回最终合并后的路线

    复杂度: O(n² log n + n³) — n≤20 时 < 10ms

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格

    Returns:
        RoutePlan: 构造的路线
    """
    if not targets:
        raise ValueError("targets list cannot be empty")

    targets_map = {t.id: t for t in targets}
    total_demand = sum(t.demand for t in targets)

    # 快速载重检查
    if total_demand > drone.payload_capacity:
        return RoutePlan(
            sequence=[],
            segments=[],
            total_geo_distance=0.0,
            total_equiv_distance=0.0,
            total_energy_consumed=0.0,
            remaining_energy=drone.battery_capacity,
            total_payload_delivered=0.0,
            feasible=False,
            warnings=[
                f"Total payload ({total_demand:.1f}kg) exceeds drone capacity "
                f"({drone.payload_capacity:.1f}kg)"
            ],
        )

    n = len(targets)

    # 单点特例
    if n == 1:
        sequence = [targets[0].id]
        plan = _build_route_plan(sequence, targets_map, home, drone)
        return plan

    # Step 1: 每个点独立路线
    # routes: key → [id1, id2, ...]  (按访问顺序)
    routes: dict[str, list[str]] = {t.id: [t.id] for t in targets}
    # node_to_route_key: 每个 node 属于哪条路线
    node_to_route: dict[str, str] = {t.id: t.id for t in targets}

    # Step 2: 计算 savings
    savings_list: list[tuple[float, str, str]] = []
    for i, t_i in enumerate(targets):
        for j, t_j in enumerate(targets):
            if i >= j:
                continue
            # s(i,j) = equiv(i→home, P−dᵢ) + equiv(home→j, P) − equiv(i→j, P−dᵢ)
            payload_i = total_demand - t_i.demand
            geo_ih = euclidean_distance(t_i.location, home)
            geo_hj = euclidean_distance(home, t_j.location)
            geo_ij = euclidean_distance(t_i.location, t_j.location)

            equiv_ih = compute_equiv_distance(
                geo_ih, payload_i, drone.alpha, drone.beta
            )
            equiv_hj = compute_equiv_distance(
                geo_hj, total_demand, drone.alpha, drone.beta
            )
            equiv_ij = compute_equiv_distance(
                geo_ij, payload_i, drone.alpha, drone.beta
            )

            saving = equiv_ih + equiv_hj - equiv_ij
            if saving > 0:
                savings_list.append((saving, t_i.id, t_j.id))

    # Step 3: 按 saving 降序排列
    savings_list.sort(key=lambda x: x[0], reverse=True)

    # Step 4: 贪心合并
    for saving, i_id, j_id in savings_list:
        route_i_key = node_to_route.get(i_id)
        route_j_key = node_to_route.get(j_id)

        # 节点可能已被合并到其他路线
        if route_i_key is None or route_j_key is None:
            continue
        if route_i_key == route_j_key:
            continue

        route_i = routes.get(route_i_key)
        route_j = routes.get(route_j_key)
        if route_i is None or route_j is None:
            continue

        # i 必须在 route_i 末尾, j 必须在 route_j 开头
        if route_i[-1] != i_id or route_j[0] != j_id:
            continue

        # 尝试合并
        merged = route_i + route_j

        # 全量可行性检查
        _, _, _, _, _, feasible, _ = simulate_route_energy(
            merged, targets_map, home, drone
        )

        if feasible:
            # 更新 routes
            del routes[route_i_key]
            routes[route_j_key] = merged
            for node_id in merged:
                node_to_route[node_id] = route_j_key

    # Step 5: 收集合并后的路线
    if not routes:
        return _build_route_plan([], targets_map, home, drone)

    # 将所有剩余路线连接起来
    # (理想情况下全部合并为一条)
    final_sequence: list[str] = []
    for seq in routes.values():
        final_sequence.extend(seq)

    plan = _build_route_plan(final_sequence, targets_map, home, drone)

    # 如果有 >1 条路线未合并, 记录警告
    if len(routes) > 1:
        plan.warnings.append(
            f"Savings: {len(routes)} routes remain unmerged"
        )

    return plan


# ====================================================================
# W4 局部搜索 — 2-opt + Or-opt + VND (增量评估)
#
# 调研结论 (A3_RESEARCH_PLAN.md §R4):
#   R4.1: 2-opt 增量公式 — 仅重算受影响段 O(k) 而非 O(n)
#   R4.2: Or-opt 增量公式 — 同框架, 分剪下+插入两步
#   R4.3: 选用 first-improvement — 快速试错, 更体现增量评估价值
#   R4.4: 选用 VND — 2-opt → Or-opt 交替, 确定性可复现
#
# 核心创新 (专利创新点 3): 增量电量校验 — 局部搜索中仅重算受影响段
# ====================================================================

# --- 增量评估辅助函数 ---

def _segment_payloads(
    sequence: list[str],
    targets_map: dict[str, Target],
    total_demand: float,
) -> list[float]:
    """计算 full_path 各段出发时载重.

    full_path = ["home"] + sequence + ["home"], 共 n+1 段.
    payloads[k] = 第 k 段出发时载重 (k=0..n).
    """
    n = len(sequence)
    payloads = [0.0] * (n + 1)
    remaining = total_demand
    for k in range(n):
        payloads[k] = remaining
        remaining -= targets_map[sequence[k]].demand
    payloads[n] = 0.0  # 返航段
    return payloads


def _segment_energy(
    from_point: GeoPoint,
    to_point: GeoPoint,
    payload: float,
    drone: DroneSpec,
) -> tuple[float, float, float]:
    """计算单段能耗: (geo, equiv, energy)."""
    geo = euclidean_distance(from_point, to_point)
    equiv = compute_equiv_distance(geo, payload, drone.alpha, drone.beta)
    energy = equiv * drone.alpha
    return geo, equiv, energy


def _get_location(
    seq_id: str,
    targets_map: dict[str, Target],
    home: GeoPoint,
) -> GeoPoint:
    """获取 sequence id 对应的坐标."""
    if seq_id == "home":
        return home
    return targets_map[seq_id].location


def _state_at_position(
    sequence: list[str],
    pos: int,
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    total_demand: float,
) -> tuple[float, float, GeoPoint]:
    """计算到达 sequence[pos] 时的状态: (battery, payload, location).

    pos: 在 sequence 中的位置 (0-indexed).
    返回到达 pos 点后 (投递前) 的状态.
    如果 pos = len(sequence), 返回到达 home 的状态.
    """
    battery = drone.battery_capacity
    payload = total_demand
    current_loc = home

    for k in range(pos):
        tid = sequence[k]
        to_loc = targets_map[tid].location
        geo, equiv, energy = _segment_energy(current_loc, to_loc, payload, drone)
        battery -= energy
        payload -= targets_map[tid].demand
        current_loc = to_loc

    return battery, payload, current_loc


# --- 2-opt 增量评估 ---

def _try_2opt_move(
    sequence: list[str],
    i: int,
    j: int,
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    total_demand: float,
    current_total_equiv: float,
) -> tuple[list[str], float] | None:
    """尝试一个 2-opt 移动: 翻转 sequence[i+1:j+1].

    增量评估: 仅重算受影响段 (i→...→j+1), O(k) 而非 O(n).

    Args:
        sequence: 当前访问序列
        i: 翻转段前锚点位置 (0 ≤ i < j ≤ n-1)
        j: 翻转段最后位置
        current_total_equiv: 当前路线的总等效距离 (用于计算 delta)

    Returns:
        (new_sequence, new_total_equiv) 如果有改进, None 否则.
    """
    n = len(sequence)
    flipped = sequence[i + 1:j + 1][::-1]
    new_seq = sequence[:i + 1] + flipped + sequence[j + 1:]

    # 到达 s_i 时的状态 (投递后, 准备出发去下一个点)
    battery_i, payload_i, loc_i = _state_at_position(
        sequence, i + 1, targets_map, home, drone, total_demand
    )
    # payload_i 是 s_i 投递后剩余载重 = s_i→s_{i+1} 的 payload

    # 原路线受影响段: s_i→p1→...→pk→s_{j+1}
    # 新路线受影响段: s_i→pk→...→p1→s_{j+1}
    # 从 s_i 出发, 遍历 flipped 段 + s_{j+1} (或 home)
    old_affected_equiv = 0.0
    new_affected_equiv = 0.0
    new_feasible = True

    # --- 计算原路线受影响段的等价距离 ---
    old_payload = payload_i
    old_loc = loc_i
    old_pts = sequence[i + 1:j + 2] if j + 1 < n else sequence[i + 1:] + ["home"]
    # old_pts: [p1, p2, ..., pk, s_{j+1}] or [p1, ..., pk, "home"]
    for pt_id in old_pts:
        to_loc = _get_location(pt_id, targets_map, home)
        geo, equiv, energy = _segment_energy(old_loc, to_loc, old_payload, drone)
        old_affected_equiv += equiv
        old_loc = to_loc
        if pt_id != "home":
            old_payload -= targets_map[pt_id].demand

    # --- 计算新路线受影响段的等价距离 + 电池可行性 ---
    new_payload = payload_i
    new_loc = loc_i
    new_battery = battery_i
    new_pts = flipped + ([sequence[j + 1]] if j + 1 < n else ["home"])
    # new_pts: [pk, ..., p1, s_{j+1}] or [pk, ..., p1, "home"]
    for pt_id in new_pts:
        to_loc = _get_location(pt_id, targets_map, home)
        geo, equiv, energy = _segment_energy(new_loc, to_loc, new_payload, drone)
        new_affected_equiv += equiv
        new_battery -= energy
        if new_battery < -1e-10:  # 容忍微小浮点误差
            new_feasible = False
            break
        new_loc = to_loc
        if pt_id != "home":
            new_payload -= targets_map[pt_id].demand

    if not new_feasible:
        return None

    delta_equiv = new_affected_equiv - old_affected_equiv
    if delta_equiv < -1e-10:
        new_total = current_total_equiv + delta_equiv
        return new_seq, new_total

    return None


# --- Or-opt 增量评估 ---

def _try_or_opt_move(
    sequence: list[str],
    seg_start: int,
    seg_end: int,
    insert_pos: int,
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    total_demand: float,
    current_total_equiv: float,
) -> tuple[list[str], float] | None:
    """尝试一个 Or-opt 移动: 移动 segment[seg_start:seg_end] 插入到 insert_pos 后.

    移动 1-3 个连续点到新位置, 增量评估.

    Args:
        sequence: 当前访问序列
        seg_start: 移动段起始位置 (inclusive)
        seg_end: 移动段结束位置 (exclusive)
        insert_pos: 插入目标位置 (插入到 sequence[insert_pos] 之后)

    Returns:
        (new_sequence, new_total_equiv) 如果有改进, None 否则.
    """
    n = len(sequence)
    seg = sequence[seg_start:seg_end]

    # 构建新序列
    if insert_pos < seg_start:
        new_seq = (
            sequence[:insert_pos + 1]
            + seg
            + sequence[insert_pos + 1:seg_start]
            + sequence[seg_end:]
        )
    else:
        new_seq = (
            sequence[:seg_start]
            + sequence[seg_end:insert_pos + 1]
            + seg
            + sequence[insert_pos + 1:]
        )

    # 找到旧序列和新序列的第一个不同位置
    first_diff = 0
    while first_diff < min(n, len(new_seq)):
        if new_seq[first_diff] != sequence[first_diff]:
            break
        first_diff += 1

    if first_diff == min(n, len(new_seq)):
        return None  # 没有变化

    # 从 first_diff 前一个点开始计算
    start_pos = max(0, first_diff - 1)

    # 获取起始状态
    if start_pos == 0:
        battery_start, payload_start, loc_start = (
            drone.battery_capacity, total_demand, home
        )
    else:
        battery_start, payload_start, loc_start = _state_at_position(
            sequence, start_pos, targets_map, home, drone, total_demand
        )

    if first_diff == 0:
        battery_start, payload_start, loc_start = drone.battery_capacity, total_demand, home

    # 计算旧序列受影响段的 equiv
    old_equiv = 0.0
    old_payload = payload_start
    old_loc = loc_start

    if start_pos == 0 and first_diff == 0:
        old_path = ["home"] + sequence + ["home"]
        for idx in range(len(old_path) - 1):
            to_pt = _get_location(old_path[idx + 1], targets_map, home)
            geo, equiv, energy = _segment_energy(
                _get_location(old_path[idx], targets_map, home), to_pt, old_payload, drone
            )
            old_equiv += equiv
            old_loc = to_pt
            if old_path[idx + 1] != "home":
                old_payload -= targets_map[old_path[idx + 1]].demand
    else:
        for pt_id in sequence[start_pos:]:
            to_loc = _get_location(pt_id, targets_map, home)
            geo, equiv, energy = _segment_energy(old_loc, to_loc, old_payload, drone)
            old_equiv += equiv
            old_loc = to_loc
            if pt_id != "home":
                old_payload -= targets_map[pt_id].demand
        geo, equiv, energy = _segment_energy(old_loc, home, 0.0, drone)
        old_equiv += equiv

    # 计算新序列受影响段的 equiv
    new_equiv = 0.0
    new_payload = payload_start
    new_loc = loc_start
    new_battery = battery_start
    new_feasible = True

    if start_pos == 0 and first_diff == 0:
        new_path = ["home"] + new_seq + ["home"]
        for idx in range(len(new_path) - 1):
            to_pt = _get_location(new_path[idx + 1], targets_map, home)
            from_pt = _get_location(new_path[idx], targets_map, home)
            geo, equiv, energy = _segment_energy(from_pt, to_pt, new_payload, drone)
            new_equiv += equiv
            new_battery -= energy
            if new_battery < -1e-10:
                new_feasible = False
                break
            new_loc = to_pt
            if new_path[idx + 1] != "home":
                new_payload -= targets_map[new_path[idx + 1]].demand
    else:
        for pt_id in new_seq[start_pos:]:
            to_loc = _get_location(pt_id, targets_map, home)
            geo, equiv, energy = _segment_energy(new_loc, to_loc, new_payload, drone)
            new_equiv += equiv
            new_battery -= energy
            if new_battery < -1e-10:
                new_feasible = False
                break
            new_loc = to_loc
            if pt_id != "home":
                new_payload -= targets_map[pt_id].demand
        if new_feasible:
            geo, equiv, energy = _segment_energy(new_loc, home, 0.0, drone)
            new_equiv += equiv
            new_battery -= energy
            if new_battery < -1e-10:
                new_feasible = False

    if not new_feasible:
        return None

    delta_equiv = new_equiv - old_equiv
    if delta_equiv < -1e-10:
        new_total = current_total_equiv + delta_equiv
        return new_seq, new_total

    return None


# --- 搜索循环 ---

def local_search_2opt(
    route: RoutePlan,
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_iterations: int = 100,
) -> RoutePlan:
    """2-opt 局部搜索 — first-improvement 策略.

    翻转一段子路径, 增量评估电量可行性.

    算法:
      1. 对每对 (i, j), 尝试翻转 sequence[i+1:j+1]
      2. 增量计算受影响段的等价距离变化
      3. 一旦发现改进且可行, 立即接受 (first-improvement)
      4. 重复直到无改进或达到 max_iterations

    复杂度: 每次 iteration O(n³·k_avg) 增量, k_avg≈n/3.
    对于 n=20: 约 8000 次段评估/iteration, < 1ms.

    Args:
        route: 初始路线 (应 feasible=True)
        targets_map: id → Target 映射
        home: 仓库位置
        drone: 无人机规格
        max_iterations: 最大外层迭代次数

    Returns:
        RoutePlan: 改进后的路线
    """
    sequence = list(route.sequence)
    if len(sequence) < 2:
        return route  # 0 或 1 个点, 无需搜索

    total_demand = sum(t.demand for t in targets_map.values())
    current_total_equiv = route.total_equiv_distance

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1
        n = len(sequence)

        for i in range(n - 1):
            for j in range(i + 1, n):
                result = _try_2opt_move(
                    sequence, i, j, targets_map, home, drone,
                    total_demand, current_total_equiv,
                )
                if result is not None:
                    sequence, current_total_equiv = result
                    improved = True
                    break
            if improved:
                break

    return _build_route_plan(sequence, targets_map, home, drone)


def local_search_or_opt(
    route: RoutePlan,
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_segment_size: int = 3,
    max_iterations: int = 100,
) -> RoutePlan:
    """Or-opt 局部搜索 — first-improvement 策略.

    移动 1-3 个连续点到新位置, 增量评估.

    算法:
      1. 对每个 segment (size=1,2,3) 和每个插入位置
      2. 增量评估受影响段的等价距离变化
      3. 一旦发现改进且可行, 立即接受 (first-improvement)
      4. 重复直到无改进或达到 max_iterations

    复杂度: 每次 iteration O(3·n²·|seg|), n=20 时 ~3600 次段评估.

    Args:
        route: 初始路线
        targets_map: id → Target 映射
        home: 仓库位置
        drone: 无人机规格
        max_segment_size: 最大移动段长度 (默认 3)
        max_iterations: 最大外层迭代次数

    Returns:
        RoutePlan: 改进后的路线
    """
    sequence = list(route.sequence)
    if len(sequence) < 2:
        return route

    total_demand = sum(t.demand for t in targets_map.values())
    current_total_equiv = route.total_equiv_distance

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1
        n = len(sequence)

        for seg_len in range(1, max_segment_size + 1):
            if seg_len > n:
                continue
            for seg_start in range(n - seg_len + 1):
                seg_end = seg_start + seg_len
                for insert_pos in range(n - seg_len + 1):
                    if seg_start <= insert_pos < seg_end:
                        continue
                    result = _try_or_opt_move(
                        sequence, seg_start, seg_end, insert_pos,
                        targets_map, home, drone,
                        total_demand, current_total_equiv,
                    )
                    if result is not None:
                        sequence, current_total_equiv = result
                        improved = True
                        break
                if improved:
                    break
            if improved:
                break

    return _build_route_plan(sequence, targets_map, home, drone)


def local_search_vnd(
    route: RoutePlan,
    targets_map: dict[str, Target],
    home: GeoPoint,
    drone: DroneSpec,
    max_iterations: int = 100,
    max_segment_size: int = 3,
) -> RoutePlan:
    """Variable Neighborhood Descent — 2-opt + Or-opt 交替搜索.

    算法 (VND, Mladenović & Hansen 1997):
      1. 2-opt first-improvement 到底
      2. Or-opt first-improvement 到底
      3. 如果任一步有改进, 回到步骤 1
      4. 直到全局无改进 或 达到 max_iterations

    这是 W4 的主搜索入口, 在 plan_multistop 中调用.

    Args:
        route: 初始路线 (来自 construct_nn)
        targets_map: id → Target 映射
        home: 仓库位置
        drone: 无人机规格
        max_iterations: 最大 VND 外层迭代次数
        max_segment_size: Or-opt 最大移动段长度

    Returns:
        RoutePlan: 改进后的路线
    """
    if len(route.sequence) < 2:
        return route

    current = route
    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        # Phase 1: 2-opt
        candidate = local_search_2opt(current, targets_map, home, drone, max_iterations=100)
        if (candidate.feasible and
                candidate.total_equiv_distance < current.total_equiv_distance - 1e-10):
            current = candidate
            improved = True

        # Phase 2: Or-opt
        candidate = local_search_or_opt(
            current, targets_map, home, drone,
            max_segment_size=max_segment_size, max_iterations=100,
        )
        if (candidate.feasible and
                candidate.total_equiv_distance < current.total_equiv_distance - 1e-10):
            current = candidate
            improved = True

    return current
