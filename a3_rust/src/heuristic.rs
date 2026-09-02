//! 构造启发式 + 局部搜索 — 对齐 Python `a3_python/heuristic.py`
//!
//! W7 实现:
//!   - NN 构造 (载重感知等效距离, N-start) + C-W Savings
//!   - 2-opt / Or-opt 局部搜索 (VND 交替, first-improvement)
//!   - 增量电量校验 (专利创新点 3: 仅重算受影响段, O(k) 而非 O(n))
//!
//! 不移植 Python 的 `full_eval` 对照版 — 那是 W5 benchmark 材料
//! (增量 vs 全量速度对比), 生产路径只用增量评估。
//!
//! 调研结论 (A3_RESEARCH_PLAN.md §R3/R4):
//!   R3.1: Solomon-style 约束 NN — cost=equiv_dist, 容量+电量双重检查
//!   R3.2: C-W Savings — s(i,j) = equiv(i→home) + equiv(home→j) − equiv(i→j)
//!   R3.3: N-start NN — n≤20 时 O(N×n²) 完全可承受
//!   R4.1~R4.4: 2-opt/Or-opt 增量公式, first-improvement, VND 交替

use std::collections::HashMap;

use crate::dto::{DroneSpecDto, GeoPointDto, RoutePlanResp, TargetDto};
use crate::energy::{equiv_distance, geo_distance, simulate_route_energy};

/// 构建 RoutePlan (封装 simulate_route_energy) — 对齐 Python `_build_route_plan`
pub(crate) fn build_route_plan(
    sequence: &[String],
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
) -> RoutePlanResp {
    let sim = simulate_route_energy(sequence, targets_map, home, drone);
    let total_payload: f64 = targets_map.values().map(|t| t.demand).sum();
    RoutePlanResp {
        sequence: sequence.to_vec(),
        segments: sim.segments,
        total_geo_distance: sim.total_geo,
        total_equiv_distance: sim.total_equiv,
        total_energy_consumed: sim.total_energy,
        remaining_energy: sim.remaining_energy,
        total_payload_delivered: total_payload,
        feasible: sim.feasible,
        warnings: sim.warnings,
    }
}

/// 检查从 from_loc 到 to_target 并返航 home 的可行性
///
/// 返回 (feasible, equiv_dist_to_target, energy_to_target, payload_after_target)
/// — 对齐 Python `_check_roundtrip_feasibility`
pub(crate) fn check_roundtrip_feasibility(
    from_loc: &GeoPointDto,
    to_target: &TargetDto,
    current_payload: f64,
    current_battery: f64,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
) -> (bool, f64, f64, f64) {
    let geo_to = geo_distance(from_loc, &to_target.location);
    let equiv_to = equiv_distance(geo_to, current_payload, drone.alpha, drone.beta);
    let energy_to = equiv_to * drone.alpha;

    let payload_after = current_payload - to_target.demand;
    let geo_home = geo_distance(&to_target.location, home);
    let equiv_home = equiv_distance(geo_home, payload_after, drone.alpha, drone.beta);
    let energy_home = equiv_home * drone.alpha;

    let feasible = current_battery >= energy_to + energy_home;
    (feasible, equiv_to, energy_to, payload_after)
}

/// 单起点 NN: 从 home 出发, 先访问 first_target, 然后贪心扩展
/// — 对齐 Python `_nn_single_start`
fn nn_single_start(
    targets: &[TargetDto],
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    total_demand: f64,
    first_target: &TargetDto,
) -> RoutePlanResp {
    let targets_map: HashMap<String, &TargetDto> =
        targets.iter().map(|t| (t.id.clone(), t)).collect();
    let n = targets.len();

    // 从 home 到 first_target
    let (feasible, _equiv_to, energy_to, payload_after) = check_roundtrip_feasibility(
        home,
        first_target,
        total_demand,
        drone.battery_capacity,
        home,
        drone,
    );

    if !feasible {
        // 第一个点就不可行, 返回空路线
        let mut plan = build_route_plan(&[], &targets_map, home, drone);
        plan.warnings.push(format!(
            "Cannot reach first target {} from home: \
             need {energy_to:.1}Wh roundtrip, battery={:.1}Wh",
            first_target.id, drone.battery_capacity
        ));
        plan.feasible = false;
        return plan;
    }

    let mut sequence = vec![first_target.id.clone()];
    let mut unvisited: Vec<&str> = targets.iter().map(|t| t.id.as_str()).collect();
    unvisited.retain(|id| *id != first_target.id);
    let mut current_loc = first_target.location;
    let mut current_payload = payload_after;
    let mut current_battery = drone.battery_capacity - energy_to;

    // 贪心扩展
    while !unvisited.is_empty() {
        let mut best_id: Option<&str> = None;
        let mut best_cost = f64::INFINITY;
        let mut best_energy = 0.0;
        let mut best_payload_after = current_payload;

        for t in targets {
            if unvisited.contains(&t.id.as_str()) {
                let (feasible_t, equiv_t, energy_t, payload_t) =
                    check_roundtrip_feasibility(
                        &current_loc,
                        t,
                        current_payload,
                        current_battery,
                        home,
                        drone,
                    );
                if feasible_t && equiv_t < best_cost {
                    best_cost = equiv_t;
                    best_id = Some(&t.id);
                    best_energy = energy_t;
                    best_payload_after = payload_t;
                }
            }
        }

        let Some(best_id) = best_id else {
            // 无可行下一跳: 返回部分路线
            let mut plan = build_route_plan(&sequence, &targets_map, home, drone);
            plan.warnings.push(format!(
                "No feasible next stop after {}: visited {}/{} targets",
                sequence.last().unwrap(),
                sequence.len(),
                n
            ));
            plan.feasible = false;
            return plan;
        };

        sequence.push(best_id.to_string());
        unvisited.retain(|id| *id != best_id);
        current_loc = targets_map[best_id].location;
        current_payload = best_payload_after;
        current_battery -= best_energy;
    }

    build_route_plan(&sequence, &targets_map, home, drone)
}

/// 电量感知最近邻构造 — N-start 变体 (对齐 Python `construct_nn`)
///
/// 从每个目标点作为第一个访问点各跑一次 NN, 取最优可行解。
/// 如果所有起点都不可行, 返回访问点数最多的部分路线。
pub(crate) fn construct_nn(
    targets: &[TargetDto],
    home: &GeoPointDto,
    drone: &DroneSpecDto,
) -> RoutePlanResp {
    assert!(!targets.is_empty(), "targets list cannot be empty");

    let total_demand: f64 = targets.iter().map(|t| t.demand).sum();

    // 快速载重检查
    if total_demand > drone.payload_capacity {
        return RoutePlanResp {
            sequence: Vec::new(),
            segments: Vec::new(),
            total_geo_distance: 0.0,
            total_equiv_distance: 0.0,
            total_energy_consumed: 0.0,
            remaining_energy: drone.battery_capacity,
            total_payload_delivered: 0.0,
            feasible: false,
            warnings: vec![format!(
                "Total payload ({total_demand:.1}kg) exceeds drone capacity ({:.1}kg)",
                drone.payload_capacity
            )],
        };
    }

    // 单点特例: 只需检查 home→target→home
    if targets.len() == 1 {
        return nn_single_start(targets, home, drone, total_demand, &targets[0]);
    }

    // N-start: 尝试每个 target 作为第一个访问点
    let mut best_plan: Option<RoutePlanResp> = None;
    let mut best_visited = 0;

    for first in targets {
        let plan = nn_single_start(targets, home, drone, total_demand, first);

        if plan.feasible {
            // 可行解: 选 total_equiv_distance 最小的
            if best_plan
                .as_ref()
                .is_none_or(|p| plan.total_equiv_distance < p.total_equiv_distance)
            {
                best_plan = Some(plan);
            }
        } else {
            // 不可行: 记录访问点数最多的 (用于降级返回)
            if plan.sequence.len() > best_visited {
                best_visited = plan.sequence.len();
                if best_plan.as_ref().is_none_or(|p| !p.feasible) {
                    best_plan = Some(plan);
                }
            }
        }
    }

    match best_plan {
        Some(plan) => plan,
        None => {
            // 不应该到这里, 但安全起见返回空计划
            let targets_map: HashMap<String, &TargetDto> =
                targets.iter().map(|t| (t.id.clone(), t)).collect();
            build_route_plan(&[], &targets_map, home, drone)
        }
    }
}

/// Clarke-Wright Savings 改造版 — 载重感知的节约算法
/// (对齐 Python `construct_savings`, solver 主路径未用, 测试覆盖)
pub fn construct_savings(
    targets: &[TargetDto],
    home: &GeoPointDto,
    drone: &DroneSpecDto,
) -> RoutePlanResp {
    assert!(!targets.is_empty(), "targets list cannot be empty");

    let targets_map: HashMap<String, &TargetDto> =
        targets.iter().map(|t| (t.id.clone(), t)).collect();
    let total_demand: f64 = targets.iter().map(|t| t.demand).sum();

    // 快速载重检查
    if total_demand > drone.payload_capacity {
        return RoutePlanResp {
            sequence: Vec::new(),
            segments: Vec::new(),
            total_geo_distance: 0.0,
            total_equiv_distance: 0.0,
            total_energy_consumed: 0.0,
            remaining_energy: drone.battery_capacity,
            total_payload_delivered: 0.0,
            feasible: false,
            warnings: vec![format!(
                "Total payload ({total_demand:.1}kg) exceeds drone capacity ({:.1}kg)",
                drone.payload_capacity
            )],
        };
    }

    let n = targets.len();

    // 单点特例
    if n == 1 {
        let sequence = vec![targets[0].id.clone()];
        return build_route_plan(&sequence, &targets_map, home, drone);
    }

    // Step 1: 每个点独立路线 (route key = 该点 id)
    // routes: (key, 访问顺序) — 用 Vec 保持插入序, 对齐 Python dict 的迭代序
    // (HashMap 无序会导致 final_sequence 拼接顺序与 Python 不一致)
    let mut routes: Vec<(String, Vec<String>)> =
        targets.iter().map(|t| (t.id.clone(), vec![t.id.clone()])).collect();
    // node_to_route: 每个 node 属于哪条路线
    let mut node_to_route: HashMap<String, String> =
        targets.iter().map(|t| (t.id.clone(), t.id.clone())).collect();

    // Step 2: 计算 savings
    let mut savings_list: Vec<(f64, String, String)> = Vec::new();
    for (i, t_i) in targets.iter().enumerate() {
        for (j, t_j) in targets.iter().enumerate() {
            if i >= j {
                continue;
            }
            // s(i,j) = equiv(i→home, P−dᵢ) + equiv(home→j, P) − equiv(i→j, P−dᵢ)
            let payload_i = total_demand - t_i.demand;
            let geo_ih = geo_distance(&t_i.location, home);
            let geo_hj = geo_distance(home, &t_j.location);
            let geo_ij = geo_distance(&t_i.location, &t_j.location);

            let equiv_ih = equiv_distance(geo_ih, payload_i, drone.alpha, drone.beta);
            let equiv_hj = equiv_distance(geo_hj, total_demand, drone.alpha, drone.beta);
            let equiv_ij = equiv_distance(geo_ij, payload_i, drone.alpha, drone.beta);

            let saving = equiv_ih + equiv_hj - equiv_ij;
            if saving > 0.0 {
                savings_list.push((saving, t_i.id.clone(), t_j.id.clone()));
            }
        }
    }

    // Step 3: 按 saving 降序排列 (稳定排序, 与 Python sort 一致)
    savings_list.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

    // Step 4: 贪心合并
    for (_, i_id, j_id) in &savings_list {
        let Some(route_i_key) = node_to_route.get(i_id).cloned() else {
            continue;
        };
        let Some(route_j_key) = node_to_route.get(j_id).cloned() else {
            continue;
        };
        if route_i_key == route_j_key {
            continue;
        }
        let Some(route_i) = routes.iter().find(|(k, _)| *k == route_i_key) else {
            continue;
        };
        let Some(route_j) = routes.iter().find(|(k, _)| *k == route_j_key) else {
            continue;
        };

        // i 必须在 route_i 末尾, j 必须在 route_j 开头
        if route_i.1.last() != Some(i_id) || route_j.1.first() != Some(j_id) {
            continue;
        }

        // 尝试合并 + 全量可行性检查
        let merged: Vec<String> =
            route_i.1.iter().chain(route_j.1.iter()).cloned().collect();
        let sim = simulate_route_energy(&merged, &targets_map, home, drone);

        if sim.feasible {
            // 对齐 Python: del routes[i]; routes[j] = merged (保持插入序)
            let idx_j = routes
                .iter()
                .position(|(k, _)| *k == route_j_key)
                .expect("route_j 存在");
            routes[idx_j].1 = merged.clone();
            let idx_i = routes
                .iter()
                .position(|(k, _)| *k == route_i_key)
                .expect("route_i 存在");
            routes.remove(idx_i);
            for node_id in &merged {
                node_to_route.insert(node_id.clone(), route_j_key.clone());
            }
        }
    }

    // Step 5: 收集合并后的路线
    if routes.is_empty() {
        return build_route_plan(&[], &targets_map, home, drone);
    }

    let final_sequence: Vec<String> = routes.iter().flat_map(|(_, seq)| seq.iter()).cloned().collect();
    let mut plan = build_route_plan(&final_sequence, &targets_map, home, drone);

    // 如果有 >1 条路线未合并, 记录警告
    if routes.len() > 1 {
        plan.warnings
            .push(format!("Savings: {} routes remain unmerged", routes.len()));
    }

    plan
}

// ====================================================================
// W4 局部搜索 — 2-opt + Or-opt + VND (增量评估, 专利创新点 3)
// ====================================================================

/// 计算 full_path 各段出发时载重 — 对齐 Python `_segment_payloads`
///
/// full_path = ["home"] + sequence + ["home"], 共 n+1 段。
/// payloads[k] = 第 k 段出发时载重 (k=0..n)。
pub fn segment_payloads(
    sequence: &[String],
    targets_map: &HashMap<String, &TargetDto>,
    total_demand: f64,
) -> Vec<f64> {
    let n = sequence.len();
    let mut payloads = vec![0.0; n + 1];
    let mut remaining = total_demand;
    for (k, tid) in sequence.iter().enumerate() {
        payloads[k] = remaining;
        remaining -= targets_map[tid].demand;
    }
    payloads[n] = 0.0; // 返航段
    payloads
}

/// 计算单段能耗: (geo, equiv, energy) — 对齐 Python `_segment_energy`
pub(crate) fn segment_energy(
    from_point: &GeoPointDto,
    to_point: &GeoPointDto,
    payload: f64,
    drone: &DroneSpecDto,
) -> (f64, f64, f64) {
    let geo = geo_distance(from_point, to_point);
    let equiv = equiv_distance(geo, payload, drone.alpha, drone.beta);
    let energy = equiv * drone.alpha;
    (geo, equiv, energy)
}

/// 获取 sequence id 对应的坐标 — 对齐 Python `_get_location`
pub(crate) fn get_location<'a>(
    seq_id: &str,
    targets_map: &'a HashMap<String, &TargetDto>,
    home: &'a GeoPointDto,
) -> &'a GeoPointDto {
    if seq_id == "home" {
        home
    } else {
        &targets_map[seq_id].location
    }
}

/// 计算到达 sequence[pos] 时的状态: (battery, payload, location)
/// — 对齐 Python `_state_at_position`
///
/// pos: 在 sequence 中的位置 (0-indexed)。返回到达 pos 点时
/// (从 sequence[pos-1] 出发、投递 sequence[pos-1] 后) 的状态;
/// pos = sequence.len() 时返回到达 home 的状态。
pub(crate) fn state_at_position(
    sequence: &[String],
    pos: usize,
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    total_demand: f64,
) -> (f64, f64, GeoPointDto) {
    let mut battery = drone.battery_capacity;
    let mut payload = total_demand;
    let mut current_loc = *home;

    for tid in sequence.iter().take(pos) {
        let to_loc = targets_map[tid].location;
        let (_, _, energy) = segment_energy(&current_loc, &to_loc, payload, drone);
        battery -= energy;
        payload -= targets_map[tid].demand;
        current_loc = to_loc;
    }

    (battery, payload, current_loc)
}

/// 尝试一个 2-opt 移动: 翻转 sequence[i+1:j+1] — 对齐 Python `_try_2opt_move`
///
/// 增量评估: 仅重算受影响段 (i→...→j+1), O(k) 而非 O(n)。
/// 返回 (new_sequence, new_total_equiv) 如果有改进, None 否则。
#[allow(clippy::too_many_arguments)]
pub(crate) fn try_2opt_move(
    sequence: &[String],
    i: usize,
    j: usize,
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    total_demand: f64,
    current_total_equiv: f64,
) -> Option<(Vec<String>, f64)> {
    let n = sequence.len();
    let mut flipped: Vec<String> = sequence[i + 1..=j].to_vec();
    flipped.reverse();
    let mut new_seq: Vec<String> = sequence[..=i].to_vec();
    new_seq.extend(flipped.iter().cloned());
    new_seq.extend(sequence[j + 1..].iter().cloned());

    // 到达 s_i 时的状态 (投递后, 准备出发去下一个点)
    let (battery_i, payload_i, loc_i) =
        state_at_position(sequence, i + 1, targets_map, home, drone, total_demand);

    // --- 计算原路线受影响段的等价距离 ---
    let mut old_affected_equiv = 0.0;
    let mut old_payload = payload_i;
    let mut old_loc = loc_i;
    // old_pts: [p1, ..., pk, s_{j+1}] 或 [p1, ..., pk, "home"]
    let mut old_pts: Vec<&str> = sequence[i + 1..].iter().map(|s| s.as_str()).collect();
    if j + 1 < n {
        old_pts.truncate(j + 2 - (i + 1)); // 保留到 s_{j+1} 含 (索引 j+1)
    } else {
        old_pts.push("home");
    }
    for pt_id in &old_pts {
        let to_loc = *get_location(pt_id, targets_map, home);
        let (_, equiv, _) = segment_energy(&old_loc, &to_loc, old_payload, drone);
        old_affected_equiv += equiv;
        old_loc = to_loc;
        if *pt_id != "home" {
            old_payload -= targets_map[*pt_id].demand;
        }
    }

    // --- 计算新路线受影响段的等价距离 + 电池可行性 ---
    let mut new_affected_equiv = 0.0;
    let mut new_payload = payload_i;
    let mut new_loc = loc_i;
    let mut new_battery = battery_i;
    let mut new_feasible = true;
    // new_pts: [pk, ..., p1, s_{j+1}] 或 [pk, ..., p1, "home"]
    let mut new_pts: Vec<&str> = flipped.iter().map(|s| s.as_str()).collect();
    if j + 1 < n {
        new_pts.push(sequence[j + 1].as_str());
    } else {
        new_pts.push("home");
    }
    for pt_id in &new_pts {
        let to_loc = *get_location(pt_id, targets_map, home);
        let (_, equiv, energy) = segment_energy(&new_loc, &to_loc, new_payload, drone);
        new_affected_equiv += equiv;
        new_battery -= energy;
        if new_battery < -1e-10 {
            // 容忍微小浮点误差
            new_feasible = false;
            break;
        }
        new_loc = to_loc;
        if *pt_id != "home" {
            new_payload -= targets_map[*pt_id].demand;
        }
    }

    if !new_feasible {
        return None;
    }

    let delta_equiv = new_affected_equiv - old_affected_equiv;
    if delta_equiv < -1e-10 {
        return Some((new_seq, current_total_equiv + delta_equiv));
    }
    None
}

/// 尝试一个 Or-opt 移动: 移动 segment[seg_start:seg_end] 插入到 insert_pos 后
/// — 对齐 Python `_try_or_opt_move`
///
/// 增量评估, 返回 (new_sequence, new_total_equiv) 如果有改进, None 否则。
#[allow(clippy::too_many_arguments)]
pub(crate) fn try_or_opt_move(
    sequence: &[String],
    seg_start: usize,
    seg_end: usize,
    insert_pos: usize,
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    total_demand: f64,
    current_total_equiv: f64,
) -> Option<(Vec<String>, f64)> {
    let n = sequence.len();
    let seg = &sequence[seg_start..seg_end];

    // 构建新序列
    let new_seq: Vec<String> = if insert_pos < seg_start {
        sequence[..=insert_pos]
            .iter()
            .chain(seg.iter())
            .chain(sequence[insert_pos + 1..seg_start].iter())
            .chain(sequence[seg_end..].iter())
            .cloned()
            .collect()
    } else {
        sequence[..seg_start]
            .iter()
            .chain(sequence[seg_end..=insert_pos].iter())
            .chain(seg.iter())
            .chain(sequence[insert_pos + 1..].iter())
            .cloned()
            .collect()
    };

    // 找到旧序列和新序列的第一个不同位置
    let mut first_diff = 0;
    while first_diff < n.min(new_seq.len()) {
        if new_seq[first_diff] != sequence[first_diff] {
            break;
        }
        first_diff += 1;
    }

    if first_diff == n.min(new_seq.len()) {
        return None; // 没有变化
    }

    // 从 first_diff 前一个点开始计算
    let start_pos = first_diff.saturating_sub(1);

    // 获取起始状态
    let (mut battery_start, mut payload_start, mut loc_start) = if start_pos == 0 {
        (drone.battery_capacity, total_demand, *home)
    } else {
        state_at_position(sequence, start_pos, targets_map, home, drone, total_demand)
    };

    if first_diff == 0 {
        battery_start = drone.battery_capacity;
        payload_start = total_demand;
        loc_start = *home;
    }

    // --- 计算旧序列受影响段的 equiv ---
    let mut old_equiv = 0.0;
    let mut old_payload = payload_start;
    let mut old_loc = loc_start;

    if start_pos == 0 && first_diff == 0 {
        let mut old_path: Vec<&str> = vec!["home"];
        old_path.extend(sequence.iter().map(|s| s.as_str()));
        old_path.push("home");
        for w in old_path.windows(2) {
            let from_pt = *get_location(w[0], targets_map, home);
            let to_pt = *get_location(w[1], targets_map, home);
            let (_, equiv, _) = segment_energy(&from_pt, &to_pt, old_payload, drone);
            old_equiv += equiv;
            if w[1] != "home" {
                old_payload -= targets_map[w[1]].demand;
            }
        }
    } else {
        for pt_id in sequence[start_pos..].iter() {
            let to_loc = *get_location(pt_id, targets_map, home);
            let (_, equiv, _) = segment_energy(&old_loc, &to_loc, old_payload, drone);
            old_equiv += equiv;
            old_loc = to_loc;
            if pt_id != "home" {
                old_payload -= targets_map[pt_id.as_str()].demand;
            }
        }
        let (_, equiv, _) = segment_energy(&old_loc, home, 0.0, drone);
        old_equiv += equiv;
    }

    // --- 计算新序列受影响段的 equiv + 电池可行性 ---
    let mut new_equiv = 0.0;
    let mut new_payload = payload_start;
    let mut new_loc = loc_start;
    let mut new_battery = battery_start;
    let mut new_feasible = true;

    if start_pos == 0 && first_diff == 0 {
        let mut new_path: Vec<&str> = vec!["home"];
        new_path.extend(new_seq.iter().map(|s| s.as_str()));
        new_path.push("home");
        for w in new_path.windows(2) {
            let from_pt = *get_location(w[0], targets_map, home);
            let to_pt = *get_location(w[1], targets_map, home);
            let (_, equiv, energy) = segment_energy(&from_pt, &to_pt, new_payload, drone);
            new_equiv += equiv;
            new_battery -= energy;
            if new_battery < -1e-10 {
                new_feasible = false;
                break;
            }
            if w[1] != "home" {
                new_payload -= targets_map[w[1]].demand;
            }
        }
    } else {
        for pt_id in new_seq[start_pos..].iter() {
            let to_loc = *get_location(pt_id, targets_map, home);
            let (_, equiv, energy) = segment_energy(&new_loc, &to_loc, new_payload, drone);
            new_equiv += equiv;
            new_battery -= energy;
            if new_battery < -1e-10 {
                new_feasible = false;
                break;
            }
            new_loc = to_loc;
            if pt_id != "home" {
                new_payload -= targets_map[pt_id.as_str()].demand;
            }
        }
        if new_feasible {
            let (_, equiv, energy) = segment_energy(&new_loc, home, 0.0, drone);
            new_equiv += equiv;
            new_battery -= energy;
            if new_battery < -1e-10 {
                new_feasible = false;
            }
        }
    }

    if !new_feasible {
        return None;
    }

    let delta_equiv = new_equiv - old_equiv;
    if delta_equiv < -1e-10 {
        return Some((new_seq, current_total_equiv + delta_equiv));
    }
    None
}

/// 2-opt 局部搜索 — first-improvement 策略 (对齐 Python `local_search_2opt`)
///
/// 复杂度: 每次 iteration O(n³·k_avg) 增量, k_avg≈n/3; n=20 时 < 1ms。
pub(crate) fn local_search_2opt(
    route: &RoutePlanResp,
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    max_iterations: usize,
) -> RoutePlanResp {
    let mut sequence = route.sequence.clone();
    if sequence.len() < 2 {
        return route.clone(); // 0 或 1 个点, 无需搜索
    }

    let total_demand: f64 = targets_map.values().map(|t| t.demand).sum();
    let mut current_total_equiv = route.total_equiv_distance;

    let mut improved = true;
    let mut iteration = 0;

    while improved && iteration < max_iterations {
        improved = false;
        iteration += 1;
        let n = sequence.len();

        'outer: for i in 0..n - 1 {
            for j in i + 1..n {
                if let Some((new_seq, new_equiv)) = try_2opt_move(
                    &sequence,
                    i,
                    j,
                    targets_map,
                    home,
                    drone,
                    total_demand,
                    current_total_equiv,
                ) {
                    sequence = new_seq;
                    current_total_equiv = new_equiv;
                    improved = true;
                    break 'outer;
                }
            }
        }
    }

    build_route_plan(&sequence, targets_map, home, drone)
}

/// Or-opt 局部搜索 — first-improvement 策略 (对齐 Python `local_search_or_opt`)
pub(crate) fn local_search_or_opt(
    route: &RoutePlanResp,
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    max_segment_size: usize,
    max_iterations: usize,
) -> RoutePlanResp {
    let mut sequence = route.sequence.clone();
    if sequence.len() < 2 {
        return route.clone();
    }

    let total_demand: f64 = targets_map.values().map(|t| t.demand).sum();
    let mut current_total_equiv = route.total_equiv_distance;

    let mut improved = true;
    let mut iteration = 0;

    while improved && iteration < max_iterations {
        improved = false;
        iteration += 1;
        let n = sequence.len();

        'outer: for seg_len in 1..=max_segment_size.min(n) {
            for seg_start in 0..=n - seg_len {
                let seg_end = seg_start + seg_len;
                for insert_pos in 0..=n - seg_len {
                    if (seg_start..seg_end).contains(&insert_pos) {
                        continue;
                    }
                    if let Some((new_seq, new_equiv)) = try_or_opt_move(
                        &sequence,
                        seg_start,
                        seg_end,
                        insert_pos,
                        targets_map,
                        home,
                        drone,
                        total_demand,
                        current_total_equiv,
                    ) {
                        sequence = new_seq;
                        current_total_equiv = new_equiv;
                        improved = true;
                        break 'outer;
                    }
                }
            }
        }
    }

    build_route_plan(&sequence, targets_map, home, drone)
}

/// Variable Neighborhood Descent — 2-opt + Or-opt 交替搜索
/// (对齐 Python `local_search_vnd`, Mladenović & Hansen 1997)
///
/// W4 主搜索入口, 在 plan_multistop 中调用。
/// W8 硬化: `deadline` 非 None 时每轮迭代前检查时限, 超时提前返回当前最优。
/// 正常求解远快于时限时不触发, 不影响输出确定性。
pub(crate) fn local_search_vnd(
    route: &RoutePlanResp,
    targets_map: &HashMap<String, &TargetDto>,
    home: &GeoPointDto,
    drone: &DroneSpecDto,
    max_iterations: usize,
    max_segment_size: usize,
    deadline: Option<std::time::Instant>,
) -> RoutePlanResp {
    if route.sequence.len() < 2 {
        return route.clone();
    }

    let mut current = route.clone();
    let mut improved = true;
    let mut iteration = 0;

    while improved && iteration < max_iterations {
        if deadline.is_some_and(|d| std::time::Instant::now() >= d) {
            break; // 超时: 返回当前最优解 (time_limit_secs 语义)
        }
        improved = false;
        iteration += 1;

        // Phase 1: 2-opt
        let candidate = local_search_2opt(&current, targets_map, home, drone, 100);
        if candidate.feasible
            && candidate.total_equiv_distance < current.total_equiv_distance - 1e-10
        {
            current = candidate;
            improved = true;
        }

        // Phase 2: Or-opt
        let candidate =
            local_search_or_opt(&current, targets_map, home, drone, max_segment_size, 100);
        if candidate.feasible
            && candidate.total_equiv_distance < current.total_equiv_distance - 1e-10
        {
            current = candidate;
            improved = true;
        }
    }

    current
}
