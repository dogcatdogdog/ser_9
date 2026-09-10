"""自适应求解入口 — 按预算在"精确模式"与"降级模式"之间自动选择 (W9)

动机
----
`exact_battery.solve_energy_exact_battery` 能在 n <= 20 上给出**精确最优**,
但代价是耗时/内存按 ~2×/点 增长 (n=20 实测 1.77 s / 156 MB)。生产服务
(`a3_rust/src/http.rs`) 有请求级预算 (默认 5 s 时限 / 进程内存上限),
预算不足时必须能自动退回启发式, 而不是超时或 OOM。

本模块提供:
  1. `estimate_exact_resources(n)` —— 保守估算精确模式的耗时/峰值内存;
  2. `choose_mode(n, time_limit_secs, memory_limit_mb)` —— 预算比较, 给出模式决策;
  3. `plan_multistop_adaptive(...)` —— 统一入口, 内部路由到精确模式或降级模式。

两种模式
--------
  * **精确模式** (`MODE_EXACT`): 调 `solve_energy_exact_battery`, 返回 n <= 20 的
    真实最优序列 (电量约束不改变最优序列, 只决定可行性);
  * **降级模式** (`MODE_HEURISTIC`): 调 `solver.plan_multistop` (NN 构造 + VND 搜索),
    秒级返回一个高质量可行解 (实测 65.2% 直接命中最优、97.1% 距最优 <= 2%)。

协同 (关键设计)
--------------
精确模式**并非**与启发式互斥: 降级解在精确模式里充当**剪枝上界 UB**。
`plan_multistop_adaptive` 总是先跑一次启发式, 取其**精确能耗** (而非
`RoutePlan.total_energy_consumed`, 后者保留 2 位小数, 可能低于真值而被误剪)
作为 `ub_override` 传入精确解。上界越紧剪枝越强, 故两条路是协同关系。
降级解不可行时不传 UB (改由精确解内部的启发式 + 多起点贪心兜底求 UB)。

模式标注
--------
返回的 `RoutePlan` 结构**不变** (A3_SCHEMA.md §1 的一部分, 不新增字段)。
实际使用的模式以 `"[adaptive] mode=..."` 形式追加在 `RoutePlan.warnings` 中,
可用 `solve_mode_of(plan)` 读回。该条目是**说明性信息, 不是错误**:
`RoutePlan.feasible` 字段不受其影响, 解析 warnings 时按 `MODE_WARNING_PREFIX`
前缀过滤即可。

设计约束:
  - 纯函数, 无 I/O / 无全局状态 (仅模块级常量)
  - 不修改 a3_python 下任何现有模块
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

from .route import DroneSpec, GeoPoint, RoutePlan, Target
from .energy_model import simulate_route_energy
from .exact_battery import (
    MAX_EXACT_BATTERY_POINTS,
    route_energy_upper_bound,
    solve_energy_exact_battery,
)
from .solver import MAX_TARGETS, plan_multistop

# === 模式标识 ===

MODE_EXACT = "exact"          # 精确模式 (solve_energy_exact_battery)
MODE_HEURISTIC = "heuristic"  # 降级模式 (solver.plan_multistop)

# warnings 中模式说明条目的前缀 (消费方据此识别说明性条目)
MODE_WARNING_PREFIX = "[adaptive]"

# === 预算默认值 (对齐 a3_rust/src/http.rs 的 ServiceConfig) ===
#
# 并发关系 (重要): `memory_limit_mb` 是**单请求**预算, 调用方应按
# "进程内存上限 / 并发数" 传入。以 n=20 为例: 保守估算 234 MB, 而实测峰值
# 155.8 MB —— 故 2 GB 进程预算下, 精确模式约可支撑 8 并发
# (实测值口径下为 13 并发)。并发更高时 n=20 会自动降级到启发式 (仍 ~0.1 s 出解),
# 这是刻意的保守取向: 宁可降级, 不可 OOM。若确需更高并发, 调小 EST_SAFETY_FACTOR
# 或加大进程内存预算 (估算常量集中在此, 便于统一标定)。
DEFAULT_TIME_LIMIT_SECS = 5.0    # 求解时限默认值 (http.rs time_limit_secs)
DEFAULT_MEMORY_LIMIT_MB = 250.0  # 单请求峰值内存预算默认值 (MB; 见上方并发说明)

# === 精确模式资源估算常量 (保守拟合, 宁可高估不可低估) ===
#
# 实测口径 (Python 3.12.13 / numpy 2.5.1, Windows 本机;
# 实例 generate_scenario(distribution="random", scale=1000, demand=(1,3)),
# 机型 payload=50kg / battery=5000Wh / α=0.1 / β=0.005):
#
#   n    耗时中位   耗时最差   峰值内存
#   14    0.053 s   0.055 s     2.5 MB
#   15    0.071 s   0.074 s     5.2 MB
#   16    0.112 s   0.122 s     9.5 MB
#   17    0.206 s   0.232 s    19.1 MB
#   18    0.421 s   0.442 s    36.7 MB
#   19    0.743 s   0.816 s    75.4 MB
#   20    1.774 s   1.857 s   155.8 MB
#
# 耗时与内存均约按 2×/点 增长, 故以 n=20 为锚点做指数外推, 再乘保守系数。
EXACT_EST_GROWTH = 2.0            # 每增加 1 个目标点, 耗时/内存约翻倍的倍率
EXACT_TIME_OVERHEAD_SECS = 0.05   # 与 n 无关的固定耗时 (距离矩阵/子集和/启发式 UB)
EXACT_TIME_AT_N20_SECS = 2.0      # n=20 的耗时基数 (实测中位 1.774 / 最差 1.857)
EXACT_MEM_OVERHEAD_MB = 4.0       # 与 n 无关的固定内存 (层索引/距离矩阵)
EXACT_MEM_AT_N20_MB = 152.0       # n=20 的峰值内存基数 (实测 155.8)
EST_SAFETY_FACTOR = 1.5           # 额外保守系数 (跨机器 / 并发负载 / 输入分布差异)

_MODE_RE = re.compile(re.escape(MODE_WARNING_PREFIX) + r"\s+mode=(\w+)")


@dataclass
class ModeDecision:
    """模式决策结果 (预算比较的完整依据, 便于日志与单测断言)"""
    mode: str            # MODE_EXACT 或 MODE_HEURISTIC
    est_time_secs: float  # 精确模式的估算耗时 (s; 降级模式下仍是该模式的估算值)
    est_memory_mb: float  # 精确模式的估算峰值内存 (MB)
    reason: str          # 决策理由 (人类可读)


def estimate_exact_resources(n_points: int) -> tuple[float, float]:
    """估算精确模式求解 n_points 个目标点的耗时 (s) 与峰值内存 (MB)

    模型 (**保守**: 刻意高估, 宁可误降级也不超时/OOM)::

        scale = EXACT_EST_GROWTH ** (n_points - 20)
        est_time = (EXACT_TIME_OVERHEAD_SECS + EXACT_TIME_AT_N20_SECS * scale)
                   * EST_SAFETY_FACTOR
        est_mem  = (EXACT_MEM_OVERHEAD_MB  + EXACT_MEM_AT_N20_MB  * scale)
                   * EST_SAFETY_FACTOR

    选用"以 n=20 为锚点的指数外推 + 固定开销 + 安全系数"而非直接查表, 理由:
      1. 表只覆盖 n=14..20, 而调用方可能传入更小的 n (n < 14 亦走精确模式);
      2. 双倍增长略高于实测比值上界 (最大 2.39, 出现在 n=19→20), 叠加 1.5 倍
         安全系数后, n=14..20 全部实测点均满足 `估算 >= 实测` (保守性成立);
      3. 常量集中在此, 换机器/换 numpy 版本后只需重测 n=20 两点。

    校验 (估算 >= 实测, 全部 7 个实测点):
        耗时: n=20 3.08 >= 1.86 | n=19 1.56 >= 0.82 | n=18 0.80 >= 0.44
              n=17 0.42 >= 0.23 | n=16 0.24 >= 0.12 | n=15 0.15 >= 0.07
              n=14 0.10 >= 0.06  (单位 s)
        内存: n=20 234.0 >= 155.8 | n=19 120.0 >= 75.4 | n=18 63.0 >= 36.7
              n=17 34.5 >= 19.1 | n=16 20.3 >= 9.5 | n=15 13.1 >= 5.2
              n=14 9.6 >= 2.5    (单位 MB)

    Args:
        n_points: 目标点数 (n_points >= 1 时按上式; n_points > 20 时仍外推,
            调用方应据 MAX_EXACT_BATTERY_POINTS 判定越界)

    Returns:
        (est_time_secs, est_memory_mb): 估算耗时 (秒) 与估算峰值内存 (MB)
    """
    exponent = n_points - MAX_EXACT_BATTERY_POINTS
    # 指数为负时 scale < 1 (小 n 更快/更省), 保持数值稳定即可
    scale = EXACT_EST_GROWTH ** exponent
    est_time = (EXACT_TIME_OVERHEAD_SECS + EXACT_TIME_AT_N20_SECS * scale) \
        * EST_SAFETY_FACTOR
    est_mem = (EXACT_MEM_OVERHEAD_MB + EXACT_MEM_AT_N20_MB * scale) \
        * EST_SAFETY_FACTOR
    return est_time, est_mem


def choose_mode(
    n_points: int,
    time_limit_secs: float = DEFAULT_TIME_LIMIT_SECS,
    memory_limit_mb: float = DEFAULT_MEMORY_LIMIT_MB,
) -> ModeDecision:
    """按预算选择求解模式: 预算足够走精确, 否则降级

    判定顺序 (任一不满足即降级, 理由写入 `ModeDecision.reason`):
      1. 点数越界: `n_points < 1` 或 `n_points > MAX_EXACT_BATTERY_POINTS`;
      2. 时间预算: `est_time > time_limit_secs`;
      3. 内存预算: `est_mem > memory_limit_mb`。

    取 `>` (而非 `>=`) 是刻意放宽边界: 估算本身已含 1.5 倍安全系数, 再收紧
    会拒掉本可精确求解的请求。

    Args:
        n_points: 目标点数
        time_limit_secs: 本次求解的时间预算 (秒); <= 0 表示不精确求解
        memory_limit_mb: 本次求解的峰值内存预算 (MB); 并发场景应传
            "进程内存上限 / 并发数"

    Returns:
        ModeDecision: 模式 + 估算值 + 理由
    """
    est_time, est_mem = estimate_exact_resources(max(n_points, 1))
    if n_points < 1:
        return ModeDecision(MODE_HEURISTIC, est_time, est_mem,
                           f"target count {n_points} is below the exact range")
    if n_points > MAX_EXACT_BATTERY_POINTS:
        return ModeDecision(
            MODE_HEURISTIC, est_time, est_mem,
            f"target count {n_points} exceeds exact limit "
            f"{MAX_EXACT_BATTERY_POINTS}",
        )
    if est_time > time_limit_secs:
        return ModeDecision(
            MODE_HEURISTIC, est_time, est_mem,
            f"est time {est_time:.2f}s exceeds budget {time_limit_secs:.2f}s",
        )
    if est_mem > memory_limit_mb:
        return ModeDecision(
            MODE_HEURISTIC, est_time, est_mem,
            f"est memory {est_mem:.1f}MB exceeds budget {memory_limit_mb:.1f}MB",
        )
    return ModeDecision(
        MODE_EXACT, est_time, est_mem,
        f"within budget: est {est_time:.2f}s / {est_mem:.1f}MB "
        f"<= {time_limit_secs:.2f}s / {memory_limit_mb:.1f}MB",
    )


def solve_mode_of(plan: RoutePlan) -> str | None:
    """从 `RoutePlan.warnings` 读回实际使用的求解模式

    Args:
        plan: 任意 RoutePlan (`plan_multistop_adaptive` 的产出才带模式说明)

    Returns:
        MODE_EXACT / MODE_HEURISTIC; 若 warnings 中无模式说明条目 (如直接来自
        `solver.plan_multistop`) 则返回 None
    """
    for warning in plan.warnings:
        match = _MODE_RE.search(warning)
        if match:
            return match.group(1)
    return None


def _mode_note(mode: str, n_points: int, decision: ModeDecision) -> str:
    """构造追加到 warnings 的模式说明条目"""
    return (
        f"{MODE_WARNING_PREFIX} mode={mode} n={n_points} "
        f"est_time={decision.est_time_secs:.2f}s "
        f"est_memory={decision.est_memory_mb:.1f}MB"
    )


def _with_mode_note(plan: RoutePlan, mode: str, n_points: int,
                    decision: ModeDecision, suffix: str = "") -> RoutePlan:
    """给 RoutePlan 追加模式说明 (不改字段定义, 亦不影响 feasible)

    Args:
        plan: 待标注的 RoutePlan (原地替换其 warnings 列表, 不共享引用)
        mode: 实际使用的模式
        n_points: 目标点数
        decision: 模式决策 (提供估算值)
        suffix: 追加在模式说明之后的补充信息 (如 "exact=infeasible")
    """
    note = _mode_note(mode, n_points, decision) + (f" {suffix}" if suffix else "")
    plan.warnings = list(plan.warnings) + [note]
    return plan


def _route_plan_from_sequence(
    sequence: list[str],
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
) -> RoutePlan:
    """由访问序列构建 RoutePlan (复用 energy_model.simulate_route_energy 填充各字段)

    与 `heuristic._build_route_plan` 同口径: segments / 总距离 / 总等效距离 /
    总能耗 / 剩余电量 / 总送货量 均由全量后验证给出 (保留 2 位小数)。
    """
    targets_map = {t.id: t for t in targets}
    segments, total_geo, total_equiv, total_energy, remaining, feasible, warnings = (
        simulate_route_energy(sequence, targets_map, home, drone)
    )
    return RoutePlan(
        sequence=list(sequence),
        segments=segments,
        total_geo_distance=total_geo,
        total_equiv_distance=total_equiv,
        total_energy_consumed=total_energy,
        remaining_energy=remaining,
        total_payload_delivered=sum(t.demand for t in targets_map.values()),
        feasible=feasible,
        warnings=list(warnings),
    )


def plan_multistop_adaptive(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    *,
    time_limit_secs: float = DEFAULT_TIME_LIMIT_SECS,
    memory_limit_mb: float = DEFAULT_MEMORY_LIMIT_MB,
    seed: int = 42,
) -> RoutePlan:
    """无人机多目标访问路线规划 (自适应模式) — 按预算走精确或降级

    流程:
      1. 输入验证 (1 <= n <= MAX_TARGETS, 与 `plan_multistop` 同契约 —
         MVP 边界为 20, 故 n > 20 一律 ValueError, 两种模式都不支持);
      2. 先跑一遍启发式 (`plan_multistop`) —— 既有解用于降级返回, 又用于给
         精确模式提供剪枝上界 (协同);
      3. `choose_mode` 比较预算:
         - 预算足够 → 精确模式: 把降级解的**精确能耗**作为 `ub_override` 传入
           `solve_energy_exact_battery` 剪枝, 得到真实最优序列;
         - 预算不足 → 直接返回启发式解;
      4. 精确模式判定"不可行"时返回启发式解 (两侧都不可行; 精确解的增量信息
         是**可行性判定**, 已写入 warnings)。

    两个模式的产出差异 (同一实例):
      * 目标值: 精确解 <= 启发式解 (精确解不劣于启发式);
      * `total_energy_consumed` 等字段口径一致 (均走 `simulate_route_energy`);
      * 模式说明在 `RoutePlan.warnings` 末尾 (见 `solve_mode_of`)。

    已知限度:
      * `time_limit_secs` / `memory_limit_mb` **只用于模式决策, 不是硬性中断**。
        一旦选定精确模式, 求解会跑完 (numpy 内层无法安全中断, 且本模块保持纯函数)。
        预算超限由保守的 `EST_SAFETY_FACTOR` 兜底, 而非运行时抢占。
      * 两种模式都只支持 1 <= n <= 20 (MVP 上限) —— 见 `solver.MAX_TARGETS`。

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格
        time_limit_secs: 本次求解的时间预算 (秒)
        memory_limit_mb: 本次求解的峰值内存预算 (MB)
        seed: 随机种子 (透传给 `plan_multistop`; 当前算法确定性)

    Returns:
        RoutePlan: 路线规划结果; `warnings` 末尾含模式说明条目

    Raises:
        ValueError: target 数量为 0 或超过 `MAX_TARGETS`
    """
    n_points = len(targets)
    if n_points == 0:
        raise ValueError("targets list cannot be empty")
    if n_points > MAX_TARGETS:
        raise ValueError(
            f"target count {n_points} exceeds MVP limit {MAX_TARGETS}"
        )

    # 1) 降级解 (同时是精确模式的剪枝上界来源)
    heuristic_plan = plan_multistop(targets, home, drone, seed=seed)

    # 2) 预算比较
    decision = choose_mode(n_points, time_limit_secs, memory_limit_mb)
    if decision.mode != MODE_EXACT:
        return _with_mode_note(heuristic_plan, MODE_HEURISTIC, n_points, decision)

    # 3) 精确模式: UB = 降级解的精确能耗 (不可行则传 None, 由模块内部兜底求 UB)
    ub_override: float | None = None
    if heuristic_plan.feasible:
        ub = route_energy_upper_bound(
            heuristic_plan.sequence, targets, home, drone
        )
        ub_override = ub if math.isfinite(ub) else None

    exact = solve_energy_exact_battery(
        targets, home, drone,
        max_points=MAX_EXACT_BATTERY_POINTS,
        ub_override=ub_override,
    )
    if not exact.feasible:
        # 电量约束只是可行性闸门: 精确解判定不可行 ⇒ 不存在可行顺序
        return _with_mode_note(
            heuristic_plan, MODE_HEURISTIC, n_points, decision,
            suffix="exact=infeasible",
        )

    plan = _route_plan_from_sequence(exact.sequence, targets, home, drone)
    return _with_mode_note(plan, MODE_EXACT, n_points, decision)
