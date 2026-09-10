# A3 · 开发计划

> 版本: v1.1 | 日期: 2026-08-05

---

## 目标

**MVP 交付 (W1-W9)**: Python 验证 → Rust 落地 → 专利交底书初稿
**交付定稿 (W10-W13)**: 专利交底书（可申报）为唯一交付物 —
  材料策略: 全程储备支撑专利实施例与有益效果的量化数据（对比/消融/扩展），
  尽量准备充分、冗余只多不少；不做论文级实验（显著性检验等，结果达不到
  论文要求则无需）（8/19 定稿）

## 总览

```
阶段一: 骨架 (W1)          阶段二: Python (W2-W5)       阶段三: Rust (W6-W9)
┌──────────────┐      ┌──────────────────────────┐      ┌──────────────────────────┐
│ 空函数 +     │      │ D1: 等效距离模型          │      │ W6: Rust crate 骨架      │
│ 3 单测 +     │ ───→ │ D2: 基线求解(PyVRP对比)   │ ───→ │ W7: 纯函数 + 单测 + svc  │
│ 目录结构     │      │ D3: 构造启发式(NN)        │      │ W8: 专利交底书初稿       │
│              │      │ D4-5: 局部搜索(2opt/Or)   │      │ W9: 初稿验收             │
└──────────────┘      │ D6: 评测 + 消融实验       │      └──────────────────────────┘
                      │ D7: 单测补充 + 指标表     │
                      └──────────────────────────┘
```

---

## W1: 骨架 — 空函数 + 目录结构 + 3 单测

### D1-D2 (8/5-8/6): 文档确立 ✅

- [x] CLAUDE.md, A3_REQUIREMENTS.md, A3_ARCHITECTURE.md, A3_SCHEMA.md, A3_DEVPLAN.md

### D3 (8/6): 空函数骨架 ✅

- [x] 创建 `a3_python/` 目录结构 (route.py / solver.py / energy_model.py / heuristic.py / benchmark.py)
- [x] 实现 `solver.py::plan_multistop()` — 输入验证 + 按输入顺序构造路线 + 能耗模拟
- [x] 实现 `energy_model.py` — 几何距离 + 等效距离变换 + 路线电量模拟 (核心公式已实现)
- [x] 24 单测 (W1 骨架时计数, 正例/退化/边界/一致性) — 全部通过

---

## W2-W5: 初稿·Python

### W2 (8/8-8/14): 数据管线 + 基线 ✅ (8/5 提前完成)

**D1-D2: energy_model.py**
- [x] 实现 `compute_geo_matrix(locations) -> np.ndarray`
- [x] 实现 `compute_equiv_distance(i, j, payload, alpha, beta) -> float`
- [x] 实现 `simulate_route_energy(sequence, targets, home, drone) -> list[Segment]`
- [x] 实现 `compute_equiv_matrix(geo_matrix, demands, alpha, beta) -> np.ndarray`
- [x] 单测: 空载 vs 满载，等效距离对比 (21 例通过)

**D3-D4: 基线求解 (PyVRP)**
- [x] 用 PyVRP 在 5/10/20 点数据集上求解（无电量约束）— `a3_python/baseline.py`
- [x] 记录 baseline 指标（总距离、求解时间）— `BaselineResult` dataclass
- [x] 确认 PyVRP 能稳定求解 — 全部 7 个实例 feasible=True

**D5-D7: 数据管线**
- [x] Solomon VRPTW 标准测试集: R101/C101/RC101 前 20 点 (fixtures/)
- [x] 自建手写场景: custom_5_heavy (重载) / custom_10_tight (电量紧) / custom_15_mixed (联合) (fixtures/)
- [x] 程序化数据生成器: `a3_python/data_generator.py` (5 种分布, 为 W5 消融预建)
- [x] 无人机参数配置表（3 种机型，不同 α/β）— `DRONE_PRESETS` in route.py

> **数据策略**: fixtures 用于集成测试 + benchmark (固定场景, 可复现对比);
> data_generator 用于 W5 消融实验 (系统性变化 N/分布/demand, 画 scaling curve)。

### W3 (8/15-8/21): 构造启发式 ✅ (8/5 提前完成)

**D1-D3: 最近邻构造**
- [x] 实现电量感知 NN: `construct_nn(targets, home, drone) -> RoutePlan`
- [x] 每次选择 equiv_dist 最小且满足剩余电量约束的点
- [x] 不可行时返回部分路线 + 不可行标记

**D4-D5: Savings 构造**
- [x] 实现 Clarke-Wright Savings 改造版
- [x] saving(i,j) = equiv_dist(home,i) + equiv_dist(home,j) - equiv_dist(i,j)
- [x] 按 saving 从大到小合并路线，每次合并检查电量可行性

**D6-D7: 对比与选择**
- [x] NN vs Savings 在自建 5/10/20 点上对比 (快速迭代)
- [x] NN vs Savings 在 Solomon R101/C101/RC101 前 20 点上对比 (标准实例验证)
- [x] 选较优者作为主构造方法
- [x] 单测: 验证构造解满足载重 + 电量约束

### W4 (8/22-8/28): 局部搜索 ✅ (8/6 提前完成)

**D1-D2: 2-opt**
- [x] 实现 2-opt 翻转操作
- [x] 实现增量电量评估（只重算受影响段，不做全局 O(n) 重算）
- [x] 单测: 验证 2-opt 后的路线仍然可行

**D3-D4: Or-opt**
- [x] 实现 Or-opt (移动 1/2/3 个连续点)
- [x] 增量评估 + 可行性检查

**D5-D7: 搜索框架**
- [x] 实现 first-improvement 搜索循环
- [x] 最大迭代次数 / 时间上限
- [x] 单测: 搜索后的解不比初始解差

### W5 (8/29-9/4): 月1中检 ✅ (8/18 提前完成)

**D1-D3: 评测**
- [x] vs OR-Tools (≤15 点精确解): 在自建 5/10/15 点上计算 gap
      — `exact.py`: CP-SAT 电路约束精确解 (n≤20, 与 Held-Karp 一致) + 能量感知 DP 精确最优 (n≤15)
      — 双基线: gap_geo (几何) + gap_energy (同能量模型公平比较, 实测多为负 = 我们更省电)
- [x] vs PyVRP (无电量约束版): 同时跑自建数据和 Solomon R101/C101/RC101, 对比解差异
      — `baseline.py`: solve_tsp_pyvrp 支持 bks_cost → gap_vs_best (W2 预留字段激活)
- [x] Solomon 标准实例验收: R101/C101/RC101 前 20 点, 与已知最优解对比
      — BKS = CP-SAT 几何最优 (截断子集无文献 BKS, R5.1 结论)
      — 集成测试断言: 能量 gap < 10% (实测 R101 −5.2% / C101 −2.1% / RC101 −16.2%)
- [x] 消融实验 (用 data_generator 批量生成, 6 变体):
  - 去掉电量约束 (no_energy) → 超大电池构造+搜索, 真实电池评估
  - 固定载重 (fixed_payload, β=0) → 忽略耦合效应, 真实 β 评估
  - 只用 NN 构造 (nn_only) / 去 2-opt (no_2opt) / 去 Or-opt (no_oropt)
      — 实测: fixed_payload 成本 +1~9%, nn_only +1~2% (10 种子 mean±std)
- [x] 规模扩展: 自建 5→10→15 点 + Solomon 全集求解时间曲线
      — 实测: our 1.1ms(5p) → 26ms(15p) → 116ms(20p); PyVRP 每实例 2s

**D4-D6: 补全**
- [x] 单测补充到 ≥10 例 (W5 新增 28 例: test_exact 15 + test_ablation 11 + test_baseline 2)
- [x] 输出论文-ready 指标表 (自建 5/10/15 + tight + Solomon R101/C101/RC101 → results/)
- [x] `plan_multistop()` 入口函数完整性检查 (纯函数 ✓ / docstring ✓ / 签名未变 ✓)
- [x] Python 必须跑通 ✓
- [x] 指标表就绪 ✓
- [x] 中检不达标则简化 MVP — 达标, 无需简化

**W5 关键结论 (详见 A3_RESEARCH_PLAN.md R5)**:
  - 能量感知优化效果显著: Solomon 三实例上我们的等效成本比几何最优路线低 2-16%
  - 标准求解器"最优解"在真实约束下可能不可行: 载重超限 (默认 demand) / 电池耗尽 (tight 区域 seed=42) — 专利论据
  - 消融: 载重-能耗耦合贡献最大 (忽略耦合 +1~9%), 搜索 +1~2%, 增量评估贡献在速度 (毫秒级)

---

## W6-W9: 初稿·Rust

### W6 (9/5-9/11): Rust crate 骨架 ✅ (8/23 提前完成)

**前置 (W6 启动前完成)**:
- [x] 安装 Rust 工具链 — rustc 1.98.0 stable + MSVC 14.27 (VS2019) 已就绪
- [x] 🔍 R6.1/R6.2 调研: sqrt 用标准库 (结论入 A3_RESEARCH_PLAN.md) + axum 设计

- [x] `cargo init a3_rust` — lib + bin 结构 (lib 核心纯函数 / bin 留 W7 HTTP)
- [x] 定义 `dto.rs` (MultiStopReq, RoutePlanResp, Segment) — 对齐 A3_SCHEMA.md §2.2
- [x] 实现 `energy.rs` — 等效距离计算, **用标准库 `f64::sqrt()` (R6.1 结论: 无需手写,
      IEEE 754 与 numpy 精度一致)**; 原则: 有合适的标准库/轻量库就不手写,
      仍禁止 geo/nalgebra/OR-Tools 级重型依赖
- [x] 实现 `solver.rs` 空壳 — 返回 INTERNAL 未实现 (W7 填算法)
- [x] Python/Rust 同输入对比: 等效距离矩阵误差 < 1e-6 — 6 个 fixture 全过
      (Rust 重算 geo/equiv 矩阵 vs numpy golden, 逐元素断言)
- [x] **强制对齐**: 两版输出 diff，误差入单测断言 — tests/cross_check.rs (2 例)
- [x] Rust 单测 16 例 (dto 4 + energy 9 + solver 1 + cross_check 2) + clippy 零告警

### W7 (9/12-9/18): 纯函数 + 服务 ✅ (8/23 提前完成)

- [x] Rust 版核心算法 — heuristic.rs (NN N-start + C-W Savings + 2-opt/Or-opt/VND
      增量评估) + solver.rs (验证 → NN → VND), 1:1 对齐 heuristic.py/solver.py
- [x] **单测与 Python 同步 (1:1 移植)**: energy 24 + heuristic 50 + solver 9 =
      83 例 (Python 54 例中跳过 full_eval 4 例 — W5 benchmark 材料, 生产路径不移植)
- [x] axum 服务: POST /plan on port 9204 — lib http.rs (Router/错误映射) + bin main.rs (serve)
- [x] **HTTP 层测试 (微服务交付验证)**:
      - 单元级: `tower::ServiceExt::oneshot` 直调 Router (4 例)
      - 集成级: tokio 起真实 TCP 服务 + reqwest `POST /plan` (4 例)
      - golden: Python 固定种子输出 (6 fixture) vs Rust 响应逐字段比对 —
        sequence 精确相等, 浮点容差 0.02; 实测全部一致
- [x] DTO 对齐 carrier 契约 — 嵌套 location 格式 (A3_SCHEMA.md §3.1)
- [x] README.md + 调用文档 (含 curl 示例)
- [x] 错误码契约: serde 解析失败 (语法/缺字段) 统一 400 BAD_REQUEST
      (axum JsonRejection 默认 422, 已显式映射); 不可行返回 200+feasible=false (与 Python 对齐)
- [x] 调试发现 2 个移植差异并修复: (1) Savings 的 dict 插入序 → Vec 保序;
      (2) golden fixture 扁平 vs 嵌套 location 格式
- [x] **W8 服务硬化 (8/24, 服务工程化, 测试 109 例)**:
      - P0 防崩溃: solver 参数域校验 (alpha>0/beta/capacity/battery/demand ≥ 0 → 400,
        防 alpha≤0 触发 assert panic) + middleware panic 兜底 (tokio task 隔离 → 500 JSON)
      - P1 并发: plan_multistop 移入 spawn_blocking (CPU 密集隔离 async worker)
        + 请求超时兜底 10s + time_limit_secs 真正生效 (VND deadline, 正常求解不触发)
      - P2 可运维: GET /healthz + tracing 请求日志 (method/path/status/耗时)
        + ServiceConfig State 注入 + env 配置 (A3_PORT/A3_MAX_ITERATIONS/A3_TIME_LIMIT_SECS)
      - 并发验证: 10 并发 20 点请求全 200 + 结果与串行一致 (确定性); 坏请求 400 隔离
      - 冒烟: healthz 200 / plan 200 / alpha=0 → 400 JSON (原为 panic 断连)

### W8 (9/19-9/25): 文档

- [ ] 专利交底书初稿 (6 章节):
  1. 背景与场景 (现有技术缺陷: PyVRP/OR-Tools 忽略载重-电量耦合)
  2. 发明目的 (解决载重感知的无人机路径规划)
  3. 技术方案 (S1 等效距离变换 → S2 电量感知构造 → S3 增量搜索 → S4 后验证)
  4. 关键创新点 (等效电量距离变换 + 载重-能耗耦合模型 + 增量电量校验)
  5. 实施例 (5/10/20 点测试数据，与 OR-Tools 对比)
  6. 有益效果 (量化对比: gap<10%, 零坠机误判)
- [ ] 论文框架（可选 — 不作为交付要求，交底书已覆盖同内容）
- [ ] 代码清理 + 注释整理

### W9 (9/26-10/2): 初稿验收

- [ ] 初稿 Deadline: Rust 落地 + 文档 + 专利交底书初稿
- [ ] 硬节点评审
- [ ] 交底书评审（交付形态已定: 专利）

**W9 附带: 精确解扩到 n≤20 + 自适应模式 (9/10 完成 ✅)**

> 来源: 探索分支 `tw-feasibility` 的 STEP1/STEP2 (分层向量化 + 上界剪枝),
> 验证有效后按"只搬已验证部分"的原则移植回主干。

- [x] `a3_python/exact_battery.py` (新模块): 带电量约束的能量最优精确 DP
      — 参照实现 (朴素三重循环) + 加速实现 (按 popcount 分层 + numpy 向量化
        + 可采纳下界剪枝), 两者**逐位一致**
      — `MAX_EXACT_BATTERY_POINTS` 由探索期 15 上调为 **20** (= `solver.MAX_TARGETS`)
      — 实测: n=20 中位 **1.774 s** / 峰值 **155.8 MB** (原 pure-Python 三重循环 96.5 s),
        即 MVP 上限内可**在线精确最优** (5 s 时限 / 250 MB 单请求预算内)
      — **理论结论**: 电量约束在本问题里只是**可行性闸门** (累计能耗单调不减 ⇒
        "任意时刻 ≤ cap" ⟺ "总能耗 ≤ cap"), 不改变最优序列; 精确解相对无约束 DP
        的增量是"可行性判定 + 性能"
      — 不修改 `exact.py` 任何既有函数 (向后兼容)
- [x] `a3_python/adaptive.py` (新模块): `plan_multistop_adaptive()` 自适应入口
      — `estimate_exact_resources()` / `choose_mode()`: 按 n 保守估算耗时与峰值内存
        (以 n=20 实测为锚点的 2×/点 指数外推 + 固定开销 + 1.5 倍安全系数),
        与传入预算比较后选精确模式或降级到 `solver.plan_multistop`
      — **协同**: 精确模式以降级解的**精确能耗**作 `ub_override` 剪枝上界
        (不能用 `RoutePlan.total_energy_consumed`, 其 2 位小数舍入可能低于真值)
      — 模式标注追加在 `RoutePlan.warnings` (`"[adaptive] mode=..."`),
        **未改** `RoutePlan` 字段定义 (A3_SCHEMA.md §1)
- [x] 单测 **122 例** (test_exact_battery 21 + test_exact_battery_fast 72 + test_adaptive 29)
      — 全量 **267 例**通过 (原 145 + 新增 122); Rust 侧 109 例保持通过
      — `plan_multistop` 行为未变: 6 个 fixture 输出与 `solver_golden.json` 逐字段一致
- [x] 文档同步: CLAUDE.md 项目结构 + 函数签名; 本 DEVPLAN; A3_REQUIREMENTS.md §6 指标

---

## W10-W13: 专利材料完善 + 定稿

> 策略（8/19 定稿）: 专利交底书为唯一交付物。W10-W11 完善实施例与有益效果
> 所需的量化材料（尽量准备充分，但只做必要的实验，不做论文级深化），
> W12 撰写，W13 定稿。

### W10: 材料完善（两项已提前至 8/19 完成 ✅）

- [x] 自建 20 点补进指标表 — 指标表现含 5/10/15/20 共 93 实例
      （circle_20p 几何 gap=0.00, random_20p 能量 gap 多为负）
- [x] 增量评估量化: 增量 vs 全量评估的速度对比 — 实测 9 实例平均加速比 **3.0×**
      （2.3-3.2×，与理论 O(n)/O(k) k≈n/3 一致），解质量 100% 一致
- [ ] 时间窗支持（进阶，不做）

### W11: 材料完善

- [ ] 消融实验按实施例需要补全（W5 已有 6 变体，通常足够）
- [ ] 可复现性检查（fixtures/种子固定 — 已具备，确认即可）
- [ ] 显著性检验 — 不做（论文级）

### W12: 交底书撰写

- [ ] 按 W8 六章节完善专利交底书（实施例/有益效果填入 W5+W10/W11 数据）
- [ ] 写作质量检查

### W13: 定稿

- [ ] 专利交底书定稿（可申报）
- [ ] 结题

---

## 四个硬节点

| 节点 | 周 | 日期 | 交付物 |
|------|----|------|--------|
| W1 骨架 | W1 | 8/9 | 空函数 + ≥3 单测 + 目录结构 |
| 月1中检 | W5 | 9/4 | Python 跑通 + 指标表 (自建 5/10/20 + Solomon R101/C101/RC101) |
| 初稿 Deadline | W9 | 10/2 | Rust 落地 + 专利交底书初稿 |
| 定稿 | W13 | 10/30 | 可申报专利交底书（唯一交付物） |

---

## 当前状态

**阶段**: W1 ✅ → W2 ✅ → W3 ✅ → W4 ✅ → W5 月1中检 ✅ → W6 Rust 骨架 ✅ → W7 纯函数+服务 ✅ → W8 专利交底书初稿 → W9 精确解扩至 n≤20 + 自适应模式 ✅ (9/10)
**阻塞**: 无
**下一步**: W8/W9 文档收口:
  - 专利交底书初稿 (6 章节) — 实施例数据已就绪 (W5 指标表 + W7 Rust 交叉验证
    + W9 精确解: n≤20 在线最优, 可直接给出启发式 vs 精确解的 gap)
  - 代码清理 + 注释整理
  - 论文框架 (可选, 不做为交付要求)
**备注 (W9)**: `plan_multistop_adaptive` 只支持 1 ≤ n ≤ 20 (两种模式同受
  `solver.MAX_TARGETS` 约束); 若实施例需要 n > 20 的对照, 需另行放开启发式上限。

**新流程**: 每阶段开始前先完成 A3_RESEARCH_PLAN.md 中的调研项，再写代码。

**W5 完成项 (8/18) + 材料完善 (8/19)**:
  - `heuristic.py`: `local_search_2opt/or_opt/vnd` 增加 `full_eval` 参数 (默认 False)
    + `_try_2opt_move_full/_try_or_opt_move_full` — 增量 vs 全量速度对比对照组
  - `benchmark.py`: 新增第 4 节"增量 vs 全量评估" — 实测加速比 3.0×, 解质量 100% 一致
  - benchmark 扩至 5/10/15/20 点 × 10 种子 + tight + Solomon = **93 实例**
  - 消融扩至 n=20: random_20p 忽略耦合 +14.6%, 仅 NN +5.2% (规模越大贡献越显著)
  - `exact.py`: `solve_tsp_exact_cpsat()` — OR-Tools CP-SAT 精确 TSP (n≤20, num_workers=1 确定性)
  - `exact.py`: `solve_tsp_exact_dp()` — Held-Karp DP (n≤15, 验证 CP-SAT 用)
  - `exact.py`: `solve_energy_exact_dp()` — 能量感知精确 DP (状态相关边权, n≤15)
  - `exact.py`: `evaluate_sequence_cost()` + `compute_gap()` — 公平比较工具
  - `ablation.py`: 6 变体消融 (full/nn_only/no_oropt/no_2opt/fixed_payload/no_energy)
  - `baseline.py`: `solve_tsp_pyvrp()` 支持 bks_cost → gap_vs_best (W2 预留字段激活)
  - `benchmark.py`: 完整评测管线 (双基线 gap + PyVRP + Solomon BKS + 消融 + 规模曲线 + 论文指标表)
  - **调研结论**: R5.1/R5.2 完成 — CP-SAT 精确解方案 + 能量感知 DP + 6 变体消融设计
  - **28 新 W5 单测** (15 exact + 11 ablation + 2 baseline gap), 集成测试补 3 个能量 gap 断言
  - 全量单测通过, 完整 benchmark 通过 (73 实例, 输出 results/table_*.md)

**W4 完成项 (8/6)**:
  - `heuristic.py`: `_try_2opt_move()` — 2-opt 增量评估, O(k) 仅重算受影响段
  - `heuristic.py`: `_try_or_opt_move()` — Or-opt 增量评估, O(|S|) 移动 1-3 点
  - `heuristic.py`: `local_search_2opt()` — first-improvement 2-opt 搜索
  - `heuristic.py`: `local_search_or_opt()` — first-improvement Or-opt 搜索
  - `heuristic.py`: `local_search_vnd()` — VND 交替框架 (2-opt → Or-opt → 重复)
  - `solver.py`: `plan_multistop()` 已集成 VND 局部搜索
  - **调研结论**: R4.1-R4.4 全部完成, 选用 first-improvement + VND
  - **25 新 W4 单测** (5 增量辅助 + 3 2-opt 正例 + 3 2-opt 边界 + 2 Or-opt 正例 + 2 Or-opt 边界 + 3 VND 正例 + 4 VND 一致性 + 2 增量vs全量 + 3 回归)
  - 全量 113 单测通过, benchmark quick smoke 通过

**W3 完成项 (8/5)**:
  - `heuristic.py`: `construct_nn()` — N-start 电量感知 NN, O(N × n²)
  - `heuristic.py`: `construct_savings()` — equiv_dist 改造版 C-W Savings
  - `solver.py`: `plan_multistop()` 已集成 NN 构造
  - **调研结论**: NN 在所有 6 个实例上优于 Savings (差距 5-29%), 选 NN 作为主构造方法
  - **25 新 heuristic 单测** (5 正例 NN + 3 正例 SV + 3 退化 NN + 2 退化 SV + 3 边界 + 4 一致性 + 3 回归 + 2 W4 占位)
  - 全量 88 单测通过, benchmark quick smoke 通过

**W2 完成项 (8/5)**:
  - `energy_model.py`: `compute_geo_matrix()` + `compute_equiv_matrix()` (numpy 向量化)
  - `baseline.py`: PyVRP TSP 基线, 7 实例全部 feasible, `BaselineResult` + `run_baseline_suite()`
  - `data_generator.py`: 5 种分布 (circle/random/grid/line/cluster) + fixture 导出
  - `route.py`: `DRONE_PRESETS` — 3 种标准机型 (light/standard/heavy)
  - `fixture_loader.py`: 共享 fixture 加载工具 (从 tests/ 提取到 package 层)
  - 67 单测全部通过 (21 energy + 9 solver + 4 heuristic + 8 integration + 6 baseline + 19 data_gen), benchmark quick smoke 通过
