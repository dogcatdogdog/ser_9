# A3 · 开发计划

> 版本: v1.1 | 日期: 2026-08-05

---

## 目标

**MVP 交付 (W1-W9)**: Python 验证 → Rust 落地 → 专利交底书初稿
**论文/专利定稿 (W10-W13)**: 对比实验 + 消融 → 可投递/可申报

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
- [x] 24 单测 (正例/退化/边界/一致性) — 全部通过

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

### W3 (8/15-8/21): 构造启发式

**D1-D3: 最近邻构造**
- [ ] 实现电量感知 NN: `construct_nn(targets, home, drone) -> RoutePlan`
- [ ] 每次选择 equiv_dist 最小且满足剩余电量约束的点
- [ ] 不可行时返回部分路线 + 不可行标记

**D4-D5: Savings 构造**
- [ ] 实现 Clarke-Wright Savings 改造版
- [ ] saving(i,j) = equiv_dist(home,i) + equiv_dist(home,j) - equiv_dist(i,j)
- [ ] 按 saving 从大到小合并路线，每次合并检查电量可行性

**D6-D7: 对比与选择**
- [ ] NN vs Savings 在 5/10/20 点上的对比
- [ ] 选较优者作为主构造方法
- [ ] 单测: 验证构造解满足载重 + 电量约束

### W4 (8/22-8/28): 局部搜索

**D1-D2: 2-opt**
- [ ] 实现 2-opt 翻转操作
- [ ] 实现增量电量评估（只重算受影响段，不做全局 O(n) 重算）
- [ ] 单测: 验证 2-opt 后的路线仍然可行

**D3-D4: Or-opt**
- [ ] 实现 Or-opt (移动 1/2/3 个连续点)
- [ ] 增量评估 + 可行性检查

**D5-D7: 搜索框架**
- [ ] 实现 first-improvement 搜索循环
- [ ] 最大迭代次数 / 时间上限
- [ ] 单测: 搜索后的解不比初始解差

### W5 (8/29-9/4): 月1中检

**D1-D3: 评测**
- [ ] vs OR-Tools (≤15 点精确解): 计算 gap
- [ ] vs PyVRP (无电量约束版): 对比解差异
- [ ] 消融实验:
  - 去掉电量约束 → 标准 TSP/VPR
  - 固定载重（不衰减）→ 忽略耦合效应
  - 只用 NN 构造（不搜索）
- [ ] 规模扩展: 5→10→20 点求解时间曲线

**D4-D6: 补全**
- [ ] 单测补充到 ≥10 例
- [ ] 输出论文-ready 指标表
- [ ] `plan_multistop()` 入口函数完整性检查
- [ ] Python 必须跑通 ✓
- [ ] 指标表就绪 ✓
- [ ] 中检不达标则简化 MVP

---

## W6-W9: 初稿·Rust

### W6 (9/5-9/11): Rust crate 骨架

- [ ] `cargo init a3_rust`
- [ ] 定义 `dto.rs` (MultiStopReq, RoutePlanResp, Segment)
- [ ] 实现 `energy.rs` (等效距离计算，手写 sqrt)
- [ ] 实现 `solver.rs` 空壳
- [ ] Python/Rust 同输入对比: 等效距离矩阵误差 < 1e-6
- [ ] **强制对齐**: 两版输出 diff，误差入单测断言

### W7 (9/12-9/18): 纯函数 + 服务

- [ ] Rust 版核心算法 (NN + 2opt/Or-opt + 增量评估)
- [ ] 单测 ≥10 例（对齐 Python 用例）
- [ ] axum 服务: POST /plan on port 9204
- [ ] DTO 对齐 carrier 契约
- [ ] README.md + 调用文档

### W8 (9/19-9/25): 文档

- [ ] 专利交底书初稿 (6 章节):
  1. 背景与场景 (现有技术缺陷: PyVRP/OR-Tools 忽略载重-电量耦合)
  2. 发明目的 (解决载重感知的无人机路径规划)
  3. 技术方案 (S1 等效距离变换 → S2 电量感知构造 → S3 增量搜索 → S4 后验证)
  4. 关键创新点 (等效电量距离变换 + 载重-能耗耦合模型 + 增量电量校验)
  5. 实施例 (5/10/20 点测试数据，与 OR-Tools 对比)
  6. 有益效果 (量化对比: gap<10%, 零坠机误判)
- [ ] 论文框架（Abstract · Introduction · Method · Experiments）
- [ ] 代码清理 + 注释整理

### W9 (9/26-10/2): 初稿验收

- [ ] 初稿 Deadline: Rust 落地 + 文档 + 交底书/论文框架
- [ ] 硬节点评审
- [ ] 定专利/论文去向

---

## W10-W13: 实验发文

### W10: 实验设计

- [ ] 对比基线选定
- [ ] 时间窗支持（进阶）
- [ ] 实验能证伪创新点

### W11: 对比 + 消融

- [ ] 对比实验 (vs OR-Tools, vs PyVRP, vs VeRyPy heuristics)
- [ ] 消融实验
- [ ] 显著性检验

### W12: 成稿

- [ ] 论文/专利成稿
- [ ] 写作质量检查

### W13: 定稿

- [ ] 可投递论文 / 可申报专利
- [ ] 结题

---

## 四个硬节点

| 节点 | 周 | 日期 | 交付物 |
|------|----|------|--------|
| W1 骨架 | W1 | 8/9 | 空函数 + ≥3 单测 + 目录结构 |
| 月1中检 | W5 | 9/4 | Python 跑通 + 指标表 |
| 初稿 Deadline | W9 | 10/2 | Rust 落地 + 专利交底书初稿 |
| 定稿 | W13 | 10/30 | 可投递论文 / 可申报专利 |

---

## 当前状态

**阶段**: W1 骨架 ✅ → W2 数据管线 + 基线 ✅ → W3 构造启发式
**阻塞**: 无
**下一步**: W3 构造启发式:
  - 实现电量感知 NN: `construct_nn(targets, home, drone) -> RoutePlan`
  - 实现 Clarke-Wright Savings 改造版
  - NN vs Savings 对比评测

**W2 完成项 (8/5)**:
  - `energy_model.py`: `compute_geo_matrix()` + `compute_equiv_matrix()` (numpy 向量化)
  - `baseline.py`: PyVRP TSP 基线, 7 实例全部 feasible, `BaselineResult` + `run_baseline_suite()`
  - `data_generator.py`: 5 种分布 (circle/random/grid/line/cluster) + fixture 导出
  - `route.py`: `DRONE_PRESETS` — 3 种标准机型 (light/standard/heavy)
  - `tests/utils.py`: 共享 fixture 加载工具 (解耦 conftest)
  - 71 单测全部通过, benchmark quick smoke 通过
