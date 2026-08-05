# A3 · 架构设计 — Python 验证 → Rust 落地

> 版本: v1.0 | 日期: 2026-08-05

---

## 1. 总体架构

```
┌─────────────────────────────────────────────────────────┐
│                    Python (实验 / 论文)                   │
│                                                         │
│  ┌──────────┐  ┌───────────┐  ┌────────────────────┐   │
│  │ 数据管线  │  │  energy_  │  │  solver.py          │   │
│  │(vrplib) │→│  model.py │→│  - construct_nn()    │   │
│  │ 仅用于   │  │  - α,β   │  │  - local_search()   │   │
│  │ 数据集加载│  │  - equiv │  │  - plan_multistop() │   │
│  └──────────┘  │  _dist() │  └─────────┬───────────┘   │
│                └───────────┘            │               │
│                                         ▼               │
│  ┌──────────────────────────────────────────────────┐   │
│  │  benchmark.py                                     │   │
│  │  - vs OR-Tools (精确解)                            │   │
│  │  - vs PyVRP (无电量约束)                           │   │
│  │  - 消融: 去掉载重耦合 / 只用最近邻                    │   │
│  │  → 指标表 + 论文图表                                │   │
│  └──────────────────────────────────────────────────┘   │
│                                                         │
│  算法核心逻辑: 纯 Python, 无外部依赖 (除 numpy)            │
├─────────────────────────────────────────────────────────┤
│                    Rust (落地 / 平台)                     │
│                                                         │
│  ┌──────────┐  ┌───────────┐  ┌────────────────────┐   │
│  │  dto.rs  │  │  solver   │  │  axum 服务           │   │
│  │  - Req   │→│  - 同算法  │→│  - POST /plan        │   │
│  │  - Resp  │  │  - 纯函数  │  │  - port 9204         │   │
│  └──────────┘  └───────────┘  └────────────────────┘   │
│                                                         │
│  算法核心: 手写几何/图算法, 不引重型依赖                    │
│  DTO 对齐 carrier 契约, 经 biz-ai 接入指挥中心             │
└─────────────────────────────────────────────────────────┘
```

## 2. 为什么 Python 和 Rust 要分开设计

| 维度 | Python 侧 | Rust 侧 |
|------|-----------|---------|
| **目的** | 算法实验、论文、对比/消融 | 生产服务、平台集成 |
| **依赖** | PyVRP, OR-Tools, numpy, matplotlib | 无重型依赖（手写几何） |
| **性能要求** | ≤20 点 < 30s (对齐 A3_REQUIREMENTS.md §6) | ≤20 点 < 1s |
| **核心算法** | 可调 PyVRP C++ 底层 | **纯手写**（几何/图算法，对齐 path-ai 范式） |
| **交付物** | 实验脚本 + 指标表 + 论文图表 | crate + 单测 + HTTP 服务 + DTO |

**关键原则**：两阶段不割裂。Python 验证了"等效距离变换 + 增量校验"的核心逻辑后，Rust 用相同的算法步骤重新实现（不是翻译代码，是翻译方法论）。

## 3. 算法流水线（两阶段一致）

```
Input: targets[], home, battery_capacity, payload_capacity, α, β
                              │
                              ▼
         ┌─────────────────────────────────────┐
         │  Step 1: 等效距离矩阵构建              │
         │  for each (i,j):                     │
         │    geo = sqrt((xi-xj)²+(yi-yj)²)    │
         │    equiv(i,j,load) = geo × (α+β·load)/α │
         │  注意: load 依赖于访问顺序，               │
         │  构造阶段用"出发时剩余载重"近似              │
         └──────────────┬──────────────────────┘
                        ▼
         ┌─────────────────────────────────────┐
         │  Step 2: 构造初始解 (NN + Savings)     │
         │  - 电量感知的最近邻 (NN):               │
         │    从 home 出发，每次选 equiv_dist    │
         │    最小且满足剩余电量约束的未访问点       │
         │  - Clarke-Wright Savings 改造版:       │
         │    saving(i,j) = equiv(0,i)+equiv(0,j) │
         │    - equiv(i,j), 按 saving 降序合并    │
         │  - 多起点 NN (N-start): 每个点作为     │
         │    第一个访问点各跑一次，取最优           │
         │  - 如果无可行点 → 返回 home            │
         └──────────────┬──────────────────────┘
                        ▼
         ┌─────────────────────────────────────┐
         │  Step 3: 局部搜索改进                 │
         │  - 2-opt: 翻转一段子路径              │
         │  - Or-opt: 移动 1-3 个连续点到新位置   │
         │  - 每次移动后增量评估:                 │
         │    · 只重算受影响段 (O(k) 而非 O(n))   │
         │    · 判定电量/capacity 可行性           │
         │  - First-improvement: 接受第一个改进解  │
         │  - 或 Best-improvement: 遍历全部邻域    │
         └──────────────┬──────────────────────┘
                        ▼
         ┌─────────────────────────────────────┐
         │  Step 4: 全量后验证 + 输出             │
         │  - 逐段模拟 payload 变化              │
         │  - 精确计算各段能耗                    │
         │  - 判定整体可行性                      │
         │  → RoutePlan { seq, total_equiv_dist, │
         │       energy_consumed, feasible }    │
         └─────────────────────────────────────┘
```

## 4. Rust 侧架构（预先设计，W6 启动）

```
a3_rust/
├── Cargo.toml
├── src/
│   ├── main.rs          # axum HTTP server, port 9204
│   ├── dto.rs           # MultiStopReq / RoutePlanResp (对齐 carrier)
│   ├── solver.rs        # plan_multistop() 纯函数
│   ├── energy.rs        # 等效距离计算 (手写几何, 无外部依赖)
│   ├── heuristic.rs     # NN 构造 + 2-opt/Or-opt 局部搜索
│   └── route.rs         # RoutePlan 数据结构 (访问序列 + 状态)
├── tests/
│   └── integration_test.rs  # ≥10 单测
└── README.md
```

### Rust 依赖策略

```toml
[dependencies]
axum = "0.7"        # HTTP 服务
serde = "1"          # JSON 序列化
serde_json = "1"
tokio = "1"          # 异步运行时
# 不加 geo/nalgebra — 使用标准库 f64::sqrt() (IEEE 754，与 numpy 精度一致)
# 不加 OR-Tools 绑定 — 算法全部手写
```

### 端口与 DTO 约定

- 端口: **9204** (path-ai 用 9203，A3 递增)
- HTTP 方法: `POST /plan`
- Content-Type: `application/json`
- DTO 对齐 `carrier` 契约（经 `biz-ai` 接入指挥中心）
  - carrier 契约定义了无人机与指挥中心的标准通信协议
  - 具体字段映射参见 carrier 项目的 `schemas/` 目录
  - 若 carrier 尚未定义多目标规划 DTO，则本项目的 `MultiStopReq`/`RoutePlanResp` 即为提案

## 5. Python→Rust 对齐策略

| 阶段 | 做法 |
|------|------|
| W1-W5 | Python 专注算法正确性，不关心性能 |
| W6 | Rust crate 骨架 + 与 Python 对比：同输入→同输出 |
| W7 | Rust 单测 ≥10 + axum 服务 |
| W8 | 交叉验证：Python/Rust 输出 diff，误差入单测断言 |
| W9 | 初稿验收：Rust 落地 + 文档 + 交底书框架 |

## 6. 不与 PyVRP 耦合的设计

PyVRP 是 Python 实验的**参考对比对象**，不是 Python 核心的**依赖**。

- `solver.py` 的核心算法是**纯手写**的 (NN + 2-opt/Or-opt)
- `benchmark.py` 中调用 PyVRP 只是为了**对比评测**
- 这样 Rust 重写时不需要翻译 PyVRP，只需要翻译自己的算法
- 这也满足专利要求：专利保护的**是自己的启发式方法**，PyVRP 只是 baseline
