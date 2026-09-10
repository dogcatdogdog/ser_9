# ser_9 — A3 · 多目标访问路线规划

## 项目概述

当前聚焦 **A3 · 多目标访问路线规划 (TSP/VRP)**：无人机从仓库出发，访问 N 个投递点/巡检点，在载重上限和电量续航的联合约束下，找到总等效能耗最小的访问顺序。

**专利主张（W9 定稿口径）**：「一种基于自适应分级求解的无人机多目标路径规划方法」
- 主张：**两模式互为支撑** — 启发式给可行解兼作精确搜索的剪枝上界；上界使状态空间削减
  41.6%~94.2%，把"在线精确最优"从 n≤15 推到 **n≤20**（1.96 s，原实现 96.5 s）；精确模式反供
  确切最优值与可行性判定（两模式齐跑时可确切得出降级解差距：实测均值 0.43%）
- **已知技术（不主张，交底书已主动承认）**：载重-能耗线性模型（Dorling 2017）、
  等效电量距离变换（等价改写，E=α·EED）、增量电量校验（Ngueveu 2010 已有 O(1)）、
  Held-Karp 式 DP 与标签支配（Christofides 1981）
- 详见 `docs/patent_disclosure.md` 与 `docs/patent/NOVELTY_VERDICT.md`（含文献核查结论）

## 技术栈

| 层 | 技术 | 用途 |
|----|------|------|
| Python 实验 | Python 3.12 + NumPy | 算法核心、对比实验 (vs OR-Tools/PyVRP)、消融、论文图表 |
| Rust 落地 | Rust ≥1.75 + axum 0.7 + serde | 生产 HTTP 服务，端口 9204，DTO 对齐 carrier |
| 对比基线 | PyVRP 0.13.x, VeRyPy, OR-Tools | **仅用于评测对比**，不作为核心求解器依赖 |

## 环境

```bash
# 激活环境
conda activate D:\ser_9\env312    # Python 3.12.13

# 安装依赖
pip install pyvrp vrplib numpy matplotlib -i https://pypi.tuna.tsinghua.edu.cn/simple

# 运行 baseline
python D:\ser_9\baseline_01_hello_pyvrp.py

# 运行单测
python -m pytest D:\ser_9\a3_python\tests\ -v
```

关键包: `pyvrp==0.13.4`, `vrplib`, `numpy`, `matplotlib`, `pytest`

## 项目结构

```
D:\ser_9\
├── CLAUDE.md                    # 本文件 — 项目约定与开发流程
├── A3_REQUIREMENTS.md           # 需求规格 (问题定义、MVP 边界、约束)
├── A3_ARCHITECTURE.md           # 架构设计 (Python→Rust 双阶段)
├── A3_SCHEMA.md                 # 数据结构 / DTO / JSON Schema / API
├── A3_DEVPLAN.md                # 13 周开发计划与里程碑
├── A3_RESEARCH_PLAN.md          # 调研计划 — 每阶段调研先行，成熟方案适配
├── baseline_01_hello_pyvrp.py   # ✅ 已跑通 — PyVRP Hello World
├── a3_python/                   # Python 实验代码 (当前阶段)
│   ├── __init__.py
│   ├── solver.py                # 核心求解入口 plan_multistop() — NN 构造 + VND 搜索
│   ├── energy_model.py          # 载重-电量耦合 + 等效距离变换
│   ├── heuristic.py             # NN 构造 + C-W Savings + 2-opt/Or-opt/VND 搜索
│   ├── route.py                 # RoutePlan/Segment 数据结构
│   ├── exact.py                 # 精确解基线 (W5): CP-SAT + Held-Karp + 能量感知 DP
│   ├── exact_battery.py         # 带电量约束精确解 (W9): 分层向量化 + 可采纳下界剪枝,
│   │                            #   n<=20 (MVP 上限) 1.8s/156MB; 电量约束只是可行性闸门
│   ├── adaptive.py              # 自适应入口 (W9): plan_multistop_adaptive() 按预算选
│   │                            #   精确/降级; 精确模式以降级解的精确能耗作剪枝上界
│   ├── ablation.py              # 消融实验 (W5): 6 变体组合现有纯函数
│   ├── benchmark.py             # 评测: vs OR-Tools / PyVRP / 消融 → 论文指标表
│   ├── baseline.py              # PyVRP 基线求解 (W2, W5 加 gap_vs_best)
│   ├── data_generator.py        # 测试数据生成器 (W2)
│   ├── fixture_loader.py        # 共享 fixture 加载 (W2)
│   └── tests/                   # 单测 (267 例 = W5 145 + W9 122)
│       ├── conftest.py           # 共享 fixtures
│       ├── utils.py              # → 委托 fixture_loader.py (向后兼容)
│       ├── test_energy_model.py  # 21 例
│       ├── test_solver.py        # 9 例
│       ├── test_heuristic.py     # 54 例 (W4 50 + W5 full_eval 4)
│       ├── test_integration.py   # 8 例 (W5 补 3 个能量 gap 断言)
│       ├── test_baseline.py      # 8 例 (W5 +2)
│       ├── test_data_generator.py # 19 例 (W2)
│       ├── test_exact.py         # 15 例 (W5)
│       ├── test_ablation.py      # 11 例 (W5)
│       ├── test_exact_battery.py      # 21 例 (W9)
│       ├── test_exact_battery_fast.py # 72 例 (W9): 向量化+剪枝 vs 参照/暴力枚举逐位一致
│       ├── test_adaptive.py           # 29 例 (W9): 模式选择/预算估算/协同/边界
│       └── fixtures/             # 标准测试数据集 (6 个)
│           ├── solomon_r101_n20.json
│           ├── solomon_c101_n20.json
│           ├── solomon_rc101_n20.json
│           ├── custom_5_heavy.json
│           ├── custom_10_tight.json
│           └── custom_15_mixed.json
├── results/                     # benchmark 输出 (W5): benchmark_*.json + table_*.md
├── a3_rust/                     # Rust 落地 (W6 ✅ W7 ✅, W8 服务硬化 ✅, 109 测试)
│   ├── Cargo.toml
│   ├── README.md                # 构建/运行/API 文档 (curl 示例)
│   ├── scripts/
│   │   ├── gen_energy_golden.py # 交叉验证 golden 生成 (env312 python)
│   │   └── gen_solver_golden.py # solver 输出 golden 生成 (HTTP 比对)
│   ├── src/
│   │   ├── lib.rs               # lib 入口: 核心纯函数 + HTTP 层
│   │   ├── main.rs              # bin 入口: axum serve + tracing + env 配置
│   │   ├── dto.rs               # MultiStopReq/RoutePlanResp (A3_SCHEMA.md §2.2)
│   │   ├── energy.rs            # 等效距离 (std f64::sqrt) + 全量路线模拟
│   │   ├── heuristic.rs         # NN (N-start) + Savings + 2-opt/Or-opt/VND 增量评估
│   │   ├── solver.rs            # plan_multistop() 纯函数 (参数校验 → NN → VND)
│   │   └── http.rs              # axum Router: ServiceConfig State + spawn_blocking
│   │                            #   + panic 兜底 + /healthz + 请求日志 (W8 硬化)
│   └── tests/
│       ├── cross_check.rs       # Python/Rust 矩阵 < 1e-6 交叉验证 (W6)
│       ├── http_integration.rs  # 真实 TCP 服务 + reqwest + golden + 并发 (W7/W8)
│       └── fixtures/
│           ├── energy_golden.json   # numpy golden (6 fixture)
│           └── solver_golden.json   # Python plan_multistop 输出 golden (6 fixture)
└── docs/                        # 专利交底书 / 论文素材
    └── patent_disclosure.md     # 专利交底书 (6 章节)
```

## 核心函数签名

### Python (当前阶段)

```python
def plan_multistop(
    targets: list[Target],   # N 个目标点
    home: GeoPoint,          # 仓库位置
    drone: DroneSpec,        # 无人机规格 (含 α/β/battery/capacity)
    seed: int = 42           # 随机种子，保证确定性
) -> RoutePlan:
```

求解流程: NN 构造 (W3) → VND 搜索 2-opt + Or-opt 交替改进 (W4)

所有类型定义见 `A3_SCHEMA.md` §1。

### Python 自适应入口 (W9, 新增)

```python
def plan_multistop_adaptive(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    *,
    time_limit_secs: float = 5.0,    # 本次求解时间预算 (对齐 http.rs ServiceConfig)
    memory_limit_mb: float = 250.0,  # 本次求解峰值内存预算 (并发时应传 总量/并发数)
    seed: int = 42
) -> RoutePlan:
```

行为: 先跑一次 `plan_multistop` (既是降级解, 又是精确模式的剪枝上界);
`adaptive.choose_mode()` 按预算估算选模式 —— 预算够走 `exact_battery` 精确模式,
不够则直接返回降级解。返回的 `RoutePlan` **结构不变**, 实际模式以
`"[adaptive] mode=..."` 追加在 `warnings` 末尾, 用 `adaptive.solve_mode_of(plan)` 读回。

约束: `plan_multistop` 与 `plan_multistop_adaptive` **均只支持 1 ≤ n ≤ 20**
(`solver.MAX_TARGETS`); n > 20 抛 `ValueError` (无降级路径 —— 启发式同样受 MVP 上限约束)。

### Rust (W6 启动，预定义)

```rust
pub fn plan_multistop(
    req: &MultiStopReq,
    cfg: &Defaults           // 含 max_iterations, time_limit_secs, seed
) -> Result<RoutePlanResp, ApiError>
```

`Defaults`、`ApiError`、`SegmentDto` 的完整定义见 `A3_SCHEMA.md` §2.2。

## MVP 边界

| 项目 | MVP (W1-W9) | 进阶 (W10+) |
|------|-------------|-------------|
| 目标点数 | ≤ 20 | 50/100 |
| 无人机数 | 1 架 | 多架 |
| 约束 | **载重上限 + 电池续航联合约束** | + 时间窗 |
| 电量模型 | 线性 α + β×load | 分段线性 |
| 时间窗 | 不做 | ✅ |
| 动态重规划 | 不做 | — |

## 开发流程

### 调研先行 (每阶段启动前)

每个算法阶段（W3 构造、W4 搜索、W5 评测、W6 Rust）启动前，先完成调研：

```
1. 文献调研 (1-2天): 找到成熟方案 ← A3_RESEARCH_PLAN.md 指定了参考文献
2. 适配分析 (1天):  确认成熟方案如何改造到本问题（如 d → equiv_dist）
3. 小规模验证 (1天): 用 5 点实例快速验证改造成立
4. 正式实现 (2-3天): 写单测 → 实现 → 跑全量单测
```

**原则**: 不发明新算法，在经典 VRP 方案上做"载重-能耗耦合"改造。调研结论先写在 `A3_RESEARCH_PLAN.md` 对应 section 中，再开始编码。

### 日常开发循环

```
1. 调研 (如果是新模块) — 见上方"调研先行"
2. 写单测 (先于实现)
3. 实现功能 → 跑单测 (pytest -v)
4. 跑 benchmark (python a3_python/benchmark.py)
5. 文档同步 (每次里程碑完成后必须):
   - DEVPLAN: 勾选已完成项 + 更新底部"当前状态"和"下一步"
   - CLAUDE.md: 如果函数签名/环境/约定变化则更新
6. Git commit (见提交规范)
```

### 提交规范

```
[阶段] 简短描述 (<50 字符)

详细说明 (可选，多行)

关联: #issue 或 A3_SCHEMA.md §X
```

示例: `[W1] plan_multistop 空函数骨架 + 3 单测`

**不要**: amend 已推送的 commit、force push、跳过 pre-commit hook。

### Code Review 流程

1. **自审清单** (提交 PR 前):
   - [ ] 所有单测通过 (`pytest -v`)
   - [ ] 核心函数是纯函数（无 I/O、无全局状态、无网络调用）
   - [ ] 新增/变更的函数有 docstring
   - [ ] 等效距离公式与 `A3_REQUIREMENTS.md` §3.2 一致
   - [ ] Python/Rust 同名函数签名对齐 (`A3_SCHEMA.md`)

2. **Review 要求**:
   - 至少 1 人 review（导师/负责人）
   - Review 重点: 算法正确性 > 代码风格 > 性能
   - 所有 feedback 解决后才能 merge

3. **W6 交叉验证 (强制)**:
   - 同输入喂 Python 和 Rust 版本
   - 等效距离矩阵误差 < 1e-6
   - 输出 diff 入单测断言

### 分支管理

```
main           # 稳定版本，通过所有单测 + review
├── w1-setup   # W1 选题 + 骨架
├── w2-energy  # W2 能量模型
├── w3-nn      # W3 构造启发式
├── w4-search  # W4 局部搜索
├── w5-eval    # W5 评测 + 中检
├── w6-rust    # W6 Rust 落地
└── ...
```

**分支依赖规则 (线性开发, 禁止并行)**:

1. **前序周未合并 → 不开始当前周**:
   - W(n) 分支必须从包含 W(n-1) 合并结果的 main 上创建
   - 严禁从 W(n-1) 未合并的 commit 上创建 W(n) 分支
   - 开始 W(n) 前检查 `git log origin/main --oneline | head -5`，确认 W(n-1) 的 merge commit 存在

2. **PR 提交前检查**:
   - `git fetch origin && git merge origin/main --no-commit --no-ff` 必须无冲突
   - 上一周 PR 未合并的, 当前 PR 不得提交 review
   - DEVPLAN "当前状态" 必须指向已完成周的下一周

3. **冲突处理**:
   - 如果出现并行开发导致的冲突, 以后提交的 PR 负责解决
   - 解决方法: rebase 到已包含前序 PR 的 main 上, 解决冲突后 force-push

## 测试策略

### 测试金字塔

```
           ┌──────────────┐
           │   Benchmark   │  Solomon 全集 + OR-Tools/PyVRP/VeRyPy
           │   10+ instances│  每周跑，产出论文指标表
           ├──────────────┤
           │  Integration  │  标准实例 (R101/C101/RC101, 自建 5/10/15 点)
           │   5-8 cases   │  验证完整流程，对比已知最优解
           ├──────────────┤
           │  Unit Tests   │  mock 数据，验证单一模块
           │   ≥10 cases   │  每次 commit 跑，< 5s
           └──────────────┘
```

### 单元测试 (pytest, ≥10 例)

- **框架**: pytest
- **数据**: Python 内联 mock (2-5 个硬编码坐标点)
- **耗时**: 全部 < 5s
- **要求**: 正例 ≥3 / 退化 ≥3 / 边界 ≥2 / 一致性 ≥2 / 回归 ≥1
- **命名**: `test_<模块>_<场景>.py`
- **断言**: 禁止 `assert True` 占位
- 所有用例定义见 `A3_SCHEMA.md` §5.2

### 集成测试 (pytest, 5-8 例)

- **数据**: Solomon VRPTW 子集 (R101/C101/RC101) + 自建配送场景
- **断言**: 对比 OR-Tools 最优解，gap < 10%
- 所有用例定义见 `A3_SCHEMA.md` §5.3

### Benchmark 管线

```bash
# 完整评测 (所有 Solomon 子集 + 4 基线对比)
python a3_python/benchmark.py --points 10,15,20 --runs 10 --output results/

# 快速冒烟 (只跑 R101_20，1 次)
python a3_python/benchmark.py --quick
```

对比基线: OR-Tools (最优解) | PyVRP (无电量) | VeRyPy (15 种经典启发式)
消融矩阵: 完整方法 vs 去 Or-opt / 去 2-opt / 仅 NN / 固定载重
详见 `A3_SCHEMA.md` §5.4

```bash
# 运行完整评测 (vs OR-Tools / PyVRP / 消融)
python a3_python/benchmark.py --points 5,10,20 --runs 10 --output results/

# 快速冒烟 (只跑 5 点，1 次)
python a3_python/benchmark.py --quick
```

## 关键约定

### 三条铁律 (对齐 path-ai)

1. **核心是纯函数** — `plan_multistop()` 接收数据 + 配置，返回结果。不依赖网络、数据库、全局状态
2. **算法全手写** — 几何/图算法不引重型依赖。Python 侧 PyVRP **仅用于 benchmark 对比**，不出现在 solver 核心路径中。Rust 侧不加 geo/nalgebra crate
3. **Python 出论文，Rust 进平台** — 两阶段不割裂，算法步骤一致

### 禁止事项

- ❌ 在 `solver.py` 或 `solver.rs` 中 import PyVRP / OR-Tools
- ❌ 在 Rust 侧引入 OR-Tools 绑定
- ❌ 使用网络调用、文件 I/O、全局可变状态
- ❌ 跳过单测直接提交核心算法代码
- ❌ 修改 `A3_SCHEMA.md` 的函数签名而不更新 `CLAUDE.md`

> OR-Tools 仅允许出现在 `exact.py` (W5 精确解评测基线, CP-SAT 电路约束, n≤20) 与
> `benchmark.py` 调用链中 — 与 PyVRP 一样只做评测对比, 不出现在 solver 核心路径。

### 编码规范

- Python: type hints 全部标注；dataclass 不写 `__init__`
- Rust: `cargo fmt` + `cargo clippy -- -D warnings`
- 文档字符串: 中文注释，英文标识符
- 硬编码数值: 必须声明为模块级常量 (如 `DEFAULT_ALPHA = 0.1`)

### 命令调用约定（配合 `.claude/settings.json` 权限白名单）

**权限匹配机理**: 命令按 `;` / `&&` / `|` 拆段逐段匹配白名单，**引号内的分号也会被拆**。
任何一段无规则 → 整条命令触发用户确认。为免确认，命令调用必须：

- 单段命令，以白名单动词开头（`& "D:\ser_9\env312\python.exe"` / `git` / `Get-Content` 等）
- 不使用 `Set-Location` 前缀 — 工作目录跨调用保留，无需 cd
- 不使用 `$env:VAR=...` 前缀 — 需要 UTF-8 输出时用 `python -X utf8`
- **禁止 `python -c "多语句"`**（代码内分号拆段必弹确认）— 多语句逻辑先写临时脚本文件再运行
- 验证 JSON 用 PowerShell 原生 `Get-Content ... -Raw | ConvertFrom-Json`

白名单/deny 列表见 `.claude/settings.json`（已入库，合并后全仓库生效）。

## 四个硬节点

| 节点 | 周 | 交付物 |
|------|----|--------|
| W1 骨架 | W1 | 空函数 + ≥3 单测 + 目录结构 |
| 月1中检 | W5 | Python 跑通 + 指标表 |
| 初稿 Deadline | W9 | Rust 落地 + 专利交底书初稿 |
| 定稿 | W13 | 可投递论文 / 可申报专利 |

## 参考资料

| 资料 | 路径/链接 |
|------|----------|
| A3 需求规格 | `D:\ser_9\A3_REQUIREMENTS.md` |
| A3 架构设计 | `D:\ser_9\A3_ARCHITECTURE.md` |
| A3 Schema 定义 | `D:\ser_9\A3_SCHEMA.md` |
| A3 开发计划 | `D:\ser_9\A3_DEVPLAN.md` |
| PyVRP 文档 | https://pyvrp.org/ |
| PyVRP 论文 | Wouda et al. (2024), INFORMS J. Computing, DOI: 10.1287/ijoc.2023.0055 |
| Solomon VRPTW | http://web.cba.neu.edu/~msolomon/problems.htm |
| path-ai 范例 | 参照 path-ai 的 README.md + CLAUDE.md + 005 文档 |
