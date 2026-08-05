# ser_9 — A3 · 多目标访问路线规划

## 项目概述

当前聚焦 **A3 · 多目标访问路线规划 (TSP/VRP)**：无人机从仓库出发，访问 N 个投递点/巡检点，在载重上限和电量续航的联合约束下，找到总等效能耗最小的访问顺序。

**核心创新（专利方向）**：「一种基于载重-能耗耦合等效距离变换的无人机多目标路径规划方法」
- 创新点 1: 等效电量距离变换 — 将几何距离矩阵改造为载重感知的等效距离矩阵
- 创新点 2: 载重-能耗耦合模型 — 能耗率 = α + β × 当前载重
- 创新点 3: 增量电量校验 — 局部搜索中仅重算受影响段，O(k) 而非 O(n)

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
├── baseline_01_hello_pyvrp.py   # ✅ 已跑通 — PyVRP Hello World
├── a3_python/                   # Python 实验代码 (当前阶段)
│   ├── __init__.py
│   ├── solver.py                # 核心求解入口 plan_multistop()
│   ├── energy_model.py          # 载重-电量耦合 + 等效距离变换
│   ├── heuristic.py             # NN 构造 + 2-opt/Or-opt 搜索
│   ├── route.py                 # RoutePlan/Segment 数据结构
│   ├── benchmark.py             # 评测: vs OR-Tools / PyVRP / 消融
│   └── tests/                   # 单测 (≥10, 对齐 path-ai 范式)
│       ├── conftest.py           # 共享 fixtures
│       ├── test_energy_model.py
│       ├── test_solver.py
│       ├── test_heuristic.py
│       ├── test_integration.py   # 5-8 集成测试
│       └── fixtures/             # 标准测试数据集
│           ├── solomon_r101_n20.json
│           ├── custom_5_heavy.json
│           └── custom_10_tight.json
├── a3_rust/                     # Rust 落地 (W6 启动)
│   ├── Cargo.toml
│   └── src/
│       ├── main.rs              # axum HTTP server, port 9204
│       ├── dto.rs               # MultiStopReq/RoutePlanResp
│       ├── solver.rs            # plan_multistop() 纯函数
│       ├── energy.rs            # 等效距离 (手写几何)
│       └── heuristic.rs         # NN + 局部搜索
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

所有类型定义见 `A3_SCHEMA.md` §1。

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

### 日常开发循环

```
1. 写单测 (先于实现)
2. 实现功能 → 跑单测 (pytest -v)
3. 跑 benchmark (python a3_python/benchmark.py)
4. 文档同步 (每次里程碑完成后必须):
   - DEVPLAN: 勾选已完成项 + 更新底部"当前状态"和"下一步"
   - CLAUDE.md: 如果函数签名/环境/约定变化则更新
5. Git commit (见提交规范)
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

## 测试策略

### 测试金字塔

```
           ┌──────────────┐
           │   Benchmark   │  Solomon 全集 + OR-Tools/PyVRP/VeRyPy
           │   10+ instances│  每周跑，产出论文指标表
           ├──────────────┤
           │  Integration  │  标准实例 (R101, 自建 5/10/20 点)
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

- ❌ 在 `solver.py` 或 `solver.rs` 中 import PyVRP
- ❌ 在 Rust 侧引入 OR-Tools 绑定
- ❌ 使用网络调用、文件 I/O、全局可变状态
- ❌ 跳过单测直接提交核心算法代码
- ❌ 修改 `A3_SCHEMA.md` 的函数签名而不更新 `CLAUDE.md`

### 编码规范

- Python: type hints 全部标注；dataclass 不写 `__init__`
- Rust: `cargo fmt` + `cargo clippy -- -D warnings`
- 文档字符串: 中文注释，英文标识符
- 硬编码数值: 必须声明为模块级常量 (如 `DEFAULT_ALPHA = 0.1`)

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
