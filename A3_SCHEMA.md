# A3 · 数据结构与 Schema 定义

> 版本: v1.0 | 日期: 2026-08-05

---

## 1. 核心数据类型（Python）

```python
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class GeoPoint:
    """地理坐标点"""
    x: float          # 经度或平面 X (米)
    y: float          # 纬度或平面 Y (米)

@dataclass
class Target:
    """目标点（投递点 / 巡检点）"""
    id: str           # 唯一标识, 如 "c1", "c2"
    location: GeoPoint
    demand: float     # 货物需求量 (kg), 巡检点为 0
    # === 进阶 (W10+) ===
    tw_ready: Optional[float] = None   # 时间窗开始 (秒)
    tw_due: Optional[float] = None     # 时间窗结束 (秒)
    service_time: float = 0.0          # 停留时间 (秒)

@dataclass
class DroneSpec:
    """无人机规格"""
    payload_capacity: float   # 最大载重 (kg)
    battery_capacity: float   # 电池总能量 (Wh)
    alpha: float              # 空载能耗率 (Wh/m)
    beta: float               # 载重敏感系数 (Wh/m/kg)
    cruise_speed: float = 10.0  # 巡航速度 (m/s), 用于时间窗计算

@dataclass
class RoutePlan:
    """路线规划结果"""
    sequence: list[str]       # 访问顺序 (id 列表), 不含 home
    segments: list[Segment]   # 各段详情
    total_geo_distance: float      # 总几何距离 (m)
    total_equiv_distance: float    # 总等效电量距离 (m)
    total_energy_consumed: float   # 总耗电 (Wh)
    remaining_energy: float        # 剩余电量 (Wh)
    total_payload_delivered: float # 总送货量 (kg)
    feasible: bool                 # 是否满足所有约束
    warnings: list[str] = field(default_factory=list)

@dataclass
class Segment:
    """路线中的一段"""
    from_id: str        # 出发点 id (home 或客户 id)
    to_id: str          # 到达点 id
    geo_distance: float           # 几何距离 (m)
    equiv_distance: float         # 等效电量距离 (m)
    energy_consumed: float        # 本段耗电 (Wh)
    payload_before: float         # 出发时载重 (kg)
    payload_after: float          # 到达+投递后载重 (kg)
    battery_before: float         # 出发时电量 (Wh)
    battery_after: float          # 到达时电量 (Wh)
```

## 2. 函数签名

### 2.1 Python 验证签名

```python
def plan_multistop(
    targets: list[Target],
    home: GeoPoint,
    drone: DroneSpec,
    seed: int = 42
) -> RoutePlan:
    """
    无人机多目标访问路线规划。

    Args:
        targets: 目标点列表 (1-20 个)
        home: 仓库位置
        drone: 无人机规格
        seed: 随机种子, 保证确定性 (默认 42)

    Returns:
        RoutePlan: 路线规划结果，包含访问序列、各段详情、可行性判定

    Raises:
        ValueError: 如果 target 数量为 0 或超过上限
    """
```

### 2.2 Rust 落地签名（预定义）

> Rust DTO 是 Python dataclass 的 1:1 翻译: `TargetDto`、`GeoPointDto`、`DroneSpecDto`
> 分别对应 `Target`、`GeoPoint`、`DroneSpec`，字段名和类型完全一致。

```rust
/// 输入请求 (JSON deserialized)
#[derive(Deserialize)]
pub struct MultiStopReq {
    pub targets: Vec<TargetDto>,
    pub home: GeoPointDto,
    pub drone: DroneSpecDto,
}

/// 输出响应 (JSON serialized)
#[derive(Serialize)]
pub struct RoutePlanResp {
    pub sequence: Vec<String>,
    pub segments: Vec<SegmentDto>,
    pub total_geo_distance: f64,
    pub total_equiv_distance: f64,
    pub total_energy_consumed: f64,
    pub remaining_energy: f64,
    pub total_payload_delivered: f64,
    pub feasible: bool,
    pub warnings: Vec<String>,
}

/// 输出响应的一个段
#[derive(Serialize)]
pub struct SegmentDto {
    pub from_id: String,
    pub to_id: String,
    pub geo_distance: f64,
    pub equiv_distance: f64,
    pub energy_consumed: f64,
    pub payload_before: f64,
    pub payload_after: f64,
    pub battery_before: f64,
    pub battery_after: f64,
}

/// 求解器配置
#[derive(Deserialize)]
pub struct Defaults {
    pub max_iterations: usize,     // 最大迭代次数
    pub time_limit_secs: f64,      // 时间上限 (秒)
    pub seed: u64,                 // 随机种子 (固定可复现)
}

/// API 错误类型
#[derive(Serialize)]
pub struct ApiError {
    pub code: String,              // 错误码, 如 "INFEASIBLE", "BAD_REQUEST"
    pub message: String,           // 人类可读的错误信息
}

/// 纯函数入口
pub fn plan_multistop(
    req: &MultiStopReq,
    cfg: &Defaults
) -> Result<RoutePlanResp, ApiError>
```

## 3. JSON Schema（HTTP API）

### POST /plan — Request

```json
{
  "targets": [
    {
      "id": "c1",
      "location": { "x": 10.0, "y": 5.0 },
      "demand": 5.0
    },
    {
      "id": "c2",
      "location": { "x": 5.0, "y": 12.0 },
      "demand": 8.0
    }
  ],
  "home": { "x": 0.0, "y": 0.0 },
  "drone": {
    "payload_capacity": 20.0,
    "battery_capacity": 5000.0,
    "alpha": 0.1,
    "beta": 0.005
  }
}
```

### POST /plan — Response (200 OK)

```json
{
  "sequence": ["c2", "c1"],
  "segments": [
    {
      "from_id": "home",
      "to_id": "c2",
      "geo_distance": 1300.0,
      "equiv_distance": 1385.0,
      "energy_consumed": 138.5,
      "payload_before": 13.0,
      "payload_after": 5.0,
      "battery_before": 5000.0,
      "battery_after": 4861.5
    },
    {
      "from_id": "c2",
      "to_id": "c1",
      "geo_distance": 860.0,
      "equiv_distance": 882.0,
      "energy_consumed": 88.2,
      "payload_before": 5.0,
      "payload_after": 0.0,
      "battery_before": 4861.5,
      "battery_after": 4773.3
    },
    {
      "from_id": "c1",
      "to_id": "home",
      "geo_distance": 1118.0,
      "equiv_distance": 1118.0,
      "energy_consumed": 111.8,
      "payload_before": 0.0,
      "payload_after": 0.0,
      "battery_before": 4773.3,
      "battery_after": 4661.5
    }
  ],
  "total_geo_distance": 3278.0,
  "total_equiv_distance": 3385.0,
  "total_energy_consumed": 338.5,
  "remaining_energy": 4661.5,
  "total_payload_delivered": 13.0,
  "feasible": true,
  "warnings": []
}
```

### POST /plan — Response (422 / 不可行)

```json
{
  "sequence": [],
  "segments": [],
  "total_geo_distance": 0,
  "total_equiv_distance": 0,
  "total_energy_consumed": 0,
  "remaining_energy": 5000.0,
  "total_payload_delivered": 0,
  "feasible": false,
  "warnings": [
    "Total payload (26.0kg) exceeds drone capacity (20.0kg)"
  ]
}
```

## 4. 约束校验规则

```
feasibility_check(route):
    payload = sum(all demands)           # 初始载重
    battery = battery_capacity           # 初始电量

    for each segment (from i to j):
        equiv_dist = geo_dist(i,j) * (alpha + beta * payload) / alpha
        energy = equiv_dist * alpha       # 本段耗电

        if payload > payload_capacity:    # 载重超限
            return INFEASIBLE("overload")

        if energy > battery:              # 电量不足
            return INFEASIBLE("low_battery")

        payload -= demand_of(j)           # 投递后载重减少
        battery -= energy                 # 剩余电量更新

    # 最后一段 (返回 home)
    if battery < alpha * geo_dist(last, home):
        return INFEASIBLE("cannot_return")

    return FEASIBLE
```

## 5. 测试基础设施

### 5.1 测试金字塔

```
           ┌──────────────┐
           │   Benchmark   │  Solomon 全量 + OR-Tools/PyVRP/VeRyPy 对比
           │   10+ instances│  每周跑一次，产出论文指标表
           ├──────────────┤
           │  Integration  │  标准实例 (R101, 自建 5/10/20 点)
           │   5-8 cases   │  验证完整求解流程，对比已知最优解
           ├──────────────┤
           │  Unit Tests   │  mock 数据，验证单一模块正确性
           │   ≥10 cases   │  每次 commit 跑，< 5s 全部通过
           └──────────────┘
```

### 5.2 单元测试 (pytest, ≥10 例)

**目的**: 验证单一函数的正确性，每次 commit 跑，必须 < 5s。

**测试数据**: Python 内联 mock — 2~5 个硬编码坐标点，无需外部文件。

| # | 类型 | 场景 | 断言 | 对应专利创新点 |
|---|------|------|------|:---:|
| 1 | 正例 | 3 点、电量充裕 | feasible=True, 所有点被访问 | 等效距离变换 |
| 2 | 正例 | 10 点、电量充裕 | feasible=True, distance 合理 | 等效距离变换 |
| 3 | 退化 | 3 点、电量只够其中 2 点 | feasible=False, warnings 包含 "low_battery" | 增量校验 |
| 4 | 退化 | 总载重 > capacity | feasible=False, warnings 包含 "overload" | 载重约束 |
| 5 | 边界 | 1 点 | feasible=True, straight line | 等效距离变换 |
| 6 | 边界 | 0 点 | ValueError | — |
| 7 | 边界 | 满载 vs 空载 同段 | equiv_dist(满载) > equiv_dist(空载) | 载重-能耗耦合 |
| 8 | 一致性 | 相同输入、相同 seed | 输出完全一致 (确定性) | — |
| 9 | 能量模型 | 先送重货 vs 先送轻货 | 访问顺序不同 → equiv_dist 不同 | 载重-能耗耦合 |
| 10 | 回归 | 5 点已知最优解 | 启发式解与最优解的 gap < 10% | 整体方法 |

### 5.3 集成测试 (pytest, 5-8 例)

**目的**: 在标准 VRP 实例上验证完整求解流程，对比已知最优解验证 gap。

**测试数据**: 
- Solomon VRPTW 前 20 点 (R101, C101, RC101 三类，覆盖不同空间分布)
- 自建无人机配送场景 (5/10/15 点，含载重和电量约束)

**Solomon 实例类型**:
| 类型 | 特点 | 实例 |
|------|------|------|
| C 类 (Clustered) | 点聚集分布 | C101, C102 |
| R 类 (Random) | 点随机分布 | R101, R102 |
| RC 类 (Mixed) | 混合分布 | RC101, RC102 |

**集成测试用例**:
| # | 数据集 | 点数 | 断言 |
|---|--------|------|------|
| 11 | Solomon R101 (截取 20 点) | 20 | feasible=True, gap vs OR-Tools < 10% |
| 12 | Solomon C101 (截取 20 点) | 20 | feasible=True, gap vs OR-Tools < 10% |
| 13 | Solomon RC101 (截取 20 点) | 20 | feasible=True, gap vs OR-Tools < 10% |
| 14 | 自建 5 点 (含载重) | 5 | feasible=True, 载重约束满足 |
| 15 | 自建 10 点 (含电量紧张) | 10 | feasible=False (电量不够), 原因明确 |
| 16 | 自建 15 点 (载重+电量联合) | 15 | feasible=True, 与 OR-Tools 无电量版的解不同 |

### 5.4 Benchmark 管线

**目的**: 生成论文-ready 指标表，每周或里程碑节点跑。

**输入**: 
- Solomon VRPTW 全集 (56 个实例，取前 20 点的子集)
- 或自建配送场景集 (10/15/20 点，各 5 个随机种子)

**对比基线**:

| 基线 | 求解器 | 用途 |
|------|--------|------|
| 最优解 (Optimal) | OR-Tools 精确求解 | 小规模 (≤15) gap 计算 |
| PyVRP 无电量 | `pyvrp.solve()` | 验证电量约束改变了最优解 |
| VeRyPy 启发式 | NN, Savings, Sweep, CI, 3-opt | 消融实验: 我们的构造+搜索 vs 经典方法 |
| 我们的方法 | `plan_multistop()` | 被测方法 |

**输出指标** (每个实例):
```
instance | points | our_cost | our_time | opt_cost | gap% | pyvrp_cost | feasible
R101_20  | 20     | 4521     | 2.3s     | 4380     | 3.2% | 4210       | true
C101_20  | 20     | 3890     | 1.8s     | 3850     | 1.0% | 3850       | true
...
```

**消融矩阵**:
```
                    总等效距离 | 求解时间 | 可行率
完整方法 (NN+2opt+Or)   100%    |  100%    | 100%
- 去掉 Or-opt           105%    |   60%    | 100%
- 去掉 2-opt            112%    |   40%    |  95%
- 只用 NN 构造          120%    |   10%    |  90%
- 固定载重 (beta=0)      *      |  100%    |  85% ← 误判可行!
```

### 5.5 测试数据目录结构

```
a3_python/
├── tests/
│   ├── conftest.py              # 共享 fixtures
│   ├── test_energy_model.py     # 单元测试 1-10
│   ├── test_solver.py
│   ├── test_heuristic.py
│   ├── test_integration.py      # 集成测试 11-16
│   └── fixtures/
│       ├── solomon_r101_n20.json   # Solomon 子集
│       ├── solomon_c101_n20.json
│       ├── custom_5_heavy.json     # 自建: 5点, 重载场景
│       ├── custom_10_tight.json    # 自建: 10点, 电量紧张
│       └── custom_15_mixed.json    # 自建: 15点, 联合约束
└── benchmark.py                # 批量评测脚本
