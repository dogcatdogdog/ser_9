# a3_rust — 无人机多目标访问路线规划 (Rust 服务)

A3 项目的 Rust 落地：载重-电量耦合约束下的多目标访问路线规划 (TSP/VRP)。

## 构建与测试

```bash
# 需要 Rust ≥ 1.75 (本机验证: rustc 1.98.0 + MSVC)
cargo build --release

# 全量测试 (99 例: 单测 93 + 交叉验证 2 + HTTP 集成 4) + lint
cargo test
cargo clippy --all-targets -- -D warnings
```

## 运行

```bash
cargo run          # 监听 0.0.0.0:9204
```

## API — POST /plan

请求 (A3_SCHEMA.md §3.1):

```json
{
  "targets": [
    {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0},
    {"id": "c2", "location": {"x": 0.0, "y": 100.0}, "demand": 5.0}
  ],
  "home": {"x": 0.0, "y": 0.0},
  "drone": {
    "payload_capacity": 30.0,
    "battery_capacity": 10000.0,
    "alpha": 0.1,
    "beta": 0.005
  }
}
```

响应 200 (RoutePlanResp, A3_SCHEMA.md §3.2) — 不可行是合法求解结果 (`feasible=false` + warnings)，非错误。

错误码 (错误体为顶层 `{"code": ..., "message": ...}`):

| 状态码 | code | 场景 |
|--------|------|------|
| 400 | `BAD_REQUEST` | JSON 解析失败 (语法/缺字段) / 参数校验失败 (空 targets / >20 点) |
| 422 | `INFEASIBLE` | 容量/电量约束无法满足 (保留, 当前路径返回 200+feasible=false 与 Python 对齐) |
| 500 | `INTERNAL` | 兜底 |

## 示例

```bash
curl -X POST http://127.0.0.1:9204/plan \
  -H "Content-Type: application/json" \
  -d '{
    "targets": [
      {"id": "c1", "location": {"x": 100.0, "y": 0.0}, "demand": 5.0},
      {"id": "c2", "location": {"x": 0.0, "y": 100.0}, "demand": 5.0}
    ],
    "home": {"x": 0.0, "y": 0.0},
    "drone": {"payload_capacity": 30.0, "battery_capacity": 10000.0,
               "alpha": 0.1, "beta": 0.005}
  }'
```

## 架构

```
src/
├── lib.rs       # 核心纯函数库 (dto/energy/heuristic/solver) + HTTP 层
├── dto.rs       # MultiStopReq / RoutePlanResp / SegmentDto / ApiError (§2.2)
├── energy.rs    # 等效距离变换 + 全量路线模拟 (标准库 f64::sqrt)
├── heuristic.rs # NN 构造 (N-start) + C-W Savings + 2-opt/Or-opt/VND (增量评估)
├── solver.rs    # plan_multistop 纯函数入口 (验证 → NN → VND)
├── http.rs      # axum Router: POST /plan (解析→调用→错误映射)
└── main.rs      # bin 启动 (port 9204)
tests/
├── cross_check.rs     # Python numpy vs Rust 矩阵 < 1e-6 (W6)
├── http_integration.rs# 真实 TCP 服务 + reqwest + golden 逐字段比对 (W7)
└── fixtures/          # energy_golden.json / solver_golden.json
scripts/
└── gen_*_golden.py    # golden 生成 (env312 python)
```

## 与 Python 的对齐 (W6/W7 强制)

- 等效距离公式: `geo × (α + β × payload) / α` — 交叉验证 < 1e-6
- 求解流程 1:1: NN 构造 (N-start) → 不可行直接返回 → VND (2-opt + Or-opt, 增量评估)
- golden: 6 个标准 fixture 的 Python 固定种子输出 vs Rust 服务响应逐字段比对 (sequence 精确, 浮点容差 0.02)
