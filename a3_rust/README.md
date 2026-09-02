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

## 运行配置 (env, 均有默认值)

| 变量 | 默认 | 说明 |
|------|------|------|
| `A3_PORT` | 9204 | 监听端口 |
| `A3_MAX_ITERATIONS` | 20 | VND 外层迭代上限 |
| `A3_TIME_LIMIT_SECS` | 5.0 | 求解时限 (<=0 不限; 正常求解 <150ms 不触发) |

## 健康检查

```bash
curl http://127.0.0.1:9204/healthz   # → 200 ok
```

## 工程化特性 (W8 硬化)

- **防崩溃**: 参数域校验 (alpha>0 等, 非法 → 400) + middleware panic 兜底 (→ 500 JSON, 不中断连接)
- **并发**: 求解在 `spawn_blocking` 线程池执行 (CPU 密集隔离 async worker), 请求超时兜底 10s
- **可观测**: tracing 结构化日志 (method/path/status/耗时)

## 架构

```
src/
├── lib.rs       # 核心纯函数库 (dto/energy/heuristic/solver) + HTTP 层
├── dto.rs       # MultiStopReq / RoutePlanResp / SegmentDto / ApiError (§2.2)
├── energy.rs    # 等效距离变换 + 全量路线模拟 (标准库 f64::sqrt)
├── heuristic.rs # NN 构造 (N-start) + C-W Savings + 2-opt/Or-opt/VND (增量评估)
├── solver.rs    # plan_multistop 纯函数入口 (参数校验 → NN → VND)
├── http.rs      # axum Router: ServiceConfig State + spawn_blocking + panic 兜底
│                #   + /healthz + 请求日志 (解析→调用→错误映射)
└── main.rs      # bin 启动: serve + tracing init + env 配置
tests/
├── cross_check.rs     # Python numpy vs Rust 矩阵 < 1e-6 (W6)
├── http_integration.rs# 真实 TCP 服务 + reqwest + golden 逐字段比对 (W7) + 并发 (W8)
└── fixtures/          # energy_golden.json / solver_golden.json
scripts/
└── gen_*_golden.py    # golden 生成 (env312 python)
```

## 与 Python 的对齐 (W6/W7 强制)

- 等效距离公式: `geo × (α + β × payload) / α` — 交叉验证 < 1e-6
- 求解流程 1:1: NN 构造 (N-start) → 不可行直接返回 → VND (2-opt + Or-opt, 增量评估)
- golden: 6 个标准 fixture 的 Python 固定种子输出 vs Rust 服务响应逐字段比对 (sequence 精确, 浮点容差 0.02)
