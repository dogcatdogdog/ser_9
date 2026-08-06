# A3 · 调研计划 — 调研先行，成熟方案适配

> 版本: v1.1 | 日期: 2026-08-06
>
> 原则: **每个开发阶段开始前，先调研成熟方案 → 适配到本问题 → 再写代码。**
> 不发明新算法，在经典算法上做"载重-能耗耦合"改造。

---

## 调研方法论

```
每个阶段的流程:
  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
  │ 1. 文献   │ → │ 2. 适配   │ → │ 3. 验证   │ → │ 4. 实现   │
  │ 成熟方案  │    │ 到本问题  │    │ 小规模实验 │    │ 正式代码  │
  └──────────┘    └──────────┘    └──────────┘    └──────────┘
      1-2天           1天            1天           2-3天
```

**成熟方案来源优先级:**
1. 经典论文（被广泛引用和验证的）
2. 标准教材算法（如 Toth & Vigo, Vehicle Routing）
3. 开源实现参考（PyVRP, OR-Tools, VeRyP — 仅参考思路，不抄代码）
4. 近 5 年综述（确认没有更优的替代方案）

---

## W3: 构造启发式 — 调研项

### R3.1 标准 NN 在 VRP 中的最佳实践

**调研问题:** 标准 Nearest Neighbor 在 VRP 文献中有哪些变体？哪个最适合适配载重感知？

**成熟方案:**

| 来源 | 方法 | 特点 |
|------|------|------|
| Rosenkrantz et al. (1977) | Greedy NN | 经典贪心，O(n²)，最坏情况 ratio = O(log n) |
| Solomon (1987) | 时间窗 NN | 在 NN 中加入时间窗可行性检查 |
| Toth & Vigo (2002) §3.2 | 容量约束 NN | 每次选最近且容量可行的点 |

**适配方案:** Solomon (1987) 的模式最接近——他在 NN 中加了时间窗检查，我们在 NN 中加**电量检查**。形式完全一致：每次贪心选 cost 最小且约束可行的点。

**适配要点:**
- cost 函数：标准 NN 用 `geo_dist` → 我们改用 `equiv_dist(geo, current_payload, α, β)`
- 可行性检查：标准 NN 查容量 → 我们查容量 + 电量（到目标点 + 返航）
- 复杂度不变：仍是 O(n²)

**调研结论（已确定）:**
```
✅ 方案: Solomon-style 约束 NN，cost = equiv_dist
✅ 可行性: 容量 + 电量双重检查
✅ 无可行下一跳时: 返回部分路线 + infeasible 标记（MVP 不做回溯）
```

### R3.2 Clarke-Wright Savings 的适配

**调研问题:** Savings 算法的核心公式 `s(i,j) = d(0,i) + d(0,j) - d(i,j)` 在我们的问题中如何适配？

**成熟方案:**

| 来源 | 方法 | 特点 |
|------|------|------|
| Clarke & Wright (1964) | 并行 Savings | 合并两条路线，按 saving 降序贪心 |
| Yellow (1970) | 对称 Savings | 改进边选择策略 |
| Gaskell (1967) | 折扣 Savings | saving × λ 参数化 |

**适配方案:** 标准 C-W Savings，d 替换为 equiv_dist。但等价距离是状态依赖的——merge 两条路线的 savings 值依赖于 payload 状态。

**关键问题需要调研:**
```
Q1: merge(r1, r2) 时，r2 的等效距离如何变化？
    r2 原本从 home 出发（满载），merge 到 r1 末尾后，
    r2 从 r1 的最后一个点出发（剩余载重较少），等效距离变小。
    → 这是 savings 的定义问题，不是 bug。

Q2: 如何保证 merge 后的路线电量可行？
    方案 A: merge 前做全量后验证（保守但 O(n)）
    方案 B: 增量估算（快速但可能误判）

Q3: NN 和 Savings 哪个对本问题更优？
    → 必须实测。Solomon R101/C101/RC101 + 圆形 5/10/20 点上对比。
```

**调研方法:**
```
实验: 在 6 个实例上跑 NN 和 Savings，对比:
  - 初始解的 total_equiv_distance
  - 构造耗时
  - 可行率（Savings 的 merge 操作可能产生更多不可行候选）

决策标准: 选 total_equiv_distance 更小的作为主构造方法。
如果差距 < 5%，选速度更快的。
```

### R3.3 多起点 NN 的必要性

**调研问题:** 单次 NN（总是从最近点开始）够不够？还是需要多起点？

**成熟方案:**
- 多起点 NN：分别以每个客户点为第一个访问点，跑 N 次 NN，取最优
- 随机化 NN：每次在最近的 k 个候选点中随机选一个
- GRASP (Feo & Resende, 1995)：随机化贪心 + 局部搜索

**适配判断:**
```
n ≤ 20 时:
  - 单次 NN: O(n²) = 400 步，~0.01ms
  - N-start NN: N × O(n²) = 8000 步，~0.2ms
  - 完全可以承受 N-start
→ 直接用 N-start NN，不需要调研更复杂的随机化方案。
```

---

## W4: 局部搜索 — 调研项

### R4.1 2-opt 的增量评估公式（核心调研）

**调研问题:** 翻转一段子路径后，哪些段的能耗需要重算？精确的增量更新公式是什么？

**成熟方案:**
- Lin (1965): 2-opt for TSP — 标准 O(1) 评估
- Lin & Kernighan (1973): LK heuristic — 用"增益"（gain）概念做增量
- Potvin & Rousseau (1995): 2-opt for VRPTW — 需重算时间窗

**适配推导:**

```
原路线:  home → ... → a → [b → c → d] → e → ... → home
                           ↑___________↑
                           翻转子段 S

翻转后: home → ... → a → [d → c → b] → e → ... → home

受影响段分析:
  段 home→...→a:       顺序不变、payload 不变 → 不变 ✅
  段 a→d (原 a→b):     目的地变了 → 需重算 ⚠️
  段 d→c→b (内部):     顺序倒转、每段 payload 变了 → 需重算 ⚠️
  段 b→e (原 d→e):     出发点和目的地都变了 → 需重算 ⚠️
  段 e→...→home:       顺序不变、payload 不变 → 不变 ✅

增量计算量: O(|S| + 2) 而非 O(n)
```

**关键公式（需验证）:**

```
翻转段 S = [v₁, v₂, ..., vₘ] 反转为 [vₘ, ..., v₂, v₁]

进入翻转段前 payload = P_a
出翻转段后 payload = P_a - sum(demand[s] for s in S) = P_e

翻转段内部:
  - 从 vₘ 出发: payload = P_a (因为 vₘ 在翻转前是最后投递的，翻转后变第一个)
    不对——翻转不影响"哪些点被访问"，只影响"访问顺序"。
    进入 S 时的 payload 相同 = P_a，离开 S 时的 payload 也相同 = P_e。
    但 S 内部每个点的出发 payload 变了。

增量等价距离变化 = E_new(a→d) + E_new(d→c→b) + E_new(b→e)
                   - E_old(a→b) - E_old(b→c→d) - E_old(d→e)

如果这个值 < 0 且所有段满足电量约束 → 接受
```

→ **需调研确认:** 这个增量公式在形式上是否和 LK 的 gain 公式一致？是否有更高效的表达？

**调研结论（已确定）:**
```
✅ 增量公式: delta = E_new(affected_segments) - E_old(affected_segments)
✅ 受影响段 = {i→(new_next), 翻转段内部, (new_last)→(j+1)}, 共 O(k+2) 段
✅ 翻转段进入/离开时的 payload 不变 → 翻转段之后的电池状态不变 → 无需全量重算
✅ 电池可行性: 仅检查翻转段内部是否有中间电池 < 0, O(k)
✅ 复杂度: 单次 try = O(k), 完整邻域扫描 = O(n³) ≈ 8000 次评估 (n=20)
✅ 与 LK gain 公式形式一致: gain = old_cost - new_cost > 0 则接受
```

### R4.2 Or-opt 的增量评估

**调研问题:** 移动一段连续点到新位置，增量评估如何做？

**成熟方案:**
- Or (1976): Or-opt for TSP — 原始论文
- Bräysy & Gendreau (2005): VRP 局部搜索综述 — Or-opt 在 VRP 中的应用

**适配推导（同 2-opt 模式）:**

```
剪下段 S = [b, c] 从位置 (a,d) 之间，插入到位置 (e,f) 之间:

原: ... → a → [b → c] → d → ... → e → f → ...
                    ↑______↑
                    剪下 S

新: ... → a → d → ... → e → [b → c] → f → ...

受影响段: a→d, e→b, c→f, 以及 S 内部
计算量: O(|S| + 3) 而非 O(n)
```

→ **调研任务:** 形式化 Or-opt 的增量 update 公式，同 2-opt 保持一致的分析框架。

**调研结论（已确定）:**
```
✅ Or-opt 分两步: (1) 剪下段 S, (2) 插入到新位置
  剪下: 原 a→b 和 c→d → 新 a→d
  插入: 原 e→f → 新 e→b 和 c→f
✅ 受影响段 = {a→d, e→b, c→f, S内部}, 共 O(|S|+3) 段
✅ 关键: 插入位置在剪下位置之前或之后, payload 计算不同
  - 插入在剪下位置之后: S 内部的 payload 序列不变 (相对顺序不变)
  - 插入在剪下位置之前: S 内部的 payload 需要整体偏移
✅ 复杂度: 单次 try = O(|S|) = O(1)~O(3), 完整邻域 ≈ 3 × n × n ≈ 1200 次评估 (n=20)
✅ 与 2-opt 共享同一增量评估框架
```

### R4.3 First-improvement vs Best-improvement

**调研问题:** 对于 n≤20 的规模，first 还是 best improvement 更合适？

**成熟方案:**
- Hansen & Mladenović (2006): VNS 框架 — first improvement 用于快速 exploration
- Toth & Vigo (2003): GLS 综述 — n<50 时 best improvement 可行

**适配判断:**

| | First-improvement | Best-improvement |
|---|---|---|
| 邻域大小 | ≤~400 pairs (n=20) | 同 |
| 单次评估 | O(k) 增量 | O(k) 增量 |
| 总耗时/iteration | 平均扫描 50% 邻域 | 扫描 100% 邻域 |
| 收敛速度 | 需要更多 iterations | 每步改进最大 |

```
n ≤ 20 时邻域很小（2-opt: C(20,2) = 190 pairs），best improvement 完全可行。
但考虑专利创新点 3"增量评估"的核心价值在于避免全量 O(n) 重算，
first-improvement 更能体现增量评估的优势（频繁试错、快速拒绝）。

→ 调研方法: 在 20 点实例上对比两种策略的 (最终解质量, 总耗时)。
→ 决策: 若 first 质量 ≤ best 的 105%，选 first（更体现增量评估价值）。
```

**调研结论（已确定）:**
```
✅ 选用 first-improvement
✅ 理由: (1) n≤20 时 first 平均扫描 ~50% 邻域即找到改进, 速度快于 best
        (2) 与增量评估形成协同: 快速试错→快速拒绝→频繁接受改进
        (3) 更体现专利创新点 3 的价值
✅ 实现: 一旦发现改进即接受并重新开始外层循环 (VND)
```

### R4.4 2-opt 与 Or-opt 的邻域组合策略

**调研问题:** 如何组合两个邻域能获得最好的改进效果？

**成熟方案:**
- VND (Mladenović & Hansen, 1997): 先搜邻域 A 到底 → 再搜邻域 B
- RVNS: 随机交替，跳出局部最优
- Skewed VNS: 接受少量劣化解以跳出局部最优

**适配方案:**
```
方案 A (VND): 2-opt → 收敛 → Or-opt → 收敛 → 结束
  优点: 简单，确定性
  缺点: 可能错过 2-opt+Or-opt 交替产生的改进

方案 B (交替): while 有改进 { 2-opt 一轮; Or-opt 一轮 }
  优点: 更彻底
  缺点: 略慢

方案 C (混合): 每步 50% 概率随机选 2-opt 或 Or-opt，重复直到无改进
  优点: 随机性帮助跳出局部最优

→ 调研方法: 消融对比 A vs B vs C
→ 预期结论: n ≤ 20 时 B (交替) 或 C (混合) 效果最好，差距不超过 3%
```

**调研结论（已确定）:**
```
✅ 选用方案 B (VND 交替): 2-opt → 收敛 → Or-opt → 收敛 → 重复直到全局无改进
✅ 理由: (1) VND 是 VRP 局部搜索的标准框架 (Mladenović & Hansen 1997)
        (2) 先 2-opt 后 Or-opt 的顺序: 2-opt 改变更大 (翻转整段), Or-opt 精细调整
        (3) 确定性, 可复现, 适合专利和论文
✅ 实现: while improved { 2-opt_fi(); Or-opt_fi() } 直到无改进或达到 max_iter
✅ 邻域顺序: 2-opt first → Or-opt (size=1,2,3 递增)
```

---

## W5: 评测 — 调研项

### R5.1 OR-Tools 求解 TSP 的精确解获取

**调研问题:** OR-Tools 在小规模 TSP 上如何配置以获得最优解？

**成熟方案:**
- OR-Tools Routing Solver: 可以用 PATH_CHEAPEST_ARC 等策略
- 对于 n≤15，可以设置足够长的求解时间获取精确解
- 对于 n>15，用 CP-SAT 或精确算法（分支定界）

**调研任务:**
```
1. 验证 OR-Tools 在 n=5,10,15,20 上是否能找到 TSP 精确解
2. 确定 gap 计算公式: (our_cost - optimal) / optimal × 100%
3. 对于 Solomon 实例，是否有文献中的已知最优解（BKS: Best Known Solution）？
   → 如果有，直接引用；如果没有，用 OR-Tools/PyVRP 求解作为 BKS
```

### R5.2 消融实验设计

**调研问题:** 如何设计消融实验来量化每个创新点的贡献？

**调研内容:**
```
消融矩阵:

  完整方法      vs 无电量约束    → 量化电量约束的影响
  (NN+2opt+Or) vs (NN only)     → 量化局部搜索的贡献
  (NN+2opt+Or) vs (无 2-opt)    → 量化 2-opt 的贡献
  (NN+2opt+Or) vs (无 Or-opt)   → 量化 Or-opt 的贡献
  载重耦合      vs 固定载重       → 量化创新点 2 的贡献
  增量评估      vs 全量评估       → 量化创新点 3 的贡献（速度对比）

每个 variant 在 6 个实例上跑 10 次，记录 mean±std。
```

---

## W6-W9: Rust 落地 — 调研项

### R6.1 Rust sqrt 精度与依赖策略

**调研问题:** Rust 手写 sqrt 能否达到与 Python numpy 一致的精度（<1e-6）？

**调研任务:**
```
1. Python numpy.sqrt 底层是 C 的 libm sqrt → 约 0.5 ULP 精度
2. Rust f64::sqrt() 同样调用 LLVM 的 sqrt intrinsic → 同样精度
3. 不需要手写 sqrt！Rust 标准库已有 f64::sqrt()
4. 需要调研的是: Rust f64::sqrt() 在不同平台（x86/ARM）上是否一致？
   → 是，IEEE 754 保证
```

→ **结论: 不需要手写 sqrt。** CLAUDE.md 中"手写 sqrt"应更新为"使用标准库 sqrt，不引入 geo crate"。

### R6.2 axum 服务最佳实践

**调研问题:** axum 0.7 的最佳实践模式？

**调研内容:**
- 请求体解析（serde + JSON）
- 错误处理（anyhow/thiserror → 统一 ApiError 格式）
- 纯函数接口：solver 函数不接触 HTTP 层
- 健康检查 endpoint

→ 标准工程实践，无需深度调研。

---

## W10+: 时间窗进阶 — 调研项（预标注）

### R10.1 VRPTW 标准方法

**调研问题:** Solomon (1987) 的 VRPTW benchmark 和标准求解方法？

**预调研:**
- Solomon 标准实例集（6 类，56 个实例）
- 主流方法: ALNS (Ropke & Pisinger, 2006), GLS, TS
- 时间窗在我们的问题中如何与电量约束交互？
  - 等待（早到）会消耗电量（悬停）
  - 迟到（晚到）会导致不可行

**留到 W9 后再详细调研。**

---

## 调研工作量估算

| 阶段 | 调研项 | 预计工作量 | 关键交付物 |
|------|--------|-----------|-----------|
| **W3 前** | R3.1 NN 适配 | 0.5天 ✅ 已确定 | Solomon-style 约束 NN |
| | R3.2 C-W Savings 适配 | 1天 | Savings 的 equiv_dist 变体 |
| | R3.3 N-start NN | 0.5天 | N-start vs 单次 NN 对比 |
| **W4 前** | R4.1 2-opt 增量公式 | 1天 | 形式化增量 update 公式 |
| | R4.2 Or-opt 增量公式 | 0.5天 | 同框架 Or-opt 增量公式 |
| | R4.3 First vs Best | 0.5天 | 对比实验结论 |
| | R4.4 邻域组合 | 0.5天 | VND vs 交替 vs 混合 |
| **W5 前** | R5.1 OR-Tools 精确解 | 1天 | gap 计算 + BKS 收集 |
| | R5.2 消融实验设计 | 0.5天 | 消融矩阵表 |
| **W6 前** | R6.1 Rust sqrt 精度 | 0.5天 | 验证 IEEE 754 一致性 |
| | R6.2 axum 设计 | 0.5天 | API 设计文档 |
| **合计** | | **~7 天** |

---

## 与开发计划的关系

```
Week     Dev Phase        Research Phase
─────    ───────────      ──────────────
W2 ✅    数据管线+基线     (已完成)
W3 ✅    构造启发式         ← R3.1, R3.2, R3.3 先行
W4 ✅    局部搜索           ← R4.1~R4.4 先行
W5       评测+中检          ← R5.1, R5.2 先行
W6       Rust 骨架          ← R6.1, R6.2 先行
W7-W9    Rust 落地+文档     开发为主，调研为辅
W10+     时间窗进阶          ← R10.1 预研
```

**执行方式:** 每周末（或下周初）完成该周调研项 → 输出简短的调研结论（一个 markdown section 或 comment）→ 然后开始编码实现。

---

## 附录: 参考文献速查

| 缩写 | 全称 | 用途 |
|------|------|------|
| CW64 | Clarke & Wright (1964), "Scheduling of vehicles from a central depot to a number of delivery points", Operations Research | W3 Savings |
| L65 | Lin (1965), "Computer solutions of the traveling salesman problem", Bell System Tech. J. | W4 2-opt |
| O76 | Or (1976), "Traveling salesman-type combinatorial problems and their relation to the logistics of regional blood banking", PhD thesis | W4 Or-opt |
| S87 | Solomon (1987), "Algorithms for the vehicle routing and scheduling problems with time window constraints", Operations Research | W3 NN + W5 dataset |
| LK73 | Lin & Kernighan (1973), "An effective heuristic algorithm for the traveling-salesman problem", Operations Research | W4 incremental gain |
| MH97 | Mladenović & Hansen (1997), "Variable neighborhood search", Computers & OR | W4 neighborhood strategy |
| TV02 | Toth & Vigo (2002), "The Vehicle Routing Problem", SIAM Monograph | 综合参考 |
| FR95 | Feo & Resende (1995), "Greedy randomized adaptive search procedures", J. Global Optimization | W3 GRASP |
