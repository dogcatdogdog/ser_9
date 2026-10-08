# 探索期材料归档（分支 `worktree-tw-feasibility`）

> **性质**：过程记录归档，**不是**交付物，也未随 W9 一并并入主干。
> **来源**：本地分支 `worktree-tw-feasibility`（末端提交 `2c3425e`，2026-09-10）。
> 该分支从未推送到远端，本目录于 W10 材料收口时整体取回，以免随机器丢失。

## 1. 为什么整目录取回

目录内文档互相引用（`STEP1_PERF` ↔ `WMLP_INSIGHT` ↔ `TW_FEASIBILITY` ↔ `TW_RESULTS`），
单独取某几个会让引用继续悬空。**整目录为一个不可拆的记录单元。**

## 2. ⚠️ 路径说明（重要）

**本目录文档中的 `a3_python/tw/...` 路径在主干中不存在**——`a3_python/tw/` 这个包
有意未并入主干（见 §4）。归档保留原文以存证，**照抄文中的复现命令会失败**。
已并入主干的对应实现与复现入口见下表右列。

| 归档文件 | 内容与结论 | 主干对应实现 / 复现入口 |
|---|---|---|
| `WMLP_INSIGHT.md` | 总能耗恒等式 $E=\alpha D_{\text{total}}+\beta\sum_i q_iD_i$；带电池精确解 | 交底书 §5.2（该恒等式经核查属已知技术，见 `NOVELTY_VERDICT.md`） |
| `STEP1_PERF.md` | 分层向量化 + 上界剪枝的完整改造过程与性能数据；**96.5 s → 1.96 s、状态削减 41.6%~94.2% 的原始出处** | `a3_python/exact_battery.py`；`docs/patent/scripts/run_exact_perf.py` |
| `STEP1_PERF.md` §3.5 | 顺带发现：**电量约束只是可行性闸门**（不改变最优序列） | `a3_python/exact_battery.py` 模块 docstring；交底书 §5.5 |
| `STEP2_THRESHOLD.md` | 服务预算（5 s / 2 GB / 10 并发）下的覆盖阈值 **n ≤ 20**；含两处对早期记录的修正（见 §3） | `a3_python/adaptive.py`；数据 `results/step1_perf_clean.json` |
| `NOVELTY_VERDICT.md` | 新颖性核查：数学与算法层面无新颖性空间，各声称点均被在先文献覆盖 | 交底书附录"创造性评述"的已知技术清单 |
| `TW_FEASIBILITY.md` / `TW_MECHANISM.md` / `TW_RESULTS.md` | 时间窗 + 悬停耗电方向的**否定性结论**（V1–V3） | 无（**有意未并入**，见 §4） |
| `step1_perf_clean.json` | 探索期原始快照（纯净计时） | **权威副本在 `results/step1_perf_clean.json`**（W9 移植时重跑），两处数值有微小计时差异，以 `results/` 为准 |
| `step1_perf_data.json` | 峰值内存（含 `tracemalloc`） | 交底书 §6.5 内存数据 |
| `step2_threshold.json` | 阈值表原始数据 | 同上 |
| `adaptive_validation.md` / `.json` | ⚠️ **已作废，勿引用** | 见 §3 |

## 3. 已作废的记录（留档以防重蹈）

`adaptive_validation.{md,json}` 是**唯一一次用 `tracemalloc` 包裹计时**跑出来的验证，
结论是"建议精确模式阈值 **n ≤ 0**"（n=15 需 19.74 s）。**该结论错误**：

- `tracemalloc` 追踪每次浮点分配，对纯 Python 参照实现的惩罚（本机实测 11×）
  远大于 numpy 实现（3×），测出的耗时严重失真；
- 纯净计时下 n=15 的参照实现只需 1.62 s，**n=20 的加速版只需 1.96 s**。

修正过程与正确数据见 `STEP2_THRESHOLD.md` §3.1 与 `STEP1_PERF.md` §0。
这两个文件**不进 `results/`**（那里是交付用的证据集），只在此留档。

## 4. `a3_python/tw/` 为什么没并入主干

其末端提交 `f9eb614` 的标题即为结论：**「时间窗 + 悬停耗电可行性验证 (V1-V3)：结论否定，不并入主干」**。
`TW_MECHANISM.md` 逐项论证了该方向没有被迫发明任何非平凡新机制。
相应代码（`a3_python/tw/**`、`tests/test_tw_*.py`、`tw_solomon_*.json` fixtures）
**仅存在于分支 `worktree-tw-feasibility`**；若需复查，从该分支检出。

## 5. 引用本目录的正式文档

- `CLAUDE.md` — 专利主张出处
- `docs/patent/DATA_CHECK.md` — §F 数据溯源
- `docs/patent/FIGURES.md` — 图 7 / 图 10 的数据来源
