# ser_9 — 无人机多目标路径规划（A3）

单架无人机从仓库出发，一次访问多个投递/巡检点后返航；在**载重上限**与**电池续航**
联合约束下，求总等效能耗最小的访问顺序。

**专利主张**：「一种基于自适应分级求解的无人机多目标路径规划方法」——
按运行时资源预算在**精确模式**与**降级（启发式）模式**间自动选择，且两模式**互为支撑**：
降级解既作预算不足时的输出，又作精确搜索的**剪枝上界**（状态空间削减 41.6%~94.2%），
从而把"在线精确最优"的适用范围从 n≤15 推至 **n≤20**（1.96 s，原实现 96.5 s）。

## 交付物

| 交付物 | 路径 | 说明 |
|---|---|---|
| **技术交底书（正式版）** | `docs/patent/技术交底书-一种基于自适应分级求解的无人机多目标路径规划方法.docx` | **在机构模板副本上填写**；42 页 / 314 个 Word 原生公式 / 18 张表 / 10 张附图 |
| 交底书（通用版） | `docs/patent_disclosure.docx` | 自建"矿业式 8 节"格式，内容相同 |
| 交底书 Markdown 源 | `docs/patent_disclosure.md` | 内容母本（改内容改这里） |
| 附图源文件 | `docs/patent/交付/附图/` | 10 张图的 PNG(300dpi) + PDF(矢量) |

> 正式版在机构模板**副本**上填写，原模板 `docs/patent/1-长沙军民先进技术研究有限公司技术交底书模板.docx` 保持未动。
> 表头人员字段（发明人/身份证/联系人）留 **【待填】**，勾选项留空由申请人勾选。

## 代码

| 目录 | 内容 |
|---|---|
| `a3_python/` | 算法实验：`solver.py` 核心入口、`exact_battery.py` 精确求解、`adaptive.py` 自适应入口（**267 单测**） |
| `a3_rust/` | 生产服务：axum HTTP（端口 9204），仅降级模式（**109 单测**）⚠️ W9 精确/自适应模式**未移植** |

## 快速开始

```bash
# 单测
"D:/ser_9/env312/python.exe" -m pytest a3_python/tests/ -q

# 交底书流水线（改 md 后重新生成，三项校验）
python -X utf8 docs/patent/docx/build_docx.py         # 通用版
python -X utf8 docs/patent/docx/build_template_docx.py # 机构模板版
python -X utf8 docs/patent/docx/verify_docx.py        # 公式/表格/图片/覆盖率
python -X utf8 docs/patent/docx/pdf_token_check.py    # ground truth: 表格是否被换行拆开
```

## 文档

- `CLAUDE.md` — 项目约定、目录结构、命令规范（**开发前必读**）
- `A3_REQUIREMENTS.md` / `A3_ARCHITECTURE.md` / `A3_SCHEMA.md` / `A3_DEVPLAN.md` / `A3_RESEARCH_PLAN.md`
- `docs/exploration/` — 探索期归档（含文献核查结论 `NOVELTY_VERDICT.md`、精确模式过程记录）
