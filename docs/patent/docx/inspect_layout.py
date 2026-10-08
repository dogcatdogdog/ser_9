"""检查 docx 表格版式 —— 找出"不该被拆开却被拆开"的单元格内容

Word 的 ComputeStatistics(wdStatisticLines) 对**表格单元格**返回 0（实测），
不能用来测量换行。故改用**确定性判据**：

  对每个单元格，估算其内容所需宽度（汉字=1字宽, 西文/数字=半字宽, 加左右边距），
  与实际列宽比较。凡"放不下"的单元格，再按其被空白切分的**标记(token)**判断：
  若某个短标记（<=12 字符，如 155.8 / t10→t9 / 0.053）自身放不下，
  该标记必被拆成两行 —— 这正是不合理的换行。

用法: python -X utf8 docs/patent/docx/inspect_layout.py [docx]
退出码: 0=无问题, 1=有问题
"""
from __future__ import annotations

import os
import sys
import zipfile

from docx import Document
from docx.shared import Emu

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT = os.path.join(REPO, "docs", "patent_disclosure.docx")

MARGIN_PT = 4.5         # 单元格左右边距合计(与 build_docx.py 一致)
TOKEN_MAX = 12          # 含数字/西文的标记超过此长度视为正常文本
CJK_MAX = 4             # 纯中文标记超过此长度视为自然换行(不算缺陷)


def est_pt(text: str, font_pt: float) -> float:
    """估算文字渲染宽度(pt)"""
    return sum(font_pt if ord(ch) > 0x2E80 else font_pt * 0.5 for ch in text)


def col_widths(doc) -> list[list[float]]:
    """取每张表每列的宽度(pt); 单元格宽度取该列各行的最小值"""
    out = []
    for tbl in doc.tables:
        ncols = len(tbl.columns)
        ws = [1e9] * ncols
        for row in tbl.rows:
            for i, cell in enumerate(row.cells[:ncols]):
                w = cell.width
                if w is not None:
                    ws[i] = min(ws[i], Emu(w).pt)
        out.append([w if w < 1e9 else 0.0 for w in ws])
    return out


def main() -> int:
    docx = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    if not os.path.exists(docx):
        print(f"[FAIL] 不存在: {docx}")
        return 1

    doc = Document(docx)
    widths = col_widths(doc)
    problems: list[tuple] = []
    total_cells = 0

    for ti, tbl in enumerate(doc.tables, 1):
        cw = widths[ti - 1]
        for ri, row in enumerate(tbl.rows, 1):
            for ci, cell in enumerate(row.cells[:len(cw)]):
                txt = "".join(p.text for p in cell.paragraphs).strip()
                if not txt:
                    continue
                total_cells += 1
                avail = cw[ci] - MARGIN_PT
                if avail <= 0:
                    continue
                # 该单元格实际字号(取各 run 最大值), 缺省按正文 10.5pt
                fsz = max((r.font.size.pt for p_ in cell.paragraphs
                           for r in p_.runs if r.font.size), default=10.5)
                # 逐行(单元格内可能有换行)检查
                for seg in txt.splitlines() or [txt]:
                    for tok in seg.split():
                        need = est_pt(tok, fsz)
                        if need <= avail:
                            continue
                        has_alnum = any(c.isalnum() and ord(c) < 128 for c in tok)
                        limit = TOKEN_MAX if has_alnum else CJK_MAX
                        if len(tok) <= limit:
                            problems.append((ti, ri, ci, tok, round(need, 1),
                                             round(avail, 1)))

    print(f"文件: {os.path.basename(docx)}")
    print(f"表格 {len(doc.tables)} 张, 非空单元格 {total_cells} 个")
    print(f"\n会被拆开的短标记: {len(problems)} 处")
    if problems:
        print(f"\n  {'表':>3} {'行':>4} {'列':>3}  {'需要':>7}{'可用':>7}  内容")
        print("  " + "-" * 52)
        for ti, ri, ci, tok, need, avail in problems:
            print(f"  {ti:>3} {ri:>4} {ci:>3}  {need:>7}{avail:>7}  {tok}")
        return 1
    print("\n✅ 无被拆开的短标记（数字/编号均可完整显示于一行）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
