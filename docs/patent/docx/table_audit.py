"""逐表审计: 网格列宽 vs 内容所需宽度 —— 直接列出放不下的列

不依赖 Word, 只读 docx: 解析 tblGrid(Word 固定布局下真正使用的列宽) 与各单元格文字,
按"汉字=1字宽、西文=半字宽、加单元格边距"估算所需宽度, 输出宽度不足的列。

用法: python -X utf8 docs/patent/docx/table_audit.py [docx]
退出码: 0=全部放得下, 1=有放不下的列
"""
from __future__ import annotations

import os
import sys
import zipfile
import re
from xml.etree import ElementTree as ET

from docx import Document
from docx.oxml.ns import qn

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT = os.path.join(REPO, "docs", "patent_disclosure.docx")
MARGIN_PT = 4.5


def est(text: str, pt: float) -> float:
    return sum(pt if ord(c) > 0x2E80 else pt * 0.5 for c in text)


def grid_widths(tbl) -> list[float]:
    """从 tblGrid 读列宽(pt) —— Word 固定布局的实际依据"""
    grid = tbl._tbl.find(qn("w:tblGrid"))
    if grid is None:
        return []
    return [int(gc.get(qn("w:w"))) / 20.0 for gc in grid.findall(qn("w:gridCol"))]


def main() -> int:
    docx = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    doc = Document(docx)
    bad_total = 0
    print(f"{'表':>3} {'形状':>8}  放不下的列 (列号: 需/可用)")
    print("  " + "-" * 68)
    for ti, tbl in enumerate(doc.tables, 1):
        gw = grid_widths(tbl)
        ncol = len(gw)
        if ncol == 0:
            continue
        need = [0.0] * ncol
        worst = [""] * ncol
        for row in tbl.rows:
            for ci, cell in enumerate(row.cells[:ncol]):
                txt = "".join(p.text for p in cell.paragraphs).strip()
                if not txt:
                    continue
                fsz = max((r.font.size.pt for p in cell.paragraphs
                           for r in p.runs if r.font.size), default=10.5)
                # 单元格内可能有换行 -> 取最长一行
                for seg in txt.splitlines() or [txt]:
                    w = est(seg, fsz) + MARGIN_PT
                    if w > need[ci]:
                        need[ci], worst[ci] = w, seg[:22]
        short = [(i, need[i], gw[i], worst[i]) for i in range(ncol)
                 if need[i] > gw[i]]
        sig = "".join(p.text for p in tbl.rows[0].cells[0].paragraphs)[:10]
        if short:
            bad_total += len(short)
            desc = ", ".join(f"c{i}({n:.0f}/{g:.0f})" for i, n, g, _ in short[:5])
            print(f"{ti:>3} {len(tbl.rows)}x{ncol:<4} {desc}")
            for i, n, g, w in short[:3]:
                print(f"        c{i} 需{n:.1f} 可用{g:.1f}  内容: {w}")
        else:
            print(f"{ti:>3} {len(tbl.rows)}x{ncol:<4} ✅ 全部放得下      [{sig}]")
    print("  " + "-" * 68)
    print(f"放不下的列合计: {bad_total}")
    return 1 if bad_total else 0


if __name__ == "__main__":
    sys.exit(main())
