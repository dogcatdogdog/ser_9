"""模板结构探针 —— 输出单元格的真实网格坐标、合并状态与宽度

用途: 模板是 21x12 的大表格, 含大量合并单元格。python-docx 的 row.cells 会把合并区
重复返回, 直接按下标写内容会写错格。本脚本解析底层 XML, 给出:
  r{行}: 该行各单元格的 [网格起列, 跨列数, 宽度pt, 合并标记, 文本前20字]

用法: python -X utf8 docs/patent/docx/probe_template.py [docx路径]
"""
from __future__ import annotations

import os
import sys
import zipfile
import re

from docx import Document
from docx.oxml.ns import qn
from docx.shared import Emu

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.join(HERE, "..",
                       "技术交底书-一种基于自适应分级求解的无人机多目标路径规划方法.docx")


def grid_cols(tbl) -> list[float]:
    g = tbl._tbl.find(qn("w:tblGrid"))
    return [int(gc.get(qn("w:w"))) / 20.0 for gc in g.findall(qn("w:gridCol"))] if g else []


def main() -> int:
    docx = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    doc = Document(docx)
    tbl = doc.tables[0]
    cols = grid_cols(tbl)
    print(f"文件: {os.path.basename(docx)}")
    print(f"表格: {len(tbl.rows)} 行, 网格列数 {len(cols)}")
    print(f"网格列宽: {[round(c,1) for c in cols]}")
    print(f"版心宽: {Emu(doc.sections[0].page_width - doc.sections[0].left_margin - doc.sections[0].right_margin).pt:.1f}pt")
    print()
    for ri, row in enumerate(tbl.rows):
        tc_list = row._tr.findall(qn("w:tc"))
        gc = 0
        parts = []
        for tc in tc_list:
            span = int(tc.find(qn("w:tcPr")).find(qn("w:gridSpan")).get(qn("w:val"))) \
                if tc.find(qn("w:tcPr")) is not None and tc.find(qn("w:tcPr")).find(qn("w:gridSpan")) is not None else 1
            w = sum(cols[gc:gc + span]) if cols else 0.0
            txt = "".join(t.text or "" for t in tc.iter(qn("w:t")))[:22].replace("\n", " ")
            vm = tc.find(qn("w:tcPr")).find(qn("w:vMerge")) \
                if tc.find(qn("w:tcPr")) is not None else None
            mark = ""
            if span > 1:
                mark += f"←跨{span}列 "
            if vm is not None:
                mark += "▲纵向合并 " if vm.get(qn("w:val")) == "restart" else "(纵向续) "
            parts.append(f"[c{gc:>2}..{gc+span-1:>2} {w:6.1f}pt {mark}| {txt}]")
            gc += span
        print(f"r{ri:>2}: " + " ".join(parts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
