"""按长沙军民模板组装技术交底书 DOCX

以模板副本为基底, 把重组后的内容写入模板表格的对应单元格。

模板结构(已探明): 整份文件是一个 21x12 大表格
  r11 发明名称 / r12 技术领域 / r13-r14 背景技术 / r15-r17 技术方案 / r18 有益效果
  内容格统一为每行最后一格 [c4..c11] 330.5pt; r13-r17 另有一格提示文字 [c1..c3] 136.6pt
  -> 把提示格并入内容格可得 [c1..c11] 467pt (提示文字填完即删, 其小节标题需保留)

流程:
  1. pandoc: content_for_template.md -> _content.docx (公式转 OMML)
  2. 样式注入: reference.docx 的样式并入模板 styles.xml (模板仅 5 条数字ID样式, 不冲突)
  3. 打开模板副本: 合并提示格 -> 清格 -> 按槽位移植内容 -> 表头填占位
  4. 表格列宽按内容格宽重排 (复用 build_docx 的算法)
  5. 文末追加说明书附图
输出: docs/patent/技术交底书-<发明名称>.docx (就地覆盖副本)

复现: python -X utf8 docs/patent/docx/build_template_docx.py
"""
from __future__ import annotations

import copy
import io
import os
import re
import shutil
import sys
import zipfile

import pypandoc
from docx import Document
from docx.shared import Pt, Emu, Cm
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
PATENT_DIR = os.path.join(REPO, "docs", "patent")
TEMPLATE = os.path.join(PATENT_DIR, "1-长沙军民先进技术研究有限公司技术交底书模板.docx")
TARGET = os.path.join(PATENT_DIR, "技术交底书-一种基于自适应分级求解的无人机多目标路径规划方法.docx")
CONTENT_MD = os.path.join(PATENT_DIR, "content_for_template.md")
REFERENCE = os.path.join(HERE, "reference.docx")

sys.path.insert(0, HERE)
from build_docx import (  # noqa: E402
    est_text_width_pt, cell_text, reorder_tblPr,
)

# 槽位标题(md 二级标题) -> 模板行号(内容格 = 该行最后一格)
SLOT_ROW = {
    "发明名称": 11,
    "技术领域": 12,
    "背景技术-缺点": 13,
    "背景技术-技术问题": 14,
    "技术方案-方案": 15,
    "技术方案-替代方案": 16,
    "技术方案-关键创新点": 17,
    "有益效果": 18,
}
# 这些行可把"提示文字"格并入内容格以拓宽 (r11/r12/r18 的标签与提示同格, 不能并)
MERGE_GUIDE_ROWS = (13, 14, 15, 16, 17)
# 需保留的小节标题(从提示格里提取)
SLOT_LABEL = {
    13: "1、现有技术的缺点：",
    14: "2、发明要解决的技术问题：",
    15: "1、本发明的技术方案",
    16: "2、替代方案",
    17: "3、本发明的关键创新点",
}
TABLE_FONT_OVERRIDE = {"实例组": 8.0, "目标点": 8.0, "目标点数": 9.0, "航段": 9.0}
WIDTH_SAFETY = 1.06
MARGIN_PT = 7.0


# ---------------------------------------------------------------- 样式注入

def inject_styles() -> None:
    """把 reference.docx 的样式并入模板副本的 styles.xml (ID 冲突者跳过)"""
    ref = zipfile.ZipFile(REFERENCE).read("word/styles.xml").decode("utf-8")
    ref_styles = re.findall(r"<w:style\b.*?</w:style>", ref, re.S)

    with zipfile.ZipFile(TARGET) as z:
        tgt = z.read("word/styles.xml").decode("utf-8")
        names = set(re.findall(r'w:styleId="([^"]+)"', tgt))
        add = [s for s in ref_styles
               if (re.search(r'w:styleId="([^"]+)"', s) or [None, None])[1] not in names]
        tgt = tgt.replace("</w:styles>", "".join(add) + "</w:styles>")
        others = {n: z.read(n) for n in z.namelist() if n != "word/styles.xml"}
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as out:
        for n, d in others.items():
            out.writestr(n, d)
        out.writestr("word/styles.xml", tgt.encode("utf-8"))
    with open(TARGET, "wb") as f:
        f.write(buf.getvalue())
    print(f"  样式注入: 新增 {len(add)} 条")


# ---------------------------------------------------------------- 内容解析

def split_sections() -> dict[str, list]:
    """pandoc 转 content_for_template.md, 按二级标题切成 {标题: [body元素]}"""
    tmp = os.path.join(HERE, "_content.docx")
    pypandoc.convert_file(CONTENT_MD, "docx", outputfile=tmp,
                          extra_args=[f"--reference-doc={REFERENCE}"])
    body = Document(tmp).element.body
    sections: dict[str, list] = {}
    cur = None
    for el in list(body):
        if el.tag == qn("w:p") and el.find(qn("w:pPr")) is not None \
                and el.find(qn("w:pPr")).find(qn("w:pStyle")) is not None \
                and el.find(qn("w:pPr")).find(qn("w:pStyle")).get(qn("w:val")) == "Heading2":
            cur = "".join(t.text or "" for t in el.iter(qn("w:t"))).strip()
            sections[cur] = []
        elif cur is not None and el.tag in (qn("w:p"), qn("w:tbl")):
            sections[cur].append(el)
    os.remove(tmp)
    return sections


# ---------------------------------------------------------------- 单元格操作

def clear_cell(cell) -> None:
    """清空单元格内容, 保留一个空段落"""
    tc = cell._tc
    for child in list(tc):
        if child.tag != qn("w:tcPr"):
            tc.remove(child)
    tc.append(OxmlElement("w:p"))


def put_paragraph(cell, text: str, bold: bool = True) -> None:
    p = OxmlElement("w:p")
    r = OxmlElement("w:r")
    if bold:
        rPr = OxmlElement("w:rPr")
        rPr.append(OxmlElement("w:b"))
        r.append(rPr)
    t = OxmlElement("w:t")
    t.text = text
    t.set(qn("xml:space"), "preserve")
    r.append(t)
    p.append(r)
    cell._tc.append(p)


def append_elements(cell, elements: list) -> int:
    """把 body 元素深拷贝进单元格"""
    n = 0
    for el in elements:
        cell._tc.append(copy.deepcopy(el))
        n += 1
    # 表格单元格必须以段落结尾
    kids = [c for c in cell._tc if c.tag != qn("w:tcPr")]
    if not kids or kids[-1].tag != qn("w:p"):
        cell._tc.append(OxmlElement("w:p"))
    return n


def merge_guide_cell(tbl, row: int) -> None:
    """把该行提示文字格 [c1..c3] 并入内容格 [c4..c11]"""
    tbl.cell(row, 1).merge(tbl.cell(row, 11))


def fit_nested_tables_of_cell(cell, usable_pt: float):
    """对单个单元格内的嵌套表重排"""
    class _W:
        def __init__(self, c): self._cells = [c]
    return fit_nested_tables(_W(cell), usable_pt)


def fit_nested_tables(tbl, usable_pt: float) -> tuple[int, int]:
    """把某单元格内的嵌套表按可用宽度重排列宽(同 build_docx 的算法)"""
    from docx.oxml.ns import qn as _qn
    done = shrunk = 0
    margin_pt = MARGIN_PT
    for cell in tbl._cells:                     # noqa: SLF001
        for nested in cell.tables:
            cols = len(nested.columns)
            rows = nested.rows
            if cols == 0 or len(rows) == 0:
                continue

            def need_at(fp):
                nd = [0.0] * cols
                for r in rows:
                    for c in range(min(cols, len(r.cells))):
                        txt = cell_text(r.cells[c]).replace(chr(10), " ").strip()
                        if txt:
                            nd[c] = max(nd[c], (est_text_width_pt(txt, fp) + margin_pt)
                                        * WIDTH_SAFETY)
                return nd

            sig = cell_text(rows[0].cells[0]) if len(rows[0].cells) else ""
            font_pt, need = 10.5, need_at(10.5)
            if sig in TABLE_FONT_OVERRIDE:
                font_pt, need = TABLE_FONT_OVERRIDE[sig], need_at(TABLE_FONT_OVERRIDE[sig])
                shrunk += 1
            elif sum(need) > usable_pt:
                for cand in (10.0, 9.5, 9.0, 8.5, 8.0, 7.5):
                    nd = need_at(cand)
                    if sum(nd) <= usable_pt:
                        font_pt, need = cand, nd
                        break
                else:
                    font_pt, need = 7.5, need_at(7.5)
                shrunk += 1

            total = sum(need)
            if total <= usable_pt:
                extra = (usable_pt - total) / cols
                widths = [n + extra for n in need]
            else:
                k = usable_pt / total
                floor = (margin_pt + font_pt * 2) * WIDTH_SAFETY
                widths = [max(floor, n * k) for n in need]
                tot = sum(widths)
                if tot > usable_pt:
                    widths = [w * usable_pt / tot for w in widths]

            nested.autofit = False
            nested.alignment = 1
            tblPr = nested._tbl.tblPr
            for tag in ("w:tblLayout", "w:tblW", "w:tblInd", "w:tblCellMar"):
                old = tblPr.find(_qn(tag))
                if old is not None:
                    tblPr.remove(old)
            ind = OxmlElement("w:tblInd")          # 清掉继承来的缩进
            ind.set(_qn("w:w"), "0"); ind.set(_qn("w:type"), "dxa")
            tblPr.append(ind)
            cm = OxmlElement("w:tblCellMar")       # 嵌套表单元格边距收紧
            for side, tw in (("top", "0"), ("left", "28"), ("bottom", "0"), ("right", "28")):
                e = OxmlElement(f"w:{side}"); e.set(_qn("w:w"), tw); e.set(_qn("w:type"), "dxa")
                cm.append(e)
            tblPr.append(cm)
            lay = OxmlElement("w:tblLayout"); lay.set(_qn("w:type"), "fixed"); tblPr.append(lay)
            tw = OxmlElement("w:tblW"); tw.set(_qn("w:w"), str(int(sum(widths) * 20)))
            tw.set(_qn("w:type"), "dxa"); tblPr.append(tw)
            reorder_tblPr(tblPr)      # schema 顺序, 否则 Word 忽略上述设置
            grid = nested._tbl.find(_qn("w:tblGrid"))
            if grid is not None:
                gcs = grid.findall(_qn("w:gridCol"))
                for i, gc in enumerate(gcs[:cols]):
                    gc.set(_qn("w:w"), str(int(round(widths[i] * 20))))
            for r in rows:
                for c in range(min(cols, len(r.cells))):
                    r.cells[c].width = Pt(widths[c])
                    if font_pt != 10.5:
                        for pp in r.cells[c].paragraphs:
                            for run in pp.runs:
                                run.font.size = Pt(font_pt)
            done += 1
    return done, shrunk


def append_figures(doc) -> int:
    """表格之后追加「说明书附图」: 每图一页 + 居中图注"""
    from build_docx import FIG_FILES, extract_captions, set_run_font_cn, PNG_DIR, FIG_WIDTH_CM
    md = open(os.path.join(REPO, "docs", "patent_disclosure.md"), encoding="utf-8").read()
    caps = extract_captions(md)

    doc.add_page_break()
    h = doc.add_paragraph()
    h.alignment = 1
    set_run_font_cn(h.add_run("说明书附图"), "黑体", 16.0)
    doc.add_paragraph()

    added = 0
    for num, stem in FIG_FILES:
        png = os.path.join(PNG_DIR, f"{stem}.png")
        if not os.path.exists(png):
            continue
        if added:
            doc.add_page_break()
        p = doc.add_paragraph()
        p.alignment = 1
        p.add_run().add_picture(png, width=Cm(FIG_WIDTH_CM))
        cap = doc.add_paragraph()
        cap.alignment = 1
        set_run_font_cn(cap.add_run(caps.get(num, f"图 {num}")), "黑体", 10.5)
        added += 1
    return added


# ---------------------------------------------------------------- 表头占位

PLACEHOLDER = "【待填】"


def fill_header(tbl) -> None:
    """表头人员类字段填占位; 勾选项保持空框不动"""
    # r5 发明人名单(格内是提示语"请确认好！", 须替换) / r6 身份证 / r7-r9 联系人
    for r in (5, 6, 7, 8, 9):
        c = tbl.cell(r, 2)
        clear_cell(c)
        put_paragraph(c, PLACEHOLDER, bold=False)
    # r7-r9 电话/邮箱列 (c6..c7 / c11)
    for r in (7, 8, 9):
        for cc in (6, 11):
            c = tbl.cell(r, cc)
            if not "".join(p.text for p in c.paragraphs).strip():
                clear_cell(c)
                put_paragraph(c, PLACEHOLDER, bold=False)


# ---------------------------------------------------------------- 主流程

def main() -> int:
    if not os.path.exists(CONTENT_MD):
        print("[!] 缺少", CONTENT_MD)
        return 1

    print("1. 从原模板重建副本")
    shutil.copyfile(TEMPLATE, TARGET)

    print("2. 注入样式")
    inject_styles()

    print("3. 解析内容")
    sections = split_sections()
    missing = [s for s in SLOT_ROW if s not in sections]
    if missing:
        print("   [!] 内容缺少槽位:", missing)

    print("4. 写入模板")
    doc = Document(TARGET)
    tbl = doc.tables[0]
    for slot, row in SLOT_ROW.items():
        content = sections.get(slot, [])
        if row in MERGE_GUIDE_ROWS:
            merge_guide_cell(tbl, row)
        cell = tbl.cell(row, 11)
        clear_cell(cell)
        if row in SLOT_LABEL:
            put_paragraph(cell, SLOT_LABEL[row], bold=True)
        n = append_elements(cell, content)
        print(f"   r{row} {slot}: {n} 个元素")

    fill_header(tbl)

    print("5. 嵌套表列宽按内容格宽重排")
    total_done = total_shrunk = 0
    for row in sorted(set(SLOT_ROW.values())):
        cell = tbl.cell(row, 11)
        cw = Emu(cell.width).pt if cell.width else 0
        usable = cw - 30          # 减去外层格边距 + 嵌套表边距的叠加损耗
        d, sh = fit_nested_tables_of_cell(cell, usable)
        if d:
            print(f"   r{row} 可用 {usable:.0f}pt: 重排 {d} 表(降字号 {sh})")
        total_done += d; total_shrunk += sh

    print("6. 追加说明书附图")
    n_fig = append_figures(doc)

    doc.save(TARGET)
    print(f"   附图 {n_fig} 张")
    print(f"[OK] {TARGET} ({os.path.getsize(TARGET)} 字节)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
