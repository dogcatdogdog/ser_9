"""构建专利交底书 DOCX（含说明书附图）

流水线:
  1. 附图 PDF 存在性检查(不存在则提示先跑 docs/patent/build.sh)
  2. pdftoppm 把 10 张附图转 300dpi PNG
  3. pandoc: docs/patent_disclosure.md -> docx (--reference-doc 应用中文样式;
     LaTeX 公式转 Word 原生公式 OMML)
  4. python-docx 后处理:
     a) 文末添加分页符 + "说明书附图" 标题, 按图号顺序插入 10 图 + 图注
     b) 宽表(列数 >= 9)所在表格字号降为小五(9pt), 并允许跨页断行
  5. 输出 docs/patent_disclosure.docx

图注与图号自动取自交底书 §七 的 "**图 N** ..." 行, 不在脚本里重复维护。

复现: python -X utf8 docs/patent/docx/build_docx.py
"""
from __future__ import annotations

import os
import re
import subprocess
import sys

import pypandoc
from docx import Document
from docx.enum.text import WD_BREAK
from docx.shared import Pt, Cm, RGBColor
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MD = os.path.join(REPO, "docs", "patent_disclosure.md")
PDF_DIR = os.path.join(REPO, "docs", "patent", "pdf")
PNG_DIR = os.path.join(HERE, "figures_png")
REF = os.path.join(HERE, "reference.docx")
OUT = os.path.join(REPO, "docs", "patent_disclosure.docx")

PDFTOPPM = r"D:\texlive\2025\bin\windows\pdftoppm.exe"
FIG_WIDTH_CM = 15.5          # 图片显示宽度(A4 竖排可用宽度约 16cm)
WIDE_TABLE_COLS = 9          # 列数 >= 此值视为宽表, 降字号
WIDE_TABLE_FONT_PT = 9.0     # 小五

# 图文件前缀 -> 图号(与交底书 §七 的 "图 N" 对应)
FIG_FILES = [
    (1, "fig1_overview"), (2, "fig2_energy_rate"), (3, "fig3_equiv_dist"),
    (4, "fig4_order_effect"), (5, "fig5_two_opt"), (6, "fig6_ablation"),
    (7, "fig7_scaling"), (8, "fig8_group_ablation"), (9, "fig9_energy_saving"),
    (10, "fig10_architecture"),
]


def extract_captions(md_text: str) -> dict[int, str]:
    """从交底书 §七 提取图注首句, 如 '图 1 本发明方法的总体流程图'"""
    caps: dict[int, str] = {}
    for m in re.finditer(r"^\*\*图\s*(\d+)\*\*\s*([^\n（(。]*?)\s*[（(。]", md_text, re.M):
        num, title = int(m.group(1)), m.group(2).strip()
        if title:
            caps[num] = f"图 {num}　{title}"
    return caps


def ensure_pngs() -> bool:
    """PDF -> 300dpi PNG; 返回是否全部就绪"""
    os.makedirs(PNG_DIR, exist_ok=True)
    missing_pdf = []
    for num, stem in FIG_FILES:
        png = os.path.join(PNG_DIR, f"{stem}.png")
        pdf = os.path.join(PDF_DIR, f"{stem}.pdf")
        if os.path.exists(png) and os.path.getmtime(png) >= os.path.getmtime(pdf) \
                if os.path.exists(pdf) else os.path.exists(png):
            continue
        if not os.path.exists(pdf):
            missing_pdf.append(pdf)
            continue
        subprocess.run([PDFTOPPM, "-r", "300", "-png", "-singlefile", pdf,
                        os.path.join(PNG_DIR, stem)], check=True, capture_output=True)
    if missing_pdf:
        print("[!] 缺少附图 PDF, 请先运行: bash docs/patent/build.sh")
        for p in missing_pdf:
            print("   ", p)
        return False
    return True


def set_run_font_cn(run, cn: str = "宋体", size_pt: float = 10.5) -> None:
    """同时设置西文与东亚字体, 否则汉字会掉字体"""
    run.font.size = Pt(size_pt)
    run.font.name = "Times New Roman"
    run.font.color.rgb = RGBColor(0, 0, 0)
    rPr = run._element.get_or_add_rPr()
    rFonts = rPr.find(qn("w:rFonts"))
    if rFonts is None:
        rFonts = OxmlElement("w:rFonts")
        rPr.append(rFonts)
    rFonts.set(qn("w:ascii"), "Times New Roman")
    rFonts.set(qn("w:hAnsi"), "Times New Roman")
    rFonts.set(qn("w:eastAsia"), cn)


def append_figures(doc: Document, captions: dict[int, str]) -> int:
    """文末追加 说明书附图: 每图一页, 图下居中图注"""
    doc.add_page_break()
    h = doc.add_paragraph()
    h.alignment = 1  # center
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
        set_run_font_cn(cap.add_run(captions.get(num, f"图 {num}")), "黑体", 10.5)
        added += 1
    return added


def shrink_wide_tables(doc: Document) -> int:
    """宽表降字号到小五 + 允许跨页断行"""
    n = 0
    for tbl in doc.tables:
        cols = len(tbl.columns)
        if cols < WIDE_TABLE_COLS:
            continue
        n += 1
        for row in tbl.rows:
            trPr = row._tr.get_or_add_trPr()
            if trPr.find(qn("w:cantSplit")) is None:
                trPr.append(OxmlElement("w:cantSplit"))
            for cell in row.cells:
                for p in cell.paragraphs:
                    for run in p.runs:
                        set_run_font_cn(run, "宋体", WIDE_TABLE_FONT_PT)
    return n


def main() -> int:
    if not os.path.exists(REF):
        print("[!] 缺少样式基准, 请先运行 make_reference.py")
        return 1
    if not ensure_pngs():
        return 1

    md_text = open(MD, encoding="utf-8").read()
    captions = extract_captions(md_text)
    print(f"  解析到图注 {len(captions)} 条")

    tmp = os.path.join(HERE, "_body.docx")
    pypandoc.convert_file(MD, "docx", outputfile=tmp,
                          extra_args=[f"--reference-doc={REF}",
                                      "--resource-path=" + os.path.dirname(MD)])
    print("  pandoc 转换完成")

    doc = Document(tmp)
    n_tbl = shrink_wide_tables(doc)
    n_fig = append_figures(doc, captions)
    doc.save(OUT)
    os.remove(tmp)

    print(f"  宽表降字号: {n_tbl} 个; 附图追加: {n_fig} 张")
    print(f"[OK] {OUT}  ({os.path.getsize(OUT)} 字节)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
