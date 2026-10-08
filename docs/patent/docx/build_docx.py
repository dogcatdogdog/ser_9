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
from docx.shared import Pt, Cm, Emu, RGBColor
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


def est_text_width_pt(text: str, font_pt: float) -> float:
    """估算一段文字渲染所需宽度(pt): 汉字/全角按 1 字宽, 西文数字按半字宽"""
    w = 0.0
    for ch in text:
        w += font_pt if ord(ch) > 0x2E80 else font_pt * 0.5
    return w


def cell_text(cell) -> str:
    return "".join(p.text for p in cell.paragraphs).strip()


def fit_table_columns(doc: Document, base_pt: float = 10.5) -> tuple[int, int]:
    """按内容实测宽度重算每张表的列宽, 消除"数字/单字被挤断"的换行

    做法:
      1. 逐列估算所需宽度 = max(各单元格文字宽) + 单元格左右边距
      2. 若总宽 <= 版心宽 → 按比例分配余量; 否则等比压缩
      3. 压缩过狠(某列 < 26pt)时, 该表整体降字号到小五并重算
      4. 关闭自动调整, 写入固定列宽(tblLayout=fixed), 并禁止行内跨页断行

    返回 (处理的表数, 降字号的表数)
    """
    sec = doc.sections[0]
    # pandoc 的 reference.docx 未定义页面尺寸 -> 显式设为 A4 + 标准页边距
    if sec.page_width is None:
        sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
    if sec.left_margin is None:
        sec.left_margin = sec.right_margin = Cm(2.54)
    if sec.top_margin is None:
        sec.top_margin = sec.bottom_margin = Cm(2.54)
    usable_pt = Emu(sec.page_width - sec.left_margin - sec.right_margin).pt
    margin_pt = 4.5   # 单元格左右边距合计(下方设为每侧 0.08cm)
    done = shrunk = 0

    for tbl in doc.tables:
        cols = len(tbl.columns)
        rows = tbl.rows
        if cols == 0 or len(rows) == 0:
            continue
        # 1) 逐列收集"最长行"所需宽度(随字号缩放, 故按字号重算)
        def compute_need(font_pt: float) -> list[float]:
            nd = [0.0] * cols
            for r in rows:
                for c in range(min(cols, len(r.cells))):
                    txt = cell_text(r.cells[c])
                    if not txt:
                        continue
                    longest = max((est_text_width_pt(seg, font_pt)
                                   for seg in txt.replace("\n", " ").split("  ")),
                                  default=0.0)
                    nd[c] = max(nd[c], longest + margin_pt)
            return nd

        # 2) 迭代选字号: 在能"放得下各列最长行"的前提下取最大字号
        font_pt = base_pt
        need = compute_need(base_pt)
        if sum(need) > usable_pt:
            for cand in (10.0, 9.5, WIDE_TABLE_FONT_PT, 8.5):
                nd = compute_need(cand)
                if sum(nd) <= usable_pt:
                    font_pt, need = cand, nd
                    break
            else:
                font_pt = 8.5
                need = compute_need(8.5)
            shrunk += 1

        # 2b) 水填充式分配: 需求小于均分的列只拿所需, 余量给紧缺的列
        fair = usable_pt / cols
        widths = [0.0] * cols
        slack_idx = [i for i in range(cols) if need[i] <= fair]
        tight_idx = [i for i in range(cols) if need[i] > fair]
        for i in slack_idx:
            widths[i] = need[i]
        rest = usable_pt - sum(widths[i] for i in slack_idx)
        if tight_idx:
            tot_need = sum(need[i] for i in tight_idx)
            if tot_need <= rest:                     # 余量足够, 按需分配再摊余
                for i in tight_idx:
                    widths[i] = need[i]
                extra = (rest - tot_need) / len(tight_idx)
                for i in tight_idx:
                    widths[i] += extra
            else:                                    # 仍不够 -> 按需等比压缩
                k = rest / tot_need
                floor = margin_pt + font_pt * 2      # 至少放得下 2 个汉字
                for i in tight_idx:
                    widths[i] = max(floor, need[i] * k)

        # 3) 写入固定列宽
        tbl.autofit = False
        tblPr = tbl._tbl.tblPr
        for tag in ("w:tblLayout", "w:tblW"):
            old = tblPr.find(qn(tag))
            if old is not None:
                tblPr.remove(old)
        old = tblPr.find(qn("w:tblCellMar"))
        if old is not None:
            tblPr.remove(old)
        cellmar = OxmlElement("w:tblCellMar")
        for side, twips in (("top", "0"), ("left", "45"),
                            ("bottom", "0"), ("right", "45")):
            e = OxmlElement(f"w:{side}")
            e.set(qn("w:w"), twips)
            e.set(qn("w:type"), "dxa")
            cellmar.append(e)
        tblPr.append(cellmar)
        layout = OxmlElement("w:tblLayout")
        layout.set(qn("w:type"), "fixed")
        tblPr.append(layout)
        tblW = OxmlElement("w:tblW")
        tblW.set(qn("w:w"), str(int(sum(widths) * 20)))
        tblW.set(qn("w:type"), "dxa")
        tblPr.append(tblW)

        for r in rows:
            trPr = r._tr.get_or_add_trPr()
            if trPr.find(qn("w:cantSplit")) is None:
                trPr.append(OxmlElement("w:cantSplit"))
            for c in range(min(cols, len(r.cells))):
                r.cells[c].width = Pt(widths[c])
                if font_pt != base_pt:
                    for p in r.cells[c].paragraphs:
                        for run in p.runs:
                            set_run_font_cn(run, "宋体", font_pt)
        done += 1
    return done, shrunk


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
    n_tbl, n_shrunk = fit_table_columns(doc)
    n_fig = append_figures(doc, captions)
    doc.save(OUT)
    os.remove(tmp)

    print(f"  列宽自适应: {n_tbl} 张表 (其中降字号 {n_shrunk} 张); 附图追加: {n_fig} 张")
    print(f"[OK] {OUT}  ({os.path.getsize(OUT)} 字节)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
