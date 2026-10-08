"""生成 pandoc 用的 reference.docx —— 中文专利交底书样式基准

做法: 取 pandoc 自带的 reference.docx(样式齐全, 49 条), 改写其 word/styles.xml:
  - docDefaults 与各样式补 w:eastAsia, 否则 Word 用西文字体渲染汉字
  - 正文 宋体 五号(10.5pt) / 标题 黑体 / 引用块 楷体
  - 正文 1.5 倍行距 + 首行缩进 2 字符

字号参考: 矿业大学母版 Normal = 10pt(五号量级); 本项目取五号 10.5pt。
字号单位为半磅(half-point): 10.5pt = 21, 12pt = 24, 14pt = 28, 16pt = 32。

复现: python -X utf8 docs/patent/docx/make_reference.py
"""
from __future__ import annotations

import io
import os
import re
import subprocess
import sys
import zipfile

import pypandoc

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "reference.docx")

SONG = "宋体"      # 正文
HEI = "黑体"       # 标题
KAI = "楷体"       # 引用/提示块
WEST = "Times New Roman"  # 西文与数字

# 样式名 -> (中文字体, 西文字体, 字号半磅 or None保留原值)
STYLE_SPEC: dict[str, tuple[str, str, int | None]] = {
    "Normal":    (SONG, WEST, 21),
    "BodyText":  (SONG, WEST, 21),
    "FirstParagraph": (SONG, WEST, 21),
    "Compact":   (SONG, WEST, 21),
    "Heading1":  (HEI, WEST, 32),
    "Heading2":  (HEI, WEST, 28),
    "Heading3":  (HEI, WEST, 24),
    "Heading4":  (HEI, WEST, 24),
    "Heading5":  (HEI, WEST, 21),
    "Heading6":  (HEI, WEST, 21),
    "BlockText": (KAI, WEST, 21),
    "Table":     (SONG, WEST, 21),
    "TableCaption": (SONG, WEST, 21),
    "ImageCaption": (SONG, WEST, 21),
}


def _rpr(font_cn: str, font_west: str, sz_half: int | None,
         line_360: bool = False, indent: bool = False) -> str:
    """构造一个 <w:rPr> 片段"""
    parts = [f'<w:rFonts w:ascii="{font_west}" w:hAnsi="{font_west}" '
             f'w:eastAsia="{font_cn}" w:cs="{font_west}"/>']
    if sz_half is not None:
        parts.append(f'<w:sz w:val="{sz_half}"/><w:szCs w:val="{sz_half}"/>')
    return "<w:rPr>" + "".join(parts) + "</w:rPr>"


def _patch_style(xml: str, sid: str, font_cn: str, font_west: str,
                 sz_half: int | None) -> tuple[str, bool]:
    """替换指定 styleId 的 <w:rPr>; 若该样式无 rPr 则插入"""
    pat = re.compile(
        r'(<w:style\b[^>]*w:styleId="' + re.escape(sid) + r'"[^>]*>)(.*?)(</w:style>)',
        re.S)
    m = pat.search(xml)
    if not m:
        return xml, False
    head, body, tail = m.group(1), m.group(2), m.group(3)
    new_rpr = _rpr(font_cn, font_west, sz_half)
    if "<w:rPr>" in body:
        body = re.sub(r"<w:rPr>.*?</w:rPr>", new_rpr, body, flags=re.S)
    else:
        body = body + new_rpr
    return xml[:m.start()] + head + body + tail + xml[m.end():], True


def main() -> None:
    pandoc = pypandoc.get_pandoc_path()
    proc = subprocess.run([pandoc, "--print-default-data-file", "reference.docx"],
                          capture_output=True, check=True)
    src = zipfile.ZipFile(io.BytesIO(proc.stdout))
    styles = src.read("word/styles.xml").decode("utf-8")

    # 1) docDefaults 补 eastAsia(全局兜底)
    styles = re.sub(
        r'(<w:rPrDefault>\s*<w:rPr>)(\s*<w:rFonts[^/>]*/>)?',
        lambda m: m.group(1) +
        f'<w:rFonts w:ascii="{WEST}" w:hAnsi="{WEST}" w:eastAsia="{SONG}" w:cs="{WEST}"/>',
        styles, count=1)

    # 2) 逐个样式设定字体字号
    done, missing = [], []
    for sid, (cn, west, sz) in STYLE_SPEC.items():
        styles, ok = _patch_style(styles, sid, cn, west, sz)
        (done if ok else missing).append(sid)

    # 3) 正文行距 1.5 + 首行缩进 2 字符(BodyText / FirstParagraph)
    for sid in ("BodyText", "FirstParagraph"):
        m = re.search(r'<w:style\b[^>]*w:styleId="' + sid + r'"[^>]*>.*?</w:style>',
                      styles, re.S)
        if m and "<w:pPr>" in m.group(0):
            block = re.sub(r"<w:spacing[^/>]*/>", "", m.group(0))
            block = re.sub(r"<w:ind[^/>]*/>", "", block)
            block = block.replace(
                "<w:pPr>",
                '<w:pPr><w:spacing w:line="360" w:lineRule="auto"/>'
                '<w:ind w:firstLineChars="200" w:firstLine="420"/>', 1)
            styles = styles[:m.start()] + block + styles[m.end():]

    # 4) 重新打包
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as dst:
        for item in src.infolist():
            data = styles.encode("utf-8") if item.filename == "word/styles.xml" \
                else src.read(item.filename)
            dst.writestr(item, data)
    with open(OUT, "wb") as f:
        f.write(buf.getvalue())

    print(f"[OK] {OUT}  ({len(buf.getvalue())} 字节)")
    print(f"  已设定样式 {len(done)}: {', '.join(done)}")
    if missing:
        print(f"  ⚠ 模板中不存在(已跳过): {', '.join(missing)}")


if __name__ == "__main__":
    sys.exit(main())
