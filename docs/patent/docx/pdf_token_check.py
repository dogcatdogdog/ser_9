"""PDF ground-truth 检查: 找出 docx 里被换行拆开的短标记

思路(不依赖宽度估算, 也不依赖 Word 的 API):
  1. 从 markdown 表格中抽取所有"短标记"(数字、形如 t10→t9 的编号等)
  2. 用 Word 把 docx 导出 PDF, pdftotext -layout 取回文字
  3. 逐个检查标记是否在 PDF 文字中**完整出现**; 未完整出现者即为被拆开

用法: python -X utf8 docs/patent/docx/pdf_token_check.py
前置: 需已装 Word 与 pdftotext(TeX Live 自带)
"""
from __future__ import annotations

import os
import re
import subprocess
import sys

import win32com.client as win32

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MD = os.path.join(REPO, "docs", "patent_disclosure.md")
DOCX = os.path.join(REPO, "docs", "patent_disclosure.docx")
PDF = os.path.join(HERE, "_chk.pdf")
TXT = os.path.join(HERE, "_chk.txt")
PDFTOTEXT = r"D:\texlive\2025\bin\windows\pdftotext.exe"

# 视为"不可拆"的短标记: 含小数点的数字、纯整数(>=2位)、带箭头的编号
TOKEN_RE = re.compile(r"(?<![\w.])(\d+\.\d+|\d{2,}|t\d+(?:→t\d+)?)(?![\w.])")


def export_pdf() -> bool:
    try:
        w = win32.gencache.EnsureDispatch("Word.Application")
        w.Visible = False
        w.DisplayAlerts = 0
        d = w.Documents.Open(DOCX, ReadOnly=True, AddToRecentFiles=False)
        try:
            d.ExportAsFixedFormat(PDF, 17)
        finally:
            d.Close(SaveChanges=False)
            w.Quit()
        return True
    except Exception as e:                                  # noqa: BLE001
        print("[!] Word 导出失败:", e)
        return False


def main() -> int:
    md = open(MD, encoding="utf-8").read()
    # 只取表格行中的标记(表格才是换行高发区)
    tokens: set[str] = set()
    for line in md.splitlines():
        if line.strip().startswith("|"):
            tokens.update(TOKEN_RE.findall(line))
    if not export_pdf():
        return 1
    subprocess.run([PDFTOTEXT, "-layout", PDF, TXT], check=True, capture_output=True)
    text = open(TXT, encoding="utf-8", errors="replace").read()
    # 去掉空白后比对, 避免 PDF 排版插入空格造成误判
    compact = re.sub(r"\s+", "", text)

    missing = []
    for tk in sorted(tokens):
        if "→" in tk:
            continue   # 箭头字形的 PDF 提取不稳定, 排除
        if re.sub(r"\s+", "", tk) not in compact:
            missing.append(tk)

    print(f"表格中的短标记: {len(tokens)} 个")
    print(f"PDF 中未完整出现的: {len(missing)} 个")
    if missing:
        print("\n被换行拆开的标记:")
        for tk in missing:
            print("   ", tk)
        return 1
    print("\n✅ 全部短标记均完整显示(未被换行拆开)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
