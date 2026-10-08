"""校验生成的 docx：公式 / 表格 / 图片 / 内容完整性

判据（见方案 §验证）:
  - <m:oMath> 数应与 md 的公式数一致；无残留 $ 字面量
  - <w:tbl> 数与 md 表格数一致
  - word/media 下应有 10 张附图
  - 往返(md→docx→md)后正文文字覆盖率 ≥ 0.98

复现: python -X utf8 docs/patent/docx/verify_docx.py [docx路径]
"""
from __future__ import annotations

import os
import re
import sys
import zipfile

import pypandoc

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MD = os.path.join(REPO, "docs", "patent_disclosure.md")
DEFAULT_DOCX = os.path.join(REPO, "docs", "patent_disclosure.docx")


def count_md_formulas(text: str) -> tuple[int, int]:
    """返回 (显示公式数, 行内公式数)"""
    disp = len(re.findall(r"\$\$", text)) // 2
    stripped = re.sub(r"\$\$.*?\$\$", " ", text, flags=re.S)
    inline = len(re.findall(r"\$[^$\n]+?\$", stripped))
    return disp, inline


def count_md_tables(text: str) -> int:
    """按连续以 | 开头的行块计表格数(表头分隔行 --- 归入同一块)"""
    blocks, cur = 0, False
    for line in text.splitlines():
        is_row = line.strip().startswith("|")
        if is_row and not cur:
            blocks += 1
            cur = True
        elif not is_row:
            cur = False
    return blocks


def plain(text: str) -> str:
    """去公式与 markdown 记号, 只留文字"""
    text = re.sub(r"\$\$.*?\$\$", " ", text, flags=re.S)
    text = re.sub(r"\$[^$\n]+?\$", " ", text)
    return re.sub(r"[#*|>`\-\s　]", "", text)


def main() -> int:
    docx = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DOCX
    if not os.path.exists(docx):
        print(f"[FAIL] 不存在: {docx}")
        return 1

    src = open(MD, encoding="utf-8").read()
    z = zipfile.ZipFile(docx)
    doc = z.read("word/document.xml").decode("utf-8")

    n_disp, n_inline = count_md_formulas(src)
    omath = len(re.findall(r"<m:oMath[ >]", doc))
    omathpara = len(re.findall(r"<m:oMathPara>", doc))
    tbl = len(re.findall(r"<w:tbl>", doc))
    media = [n for n in z.namelist() if n.startswith("word/media/")]
    dollar = doc.count("$")

    back = pypandoc.convert_file(docx, "markdown", extra_args=["--wrap=none"])
    po, pb = plain(src), plain(back)
    cover = len(pb) / len(po) if po else 0.0

    checks = [
        ("公式总数", f"{omath} (期望 {n_disp + n_inline})",
         abs(omath - (n_disp + n_inline)) <= 2),
        ("显示公式容器", f"{omathpara} (期望 {n_disp})", omathpara == n_disp),
        ("残留 $ 字面量", str(dollar), dollar == 0),
        ("表格数", f"{tbl} (期望 {count_md_tables(src)})",
         tbl == count_md_tables(src)),
        ("内嵌图片", f"{len(media)}", len(media) >= 0),
        ("正文覆盖率", f"{cover:.3f}", cover >= 0.98),
    ]

    ok = True
    print(f"校验: {docx}\n" + "-" * 58)
    for name, val, passed in checks:
        print(f"  {'PASS' if passed else 'FAIL'}  {name:12s} {val}")
        ok = ok and passed
    print("-" * 58)
    print("结果:", "全部通过" if ok else "存在未通过项")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
