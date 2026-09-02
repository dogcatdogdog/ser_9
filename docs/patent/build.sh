#!/usr/bin/env bash
# Compile the 7 standalone patent figures with xelatex (TeX Live 2025).
# Usage: bash docs/patent/build.sh   -> docs/patent/pdf/fig*.pdf
set -u
cd "$(dirname "$0")" || exit 1
mkdir -p pdf
fail=0
for f in tikz/fig*.tex; do
  base=$(basename "$f" .tex)
  echo "== compiling $base =="
  if ! latexmk -pdf -xelatex -interaction=nonstopmode -halt-on-error -outdir=pdf "$f" >/dev/null 2>&1; then
    echo "FAIL: $base"
    fail=1
  fi
done
# keep PDFs, clean aux/log/fls
latexmk -c -outdir=pdf tikz/fig*.tex >/dev/null 2>&1
if [ "$fail" = "0" ]; then
  echo "ALL 7 PDFs compiled OK -> pdf/"
else
  echo "SOME FIGURES FAILED (see above)"
  exit 1
fi
