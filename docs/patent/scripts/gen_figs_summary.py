"""图8/图9 数据解析 — 从全量评测表提取, 不重跑实验

图8 (fig8_group_ablation): 分组消融 —— circle/random × 5/10/15/20 各变体相对成本%
图9 (fig9_energy_saving): 逐实例省电 —— gap_eng% 按实例类型/规模分组 mean±std

输入: results/table_20260819_105511.md (93 实例评测)
输出: fig8_group_ablation.json / fig9_energy_saving.json
复现: python -X utf8 docs/patent/scripts/gen_figs_summary.py
"""
import json
import os
import re
import statistics

HERE = os.path.dirname(__file__)
TABLE = os.path.abspath(os.path.join(HERE, "..", "..", "..", "results",
                                     "table_20260819_105511.md"))

VARIANTS = ["full", "nn_only", "no_oropt", "no_2opt", "fixed_payload", "no_energy"]
GROUPS = [("circle", n) for n in (5, 10, 15, 20)] + \
         [("random", n) for n in (5, 10, 15, 20)]


def _cells(line):
    return [c.strip() for c in line.strip().strip("|").split("|")]


def parse_ablation(text):
    """解析 "## 2. 消融矩阵" 下的分组表 -> {group: {variant: rel%}}"""
    out = {}
    cur = None
    for line in text.splitlines():
        m = re.match(r"\*\*(\w+_(\d+)p)\*\*", line)
        if m:
            cur = m.group(1)
            out[cur] = {}
            continue
        if cur and line.startswith("|"):
            cells = _cells(line)
            if len(cells) >= 3 and cells[0] in VARIANTS:
                rel = cells[2].rstrip("%")
                out[cur][cells[0]] = float(rel)
    return out


def parse_main_gap(text):
    """解析主表 Gap_eng% 列 (第 10 列) 与 Feas 列 (第 6 列)"""
    rows = []
    started = False
    for line in text.splitlines():
        if line.startswith("| Instance | N | Our_equiv"):
            started = True
            continue
        if started:
            if not line.startswith("|"):
                break
            cells = _cells(line)
            if len(cells) < 16 or cells[0].startswith("---"):
                continue
            name = cells[0]
            if name.startswith("solomon_"):
                kind = "solomon"
            elif name.startswith("tight_"):
                kind = "tight"
            elif name.startswith("circle_"):
                kind = "circle"
            elif name.startswith("random_"):
                kind = "random"
            else:
                continue
            rows.append({
                "name": name, "kind": kind, "n": int(cells[1]),
                "feasible": cells[5] == "✓",
                "gap_eng": float(cells[9]),
            })
    return rows


def main():
    with open(TABLE, encoding="utf-8") as f:
        text = f.read()

    ab = parse_ablation(text)
    rows = parse_main_gap(text)

    # ---- 图8: 分组消融 ----
    fig8 = {"variants": [v for v in VARIANTS if v != "no_energy"],
            "groups": [], "series": {v: [] for v in VARIANTS if v != "no_energy"}}
    for dist, n in GROUPS:
        key = f"{dist}_{n}p"
        if key not in ab:
            continue
        fig8["groups"].append({"dist": dist, "n": n, "key": key})
        for v in fig8["variants"]:
            fig8["series"][v].append(ab[key][v])

    # ---- 图9: 逐实例省电 (按类型/规模聚合 mean±std) ----
    fig9 = []
    for kind, n in [(k, n) for k in ("circle", "random", "tight")
                    for n in (5, 10, 15, 20)] + [("solomon", 20)]:
        vals = [r["gap_eng"] for r in rows
                if r["kind"] == kind and r["n"] == n and r["feasible"]]
        if not vals:
            continue
        fig9.append({
            "label": ("Solomon" if kind == "solomon" else f"{dist_label(kind)}{n}"),
            "kind": kind, "n": n, "count": len(vals),
            "mean": round(statistics.mean(vals), 2),
            "std": round(statistics.stdev(vals), 2) if len(vals) > 1 else 0.0,
            "min": round(min(vals), 2), "max": round(max(vals), 2),
        })

    # solomon 逐实例
    sol = [{"label": r["name"].replace("solomon_", "").replace("_n20", ""),
            "gap_eng": r["gap_eng"]} for r in rows if r["kind"] == "solomon"]

    with open(os.path.join(HERE, "fig8_group_ablation.json"), "w", encoding="utf-8") as f:
        json.dump(fig8, f, ensure_ascii=False, indent=2)
    with open(os.path.join(HERE, "fig9_energy_saving.json"), "w", encoding="utf-8") as f:
        json.dump({"groups": fig9, "solomon": sol}, f, ensure_ascii=False, indent=2)

    print("== 图8 分组消融 (相对成本%) ==")
    for key in [g["key"] for g in fig8["groups"]]:
        print(f"  {key:12s}", {v: fig8["series"][v][fig8['groups'].index(
            next(g for g in fig8['groups'] if g['key'] == key))] for v in fig8["variants"]})
    print("== 图9 省电 gap_eng% (负=更省) ==")
    for g in fig9:
        print(f"  {g['label']:12s} n={g['n']:<3d} cnt={g['count']:<3d} "
              f"mean={g['mean']:>7.2f} std={g['std']:>5.2f} [{g['min']},{g['max']}]")
    print("  Solomon:", sol)


def dist_label(kind):
    return {"circle": "circle_", "random": "random_", "tight": "tight_"}[kind]


if __name__ == "__main__":
    main()
