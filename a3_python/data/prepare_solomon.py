"""从下载的原始 Solomon JSON 中提取前 20 点, 生成测试 fixtures。

数据来源: CervEdin/solomon-vrptw-benchmarks (GitHub)
原始发布: Solomon, M. M. (1987). "Algorithms for the Vehicle Routing and
Scheduling Problems with Time Window Constraints." Operations Research, 35(2), 254-265.

用法:
    python a3_python/data/prepare_solomon.py
"""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES_DIR = os.path.join(os.path.dirname(HERE), "tests", "fixtures")
DATA_DIR = HERE  # downloaded Solomon JSON/TXT live here

N_POINTS = 20  # 取前 N 个客户


def extract_instance(name: str, json_filename: str):
    """从下载的 Solomon JSON 提取前 N_POINTS 个客户, 写入 fixture JSON"""
    src = os.path.join(DATA_DIR, "solomon", json_filename)
    if not os.path.exists(src):
        print(f"  SKIP: {src} not found — run _fetch_solomon.py first")
        return None

    with open(src) as f:
        data = json.load(f)

    customers = data["customers"]
    depot = customers[0]  # id=0 is depot
    first_n = customers[1 : 1 + N_POINTS]

    fixture = {
        "description": (
            f"Solomon {name.upper()} — first {N_POINTS} customers. "
            f"Source: CervEdin/solomon-vrptw-benchmarks (GitHub). "
            f"Original: Solomon (1987), Operations Research 35(2), 254-265."
        ),
        "home": {"x": depot["x"], "y": depot["y"]},
        "targets": [
            {
                "id": f"{name.upper()}{c['id']:02d}",
                "x": c["x"],
                "y": c["y"],
                "demand": c["demand"],
            }
            for c in first_n
        ],
    }

    dst = os.path.join(FIXTURES_DIR, f"solomon_{name}_n{N_POINTS}.json")
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(fixture, f, indent=2, ensure_ascii=False)

    total_demand = sum(c["demand"] for c in first_n)
    print(f"  OK: {dst} ({len(first_n)} points, total_demand={total_demand})")
    return fixture


def main():
    print(f"Extracting first {N_POINTS} customers from Solomon instances...\n")

    insts = [
        ("r101", "r101.json"),
        ("c101", "c101.json"),
        ("rc101", "rc101.json"),
    ]
    for name, fn in insts:
        extract_instance(name, fn)

    print(f"\nDone. Fixtures written to {FIXTURES_DIR}")


if __name__ == "__main__":
    main()
