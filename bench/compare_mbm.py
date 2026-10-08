"""Builds the MotionBenchMaker comparison table from the raw result files in bench/results/.

Usage: python bench/compare_mbm.py > bench/results/mbm-comparison.md
       python bench/compare_mbm.py ur5 > bench/results/mbm-ur5-comparison.md
"""

import json
import math
import sys
from pathlib import Path

RESULTS = Path(__file__).parent / "results"
PLANNERS = [
    ("VAMP 0.6.4, default (dynamic domain)", "mbm-vamp-default.json"),
    ("VAMP 0.6.4, dynamic domain off", "mbm-vamp-no-dd.json"),
    ("motionAmigo, AVX2", "mbm-motionamigo-simd.json"),
    ("motionAmigo, AVX2, greedy shortcutting only", "mbm-motionamigo-simd-greedy.json"),
    ("motionAmigo, portable SIMD", "mbm-motionamigo-portable.json"),
    ("motionAmigo, scalar", "mbm-motionamigo-scalar.json"),
]


def percentile(values, p):
    if not values:
        return math.nan
    v = sorted(values)
    pos = p / 100 * (len(v) - 1)
    lo, hi = math.floor(pos), math.ceil(pos)
    return v[lo] + (v[hi] - v[lo]) * (pos - lo)


def fmt(us):
    if us != us:
        return "n/a"
    return f"{us:.0f} µs" if us < 1000 else f"{us / 1000:.2f} ms"


def load(name):
    data = json.loads((RESULTS / name).read_text(encoding="utf-8"))
    return {k: [r for r in v] for k, v in data.items()}


def stats(records):
    ok = [r for r in records if r["solved"]]
    col = lambda k: [r[k] for r in ok]
    return {
        "n": len(records),
        "success": len(ok) / max(1, len(records)),
        "plan50": percentile(col("planning_us"), 50),
        "plan95": percentile(col("planning_us"), 95),
        "simp50": percentile(col("simplify_us"), 50),
        "total50": percentile(col("total_us"), 50),
        "total95": percentile(col("total_us"), 95),
        "len50": percentile(col("length"), 50),
    }


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else "panda"
    prefix = "mbm-" if robot == "panda" else f"mbm-{robot}-"
    files = [(label, f.replace("mbm-", prefix, 1)) for label, f in PLANNERS]
    loaded = [(label, load(f)) for label, f in files if (RESULTS / f).exists()]
    scenarios = sorted(loaded[0][1])
    n = sum(len(loaded[0][1][sc]) for sc in scenarios)
    print(f"### All {n} valid problems pooled\n")
    print("| planner | success | planning median | planning P95 | simplification median | total median | total P95 | path length median (rad) |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|")
    for label, data in loaded:
        s = stats([r for sc in scenarios for r in data[sc]])
        print(
            f"| {label} | {100 * s['success']:.1f}% | {fmt(s['plan50'])} | {fmt(s['plan95'])} | "
            f"{fmt(s['simp50'])} | {fmt(s['total50'])} | {fmt(s['total95'])} | {s['len50']:.2f} |"
        )
    print("\n### Median planning time per scenario\n")
    print("| scenario | " + " | ".join(l for l, _ in loaded) + " |")
    print("|---|" + "---:|" * len(loaded))
    for sc in scenarios:
        cells = [fmt(stats(d[sc])["plan50"]) for _, d in loaded]
        print(f"| {sc} | " + " | ".join(cells) + " |")
    print("\n### Median total time (planning + simplification) per scenario\n")
    print("| scenario | " + " | ".join(l for l, _ in loaded) + " |")
    print("|---|" + "---:|" * len(loaded))
    for sc in scenarios:
        cells = [fmt(stats(d[sc])["total50"]) for _, d in loaded]
        print(f"| {sc} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
