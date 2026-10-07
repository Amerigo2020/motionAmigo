"""Run VAMP's RRT-Connect on the MotionBenchMaker problems for the Panda.

This mirrors VAMP's scripts/evaluate_mbm.py (one trial per valid problem, Halton sampler, default
settings from vamp.configure_robot_and_planner_with_kwargs) and reports the same statistics as
motionamigo-bench mbm. Times are VAMP's internal nanosecond timers.

Usage: python run_vamp_mbm.py vamp-src/resources/panda/problems.pkl [--dynamic_domain False]
"""

import json
import pickle
import platform
import sys
from pathlib import Path

import numpy as np
import vamp


def percentile(values, p):
    return float(np.percentile(values, p)) if values else float("nan")


def fmt_us(us):
    if us != us:
        return "n/a"
    if us < 1000:
        return f"{us:.0f} µs"
    if us < 1e6:
        return f"{us / 1000:.2f} ms"
    return f"{us / 1e6:.2f} s"


def cpu_name():
    try:
        for line in open("/proc/cpuinfo"):
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def main(pkl, dynamic_domain="True", out=None):
    with open(pkl, "rb") as f:
        problems = pickle.load(f)["problems"]
    kwargs = {}
    if dynamic_domain == "False":
        kwargs["dynamic_domain"] = False
    robot, planner, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(
        "panda", "rrtc", **kwargs
    )
    sampler = robot.halton()
    rows, raw = [], {}
    for name in sorted(problems):
        records = []
        for i, data in enumerate(problems[name]):
            if not data["valid"]:
                continue
            env = vamp.problem_dict_to_vamp(data)
            sampler.reset()
            result = planner(data["start"], data["goals"], env, plan_settings, sampler)
            if not result.solved:
                records.append({"problem": f"{name}/{i}", "solved": False})
                continue
            simple = robot.simplify(result.path, env, simp_settings, sampler)
            planning_us = result.nanoseconds / 1e3
            simplify_us = simple.nanoseconds / 1e3
            records.append(
                {
                    "problem": f"{name}/{i}",
                    "solved": True,
                    "planning_us": planning_us,
                    "simplify_us": simplify_us,
                    "total_us": planning_us + simplify_us,
                    "initial_length": float(result.path.cost()),
                    "length": float(simple.path.cost()),
                    "iterations": int(result.iterations),
                }
            )
        ok = [r for r in records if r["solved"]]
        col = lambda k: [r[k] for r in ok]
        rows.append(
            (
                name,
                len(records),
                len(ok) / max(len(records), 1),
                percentile(col("planning_us"), 50),
                percentile(col("planning_us"), 95),
                percentile(col("simplify_us"), 50),
                percentile(col("total_us"), 50),
                percentile(col("total_us"), 95),
                percentile(col("length"), 50),
            )
        )
        raw[name] = records
    label = "VAMP rrtc (default settings)" if dynamic_domain != "False" else "VAMP rrtc (dynamic domain off)"
    print(f"\n{label}, vamp-planner {getattr(vamp, '__version__', '0.6.4')}\nHardware: {cpu_name()}\n")
    print("| scenario | problems | success | planning median | planning P95 | simplification median | total median | total P95 | path length median |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        print(
            f"| {r[0]} | {r[1]} | {100 * r[2]:.0f}% | {fmt_us(r[3])} | {fmt_us(r[4])} | {fmt_us(r[5])} | "
            f"{fmt_us(r[6])} | {fmt_us(r[7])} | {r[8]:.2f} |"
        )
    if out:
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w") as f:
            json.dump(raw, f, indent=1)


if __name__ == "__main__":
    args = sys.argv[1:]
    pkl = args[0]
    dd = "True"
    out = None
    if "--dynamic_domain" in args:
        dd = args[args.index("--dynamic_domain") + 1]
    if "--out" in args:
        out = args[args.index("--out") + 1]
    main(pkl, dd, out)
