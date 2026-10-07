"""Export VAMP's MotionBenchMaker problems (problems.pkl) to JSON for motionAmigo.

The obstacles are converted exactly like vamp.problem_dict_to_vamp does it: cylinders become
capsules, except in the "box" scenario where VAMP over-approximates them with boxes. Rotations
are taken from the stored quaternions, so no Euler convention is involved.

Usage: python export_mbm.py vamp-src/resources/panda/problems.pkl ../data/mbm/panda_mbm.json
"""

import json
import pickle
import sys
from pathlib import Path

import numpy as np


def quat_to_matrix(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def convert(problem):
    out = {
        "valid": bool(problem["valid"]),
        "start": [float(v) for v in problem["start"]],
        "goal": [float(v) for v in problem["goals"][0]],
        "spheres": [],
        "capsules": [],
        "cuboids": [],
    }
    for s in problem["sphere"]:
        out["spheres"].append({"center": list(map(float, s["position"])), "radius": float(s["radius"])})
    for c in problem["cylinder"]:
        rot = quat_to_matrix(c["orientation_quat_xyzw"])
        center = np.array(c["position"], dtype=float)
        if problem["problem"] == "box":
            out["cuboids"].append(
                {
                    "center": center.tolist(),
                    "rotation": rot.tolist(),
                    "half_extents": [c["radius"], c["radius"], c["length"] / 2],
                }
            )
        else:
            axis = rot[:, 2] * c["length"] / 2
            out["capsules"].append(
                {"a": (center + axis).tolist(), "b": (center - axis).tolist(), "radius": float(c["radius"])}
            )
    for b in problem["box"]:
        out["cuboids"].append(
            {
                "center": list(map(float, b["position"])),
                "rotation": quat_to_matrix(b["orientation_quat_xyzw"]).tolist(),
                "half_extents": list(map(float, b["half_extents"])),
            }
        )
    return out


def main(src, dst):
    with open(src, "rb") as f:
        data = pickle.load(f)
    exported = {name: [convert(p) for p in problems] for name, problems in data["problems"].items()}
    Path(dst).parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "w") as f:
        json.dump(exported, f)
    print(f"exported {sum(len(v) for v in exported.values())} problems to {dst}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
