"""Pick the mug: plan to a pre-grasp pose, approach linearly, grasp, lift and go home.

    cd crates/motionamigo-py && uv run python ../../examples/python/pick.py
"""

from pathlib import Path

import numpy as np

import motionamigo as ma

SCENE = Path(__file__).resolve().parents[1] / "scenes" / "tabletop.json"

robot = ma.Robot.panda()
result = ma.plan_pick(robot, SCENE, "mug_1", ma.PANDA_READY, place=ma.PANDA_READY)
print(result)
print(f"to pre-grasp: {len(result.to_pregrasp)} waypoints")
print(f"approach:     {len(result.approach)} waypoints, TCP down to {np.round(result.grasp_pose[:3, 3], 3)}")
print(f"retreat:      {len(result.retreat)} waypoints with {len(result.attached_spheres)} object spheres")
print(f"home:         {len(result.place)} waypoints")

# The robot holding the mug, e.g. for further planning or visualization.
held = robot.with_attached("mug_1", result.attached_spheres)
lifted = held.spheres(result.retreat[-1])[-len(result.attached_spheres) :]
print(f"mug bottom after the lift: {np.min(lifted[:, 2] - lifted[:, 3]):.3f} m")
