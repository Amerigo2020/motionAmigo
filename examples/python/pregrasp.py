"""Language to motion: move the Panda above the object that a spatial expression refers to.

spatialAmigo resolves expressions like "the mug left of the laptop" in a scene. Both projects
share the scene format, so the resolved object id can be handed to motionAmigo directly. Here the
resolution step is a stand-in (a lookup table) so that the example runs on its own.

    cd crates/motionamigo-py && uv run python ../../examples/python/pregrasp.py
"""

from pathlib import Path

import numpy as np

import motionamigo as ma

SCENE = Path(__file__).resolve().parents[1] / "scenes" / "tabletop.json"

# Stand-in for spatialAmigo: expression -> object id in the shared scene. Left and right are
# seen from the scene's viewpoint, which looks along +x (so "left" means larger y).
RESOLVED = {
    "the mug right of the laptop": "mug_1",
    "the mug left of the laptop": "mug_2",
    "the bowl": "bowl_1",
}

robot = ma.Robot.panda()
start = ma.PANDA_READY
for expression, object_id in RESOLVED.items():
    try:
        result = ma.plan_to_pregrasp(robot, SCENE, object_id, start)
    except ma.PlanningError as err:
        print(f"{expression!r} -> {object_id}: {err}")
        continue
    pose = result.pose
    tilt = np.degrees(np.arccos(np.clip(-pose[2, 2], -1, 1)))
    print(
        f"{expression!r} -> {object_id}: TCP {np.round(pose[:3, 3], 3)}, tilt {tilt:.0f} deg, "
        f"{len(result.plan)} waypoints, {result.plan.total_time * 1e6:.0f} us"
    )
