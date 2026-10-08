"""Language to motion: pick the object an instruction refers to.

Uses spatialAmigo for the resolution when it is installed, otherwise the built-in resolver.

    cd crates/motionamigo-py && uv run python ../../examples/python/plan_from_instruction.py
"""

from pathlib import Path

import motionamigo as ma

SCENE = Path(__file__).resolve().parents[1] / "scenes" / "tabletop.json"

robot = ma.Robot.panda()
for instruction in ["pick up the mug right of the laptop", "the mug left of the laptop", "the mug"]:
    try:
        out = ma.plan_from_instruction(SCENE, instruction, robot=robot, start=ma.PANDA_READY)
    except (ma.ResolutionError, ma.PlanningError) as err:
        print(f"{instruction!r}: {err}")
        continue
    print(f"{instruction!r} -> {out.object_id}: {len(out.pick.to_pregrasp)} + {len(out.pick.approach)} waypoints")
