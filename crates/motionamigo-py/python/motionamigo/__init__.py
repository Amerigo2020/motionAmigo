"""motionAmigo: fast SIMD-vectorized sampling-based motion planning for robot arms.

Example::

    import motionamigo as ma

    robot = ma.Robot.panda()
    env = ma.Environment.from_scene("examples/scenes/tabletop.json")
    planner = ma.Planner(robot, env)
    result = planner.plan(ma.PANDA_READY, [0.06, 0.41, -1.16, -1.02, 0.55, 1.36, 0.52], seed=0)
    print(result.path.shape, result.planning_time)
"""

import numpy as _np

from ._motionamigo import (
    Environment,
    PlanningError,
    PlanResult,
    Planner,
    Robot,
    __version__,
    simd_backend,
)
from ._motionamigo import PANDA_READY as _PANDA_READY

#: The common "ready" configuration of the Panda.
PANDA_READY = _np.array(_PANDA_READY, dtype=float)
PANDA_READY.setflags(write=False)

__all__ = [
    "Environment",
    "PANDA_READY",
    "PlanningError",
    "PlanResult",
    "Planner",
    "Robot",
    "__version__",
    "simd_backend",
]
