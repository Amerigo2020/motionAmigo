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
    PickResult,
    PlanningError,
    PlanResult,
    Planner,
    PregraspResult,
    Robot,
    __version__,
    plan_pick,
    plan_to_pregrasp,
    simd_backend,
)
from ._motionamigo import PANDA_READY as _PANDA_READY
from ._motionamigo import UR5_HOME as _UR5_HOME
from .language import (
    InstructionResult,
    ResolutionError,
    Resolver,
    builtin_resolver,
    default_resolver,
    plan_from_instruction,
    spatialamigo_resolver,
)

#: The common "ready" configuration of the Panda.
PANDA_READY = _np.array(_PANDA_READY, dtype=float)
PANDA_READY.setflags(write=False)

#: A collision-free UR5 configuration with the arm raised and the gripper pointing down.
UR5_HOME = _np.array(_UR5_HOME, dtype=float)
UR5_HOME.setflags(write=False)

__all__ = [
    "Environment",
    "InstructionResult",
    "PANDA_READY",
    "PickResult",
    "PlanningError",
    "PlanResult",
    "Planner",
    "PregraspResult",
    "ResolutionError",
    "Resolver",
    "Robot",
    "plan_from_instruction",
    "plan_pick",
    "plan_to_pregrasp",
    "__version__",
    "builtin_resolver",
    "default_resolver",
    "spatialamigo_resolver",
    "simd_backend",
    "UR5_HOME",
]
