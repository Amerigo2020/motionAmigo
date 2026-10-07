"""Plan a pick-and-place style motion in the shared tabletop scene.

Run from the repository root after building the bindings:

    cd crates/motionamigo-py && uv sync && uv run maturin develop --release --uv
    uv run python ../../examples/python/plan_tabletop.py
"""

from pathlib import Path

import numpy as np

import motionamigo as ma

SCENE = Path(__file__).resolve().parents[1] / "scenes" / "tabletop.json"

robot = ma.Robot.panda()
env = ma.Environment.from_scene(SCENE)
planner = ma.Planner(robot, env)
print(robot, env, planner, sep="\n")

# Hand-down configurations above mug_1 and mug_2.
above_mug_1 = np.array([0.06, 0.41, -1.16, -1.02, 0.55, 1.36, 0.52])
above_mug_2 = np.array([-0.18, 0.22, 0.89, -1.21, -0.08, 1.54, 1.31])

current = ma.PANDA_READY
for name, goal in [("mug_1", above_mug_1), ("mug_2", above_mug_2), ("ready", ma.PANDA_READY)]:
    result = planner.plan(current, goal, seed=0)
    tcp = robot.fk(result.path[-1])[:3, 3]
    print(
        f"to {name:6s}: {len(result)} waypoints, {result.length:.2f} rad, "
        f"{result.total_time * 1e6:.0f} us, TCP at {np.round(tcp, 3)}"
    )
    current = goal

# Dense trajectory for execution or animation (at most 0.05 rad between samples).
trajectory = result.interpolate(0.05)
assert planner.configs_valid(trajectory).all()
print(f"last trajectory: {trajectory.shape[0]} samples, all collision-free")

# Batch validity checks are vectorized over NumPy arrays.
samples = np.random.default_rng(0).uniform(robot.lower_limits, robot.upper_limits, (10000, 7))
print(f"{planner.configs_valid(samples).mean():.1%} of random configurations are collision-free")
