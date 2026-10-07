# motionamigo (Python)

Python bindings for [motionAmigo](https://github.com/Amerigo2020/motionAmigo), a fast SIMD-vectorized
sampling-based motion planner for robot arms written in Rust.

```python
import motionamigo as ma

robot = ma.Robot.panda()
env = ma.Environment.from_scene("examples/scenes/tabletop.json")
planner = ma.Planner(robot, env)
result = planner.plan(ma.PANDA_READY, [0.06, 0.41, -1.16, -1.02, 0.55, 1.36, 0.52], seed=0)
print(result.path)                 # (K, 7) NumPy array of waypoints
print(result.interpolate(0.05))    # dense trajectory
```

Development with [uv](https://docs.astral.sh/uv/):

```bash
uv sync
uv run maturin develop --release --uv
uv run pytest
```
