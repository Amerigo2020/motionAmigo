# Benchmarks

Three benchmarks, all reproducible from this directory.

## 1. Own scenes: tabletop, shelf, cage

Scenes are in the shared scene format (`examples/scenes/*.json`), problems in
`bench/problems/*.json` (100 per scene). Every problem connects two collision-free task
configurations in different regions of the scene (for example two shelf compartments, or inside and
outside the cage), and problems that a straight line solves are rejected. The problem files were
generated once with a seeded generator and are committed:

```bash
cargo run --release -p motionamigo-bench -- gen --count 100   # regenerates bench/problems/
cargo run --release -p motionamigo-bench -- run --seeds 10    # 10 seeds per problem
cargo run --release -p motionamigo-bench -- run --seeds 10 --checker scalar
```

Reported: success rate, median and 95th percentile of the total time (RRT-Connect plus
shortcutting), median planning time and median joint-space path length. Times are statistics over
the solved runs. Results: `results/scenes-*.md` and raw per-run data in `results/scenes-*.json`.

## 2. Scalar against SIMD (criterion)

```bash
cargo bench -p motionamigo --bench collision
```

Results: `results/criterion-scalar-vs-simd.md`.

Nearest-neighbour queries, linear scan against the kd-tree:

```bash
cargo bench -p motionamigo --bench nn
```

Results: `results/nn-linear-vs-kdtree.md`.

## 3. VAMP on the MotionBenchMaker problems

`vamp/run_vamp.sh` installs `vamp-planner` from PyPI into a uv environment, clones the VAMP
repository at a pinned commit for its MotionBenchMaker problem set (7 Panda scenarios with 100
problems each, 699 valid), runs VAMP's RRT-Connect with its default Panda settings exactly as
`scripts/evaluate_mbm.py` does, exports the same problems to JSON and runs motionAmigo on them on the
same machine. The problem files are downloaded at benchmark time and not redistributed.

```bash
bench/vamp/run_vamp.sh   # needs uv, CMake, a C++ compiler and Eigen 3
```

Results: `results/mbm-comparison.md`, raw data in `results/mbm-*.json`.

Fairness notes:

* Same problems, same obstacle conversion (cylinders as capsules, boxes in the "box" scenario as in
  `vamp.problem_dict_to_vamp`), same joint limits (VAMP's URDF limits; 4 of the 1400 start and goal
  configurations lie outside the tighter Franka datasheet limits), same RRT-Connect range (1.0) and
  edge resolution (32 per radian), one trial per problem.
* VAMP is compiled by pip with `-O3 -march=native` and fast-math flags; motionAmigo is a portable
  build with runtime AVX2 dispatch and no FMA.
* Differences that remain: VAMP samples with a Halton sequence (motionAmigo uses a seeded PRNG),
  VAMP's default RRT-Connect uses dynamic domain sampling (a second VAMP row turns it off), and the
  path simplification algorithms differ (VAMP: greedy shortcut plus B-spline smoothing;
  motionAmigo: greedy plus randomized partial shortcuts, which costs more time and yields slightly
  shorter paths; a second motionAmigo row uses greedy shortcutting only).
* Both planners report their own internal timers for planning and simplification.

### UR5

`run_vamp.sh` also runs the UR5 problems of the same problem set (7 scenarios, 689 valid problems):
VAMP's `ur5` robot with its default RRT-Connect settings, then motionAmigo with
`motionamigo-bench mbm --robot ur5`. The UR5 is mounted like in VAMP's `ur5_spherized.urdf` (base
link on a 0.9144 m pedestal, rotated by 1.57 rad), joint limits are +-pi as in that URDF, and the
same 55 self-collision link pairs are checked as in VAMP's generated UR5 model.

Results: `results/mbm-ur5-comparison.md`, raw data in `results/mbm-ur5-*.json`. They were measured
on an Intel Core i9-13900H laptop under WSL2 (Ubuntu 24.04, uv-managed Python 3.12, Eigen 3.4
installed locally), not on the cloud VM of the Panda numbers, so the two tables must not be
compared with each other. The greedy-only motionAmigo row was not run for the UR5.
