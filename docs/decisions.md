# Design decisions

This log records the decisions taken while building motionAmigo, with a short rationale for each.
Newest entries are appended at the bottom of each section.

## Project and process

* **Repository access.** The build session had no GitHub credentials for `Amerigo2020/motionAmigo`
  (no `add_repo` tool was available, the API answered "GitHub access to this repository is not
  enabled for this session"). Work happens in a local git repository with the requested author
  identity; it is pushed as soon as access exists. History is never rewritten for that.
* **Commit signing disabled locally.** The sandbox signs commits with a key that does not belong to
  Amerigo, which GitHub would show as "Unverified" next to his name. The repository therefore sets
  `commit.gpgsign=false`.
* **License.** `MIT OR Apache-2.0`, the Rust ecosystem default. Third-party data is listed in
  `NOTICE`.

## Scene format

* **Schema is strict, parser is lenient.** `schema/scene-v0.1.json` uses
  `additionalProperties: false` so typos are caught by validators. The Rust parser ignores unknown
  fields so that a slightly newer spatialAmigo output still loads.
* **Every object is an oriented box.** As specified, `front` and `viewpoint` are parsed but ignored
  by the planner. `yaw` rotates about world z, `size` holds full edge lengths.

## Build and CI

* **Rust edition 2021, MSRV 1.86.** 1.86 is the first release with safe `#[target_feature]`
  functions (target_feature 1.1), which keeps the amount of `unsafe` in the SIMD layer minimal.
* **WASM always uses simd128.** Set in `.cargo/config.toml`; every evergreen browser supports it.
* **Python packaging.** `crates/motionamigo-py` is a maturin project. `[tool.uv] package = false`
  lets `uv sync` install only the dev tools, and `uv run maturin develop --uv` builds the extension.
  This keeps the requested `uv sync`, `uv run maturin develop`, `uv run pytest` workflow.
* **abi3 wheels.** One wheel per platform covers Python 3.9 and newer.
* **CI matrix** covers x86_64 Linux, aarch64 Linux (NEON backend), macOS (aarch64) and Windows.

## Robot model (M1)

* **Robot description as TOML.** Human-editable, comments allowed, small parser. A robot is a
  serial chain of revolute joints in modified (Craig) or standard DH convention, a TCP transform,
  links made of spheres attached to a joint frame, and explicit self-collision link pairs. Adding a
  UR5 later means writing one more TOML file.
* **Panda kinematics from Franka's modified DH table**, limits from the official Panda datasheet
  (for example joint 4 in `[-3.0718, -0.0698]`). A test builds the URDF joint chain independently
  and checks that the DH frames coincide with the URDF link frames, plus literal reference poses
  computed with numpy.
* **Spheres from VAMP's `panda_spherized.urdf`** (59 spheres, Apache-2.0, originally from
  robowflex_resources, MIT). `tools/gen_panda_toml.py` converts it; spheres of the rigidly attached
  hand and fingers are re-expressed in the joint 7 frame so the kernel needs no fixed frames.
  Self-collision pairs are all link pairs not disabled in the SRDF (21 pairs).
* **Fingers fixed at the MBM opening.** The finger spheres keep the MotionBenchMaker separation of
  the URDF, which makes the VAMP comparison fair. A movable gripper is out of scope.
* **Bounding spheres per link** are computed at load time (bounding-box center, conservative radius
  plus 0.1 mm, rounded up when converted to `f32`) for a two-level hierarchical check.

## Numerics and SIMD design (M1)

* **`f32` everywhere in the hot path.** Millimeter-level accuracy is far below the sphere
  approximation error, and `f32` doubles the SIMD width compared to `f64`.
* **One generic kernel, many backends.** FK and collision kernels are written once against a
  small `Real` trait. `f32` is the scalar reference (one configuration at a time), eight-lane types
  give the SoA vectorized version. Same code, same operation order.
* **No fused multiply-add, bit-identical results.** All backends only use exactly rounded
  operations, so scalar, portable, AVX2, NEON and WASM results are bit-identical. This makes the
  scalar/SIMD equivalence tests exact instead of tolerance based, and makes plans reproducible
  across platforms (the browser computes the same path as the native library for the same seed).
  The price is a few percent of throughput on FMA hardware.
* **Own sine and cosine.** Library `sin`/`cos` differ across platforms and are not vectorized.
  The kernel uses a Cody-Waite range reduction to `[-pi, pi]`, a reflection to `[-pi/2, pi/2]` and
  Taylor polynomials of degree 11 and 12 (error below 5e-7 rad for `|x| < 40`).
* **Twist special cases.** The constant DH twist is applied with branches for `0`, `+-pi/2` and
  `pi`, avoiding multiplications by zero and one. The branch depends on robot data only, never on
  lane data, so it is perfectly predictable.

## Collision checking (M2)

* **Supported obstacles:** spheres, capsules, oriented boxes and point clouds. Scene objects become
  oriented boxes. Contact is strict: touching (distance exactly zero) counts as free.
* **Sphere against oriented box** uses the exact squared distance to the box (clamped projections
  on the three box axes), so the test is exact, not an approximation.
* **Hierarchical, lazy evaluation.** Links are processed in kinematic order. A link's frame is
  computed only when needed, then its bounding sphere is tested; the individual spheres are only
  tested if the bounding sphere touches something. Collisions near the base therefore exit after
  very little work. Self-collision is checked at the end, with the same bounding-sphere filter.
* **Point clouds in a uniform grid** with per-cell structure-of-arrays storage. The query visits
  only cells overlapped by the sphere's bounding box. Simpler than VAMP's CAPT structure and exact;
  CAPT-like precomputation is on the roadmap.
* **Edge discretization like VAMP:** `ceil(length * resolution / 8)` passes of eight configurations,
  resolution 32 per radian by default (VAMP's Panda setting).
* **Direction-independent edges.** Edge points in the first half are interpolated from the start,
  points in the second half from the end, and the midpoint symmetrically as `(a + b) / 2`. Since
  `a - b == -(b - a)` exactly in IEEE arithmetic, checking `a -> b` and `b -> a` evaluates the very
  same configurations. Without this, a path whose edge was validated in one direction could, in
  rare boundary cases, fail validation in the other direction. A property test checks this.

## Planning (M2)

* **RRT-Connect** with balanced trees, VAMP's Panda defaults (range 1.0 rad, direct start-goal
  connection attempted first, multiple goals as roots of the goal tree).
* **Nearest neighbours by linear scan** over a flat array for small trees, a kd-tree for large
  ones; see "Nearest neighbours (C)".
* **Own deterministic RNG** (xoshiro256** seeded by SplitMix64) instead of the `rand` crate, so a
  seed gives the same plan on every platform and in every version. A test checks the first output
  against the reference implementation.
* **Shortcutting** = VAMP-style greedy vertex shortcutting plus randomized partial shortcuts between
  random points on the path (new cut points and the partial edges to them are validated too). It
  never lengthens a path.
* **Time measurement compiles on wasm32**, where `std::time::Instant` is unavailable: the core
  reports zero there and the WASM bindings measure with `performance.now()`.

## SIMD (M3)

* **Own abstraction over `std::arch`, no `wide` crate.** Runtime dispatch on x86_64 requires
  control over `#[target_feature]` boundaries, and bit-identical results require control over
  min/max semantics (see below). Both are awkward with `wide`, whose backend is fixed at compile
  time. The abstraction is one trait (`Real`) with five implementations, about 200 lines each.
* **Runtime dispatch for AVX2.** Python wheels and native binaries must run on any x86_64 CPU,
  so AVX2 is not enabled at compile time. Two small `#[target_feature(enable = "avx2")]` entry
  points (edge validation and block check) instantiate the generic kernel with the AVX2 type.
  Disassembly confirms the whole kernel is inlined into AVX code without calls.
* **NEON and WASM are compile-time backends.** NEON is part of the aarch64 baseline; WASM SIMD is
  enabled for the web build via `.cargo/config.toml`.
* **Min/max are compare plus select.** `fmin`/`fmax` style instructions treat `-0.0`/`+0.0` and NaN
  differently across ISAs. x86 `minps` happens to be exactly `if a < b { a } else { b }`; NEON and
  WASM use an explicit compare plus bit-select to match it.
* **`unsafe` is confined to `simd/`.** x86 intrinsics are unsafe to call outside a
  `#[target_feature]` context and are guarded by the runtime check; NEON loads and stores take raw
  pointers; `simd/stack.rs` is a fixed-capacity stack vector over `MaybeUninit`. Profiling
  (callgrind) showed that zero-initializing the per-call scratch arrays of eight-lane vectors cost
  more instructions than the kinematics itself, so the kernel uses that buffer instead.
* **Obstacle hit lists.** A link's bounding sphere test records which obstacles it touches; the
  link's spheres are then only tested against those. This halved the cost per edge.
* **Three-level self-collision.** Bounding sphere against bounding sphere, then each sphere of one
  link against the other link's bounding sphere (both directions, as bit masks), then the remaining
  sphere pairs.
* **Equivalence is tested on all backends,** including NEON (aarch64 runner in CI, qemu locally)
  and WASM SIMD (the whole test suite runs on `wasm32-wasip1` under wasmtime in CI). Tests compare
  FK sphere centers bit for bit, block results, edge results and complete plans.
* **The SIMD checker is the default.** Single configurations are still checked with the scalar
  kernel (nothing to vectorize); edges are checked eight configurations at a time.

## Benchmarks (M4)

* **Own problems are generated once and committed.** A seeded stochastic search places the TCP in
  task regions (table areas, shelf compartments, inside and outside the cage) with a given approach
  direction; straight-line problems are rejected so every problem needs search. Committing the
  problem files keeps the benchmark stable even if the generator changes.
* **The shelf sits at x = 0.525 to 0.875 m.** A reachability scan showed that horizontal grasps at
  shelf heights are not possible much further away for the Panda.
* **VAMP comparison uses VAMP's own pipeline.** `vamp-planner` 0.6.4 from PyPI (sdist, compiled by
  pip with `-march=native`), its MotionBenchMaker problem files and converter from a pinned commit,
  and a runner that mirrors `scripts/evaluate_mbm.py`. VAMP's official evaluation script was also
  run on the same machine and gives the same median planning time (89 µs), and the iteration counts
  match VAMP's published reference exactly, so the setup is faithful.
* **Problems are exported via quaternions,** not Euler angles, to avoid convention mismatches; the
  "box" scenario keeps VAMP's box over-approximation of cylinders.
* **MBM runs use VAMP's URDF joint limits** (4 of 1400 start/goal configurations violate the tighter
  datasheet limits that motionAmigo uses by default).
* **Shortcutting default reduced to 3 rounds of 32 random attempts.** A parameter sweep showed that
  rounds 4 and 5 cost about 15% more time for under 1% shorter paths. Trying the shortcut edge
  before checking the cut points made each attempt cheaper without changing results.
* **Results are reported honestly, including where VAMP is faster** (P95 planning time and
  simplification time).

## Python bindings (M5)

* **PyO3 0.29 + maturin, abi3 for Python 3.9 and newer.** One wheel per platform.
* **Array-like inputs.** All inputs accept anything NumPy can convert (lists, tuples, arrays of
  any float dtype); outputs are `float64` NumPy arrays.
* **Three objects mirror the Rust API:** `Robot`, `Environment` and `Planner` (checker plus
  planner bound to one environment, so the environment is broadcast to SIMD lanes only once).
  `PlanResult.interpolate(step)` returns a dense trajectory for execution or animation.
* **Errors:** invalid arguments raise `ValueError`, failed or invalid planning queries raise
  `motionamigo.PlanningError` (a `RuntimeError`).
* **The GIL is released** during planning and batch validity checks, tested with threads.
* **Type stubs and `py.typed`** ship with the package for editor support.
* **Not published to PyPI**; building from source with uv is documented instead.

## WebAssembly and browser demo (M6)

* **wasm-bindgen via wasm-pack, `--target web`.** No bundler: the demo is plain ES modules with an
  import map, so GitHub Pages can serve `web/` directly.
* **three.js 0.170 is vendored** (`web/vendor/three`, MIT license included) instead of loaded from a
  CDN, so the demo works offline, in headless CI and without third-party requests.
* **The robot is drawn as capsules** between the joint origins plus a box hand and two finger
  capsules, no manufacturer meshes. The 59 collision spheres can be overlaid.
* **Timing in the browser** uses `performance.now()` imported into the WASM module, measured around
  RRT-Connect and shortcutting separately. Browsers coarsen this timer (about 0.1 ms), which the
  stats panel inherits. The module plans a few warm-up queries at startup so the first measured
  plan is not dominated by tier-up of the JIT.
* **Dragging** moves objects in the horizontal plane through their grab point; every move rebuilds
  the collision environment (cheap: a few broadcasts) and recolors the robot red if it is now in
  collision.
* **wasm-opt is run by `web/build.sh`**, not by wasm-pack: wasm-pack's own binaryen download did
  not work behind the build sandbox's TLS proxy, and binaryen releases older than 116 corrupt the
  externref table emitted by current wasm-bindgen. The script only uses wasm-opt 116 or newer.
* **The README GIF is recorded deterministically**: a Playwright script plans, then steps through
  the dense trajectory frame by frame and screenshots each frame; Pillow assembles a 128-color GIF
  (about 0.5 MB). Headless Chromium renders WebGL through SwiftShader.
* **CI smoke tests the demo** in headless Chromium: it must load, plan successfully, and a scripted
  mouse drag must move an object both in three.js and in the WASM scene.

## Inverse kinematics and pre-grasp (M7)

* **Damped least squares on the full 6D pose error,** with the geometric Jacobian computed from the
  FK frames (checked against finite differences), the rotation error as the log map of
  `R_target * R^T`, step clamping, joint-limit clamping and seeded random restarts. The first attempt
  starts from the current configuration, so solutions tend to stay close to it. A test solves 50
  random reachable poses and requires at least 48 successes (the result is deterministic).
* **IK runs in `f64`, collision checks in `f32`.** Solutions are accepted only if the `f32`-rounded
  configuration is collision-free, which is exactly what the planner will check later.
* **Pre-grasp poses are top-down** with the TCP 10 cm above the top face of the object box. The
  fingers close across the narrower horizontal side; sides that fit into the 8 cm gripper are tried
  first, each in both hand orientations.
* **Tilted fallback.** Objects near the edge of the workspace (for example `mug_2` at 0.84 m) are
  not reachable with a vertical tool axis. Tilts of 0.35, 0.7 and 1.0 rad away from the base are
  tried next; the TCP position stays the same.
* **The approach itself is not planned** (it would need Cartesian or constrained planning). The
  Rust example checks it as a straight joint-space motion, with the target object removed from the
  environment because the fingers must enclose it. It reports honestly when no approach is found.
* **Spatial expressions in the examples use the viewpoint of the scene,** which looks along +x, so
  "left" means larger y. The examples use a lookup table instead of spatialAmigo, to stay
  self-contained.

## UR5 (A)

* **Kinematics from the official Universal Robots standard DH table** (d1 = 0.089159,
  a2 = -0.425, a3 = -0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823), no theta offsets. The UR5
  is the first bundled robot in standard DH convention; the kernels already supported it.
* **Joint limits +-pi, not +-2pi.** VAMP's `ur5_spherized.urdf` (and therefore VAMP's planner and the
  MotionBenchMaker problems) uses +-3.14159265. Matching it keeps the benchmark fair; a user who
  needs the full +-2pi range can edit the TOML.
* **Frame 0 is the UR DH base frame,** which is the classic ROS `base_link` rotated by pi about z.
  With that rotation every URDF link frame is rigidly attached to one DH frame, and the URDF
  `tool0` frame coincides with DH frame 6, so the TCP is the identity. The pedestal of VAMP's URDF
  (`offset_link`, 0.9144 m up, yaw 1.57) is not part of the model; the benchmark sets it as the
  robot base.
* **Spheres from VAMP's `resources/ur5/ur5_spherized.urdf`** at the pinned VAMP commit
  `27cb9b66` (40 spheres, Apache-2.0, originally robowflex_resources, MIT). The URDF describes a
  UR5 with a Robotiq FT sensor and a Robotiq 2F-85 gripper whose finger joints are all fixed, so
  the gripper opening is fixed like the Panda's fingers. `tools/gen_ur5_toml.py` builds the URDF
  and DH chains, checks that the offset between each link frame and its DH frame is the same for
  several random configurations, and moves the spheres into the DH frames. Gripper and sensor links
  are attached to frame 6, so there are 17 links.
* **Self-collision pairs: SRDF plus two rules.** Starting from all link pairs not disabled in
  `ur5.srdf`, pairs on the same frame (rigidly attached, their distance never changes) and pairs
  that collide in every configuration (`wrist_2_link` against `fts_robotside`, both centered on the
  wrist 3 axis) are dropped, like MoveIt's "always in collision" rule. The result is exactly the 55
  pairs that VAMP's generated `ur5.hh` checks.
* **Tests.** `tests/fk_reference.rs` builds the URDF joint chain (origins and y or z axes copied
  from the URDF) independently of the DH table and checks that the frame offsets are constant and
  that the TCP equals `tool0`; it also checks reference TCP poses computed in Python, one sphere
  against its URDF position, and the `f32` kernel against `f64`. The scalar against SIMD tests and
  the identical-plans test now run for both robots. The kernels are generic over the number of
  joints (stack arrays sized by `MAX_DOF = 8`), so 6 joints needed no kernel change.
* **Benchmark run in WSL2, not on the cloud VM.** VAMP 0.6.4 builds from source; on this Windows
  machine it was built in WSL2 Ubuntu 24.04 with a uv-managed Python (the system Python lacks
  development headers) and a locally installed Eigen 3.4 (no root access for apt). VAMP and
  motionAmigo ran on the same laptop (i9-13900H), all 689 valid UR5 problems were solved by both.
  The Panda numbers were not re-measured on the laptop.
* **Not covered for the UR5:** the pre-grasp helper (`grasp.rs`) is untested with it. The UR5 TCP is
  the flange, not the grasp point between the fingers, and the gripper width constant is the
  Panda's. The browser demo still shows only the Panda.

## Attached objects and pick (B)

* **An attached object is just another collision link.** `RobotModel::attach_object` transforms
  the spheres from the TCP frame into the frame of the last joint (the TCP is a constant offset from
  it) and appends a link on that frame. Forward kinematics, the hierarchical environment check and
  the self-collision code therefore needed no change at all, and the scalar and SIMD kernels stay
  bit-identical by construction. The SIMD equivalence tests (proptest blocks and edges, identical
  plans) now also run for both robots holding an object.
* **Self-collision exclusion: links on the last frame.** The held object is checked against every
  link that is not on the last joint frame. Links on that frame (Panda `link7`, hand and fingers;
  UR5 `wrist_3_link`, FT sensor and gripper) move rigidly with the object, so their distance never
  changes and a check would report either always or never; the fingers touch the object by design.
  This is the same rule that removes rigidly attached pairs from the pair list of the robot itself.
* **Attach and detach on the robot model, not on the checker.** Checkers are immutable and cheap to
  build, so a pick builds one checker per phase. Python follows its frozen style:
  `Robot.with_attached` and `Robot.without_attached` return modified copies.
* **The limits stay as they were:** at most 64 spheres per attached object (the self-collision
  bitmask is a `u64`) and the robot-wide limits of 128 spheres and 24 links.
* **Note on the UR5 TCP.** The UR5 TCP is the `tool0` flange, not a point between the fingers, so
  spheres attached to the UR5 must be placed about 0.15 m further out along the tool axis.
* **Linear approach and retreat by dense IK.** The line is cut into 5 mm steps. Each step runs the
  damped least squares solver seeded with the previous solution and without random restarts, so it
  cannot jump to another IK branch; a joint change above 0.1 rad per step is rejected anyway. Every
  joint-space segment between steps is checked with `motion_valid`, so the whole motion is covered
  at the planner resolution, not only the step configurations. The orientation is held fixed.
* **Pick sequence.** Pre-grasp candidates are tried in order (vertical first, then tilted) until
  one admits a collision-free approach and retreat; only then is the motion to the pre-grasp pose
  planned. The approach is checked without the target object, because the fingers enclose it. The
  TCP stops 2 cm below the top face (at most half the object height). The retreat is the approach
  in reverse, which keeps it exactly on the line, and it is checked again with the object attached,
  still without its old box. The optional place motion is planned with the object attached.
* **Box to spheres, inside the box.** The sphere radius is half the smallest box side minus 3 mm,
  and the centers form a grid along the other two sides (at most 8 per side, so at most 64
  spheres). The spheres stay inside the box: an enclosing approximation would penetrate the table
  the object rests on and make the grasp configuration invalid. The price is rounded edges and
  corners, documented as a limitation. A mug becomes 2 spheres of 3.7 cm radius.
* **No gripper actuation.** Finger opening and closing is not modeled, like the fixed gripper
  model of the robots. The browser demo animates the same segments and lets the box follow the TCP
  after the grasp sample; the headless smoke test checks that a mug ends up in the hand.
* **mug_2 is not pickable for the Panda** with these settings: its tilted pre-grasp poses are
  reachable, but going straight down along the tool axis leaves the workspace after about 6 cm.
  A test pins this behaviour (`NoApproach`) so that a change is noticed.

## Nearest neighbours (C)

* **Kd-tree, not GNAT.** RRT-Connect only ever inserts and queries one nearest neighbour, in 6 or
  7 dimensions with the plain L2 metric. An incremental kd-tree whose nodes are the points
  themselves needs two child indices per node, no rebalancing and no distance evaluations to
  insert. GNAT pays off for expensive or non-Euclidean metrics, which motionAmigo does not have.
  Random samples are inserted in random order, so the unbalanced tree stays shallow in practice.
* **Exactly the linear scan's result.** Distances are the same f32 sum of squares in dimension
  order, ties go to the lowest index. Pruning compares a lower bound (squared offsets of `q` to the
  subtree's cell, summed in dimension order) against the best distance and only skips when it is
  strictly larger. Rounded f32 subtraction, squaring and adding non-negative values are monotone,
  so no point's computed distance can fall below the bound: the result is identical, not only
  within a tolerance. Property tests compare against the scan with many ties and duplicate points,
  and a test checks that RRT-Connect plans are identical with the scan, the kd-tree and the hybrid.
* **Hybrid by measurement.** The kd-tree is always maintained (insertion is cheap), but queries
  scan linearly below 2048 nodes per tree. Criterion puts the crossover there for 7 DOF; below it
  the scan is up to 5x faster, at 16384 nodes the kd-tree is 4.8x faster. On the own scenes the
  effect is within noise for tabletop and shelf and a few percent for the cage, because collision
  checking dominates. Numbers: `bench/results/nn-linear-vs-kdtree.md`.
* **No recursion.** The search uses an explicit stack, so degenerate (deep) trees cannot overflow
  the call stack, which matters on WebAssembly.

## spatialAmigo coupling (D)

* **spatialAmigo is not public on GitHub yet.** The coupling was written against its Python API as
  read from a local copy (`spatialamigo.Resolver(Scene(dict)).resolve(text)` returning a
  `Resolution` with `best`, `margin` and `explanation`). It is an optional import, not a
  dependency, and the adapter is tested with a stub module.
* **A protocol, not a dependency.** `Resolver` is any callable `(scene_dict, instruction) -> id`.
  `plan_from_instruction` loads the scene once, resolves, then calls `plan_pick` unchanged, so the
  scene format v0.1 did not change and the Rust side was not touched.
* **Ambiguity is an error.** spatialAmigo always returns a best candidate; the adapter rejects a
  margin below 0.05 (adjustable) so that a robot asks back instead of guessing.
* **Fallback resolver.** Pure Python: ids, labels in order of appearance (first is the target,
  second the anchor) and one relation. Left, right, front and behind are half-planes in the
  viewpoint frame (viewpoint looks along +x by default, so "left" means larger y); "near" is the
  closest candidate. No colors: the scene format has no color attribute.

## Release (v0.1.0)

* **Wheels on the GitHub release** come from the regular Python CI run (Linux x86_64, macOS arm64, Windows x86_64).
* **Registry publishing is prepared but off.** No PyPI or crates.io credentials exist yet. `release.yml` publishes to PyPI via Trusted Publishing and to crates.io with `CARGO_REGISTRY_TOKEN`, each gated by a repository variable (`PYPI_ENABLED`, `CRATES_ENABLED`) so a release never fails for missing setup.
* **The commit history was imported unchanged** from the original bundle (11 commits, not 12 as assumed in the brief).
