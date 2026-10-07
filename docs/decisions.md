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
* **Nearest neighbours by linear scan** over a flat array. Trees for typical problems hold tens to
  a few hundred nodes, where a scan beats a kd-tree. A GNAT or kd-tree is on the roadmap.
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
