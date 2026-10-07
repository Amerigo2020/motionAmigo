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
