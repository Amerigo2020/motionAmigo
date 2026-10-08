# Changelog

## 0.1.0 (2026-10-08)

First public release.

* Franka Panda and UR5 models (DH kinematics, sphere models from VAMP), forward kinematics and collision checking against spheres, capsules, oriented boxes and point clouds.
* SIMD collision checking with AVX2 (runtime dispatch), NEON and WASM simd128 backends, bit-identical to the scalar reference.
* RRT-Connect with shortcutting, deterministic seeded RNG, hybrid linear scan and kd-tree nearest neighbour search with identical results.
* Damped least squares IK, pre-grasp poses, straight-line Cartesian approach and retreat, attached objects and a full pick (`plan_pick`).
* Python bindings (PyO3, abi3 wheels) including `plan_from_instruction` with an optional spatialAmigo resolver and a built-in fallback.
* Browser demo (WebAssembly and three.js) with planning, dragging and picking: https://amerigo2020.github.io/motionAmigo/
* Benchmarks on own scenes and a MotionBenchMaker comparison with VAMP for Panda and UR5.
