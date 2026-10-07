# motionAmigo

**A fast, SIMD-vectorized sampling-based motion planner for robot arms, written in Rust, with Python bindings and a live browser demo.**

motionAmigo plans collision-free motions for a Franka Emika Panda (7 DoF) with RRT-Connect and
shortcutting. Following [VAMP](https://github.com/KavrakiLab/vamp) (Thomason, Kingston, Kavraki,
ICRA 2024), the robot is approximated by spheres, and forward kinematics plus collision checking run
on eight configurations along an edge at once with SIMD instructions. motionAmigo is an independent
Rust implementation of these ideas, not a port of the VAMP code.

* Plans in about 0.1 ms (median) on the MotionBenchMaker Panda problems, on par with VAMP on the
  same machine ([numbers below](#benchmarks)).
* One generic kernel for all backends: scalar reference, AVX2 (runtime dispatch), NEON, WebAssembly
  SIMD and a portable fallback. All of them produce **bit-identical** results, so a seed gives the
  same plan on every platform.
* Shares its scene format with the sister project **spatialAmigo**, which turns language such as
  "the mug left of the laptop" into a target object.

## Benchmarks

All numbers below were measured on the same machine: a cloud VM with an Intel Xeon @ 2.80 GHz,
2 vCPUs, Linux, single-threaded. This is a noisy, rather slow machine (VAMP's own reference timings
on a Ryzen 9 7950X are about 2.5x faster than what VAMP achieves here); repeated runs varied by
about 15%. Raw data and scripts are in [`bench/`](bench/).

### Comparison with VAMP (MotionBenchMaker, Panda, 699 problems)

VAMP 0.6.4 from PyPI (compiled with `-O3 -march=native`), default RRT-Connect settings, one trial
per problem, same problems, obstacles, joint limits and edge resolution for both planners. See
[`bench/README.md`](bench/README.md) for the fairness notes.

| planner | success | planning median | planning P95 | simplification median | total median | total P95 | path length median |
|---|---:|---:|---:|---:|---:|---:|---:|
| VAMP, default (dynamic domain) | 100% | 115 µs | 1.03 ms | 208 µs | 348 µs | 1.47 ms | 4.90 rad |
| VAMP, dynamic domain off | 100% | 88 µs | 1.41 ms | 208 µs | 315 µs | 1.89 ms | 4.84 rad |
| **motionAmigo, AVX2** | 100% | 119 µs | 1.54 ms | 428 µs | 589 µs | 2.20 ms | 4.82 rad |
| motionAmigo, AVX2, greedy shortcutting only | 100% | 118 µs | 1.50 ms | 48 µs | 174 µs | 1.61 ms | 5.44 rad |
| motionAmigo, scalar reference | 100% | 584 µs | 5.66 ms | 2.33 ms | 3.08 ms | 9.01 ms | 4.82 rad |

In short: RRT-Connect planning time is on par with VAMP (VAMP is up to 1.35x faster at the median,
depending on its settings, and has a lower P95). motionAmigo's default shortcutting spends about 2x
more time than VAMP's simplification for slightly shorter paths; with greedy shortcutting only it is
faster than VAMP's, but leaves longer paths. Per-scenario tables:
[`bench/results/mbm-comparison.md`](bench/results/mbm-comparison.md).

### Scalar reference against SIMD (criterion)

Tabletop scene, Panda with 59 spheres, `cargo bench -p motionamigo --bench collision`:

| benchmark | scalar | SIMD portable | SIMD AVX2 | AVX2 speedup |
|---|---:|---:|---:|---:|
| collision-free configuration (FK + collision check) | 1.13 µs | 754 ns | 236 ns | 4.8x |
| edge between two random valid configurations | 120 µs | 69 µs | 18.2 µs | 6.6x |
| planning problem (RRT-Connect + shortcutting) | 2.78 ms | 1.61 ms | 0.48 ms | 5.8x |

### Own scenes (tabletop, shelf, cage)

100 problems per scene (start and goal in different regions, for example two shelf compartments or
inside and outside the cage; problems a straight line solves are excluded), 10 seeds each:

| scene | runs | success | median total | P95 total | median planning | median path length |
|---|---:|---:|---:|---:|---:|---:|
| tabletop | 1000 | 100% | 356 µs | 653 µs | 40 µs | 4.96 rad |
| shelf | 1000 | 100% | 784 µs | 1.56 ms | 131 µs | 6.77 rad |
| cage | 1000 | 100% | 2.57 ms | 21.8 ms | 1.80 ms | 8.09 rad |

With the scalar checker the median totals are 1.90 ms, 4.43 ms and 8.42 ms.

## Scene format

motionAmigo shares its scene format with spatialAmigo. Together they form the chain
language, target object, collision-free motion.

* Schema: [`schema/scene-v0.1.json`](schema/scene-v0.1.json)
* Examples: [`examples/scenes/`](examples/scenes/) (tabletop, shelf, cage)

Units are meters, z points up, the frame is right-handed. Each object is an oriented box with a
`center`, full edge lengths `size` and a `yaw` angle about z. The optional `front` and
`viewpoint` fields are used by spatialAmigo and ignored by the planner.

## Design notes

Design decisions and their rationale are logged in [`docs/decisions.md`](docs/decisions.md).

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT)
at your option. See [NOTICE](NOTICE) for third-party data.
