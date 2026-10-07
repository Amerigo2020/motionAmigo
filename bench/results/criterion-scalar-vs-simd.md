# Scalar reference vs. SIMD checker (criterion)

`cargo bench -p motionamigo --bench collision -- --warm-up-time 2 --measurement-time 5`

Machine: cloud VM, Intel Xeon @ 2.80 GHz (2 vCPUs, `avx2` and `avx512f` flags; motionAmigo uses
AVX2), Linux 6.18, rustc 1.97.0, single thread. Scene: tabletop (7 oriented boxes), Panda with 59
spheres and 21 self-collision link pairs. Median of criterion's estimate.

| benchmark | scalar | SIMD portable | SIMD AVX2 | AVX2 speedup |
|---|---:|---:|---:|---:|
| 256 collision-free configurations (FK + CC) | 290 µs | 193 µs | 60.4 µs | 4.8x |
| 256 edges between random valid configurations | 30.8 ms | 17.6 ms | 4.67 ms | 6.6x |
| 16 planning problems (RRT-Connect + shortcutting) | 44.4 ms | 25.8 ms | 7.71 ms | 5.8x |

Per configuration this is 1.13 µs (scalar) against 236 ns (AVX2). Edge checks gain more than
single configurations because the rake spreads the eight lanes over the edge, so blocked edges are
rejected after fewer kernel calls. Planning gains less because nearest-neighbour search and the
single-configuration checks of new samples stay scalar.
