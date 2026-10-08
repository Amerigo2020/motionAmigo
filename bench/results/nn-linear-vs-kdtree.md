# Nearest neighbours: linear scan vs. kd-tree (Milestone C)

Machine: Intel Core i9-13900H laptop, Windows 11 (native, not WSL2), rustc 1.88.0, single thread,
AVX2 checker. Laptop timings are noisy (about 10% between identical runs), so every number below
is from runs interleaved with the baseline.

## Query cost (criterion)

`cargo bench -p motionamigo --bench nn -- --warm-up-time 2 --measurement-time 4`

256 uniform 7-DOF queries against a tree grown like an obstacle-free RRT (range 1.0 rad, limits
+-2.9 rad). Median of criterion's estimate.

| nodes | linear scan | kd-tree |
|---:|---:|---:|
| 16 | 13.9 µs | 79.5 µs |
| 64 | 38.3 µs | 206 µs |
| 256 | 145 µs | 527 µs |
| 1024 | 743 µs | 1.64 ms |
| 2048 | 1.73 ms | 1.59 ms |
| 4096 | 3.39 ms | 2.44 ms |
| 16384 | 14.7 ms | 3.07 ms |

The crossover is at about 2048 nodes, so RRT-Connect scans linearly below 2048 nodes per tree and
queries the kd-tree from there on (`KD_MIN` in `planner/rrtc.rs`).

## Own scenes (`motionamigo-bench run --seeds 10`)

Plans are bit-identical, so success and path length do not change. Three interleaved runs each,
"before" is the linear scan only, "after" is the hybrid:

| scene | median total before | median total after | P95 total before | P95 total after | median planning before | median planning after |
|---|---:|---:|---:|---:|---:|---:|
| tabletop | 211 / 236 / 229 µs | 230 / 227 / 212 µs | 400 / 472 / 422 µs | 412 / 408 / 386 µs | 27 / 31 / 29 µs | 30 / 28 / 26 µs |
| shelf | 460 / 487 / 448 µs | 471 / 478 / 448 µs | 893 / 905 / 847 µs | 867 / 891 / 836 µs | 76 / 79 / 75 µs | 79 / 79 / 75 µs |
| cage | 1.51 / 1.48 / 1.58 ms | 1.36 / 1.37 / 1.55 ms | 12.74 / 12.62 / 12.88 ms | 11.86 / 12.18 / 12.40 ms | 1.01 / 1.01 / 1.01 ms | 897 / 912 / 992 µs |

Tabletop and shelf trees almost never reach 2048 nodes, so they are unchanged within noise. The
cage gains a few percent in median and P95, because collision checking, not the nearest neighbour
query, dominates planning time. Using the kd-tree from the first node (`KD_MIN = 0`) made the cage
P95 clearly worse (about 23 ms instead of 12 to 17 ms in the same session).

The MotionBenchMaker comparison (`mbm-*.md`) was not rerun: plans are identical, its median trees
are far below 2048 nodes, and the expected change is within the noise of the laptop.
