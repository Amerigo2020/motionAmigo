### All 689 valid problems pooled

| planner | success | planning median | planning P95 | simplification median | total median | total P95 | path length median (rad) |
|---|---:|---:|---:|---:|---:|---:|---:|
| VAMP 0.6.4, default (dynamic domain) | 100.0% | 56 µs | 753 µs | 142 µs | 211 µs | 937 µs | 6.53 |
| VAMP 0.6.4, dynamic domain off | 100.0% | 61 µs | 1.27 ms | 163 µs | 235 µs | 1.57 ms | 6.71 |
| motionAmigo, AVX2 | 100.0% | 95 µs | 1.56 ms | 296 µs | 453 µs | 1.96 ms | 6.74 |
| motionAmigo, portable SIMD | 100.0% | 373 µs | 5.42 ms | 1.21 ms | 1.80 ms | 7.36 ms | 6.74 |
| motionAmigo, scalar | 100.0% | 539 µs | 7.71 ms | 2.15 ms | 3.04 ms | 10.32 ms | 6.74 |

### Median planning time per scenario

| scenario | VAMP 0.6.4, default (dynamic domain) | VAMP 0.6.4, dynamic domain off | motionAmigo, AVX2 | motionAmigo, portable SIMD | motionAmigo, scalar |
|---|---:|---:|---:|---:|---:|
| bookshelf_small | 39 µs | 43 µs | 56 µs | 198 µs | 328 µs |
| bookshelf_tall | 73 µs | 75 µs | 116 µs | 435 µs | 612 µs |
| bookshelf_thin | 67 µs | 66 µs | 144 µs | 553 µs | 653 µs |
| box | 38 µs | 38 µs | 67 µs | 258 µs | 462 µs |
| cage | 455 µs | 926 µs | 1.00 ms | 3.90 ms | 5.05 ms |
| table_pick | 38 µs | 42 µs | 45 µs | 194 µs | 290 µs |
| table_under_pick | 62 µs | 64 µs | 110 µs | 459 µs | 687 µs |

### Median total time (planning + simplification) per scenario

| scenario | VAMP 0.6.4, default (dynamic domain) | VAMP 0.6.4, dynamic domain off | motionAmigo, AVX2 | motionAmigo, portable SIMD | motionAmigo, scalar |
|---|---:|---:|---:|---:|---:|
| bookshelf_small | 143 µs | 172 µs | 297 µs | 1.12 ms | 2.12 ms |
| bookshelf_tall | 258 µs | 266 µs | 500 µs | 2.09 ms | 3.52 ms |
| bookshelf_thin | 240 µs | 250 µs | 573 µs | 2.25 ms | 3.65 ms |
| box | 132 µs | 153 µs | 299 µs | 1.24 ms | 2.08 ms |
| cage | 653 µs | 1.12 ms | 1.34 ms | 5.31 ms | 7.30 ms |
| table_pick | 169 µs | 206 µs | 272 µs | 1.22 ms | 2.07 ms |
| table_under_pick | 247 µs | 260 µs | 573 µs | 2.36 ms | 3.52 ms |
