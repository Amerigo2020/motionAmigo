### All 699 valid problems pooled

| planner | success | planning median | planning P95 | simplification median | total median | total P95 | path length median (rad) |
|---|---:|---:|---:|---:|---:|---:|---:|
| VAMP 0.6.4, default (dynamic domain) | 100.0% | 115 µs | 1.03 ms | 208 µs | 348 µs | 1.47 ms | 4.90 |
| VAMP 0.6.4, dynamic domain off | 100.0% | 88 µs | 1.41 ms | 208 µs | 315 µs | 1.89 ms | 4.84 |
| motionAmigo, AVX2 | 100.0% | 119 µs | 1.54 ms | 428 µs | 589 µs | 2.20 ms | 4.82 |
| motionAmigo, AVX2, greedy shortcutting only | 100.0% | 118 µs | 1.50 ms | 48 µs | 174 µs | 1.61 ms | 5.44 |
| motionAmigo, portable SIMD | 100.0% | 412 µs | 4.97 ms | 1.43 ms | 1.99 ms | 6.87 ms | 4.82 |
| motionAmigo, scalar | 100.0% | 584 µs | 5.66 ms | 2.33 ms | 3.08 ms | 9.01 ms | 4.82 |

### Median planning time per scenario

| scenario | VAMP 0.6.4, default (dynamic domain) | VAMP 0.6.4, dynamic domain off | motionAmigo, AVX2 | motionAmigo, AVX2, greedy shortcutting only | motionAmigo, portable SIMD | motionAmigo, scalar |
|---|---:|---:|---:|---:|---:|---:|
| bookshelf_small | 120 µs | 133 µs | 77 µs | 75 µs | 240 µs | 419 µs |
| bookshelf_tall | 70 µs | 57 µs | 99 µs | 86 µs | 344 µs | 441 µs |
| bookshelf_thin | 72 µs | 70 µs | 143 µs | 118 µs | 447 µs | 572 µs |
| box | 146 µs | 136 µs | 157 µs | 162 µs | 466 µs | 809 µs |
| cage | 508 µs | 974 µs | 869 µs | 880 µs | 2.83 ms | 4.04 ms |
| table_pick | 60 µs | 50 µs | 50 µs | 49 µs | 192 µs | 302 µs |
| table_under_pick | 127 µs | 81 µs | 104 µs | 103 µs | 375 µs | 596 µs |

### Median total time (planning + simplification) per scenario

| scenario | VAMP 0.6.4, default (dynamic domain) | VAMP 0.6.4, dynamic domain off | motionAmigo, AVX2 | motionAmigo, AVX2, greedy shortcutting only | motionAmigo, portable SIMD | motionAmigo, scalar |
|---|---:|---:|---:|---:|---:|---:|
| bookshelf_small | 302 µs | 369 µs | 444 µs | 123 µs | 1.41 ms | 2.68 ms |
| bookshelf_tall | 239 µs | 224 µs | 544 µs | 123 µs | 1.96 ms | 2.97 ms |
| bookshelf_thin | 260 µs | 264 µs | 698 µs | 177 µs | 2.53 ms | 3.58 ms |
| box | 352 µs | 332 µs | 578 µs | 202 µs | 1.71 ms | 2.70 ms |
| cage | 942 µs | 1.44 ms | 1.43 ms | 964 µs | 4.59 ms | 6.85 ms |
| table_pick | 278 µs | 222 µs | 367 µs | 84 µs | 1.40 ms | 2.19 ms |
| table_under_pick | 403 µs | 357 µs | 607 µs | 173 µs | 2.20 ms | 3.60 ms |
