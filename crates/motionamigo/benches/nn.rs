//! Nearest-neighbour query: linear scan against the kd-tree, on RRT-like trees.
//!
//! Run with `cargo bench -p motionamigo --bench nn`.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use motionamigo::planner::nn::{nearest_linear, KdTree};
use motionamigo::rng::Rng;
use std::hint::black_box;

const DOF: usize = 7;
const LIMIT: f32 = 2.9;

fn sample(rng: &mut Rng) -> Vec<f32> {
    (0..DOF).map(|_| rng.uniform(-LIMIT, LIMIT)).collect()
}

/// Grows a tree like RRT without obstacles: steer the nearest node 1 rad towards a sample.
fn rrt_tree(n: usize, rng: &mut Rng) -> (Vec<f32>, KdTree) {
    let mut points = sample(rng);
    let mut kd = KdTree::default();
    kd.insert(&points, DOF);
    while points.len() < n * DOF {
        let s = sample(rng);
        let near = kd.nearest(&points, DOF, &s);
        let from = points[near * DOF..(near + 1) * DOF].to_vec();
        let d = motionamigo::planner::distance(&from, &s);
        let t = (1.0 / d).min(1.0);
        points.extend(from.iter().zip(&s).map(|(a, b)| a + (b - a) * t));
        kd.insert(&points, DOF);
    }
    (points, kd)
}

fn bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("nearest");
    for n in [16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 16384] {
        let mut rng = Rng::new(1);
        let (points, mut kd) = rrt_tree(n, &mut rng);
        let queries: Vec<Vec<f32>> = (0..256).map(|_| sample(&mut rng)).collect();
        group.bench_with_input(BenchmarkId::new("linear", n), &n, |b, _| {
            b.iter(|| {
                for q in &queries {
                    black_box(nearest_linear(&points, DOF, q));
                }
            })
        });
        group.bench_with_input(BenchmarkId::new("kd-tree", n), &n, |b, _| {
            b.iter(|| {
                for q in &queries {
                    black_box(kd.nearest(&points, DOF, q));
                }
            })
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
