//! Scalar reference against the vectorized checker.
//!
//! Run with `cargo bench -p motionamigo --bench collision`.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use motionamigo::rng::Rng;
use motionamigo::simd::LANES;
use motionamigo::{
    plan_with, Backend, CollisionChecker, Environment, PlanSettings, RobotModel, ScalarChecker,
    Scene, SimdChecker,
};
use std::hint::black_box;

fn tabletop() -> Environment {
    let scene = Scene::from_json(include_str!("../../../examples/scenes/tabletop.json")).unwrap();
    Environment::from_scene(&scene)
}

struct Fixture {
    scalar: ScalarChecker,
    simd: Vec<SimdChecker>,
    configs: Vec<Vec<f32>>,
    edges: Vec<(Vec<f32>, Vec<f32>)>,
}

fn fixture() -> Fixture {
    let robot = RobotModel::panda();
    let env = tabletop();
    let scalar = ScalarChecker::new(&robot, &env, 32.0);
    let simd = Backend::available()
        .into_iter()
        .map(|b| SimdChecker::with_backend(&robot, &env, 32.0, b))
        .collect();
    let mut rng = Rng::new(7);
    let mut sample = |valid_only: bool| loop {
        let q: Vec<f32> = (0..7)
            .map(|k| rng.uniform(scalar.lower()[k], scalar.upper()[k]))
            .collect();
        if !valid_only || scalar.config_valid(&q) {
            return q;
        }
    };
    // Collision-free configurations: every check does the full amount of work.
    let configs = (0..256).map(|_| sample(true)).collect();
    // Edges between valid configurations: a realistic mix of free and blocked motions.
    let edges = (0..256).map(|_| (sample(true), sample(true))).collect();
    Fixture {
        scalar,
        simd,
        configs,
        edges,
    }
}

fn bench_configs(c: &mut Criterion) {
    let f = fixture();
    let mut g = c.benchmark_group("configurations");
    g.throughput(Throughput::Elements(f.configs.len() as u64));
    g.bench_function("scalar", |b| {
        b.iter(|| {
            f.configs
                .iter()
                .filter(|q| f.scalar.in_collision(black_box(q)))
                .count()
        })
    });
    let blocks: Vec<Vec<[f32; LANES]>> = f
        .configs
        .chunks(LANES)
        .map(|chunk| {
            (0..7)
                .map(|k| core::array::from_fn(|l| chunk[l][k]))
                .collect()
        })
        .collect();
    for s in &f.simd {
        g.bench_function(BenchmarkId::new("simd-block", s.backend().name()), |b| {
            b.iter(|| {
                blocks
                    .iter()
                    .filter(|blk| s.any_in_collision(black_box(blk)))
                    .count()
            })
        });
    }
    g.finish();
}

fn bench_edges(c: &mut Criterion) {
    let f = fixture();
    let mut g = c.benchmark_group("edges");
    g.throughput(Throughput::Elements(f.edges.len() as u64));
    g.bench_function("scalar", |b| {
        b.iter(|| {
            f.edges
                .iter()
                .filter(|(a, e)| f.scalar.motion_valid(black_box(a), black_box(e)))
                .count()
        })
    });
    for s in &f.simd {
        g.bench_function(BenchmarkId::new("simd", s.backend().name()), |b| {
            b.iter(|| {
                f.edges
                    .iter()
                    .filter(|(a, e)| s.motion_valid(black_box(a), black_box(e)))
                    .count()
            })
        });
    }
    g.finish();
}

fn bench_planning(c: &mut Criterion) {
    let f = fixture();
    // Problems that need search: valid pairs whose straight line is blocked.
    let problems: Vec<(Vec<f64>, Vec<f64>)> = f
        .edges
        .iter()
        .filter(|(a, b)| !f.scalar.motion_valid(a, b))
        .take(16)
        .map(|(a, b)| {
            (
                a.iter().map(|&v| v as f64).collect(),
                b.iter().map(|&v| v as f64).collect(),
            )
        })
        .collect();
    let settings = PlanSettings::default();
    let mut g = c.benchmark_group("planning");
    g.sample_size(20);
    g.throughput(Throughput::Elements(problems.len() as u64));
    g.bench_function("scalar", |b| {
        b.iter(|| {
            problems
                .iter()
                .filter(|(s, e)| {
                    plan_with(&f.scalar, s, std::slice::from_ref(e), &settings).is_ok()
                })
                .count()
        })
    });
    for s in &f.simd {
        g.bench_function(BenchmarkId::new("simd", s.backend().name()), |b| {
            b.iter(|| {
                problems
                    .iter()
                    .filter(|(st, e)| plan_with(s, st, std::slice::from_ref(e), &settings).is_ok())
                    .count()
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench_configs, bench_edges, bench_planning);
criterion_main!(benches);
