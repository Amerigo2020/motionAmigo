//! The vectorized checker must agree with the scalar reference on every query, bit for bit.

use motionamigo::rng::Rng;
use motionamigo::simd::LANES;
use motionamigo::{
    plan_with, Backend, CollisionChecker, Cuboid, Environment, PlanSettings, PointCloud,
    RobotModel, ScalarChecker, SimdChecker,
};
use proptest::prelude::*;

fn env() -> Environment {
    let mut env = Environment::new();
    env.add_cuboid(Cuboid::with_yaw([0.6, 0.0, 0.2], [0.5, 1.2, 0.4], 0.2))
        .add_cuboid(Cuboid::aabb([0.55, 0.0, 0.75], [0.3, 0.04, 0.5]))
        .add_sphere([0.0, 0.55, 0.6], 0.1)
        .add_capsule([-0.5, -0.5, 0.0], [-0.5, 0.5, 1.0], 0.05);
    let pts: Vec<[f32; 3]> = (0..500)
        .map(|i| {
            let a = i as f32 * 0.37;
            [
                -0.2 + 0.15 * a.cos(),
                -0.5 + 0.15 * a.sin(),
                0.2 + (i % 50) as f32 * 0.01,
            ]
        })
        .collect();
    env.add_pointcloud(PointCloud::new(&pts, 0.01, 0.05));
    env
}

/// Every bundled robot: the 7-DoF Panda and the 6-DoF UR5 (a partially filled joint block).
fn robots() -> [RobotModel; 2] {
    [RobotModel::panda(), RobotModel::ur5()]
}

fn checkers(robot: &RobotModel) -> (ScalarChecker, Vec<SimdChecker>) {
    let env = env();
    let simd = Backend::available()
        .into_iter()
        .map(|b| SimdChecker::with_backend(robot, &env, 32.0, b))
        .collect();
    (ScalarChecker::new(robot, &env, 32.0), simd)
}

fn random_config(rng: &mut Rng, c: &ScalarChecker) -> Vec<f32> {
    (0..c.lower().len())
        .map(|k| rng.uniform(c.lower()[k], c.upper()[k]))
        .collect()
}

#[test]
fn all_backends_are_tested() {
    let (_, simd) = checkers(&RobotModel::panda());
    let names: Vec<String> = simd.iter().map(|c| c.name()).collect();
    assert!(names.contains(&"simd-portable".to_string()));
    #[cfg(target_arch = "aarch64")]
    assert!(names.contains(&"simd-neon".to_string()));
    #[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
    assert!(names.contains(&"simd-wasm-simd128".to_string()));
    eprintln!("backends under test: {names:?}");
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

    #[test]
    fn blocks_agree_with_scalar(seed in any::<u64>()) {
        for robot in robots() {
        let (scalar, simd) = checkers(&robot);
        let mut rng = Rng::new(seed);
        let configs: Vec<Vec<f32>> = (0..LANES).map(|_| random_config(&mut rng, &scalar)).collect();
        let expected: Vec<bool> = configs.iter().map(|q| scalar.in_collision(q)).collect();
        let block: Vec<[f32; LANES]> =
            (0..robot.dof()).map(|k| core::array::from_fn(|lane| configs[lane][k])).collect();
        for c in &simd {
            // The block as a whole...
            prop_assert_eq!(c.any_in_collision(&block), expected.iter().any(|&e| e));
            // ...and every lane on its own (broadcast to all lanes).
            for (q, &e) in configs.iter().zip(&expected) {
                let single: Vec<[f32; LANES]> = q.iter().map(|&v| [v; LANES]).collect();
                prop_assert_eq!(c.any_in_collision(&single), e);
            }
        }
        }
    }

    #[test]
    fn edges_agree_with_scalar(seed in any::<u64>()) {
        for robot in robots() {
        let (scalar, simd) = checkers(&robot);
        let mut rng = Rng::new(seed);
        let a = random_config(&mut rng, &scalar);
        // Mix of short and long edges.
        let scale = [0.05f32, 0.3, 1.0][rng.below(3)];
        let b: Vec<f32> = a
            .iter()
            .enumerate()
            .map(|(k, &v)| (v + rng.uniform(-3.0, 3.0) * scale).clamp(scalar.lower()[k], scalar.upper()[k]))
            .collect();
        let expected = scalar.motion_valid(&a, &b);
        for c in &simd {
            prop_assert_eq!(c.motion_valid(&a, &b), expected, "{}", c.name());
        }
        }
    }
}

#[test]
fn plans_are_identical_across_checkers() {
    for robot in robots() {
        plans_are_identical(&robot);
    }
}

fn plans_are_identical(robot: &RobotModel) {
    let (scalar, simd) = checkers(robot);
    let mut rng = Rng::new(2024);
    let mut compared = 0;
    for seed in 0..12 {
        let valid = |rng: &mut Rng| loop {
            let q = random_config(rng, &scalar);
            if scalar.config_valid(&q) {
                return q.iter().map(|&v| v as f64).collect::<Vec<_>>();
            }
        };
        let (start, goal) = (valid(&mut rng), valid(&mut rng));
        let settings = PlanSettings {
            seed,
            ..PlanSettings::default()
        };
        let reference = plan_with(&scalar, &start, std::slice::from_ref(&goal), &settings);
        for c in &simd {
            let other = plan_with(c, &start, std::slice::from_ref(&goal), &settings);
            match (&reference, &other) {
                (Ok(a), Ok(b)) => {
                    assert_eq!(a.path, b.path, "{}", c.name());
                    assert_eq!(a.iterations, b.iterations);
                    compared += 1;
                }
                (Err(a), Err(b)) => assert_eq!(a, b),
                _ => panic!("checkers disagree on solvability ({})", c.name()),
            }
        }
    }
    assert!(compared >= 10);
}
