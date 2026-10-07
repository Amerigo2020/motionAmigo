//! Planner properties: valid paths, determinism, simplification.

use motionamigo::checker::edge_configurations;
use motionamigo::rng::Rng;
use motionamigo::{
    plan_with, CollisionChecker, Cuboid, Environment, PlanError, PlanSettings, RobotModel,
    ScalarChecker,
};
use proptest::prelude::*;

/// A cluttered environment: table, a wall above it and a few spheres.
fn cluttered() -> Environment {
    let mut env = Environment::new();
    env.add_cuboid(Cuboid::aabb([0.6, 0.0, 0.2], [0.5, 1.2, 0.4]))
        .add_cuboid(Cuboid::aabb([0.55, 0.0, 0.75], [0.3, 0.04, 0.5]))
        .add_sphere([0.0, 0.55, 0.6], 0.1)
        .add_sphere([0.0, -0.55, 0.6], 0.1)
        .add_capsule([-0.5, -0.5, 0.0], [-0.5, 0.5, 1.0], 0.05);
    env
}

fn random_valid(c: &ScalarChecker, rng: &mut Rng) -> Vec<f32> {
    loop {
        let q: Vec<f32> = (0..c.dof())
            .map(|k| rng.uniform(c.lower()[k], c.upper()[k]))
            .collect();
        if c.config_valid(&q) {
            return q;
        }
    }
}

fn to64(q: &[f32]) -> Vec<f64> {
    q.iter().map(|&v| v as f64).collect()
}

fn to32(q: &[f64]) -> Vec<f32> {
    q.iter().map(|&v| v as f32).collect()
}

fn assert_valid_path(c: &ScalarChecker, path: &[Vec<f64>]) {
    for q in path {
        assert!(c.config_valid(&to32(q)), "invalid waypoint {q:?}");
    }
    for w in path.windows(2) {
        assert!(c.motion_valid(&to32(&w[0]), &to32(&w[1])), "invalid edge");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(40))]
    #[test]
    fn planned_paths_are_collision_free(seed in 0u64..1_000_000) {
        let robot = RobotModel::panda();
        let env = cluttered();
        let c = ScalarChecker::new(&robot, &env, 32.0);
        let mut rng = Rng::new(seed);
        let (start, goal) = (random_valid(&c, &mut rng), random_valid(&c, &mut rng));
        let settings = PlanSettings { seed, ..PlanSettings::default() };
        match plan_with(&c, &to64(&start), &[to64(&goal)], &settings) {
            Ok(plan) => {
                prop_assert_eq!(&to32(&plan.path[0]), &start);
                prop_assert_eq!(&to32(plan.path.last().unwrap()), &goal);
                assert_valid_path(&c, &plan.path);
                prop_assert!(plan.length <= plan.initial_length + 1e-4);
            }
            // Random start/goal pairs can lie in disconnected regions; that is not an error.
            Err(PlanError::NoSolution { .. }) => {}
            Err(e) => prop_assert!(false, "unexpected error {e}"),
        }
    }

    #[test]
    fn edge_validity_does_not_depend_on_direction(seed in 0u64..1_000_000) {
        let robot = RobotModel::panda();
        let c = ScalarChecker::new(&robot, &cluttered(), 32.0);
        let mut rng = Rng::new(seed);
        let (a, b) = (random_valid(&c, &mut rng), random_valid(&c, &mut rng));
        prop_assert_eq!(c.motion_valid(&a, &b), c.motion_valid(&b, &a));
        // The interior points are bit-identical in both directions.
        let mut fwd = edge_configurations(&a, &b, 32.0);
        let mut bwd = edge_configurations(&b, &a, 32.0);
        fwd.retain(|q| q != &b);
        bwd.retain(|q| q != &a);
        let key = |q: &Vec<f32>| q.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
        fwd.sort_by_key(key);
        bwd.sort_by_key(key);
        prop_assert_eq!(fwd, bwd);
    }
}

#[test]
fn same_seed_same_path() {
    let robot = RobotModel::panda();
    let c = ScalarChecker::new(&robot, &cluttered(), 32.0);
    let mut rng = Rng::new(11);
    let (start, goal) = (
        to64(&random_valid(&c, &mut rng)),
        to64(&random_valid(&c, &mut rng)),
    );
    let settings = PlanSettings {
        seed: 3,
        ..PlanSettings::default()
    };
    let a = plan_with(&c, &start, std::slice::from_ref(&goal), &settings);
    let b = plan_with(&c, &start, std::slice::from_ref(&goal), &settings);
    match (a, b) {
        (Ok(a), Ok(b)) => {
            assert_eq!(a.path, b.path);
            assert_eq!(a.iterations, b.iterations);
        }
        (a, b) => assert_eq!(a.is_err(), b.is_err()),
    }
}

#[test]
fn rejects_invalid_queries() {
    let robot = RobotModel::panda();
    let c = ScalarChecker::new(&robot, &cluttered(), 32.0);
    let ok = motionamigo::PANDA_READY.to_vec();
    let s = PlanSettings::default();
    assert!(matches!(
        plan_with(&c, &[0.0; 3], std::slice::from_ref(&ok), &s),
        Err(PlanError::Dimension { .. })
    ));
    assert_eq!(
        plan_with(&c, &[0.0; 7], std::slice::from_ref(&ok), &s).unwrap_err(),
        PlanError::InvalidStart
    );
    assert_eq!(plan_with(&c, &ok, &[], &s).unwrap_err(), PlanError::NoGoal);
}

#[test]
fn solves_a_problem_that_needs_search() {
    // Start and goal on opposite sides of the wall, hands below its top edge.
    let robot = RobotModel::panda();
    let c = ScalarChecker::new(&robot, &cluttered(), 32.0);
    let mut rng = Rng::new(99);
    let mut solved = 0;
    let mut searched = 0;
    for i in 0..10 {
        let (start, goal) = (random_valid(&c, &mut rng), random_valid(&c, &mut rng));
        let settings = PlanSettings {
            seed: i,
            ..PlanSettings::default()
        };
        if let Ok(plan) = plan_with(&c, &to64(&start), &[to64(&goal)], &settings) {
            solved += 1;
            searched += (plan.iterations > 0) as usize;
            assert_valid_path(&c, &plan.path);
        }
    }
    assert!(solved >= 8, "solved only {solved} of 10");
    assert!(
        searched >= 3,
        "too few problems required search ({searched})"
    );
}
