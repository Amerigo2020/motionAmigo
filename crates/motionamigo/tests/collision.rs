//! The f32 collision kernel against an independent f64 brute-force implementation.

use motionamigo::{Cuboid, Environment, PointCloud, RobotModel, ScalarChecker, PANDA_READY};
use proptest::prelude::*;

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn f64s(v: [f32; 3]) -> [f64; 3] {
    v.map(|x| x as f64)
}

/// Signed clearance of a sphere to the environment (negative means penetration).
fn clearance(env: &Environment, c: [f64; 3], r: f64) -> f64 {
    let mut best = f64::INFINITY;
    for s in &env.spheres {
        let d = sub(c, f64s(s.center));
        best = best.min(dot(d, d).sqrt() - r - s.radius as f64);
    }
    for cap in &env.capsules {
        let (a, b) = (f64s(cap.a), f64s(cap.b));
        let v = sub(b, a);
        let t = (dot(sub(c, a), v) / dot(v, v)).clamp(0.0, 1.0);
        let p = [a[0] + v[0] * t, a[1] + v[1] * t, a[2] + v[2] * t];
        let d = sub(c, p);
        best = best.min(dot(d, d).sqrt() - r - cap.radius as f64);
    }
    for b in &env.cuboids {
        let d = sub(c, f64s(b.center));
        let mut e2 = 0.0;
        for k in 0..3 {
            let e = (dot(d, f64s(b.axes[k])).abs() - b.half_extents[k] as f64).max(0.0);
            e2 += e * e;
        }
        best = best.min(e2.sqrt() - r);
    }
    for pc in &env.pointclouds {
        for i in 0..pc.len() {
            let d = sub(c, f64s(pc.point(i)));
            best = best.min(dot(d, d).sqrt() - r - pc.point_radius() as f64);
        }
    }
    best
}

/// Minimum clearance over all robot spheres and self-collision pairs.
fn robot_clearance(robot: &RobotModel, env: &Environment, q: &[f64]) -> f64 {
    let frames = robot.frames(q);
    let world: Vec<Vec<[f64; 4]>> = robot
        .links
        .iter()
        .map(|l| {
            l.spheres
                .iter()
                .map(|s| {
                    let p = frames[l.frame].transform_point([s[0], s[1], s[2]]);
                    [p[0], p[1], p[2], s[3]]
                })
                .collect()
        })
        .collect();
    let mut best = f64::INFINITY;
    for link in &world {
        for s in link {
            best = best.min(clearance(env, [s[0], s[1], s[2]], s[3]));
        }
    }
    for &(a, b) in &robot.self_collision {
        for sa in &world[a] {
            for sb in &world[b] {
                let d = sub([sa[0], sa[1], sa[2]], [sb[0], sb[1], sb[2]]);
                best = best.min(dot(d, d).sqrt() - sa[3] - sb[3]);
            }
        }
    }
    best
}

fn test_env() -> Environment {
    let mut env = Environment::new();
    env.add_sphere([0.5, 0.2, 0.5], 0.12)
        .add_capsule([0.3, -0.5, 0.2], [0.6, -0.2, 0.9], 0.05)
        .add_cuboid(Cuboid::with_yaw([0.55, 0.0, 0.15], [0.5, 0.9, 0.3], 0.3))
        .add_cuboid(Cuboid::from_rotation(
            [-0.4, 0.3, 0.6],
            [[0.0, -1.0, 0.0], [0.6, 0.0, -0.8], [0.8, 0.0, 0.6]],
            [0.1, 0.2, 0.05],
        ));
    let pts: Vec<[f32; 3]> = (0..400)
        .map(|i| {
            let a = i as f32 * 0.1;
            [
                -0.3 + 0.1 * a.cos(),
                -0.4 + 0.1 * a.sin(),
                0.3 + i as f32 * 0.001,
            ]
        })
        .collect();
    env.add_pointcloud(PointCloud::new(&pts, 0.005, 0.05));
    env
}

#[test]
fn ready_pose_is_free_and_self_collision_detected() {
    let robot = RobotModel::panda();
    let c = ScalarChecker::new(&robot, &Environment::new(), 32.0);
    let ready: Vec<f32> = PANDA_READY.iter().map(|&v| v as f32).collect();
    assert!(!c.in_collision(&ready));
    // Folding the elbow fully and swinging the wrist into the upper arm collides with itself.
    let folded = [0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0];
    let folded64: Vec<f64> = folded.iter().map(|&v| v as f64).collect();
    let expected = robot_clearance(&robot, &Environment::new(), &folded64) < 0.0;
    assert_eq!(c.in_collision(&folded), expected);
    assert!(
        expected,
        "the folded configuration should be in self-collision"
    );
}

#[test]
fn scene_objects_block_the_table() {
    let robot = RobotModel::panda();
    let scene =
        motionamigo::Scene::from_json(include_str!("../../../examples/scenes/tabletop.json"))
            .unwrap();
    let env = Environment::from_scene(&scene);
    assert_eq!(env.cuboids.len(), 7);
    let c = ScalarChecker::new(&robot, &env, 32.0);
    // Reaching straight forward and down puts the hand into the table.
    let down = [0.0, 1.2, 0.0, -0.8, 0.0, 2.0, 0.8];
    assert!(c.in_collision(&down));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(3000))]
    #[test]
    fn kernel_matches_f64_brute_force(q in proptest::collection::vec(-3.0f32..3.7, 7)) {
        let robot = RobotModel::panda();
        let env = test_env();
        let c = ScalarChecker::new(&robot, &env, 32.0);
        let q64: Vec<f64> = q.iter().map(|&v| v as f64).collect();
        let margin = robot_clearance(&robot, &env, &q64);
        // Away from the contact boundary, f32 and f64 must agree on the outcome.
        if margin.abs() > 1e-4 {
            prop_assert_eq!(c.in_collision(&q), margin < 0.0, "margin {}", margin);
        }
    }
}

#[test]
fn brute_force_test_sees_both_outcomes() {
    // Guards the property test above against a degenerate environment.
    let robot = RobotModel::panda();
    let env = test_env();
    let c = ScalarChecker::new(&robot, &env, 32.0);
    let mut rng = motionamigo::rng::Rng::new(5);
    let n = 2000;
    let mut hits = 0;
    for _ in 0..n {
        let q: Vec<f32> = (0..7).map(|_| rng.uniform(-3.0, 3.7)).collect();
        let q64: Vec<f64> = q.iter().map(|&v| v as f64).collect();
        let margin = robot_clearance(&robot, &env, &q64);
        let hit = c.in_collision(&q);
        if margin.abs() > 1e-4 {
            assert_eq!(hit, margin < 0.0);
        }
        hits += hit as usize;
    }
    let frac = hits as f64 / n as f64;
    assert!((0.2..0.95).contains(&frac), "collision fraction {frac}");
}
