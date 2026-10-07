//! Plans a sequence of motions for the Panda in the tabletop scene shared with spatialAmigo.
//!
//! Run with `cargo run --release -p motionamigo --example plan_tabletop`.

use motionamigo::{
    plan_with, Environment, PlanSettings, RobotModel, ScalarChecker, Scene, PANDA_READY,
};

fn main() {
    let robot = RobotModel::panda();
    let scene_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/scenes/tabletop.json"
    );
    let scene = Scene::from_path(scene_path).expect("tabletop scene");
    let env = Environment::from_scene(&scene);
    println!(
        "scene: {} objects, robot: {} ({} DoF, {} spheres)",
        scene.objects.len(),
        robot.name,
        robot.dof(),
        robot.num_spheres()
    );

    // Hand-down configurations above the table (milestone M7 adds IK to compute these from
    // poses): near mug_1 and the bowl, near mug_2, and above the book.
    let waypoints: [(&str, [f64; 7]); 4] = [
        ("above mug_1", [0.06, 0.41, -1.16, -1.02, 0.55, 1.36, 0.52]),
        ("above mug_2", [-0.18, 0.22, 0.89, -1.21, -0.08, 1.54, 1.31]),
        (
            "above book_1",
            [-1.17, -0.79, 1.27, -1.26, 0.81, 1.06, -0.48],
        ),
        ("ready", PANDA_READY),
    ];
    let checker = ScalarChecker::new(&robot, &env, 32.0);
    let settings = PlanSettings::default();
    let mut current = PANDA_READY.to_vec();
    for (name, goal) in waypoints {
        match plan_with(&checker, &current, &[goal.to_vec()], &settings) {
            Ok(plan) => {
                println!(
                    "-> {name:<13} {:>2} waypoints, length {:.2} rad (before shortcutting {:.2}), \
                     {:>4} iterations, {:>9.1?} planning + {:>8.1?} shortcutting",
                    plan.path.len(),
                    plan.length,
                    plan.initial_length,
                    plan.iterations,
                    plan.planning_time,
                    plan.simplify_time
                );
                let tcp = robot.tcp_pose(plan.path.last().unwrap());
                println!("   TCP at {:.3?}", tcp.trans);
                current = goal.to_vec();
            }
            Err(e) => println!("-> {name}: {e}"),
        }
    }
}
