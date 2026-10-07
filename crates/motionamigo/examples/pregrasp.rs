//! Moves the Panda to a pre-grasp pose above an object of the shared tabletop scene, then checks
//! the straight approach down to the grasp.
//!
//! In the full pipeline, spatialAmigo resolves an instruction such as "the mug right of the
//! laptop" to an object id; here the id is given on the command line:
//!
//! ```text
//! cargo run --release -p motionamigo --example pregrasp -- mug_1
//! ```

use motionamigo::grasp::{plan_to_pregrasp, PregraspSettings};
use motionamigo::ik::{solve_with, IkSettings};
use motionamigo::math::Pose;
use motionamigo::{CollisionChecker, Environment, RobotModel, Scene, SimdChecker, PANDA_READY};

fn main() {
    let object_id = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "mug_1".to_string());
    let scene_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/scenes/tabletop.json"
    );
    let scene = Scene::from_path(scene_path).expect("tabletop scene");
    let robot = RobotModel::panda();
    let object = scene.object(&object_id).unwrap_or_else(|| {
        let ids: Vec<_> = scene.objects.iter().map(|o| o.id.as_str()).collect();
        panic!("unknown object {object_id:?}, choose one of {ids:?}")
    });
    println!(
        "target: {} ({}), center {:?}, size {:?}",
        object.id, object.label, object.center, object.size
    );

    // 1. Pre-grasp: IK for a pose 10 cm above the object, then plan a collision-free motion.
    let settings = PregraspSettings::default();
    let result = match plan_to_pregrasp(&robot, &scene, &object_id, &PANDA_READY, &settings) {
        Ok(r) => r,
        Err(e) => {
            println!("cannot reach a pre-grasp pose: {e}");
            return;
        }
    };
    let tilt = (-result.pose.axis(2)[2])
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();
    println!(
        "pre-grasp: TCP at {:.3?}, tool axis tilted {tilt:.0} degrees from vertical, fingers across {:.0} mm",
        result.pose.trans,
        result.grasp_width * 1000.0
    );
    println!("pre-grasp configuration: {:.3?}", result.goal);
    let plan = &result.plan;
    println!(
        "motion: {} waypoints, {:.2} rad, planned in {:?} (+ {:?} shortcutting) with {}",
        plan.path.len(),
        plan.length,
        plan.planning_time,
        plan.simplify_time,
        plan.checker
    );

    // 2. Approach: move straight down to the grasp pose. The target object itself is removed from
    //    the collision environment, because the fingers are supposed to enclose it.
    let grasp_height = (object.size[2] * 0.5).min(0.03) + settings.clearance;
    let axis = result.pose.axis(2);
    let grasp = Pose {
        rot: result.pose.rot,
        trans: [0, 1, 2].map(|k| result.pose.trans[k] + axis[k] * grasp_height),
    };
    let env = Environment::from_scene_without(&scene, &[object_id.as_str()]);
    let checker = SimdChecker::new(&robot, &env, 32.0);
    let to32 = |q: &[f64]| q.iter().map(|&v| v as f32).collect::<Vec<_>>();
    let ik = IkSettings {
        restarts: 0,
        ..IkSettings::default()
    };
    match solve_with(&robot, &grasp, Some(&result.goal), &ik, |q| {
        checker.config_valid(&to32(q))
    }) {
        Some(sol) => {
            let straight = checker.motion_valid(&to32(&result.goal), &to32(&sol.q));
            println!(
                "approach: grasp configuration found {:.0} mm lower, straight joint-space approach is {}",
                grasp_height * 1000.0,
                if straight { "collision-free" } else { "blocked" }
            );
        }
        None => println!("approach: no grasp configuration near the pre-grasp configuration"),
    }
}
