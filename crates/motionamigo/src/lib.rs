//! # motionAmigo
//!
//! Fast sampling-based motion planning for robot arms. The robot is approximated by spheres,
//! and forward kinematics plus collision checking are vectorized over several configurations
//! along an edge at once, following the ideas of VAMP (Thomason, Kingston, Kavraki, ICRA 2024).
//!
//! ```
//! use motionamigo::{plan, Environment, PlanSettings, RobotModel, Scene, PANDA_READY};
//!
//! let robot = RobotModel::panda();
//! let scene = Scene::from_json(include_str!("../../../examples/scenes/tabletop.json")).unwrap();
//! let env = Environment::from_scene(&scene);
//! let goal = [0.06, 0.41, -1.16, -1.02, 0.55, 1.36, 0.52]; // hand above mug_1
//! let result = plan(&robot, &env, &PANDA_READY, &goal, &PlanSettings::default()).unwrap();
//! println!("{} waypoints, length {:.2} rad", result.path.len(), result.length);
//! ```
#![forbid(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]
// Lane loops index several arrays in lockstep; iterators would obscure that.
#![allow(clippy::needless_range_loop)]

pub mod checker;
mod collision;
pub mod environment;
pub mod grasp;
pub mod ik;
pub mod kinematics;
pub mod math;
pub mod plan;
pub mod planner;
pub mod pointcloud;
pub mod rng;
pub mod robot;
pub mod scene;
pub mod simd;
pub mod time;

pub use checker::{CollisionChecker, ScalarChecker, SimdChecker};
pub use environment::{Capsule, Cuboid, Environment, Sphere};
pub use plan::{plan, plan_with, CheckerKind, Plan, PlanError, PlanSettings};
pub use pointcloud::PointCloud;
pub use robot::{RobotModel, PANDA_READY};
pub use scene::Scene;
pub use simd::Backend;
