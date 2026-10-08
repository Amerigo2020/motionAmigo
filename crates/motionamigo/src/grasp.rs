//! Pre-grasp poses above scene objects, and planning to them.
//!
//! This closes the loop with spatialAmigo: language such as "the mug right of the laptop" resolves
//! to an object id in the shared scene, and [`plan_to_pregrasp`] moves the hand to a collision-free
//! pose above that object, ready for a top-down grasp.
//!
//! ```
//! use motionamigo::grasp::{plan_to_pregrasp, PregraspSettings};
//! use motionamigo::{RobotModel, Scene, PANDA_READY};
//! let scene = Scene::from_json(include_str!("../../../examples/scenes/tabletop.json")).unwrap();
//! let robot = RobotModel::panda();
//! let result = plan_to_pregrasp(&robot, &scene, "mug_1", &PANDA_READY, &PregraspSettings::default()).unwrap();
//! let tcp = robot.tcp_pose(&result.goal);
//! assert!((tcp.trans[2] - (0.45 + 0.05 + 0.1)).abs() < 1e-3); // 10 cm above the mug
//! ```

use crate::cartesian::{linear_path, LinearSettings};
use crate::checker::{CollisionChecker, SimdChecker};
use crate::environment::Environment;
use crate::ik::{solve_with, IkSettings};
use crate::math::Pose;
use crate::plan::{plan_with, Plan, PlanError, PlanSettings};
use crate::robot::RobotModel;
use crate::scene::{Scene, SceneObject};

/// Maximum opening of the Panda gripper in meters.
pub const PANDA_MAX_OPENING: f64 = 0.08;

/// Settings of [`plan_to_pregrasp`].
#[derive(Debug, Clone, PartialEq)]
pub struct PregraspSettings {
    /// Height of the tool center point above the top face of the object, in meters.
    pub clearance: f64,
    /// Gripper opening; grasp axes wider than this are tried last.
    pub max_opening: f64,
    /// Tilt angles (radians) tried after the vertical approach, tilting the tool axis away from
    /// the robot base. This makes objects near the edge of the workspace reachable.
    pub tilts: Vec<f64>,
    /// IK settings.
    pub ik: IkSettings,
    /// Planner settings.
    pub plan: PlanSettings,
}

impl Default for PregraspSettings {
    fn default() -> Self {
        PregraspSettings {
            clearance: 0.10,
            max_opening: PANDA_MAX_OPENING,
            tilts: vec![0.35, 0.7, 1.0],
            ik: IkSettings::default(),
            plan: PlanSettings::default(),
        }
    }
}

/// Errors of [`plan_to_pregrasp`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum GraspError {
    /// The object id does not exist in the scene.
    #[error("no object {0:?} in the scene")]
    UnknownObject(String),
    /// No collision-free IK solution was found for any candidate pose.
    #[error("no collision-free pre-grasp configuration above {0:?}")]
    NoIkSolution(String),
    /// No pre-grasp configuration allows a collision-free linear approach and retreat.
    #[error("no collision-free linear approach and retreat for {0:?}")]
    NoApproach(String),
    /// Planning failed.
    #[error("planning failed: {0}")]
    Plan(#[from] PlanError),
}

/// Result of [`plan_to_pregrasp`].
#[derive(Debug, Clone, PartialEq)]
pub struct PregraspPlan {
    /// The chosen pre-grasp pose of the tool center point.
    pub pose: Pose,
    /// Width of the object along the chosen closing direction of the fingers.
    pub grasp_width: f64,
    /// The pre-grasp joint configuration.
    pub goal: Vec<f64>,
    /// The motion from the start to `goal`.
    pub plan: Plan,
}

/// Candidate top-down pre-grasp poses above `object`, best first.
///
/// The tool z axis points down, the TCP sits `clearance` above the top face, and the fingers
/// (which close along the tool y axis) close across one of the two horizontal box axes. Axes that
/// fit into the gripper come first, narrower first; each is offered in both hand orientations.
/// Returns `(pose, width across the fingers)` pairs.
pub fn top_down_pregrasp_poses(
    object: &SceneObject,
    clearance: f64,
    max_opening: f64,
) -> Vec<(Pose, f64)> {
    let [cx, cy, cz] = object.center;
    let top = cz + 0.5 * object.size[2];
    let mut axes = [
        (object.size[0], 0.0),
        (object.size[1], core::f64::consts::FRAC_PI_2),
    ];
    // Fitting axes first, then by width.
    axes.sort_by(|a, b| {
        (a.0 > max_opening)
            .cmp(&(b.0 > max_opening))
            .then(a.0.partial_cmp(&b.0).unwrap())
    });
    let mut out = Vec::new();
    for (width, offset) in axes {
        for flip in [0.0, core::f64::consts::PI] {
            // Fingers close along the tool y axis, which must be aligned with the object axis
            // `object.yaw + offset`. With z pointing down, y = (s, -c, 0) for x = (c, s, 0).
            let psi = object.yaw + offset + flip + core::f64::consts::FRAC_PI_2;
            let (s, c) = psi.sin_cos();
            let pose = Pose {
                rot: [[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]],
                trans: [cx, cy, top + clearance],
            };
            out.push((pose, width));
        }
    }
    out
}

/// Computes a collision-free pre-grasp configuration above `object_id` and plans a motion to it.
///
/// Every candidate pose of [`top_down_pregrasp_poses`] is tried in order; IK starts from `start`
/// (so the solution tends to be close to the current configuration) and falls back to random
/// restarts. Only collision-free solutions are accepted.
pub fn plan_to_pregrasp(
    robot: &RobotModel,
    scene: &Scene,
    object_id: &str,
    start: &[f64],
    settings: &PregraspSettings,
) -> Result<PregraspPlan, GraspError> {
    let object = scene
        .object(object_id)
        .ok_or_else(|| GraspError::UnknownObject(object_id.to_string()))?;
    let env = Environment::from_scene(scene);
    let checker = SimdChecker::new(robot, &env, settings.plan.resolution);
    let (pose, width, goal) = pregrasp_configuration(robot, &checker, object, start, settings)
        .ok_or_else(|| GraspError::NoIkSolution(object_id.to_string()))?;
    let plan = plan_with(&checker, start, std::slice::from_ref(&goal), &settings.plan)?;
    Ok(PregraspPlan {
        pose,
        grasp_width: width,
        goal,
        plan,
    })
}

/// Rotation by `angle` about the unit axis `a` (Rodrigues' formula).
fn axis_angle(a: [f64; 3], angle: f64) -> Pose {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    let [x, y, z] = a;
    Pose {
        rot: [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        trans: [0.0; 3],
    }
}

/// Tilts a downward pose so that its tool axis leans away from the robot base by `tilt`.
pub fn tilt_away_from_base(pose: &Pose, tilt: f64) -> Pose {
    let (px, py) = (pose.trans[0], pose.trans[1]);
    let n = (px * px + py * py).sqrt();
    if n < 1e-9 || tilt == 0.0 {
        return *pose;
    }
    let (rx, ry) = (px / n, py / n);
    let r = axis_angle([ry, -rx, 0.0], tilt)
        * Pose {
            trans: [0.0; 3],
            ..*pose
        };
    Pose {
        rot: r.rot,
        trans: pose.trans,
    }
}

/// Finds a collision-free configuration for the first feasible pre-grasp pose above `object`.
pub fn pregrasp_configuration<C: CollisionChecker + ?Sized>(
    robot: &RobotModel,
    checker: &C,
    object: &SceneObject,
    start: &[f64],
    settings: &PregraspSettings,
) -> Option<(Pose, f64, Vec<f64>)> {
    pregrasp_configurations(robot, checker, object, start, settings).next()
}

/// All collision-free pre-grasp configurations above `object`, one per candidate pose for
/// which IK succeeds, in the order of [`top_down_pregrasp_poses`] (vertical poses first, then
/// increasingly tilted ones). Lazy: IK runs only for the candidates that are consumed.
fn pregrasp_configurations<'a, C: CollisionChecker + ?Sized>(
    robot: &'a RobotModel,
    checker: &'a C,
    object: &SceneObject,
    start: &'a [f64],
    settings: &'a PregraspSettings,
) -> impl Iterator<Item = (Pose, f64, Vec<f64>)> + 'a {
    let candidates = top_down_pregrasp_poses(object, settings.clearance, settings.max_opening);
    std::iter::once(0.0)
        .chain(settings.tilts.iter().copied())
        .flat_map(move |t| {
            candidates
                .clone()
                .into_iter()
                .map(move |(p, w)| (tilt_away_from_base(&p, t), w))
        })
        .filter_map(move |(pose, width)| {
            let valid = |q: &[f64]| checker.config_valid(&to_f32(q));
            solve_with(robot, &pose, Some(start), &settings.ik, valid).map(|s| (pose, width, s.q))
        })
}

fn to_f32(q: &[f64]) -> Vec<f32> {
    q.iter().map(|&v| v as f32).collect()
}

/// Settings of [`plan_pick`].
#[derive(Debug, Clone, PartialEq)]
pub struct PickSettings {
    /// Pre-grasp pose search and planning settings.
    pub pregrasp: PregraspSettings,
    /// Depth of the TCP below the top face of the object at the grasp, in meters (at most half
    /// the object height).
    pub grasp_depth: f64,
    /// Settings of the linear approach and retreat.
    pub linear: LinearSettings,
    /// Optional place or home configuration to plan to after the retreat, with the object held.
    pub place: Option<Vec<f64>>,
}

impl Default for PickSettings {
    fn default() -> Self {
        PickSettings {
            pregrasp: PregraspSettings::default(),
            grasp_depth: 0.02,
            linear: LinearSettings::default(),
            place: None,
        }
    }
}

/// Result of [`plan_pick`]: the segments of a pick, to be executed in order.
#[derive(Debug, Clone, PartialEq)]
pub struct PickPlan {
    /// Pre-grasp pose of the TCP.
    pub pregrasp_pose: Pose,
    /// Grasp pose of the TCP.
    pub grasp_pose: Pose,
    /// Width of the object across the fingers.
    pub grasp_width: f64,
    /// Planned motion from the start to the pre-grasp configuration.
    pub to_pregrasp: Plan,
    /// Linear approach from the pre-grasp to the grasp configuration (dense waypoints).
    pub approach: Vec<Vec<f64>>,
    /// Linear retreat from the grasp back to the pre-grasp pose with the object attached.
    pub retreat: Vec<Vec<f64>>,
    /// Planned motion from the end of the retreat to the place configuration, if one was given.
    pub place: Option<Plan>,
    /// Spheres `[x, y, z, r]` approximating the object, in the TCP frame. They are attached to
    /// the robot (as link [`HELD_OBJECT`]) for the retreat and the place motion.
    pub attached_spheres: Vec<[f64; 4]>,
    /// Pose of the object in the TCP frame while it is held.
    pub object_in_tcp: Pose,
}

/// Link name of the object attached during a pick.
pub const HELD_OBJECT: &str = "held_object";

/// Approximates a box (full edge lengths `size`, centered at the origin) by at most 64 spheres
/// that stay inside the box, `margin` away from its faces.
///
/// The sphere radius is half the smallest edge length minus the margin, and the centers lie on a
/// grid along the other two axes. The spheres reach every face (up to the margin), but the edges
/// and corners of the box are rounded off, so the approximation is slightly smaller than the box.
/// It stays inside on purpose: an object resting on a table must not collide with the table
/// right at the grasp.
pub fn box_spheres(size: [f64; 3], margin: f64) -> Vec<[f64; 4]> {
    let kmin = (0..3).min_by(|&a, &b| size[a].total_cmp(&size[b])).unwrap();
    let r = (0.5 * size[kmin] - margin).max(0.005);
    let axis = |k: usize| -> Vec<f64> {
        let span = (size[k] - 2.0 * (r + margin)).max(0.0);
        if k == kmin || span == 0.0 {
            return vec![0.0];
        }
        let n = ((span / r).ceil() as usize + 1).min(8);
        (0..n)
            .map(|i| -0.5 * span + span * i as f64 / (n - 1) as f64)
            .collect()
    };
    let (xs, ys, zs) = (axis(0), axis(1), axis(2));
    let mut out = Vec::new();
    for &x in &xs {
        for &y in &ys {
            for &z in &zs {
                out.push([x, y, z, r]);
            }
        }
    }
    out
}

/// Plans a complete top-down pick of `object_id`:
///
/// 1. plan from `start` to a pre-grasp configuration above the object,
/// 2. move the TCP straight along the tool axis to the grasp (the target object is left out of
///    the collision checks, because the fingers enclose it),
/// 3. attach the object (approximated by [`box_spheres`]) to the TCP,
/// 4. move straight back to the pre-grasp pose with the object attached (the retreat is the
///    approach in reverse, checked again with the object),
/// 5. optionally plan to `settings.place` with the object attached.
///
/// Pre-grasp candidates are tried in order until one admits a collision-free approach and
/// retreat. Opening and closing the gripper are not modeled.
pub fn plan_pick(
    robot: &RobotModel,
    scene: &Scene,
    object_id: &str,
    start: &[f64],
    settings: &PickSettings,
) -> Result<PickPlan, GraspError> {
    let object = scene
        .object(object_id)
        .ok_or_else(|| GraspError::UnknownObject(object_id.to_string()))?;
    let res = settings.pregrasp.plan.resolution;
    let checker = SimdChecker::new(robot, &Environment::from_scene(scene), res);
    let free_env = Environment::from_scene_without(scene, &[object_id]);
    let free = SimdChecker::new(robot, &free_env, res);
    let object_pose = Pose {
        trans: object.center,
        ..Pose::rot_z(object.yaw)
    };
    let local_spheres = box_spheres(object.size, 0.003);
    let depth = settings.pregrasp.clearance + settings.grasp_depth.min(0.5 * object.size[2]);
    let mut any_ik = false;
    for (pose, width, q_pre) in
        pregrasp_configurations(robot, &checker, object, start, &settings.pregrasp)
    {
        any_ik = true;
        let offset = pose.axis(2).map(|v| v * depth);
        let Ok(approach) = linear_path(robot, &free, &q_pre, offset, &settings.linear) else {
            continue;
        };
        let q_grasp = approach.last().unwrap();
        let grasp_pose = robot.tcp_pose(q_grasp);
        let object_in_tcp = grasp_pose.inverse() * object_pose;
        let attached_spheres: Vec<[f64; 4]> = local_spheres
            .iter()
            .map(|s| {
                let p = object_in_tcp.transform_point([s[0], s[1], s[2]]);
                [p[0], p[1], p[2], s[3]]
            })
            .collect();
        let mut held = robot.clone();
        if held.attach_object(HELD_OBJECT, &attached_spheres).is_err() {
            continue;
        }
        let held_checker = SimdChecker::new(&held, &free_env, res);
        let retreat: Vec<Vec<f64>> = approach.iter().rev().cloned().collect();
        let retreat_ok = held_checker.config_valid(&to_f32(&retreat[0]))
            && retreat.windows(2).all(|w| {
                let (a, b) = (to_f32(&w[0]), to_f32(&w[1]));
                held_checker.config_valid(&b) && held_checker.motion_valid(&a, &b)
            });
        if !retreat_ok {
            continue;
        }
        let to_pregrasp = plan_with(&checker, start, &[q_pre], &settings.pregrasp.plan)?;
        let place = match &settings.place {
            Some(goal) => Some(plan_with(
                &held_checker,
                retreat.last().unwrap(),
                std::slice::from_ref(goal),
                &settings.pregrasp.plan,
            )?),
            None => None,
        };
        return Ok(PickPlan {
            pregrasp_pose: pose,
            grasp_pose,
            grasp_width: width,
            to_pregrasp,
            approach,
            retreat,
            place,
            attached_spheres,
            object_in_tcp,
        });
    }
    Err(if any_ik {
        GraspError::NoApproach(object_id.to_string())
    } else {
        GraspError::NoIkSolution(object_id.to_string())
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::PANDA_READY;

    fn tabletop() -> Scene {
        Scene::from_json(include_str!("../../../examples/scenes/tabletop.json")).unwrap()
    }

    #[test]
    fn candidate_poses_point_down_and_are_rotations() {
        let scene = tabletop();
        let book = scene.object("book_1").unwrap();
        let poses = top_down_pregrasp_poses(book, 0.1, PANDA_MAX_OPENING);
        assert_eq!(poses.len(), 4);
        // The book is 0.16 x 0.22: both axes are too wide, the narrower one comes first.
        assert!((poses[0].1 - 0.16).abs() < 1e-12);
        for (p, _) in &poses {
            assert_eq!(p.axis(2), [0.0, 0.0, -1.0]);
            let x = p.axis(0);
            let y = p.axis(1);
            let z = [
                x[1] * y[2] - x[2] * y[1],
                x[2] * y[0] - x[0] * y[2],
                x[0] * y[1] - x[1] * y[0],
            ];
            assert!((z[2] + 1.0).abs() < 1e-12, "right-handed");
            assert!((p.trans[2] - (0.415 + 0.015 + 0.1)).abs() < 1e-12);
        }
    }

    #[test]
    fn fingers_close_across_the_narrow_axis() {
        let mut scene = tabletop();
        scene.objects.push(SceneObject {
            id: "bar".into(),
            label: "bar".into(),
            center: [0.5, 0.0, 0.45],
            size: [0.04, 0.2, 0.1],
            yaw: 0.3,
            front: None,
        });
        let bar = scene.object("bar").unwrap();
        let (pose, width) = top_down_pregrasp_poses(bar, 0.1, PANDA_MAX_OPENING)[0];
        assert!((width - 0.04).abs() < 1e-12);
        // The finger axis (tool y) is parallel to the bar's local x axis.
        let y = pose.axis(1);
        let bar_x = [0.3f64.cos(), 0.3f64.sin(), 0.0];
        let dot: f64 = (0..3).map(|k| y[k] * bar_x[k]).sum();
        assert!((dot.abs() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn plans_to_every_small_tabletop_object() {
        let scene = tabletop();
        let robot = RobotModel::panda();
        for id in ["mug_1", "mug_2", "bowl_1", "book_1", "laptop_1"] {
            let r = plan_to_pregrasp(
                &robot,
                &scene,
                id,
                &PANDA_READY,
                &PregraspSettings::default(),
            )
            .unwrap_or_else(|e| panic!("{id}: {e}"));
            let tcp = robot.tcp_pose(&r.goal);
            for k in 0..3 {
                assert!((tcp.trans[k] - r.pose.trans[k]).abs() < 1e-3, "{id}");
            }
            assert_eq!(r.plan.path.last().unwrap().len(), 7);
        }
    }

    #[test]
    fn tilting_leans_the_tool_axis_outwards() {
        let down = top_down_pregrasp_poses(tabletop().object("mug_2").unwrap(), 0.1, 0.08)[0].0;
        let t = tilt_away_from_base(&down, 0.3);
        let z = t.axis(2);
        assert!((z[2] + 0.3f64.cos()).abs() < 1e-12);
        // Leaning away from the base: positive component along the radial direction.
        assert!(z[0] * down.trans[0] + z[1] * down.trans[1] > 0.0);
        assert_eq!(t.trans, down.trans);
    }

    #[test]
    fn box_spheres_stay_inside_and_reach_the_faces() {
        for size in [[0.08, 0.08, 0.1], [0.16, 0.22, 0.03], [0.6, 0.02, 0.3]] {
            let s = box_spheres(size, 0.003);
            assert!(!s.is_empty() && s.len() <= 64);
            for k in 0..3 {
                let lo = s.iter().map(|v| v[k] - v[3]).fold(f64::INFINITY, f64::min);
                let hi = s
                    .iter()
                    .map(|v| v[k] + v[3])
                    .fold(f64::NEG_INFINITY, f64::max);
                let face = 0.5 * size[k] - 0.003;
                assert!(
                    hi <= face + 1e-12 && lo >= -face - 1e-12,
                    "{size:?} axis {k}"
                );
                assert!(hi >= face - 1e-12, "{size:?} axis {k}");
            }
        }
    }

    #[test]
    fn picks_a_mug_and_lifts_it() {
        let scene = tabletop();
        let robot = RobotModel::panda();
        {
            let id = "mug_1";
            let settings = PickSettings {
                place: Some(PANDA_READY.to_vec()),
                ..PickSettings::default()
            };
            let p = plan_pick(&robot, &scene, id, &PANDA_READY, &settings)
                .unwrap_or_else(|e| panic!("{id}: {e}"));
            let pre: Vec<f64> = p.approach[0].iter().map(|&v| v as f32 as f64).collect();
            assert_eq!(p.to_pregrasp.path.last().unwrap(), &pre);
            assert_eq!(p.retreat.first(), p.approach.last());
            let mug = scene.object(id).unwrap();
            // The grasp TCP is 2 cm below the top face of the mug.
            assert!((p.grasp_pose.trans[2] - (mug.center[2] + 0.05 - 0.02)).abs() < 1e-3);
            // At the end of the retreat the held mug is lifted off the table.
            let mut held = robot.clone();
            held.attach_object(HELD_OBJECT, &p.attached_spheres)
                .unwrap();
            let end = held.spheres_world(p.retreat.last().unwrap());
            let n = p.attached_spheres.len();
            for s in &end[end.len() - n..] {
                assert!(s[2] - s[3] > 0.45, "{id}: object lifted");
            }
            assert!(p.place.is_some());
        }
        // mug_2 sits at the edge of the workspace: the pre-grasp pose is reachable, but the arm
        // cannot reach down to the grasp along a straight line.
        let r = plan_pick(
            &robot,
            &scene,
            "mug_2",
            &PANDA_READY,
            &PickSettings::default(),
        );
        assert_eq!(r.unwrap_err(), GraspError::NoApproach("mug_2".into()));
    }

    #[test]
    fn unknown_object_is_an_error() {
        let robot = RobotModel::panda();
        let err = plan_to_pregrasp(
            &robot,
            &tabletop(),
            "unicorn",
            &PANDA_READY,
            &PregraspSettings::default(),
        );
        assert_eq!(
            err.unwrap_err(),
            GraspError::UnknownObject("unicorn".into())
        );
    }
}
