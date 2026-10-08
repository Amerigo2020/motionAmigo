//! Straight-line (Cartesian) motions of the tool center point.
//!
//! [`linear_path`] moves the TCP along a straight line with fixed orientation, for example from
//! a pre-grasp pose down to the grasp and back up. The line is cut into small steps; each step is
//! solved with IK seeded with the previous solution (no random restarts, so the arm does not
//! jump to another IK branch), joint jumps are bounded, and every joint-space segment between
//! consecutive steps is collision checked.
//!
//! ```
//! use motionamigo::cartesian::{linear_path, LinearSettings};
//! use motionamigo::{Environment, RobotModel, SimdChecker, PANDA_READY};
//! let robot = RobotModel::panda();
//! let checker = SimdChecker::new(&robot, &Environment::new(), 32.0);
//! let path = linear_path(&robot, &checker, &PANDA_READY, [0.0, 0.0, -0.1], &LinearSettings::default()).unwrap();
//! let end = robot.tcp_pose(path.last().unwrap());
//! let start = robot.tcp_pose(&PANDA_READY);
//! assert!((end.trans[2] - (start.trans[2] - 0.1)).abs() < 1e-3);
//! ```

use crate::checker::CollisionChecker;
use crate::ik::{solve, IkSettings};
use crate::math::Pose;
use crate::robot::RobotModel;

/// Settings of [`linear_path`].
#[derive(Debug, Clone, PartialEq)]
pub struct LinearSettings {
    /// Largest Cartesian step between consecutive IK solutions, in meters.
    pub step: f64,
    /// Largest change of any single joint between consecutive steps, in radians.
    pub max_joint_step: f64,
    /// IK settings for each step. Restarts are ignored: every step is seeded with the previous
    /// solution only.
    pub ik: IkSettings,
}

impl Default for LinearSettings {
    fn default() -> Self {
        LinearSettings {
            step: 0.005,
            max_joint_step: 0.1,
            ik: IkSettings::default(),
        }
    }
}

/// Why a straight-line motion failed. `step` counts from 1 (the first point after the start).
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum LinearError {
    /// The start configuration violates the joint limits or is in collision.
    #[error("start configuration of the linear motion is invalid")]
    InvalidStart,
    /// IK did not converge at a point on the line.
    #[error("no IK solution at step {0} of the linear motion")]
    NoIk(usize),
    /// A joint would have moved more than the allowed joint step.
    #[error("joint jump at step {0} of the linear motion")]
    JointJump(usize),
    /// The motion collides.
    #[error("collision at step {0} of the linear motion")]
    Collision(usize),
}

fn to_f32(q: &[f64]) -> Vec<f32> {
    q.iter().map(|&v| v as f32).collect()
}

/// Moves the TCP from its pose at `start` by `offset` (world frame, meters) along a straight
/// line, keeping the orientation. Returns the joint configurations along the line, starting with
/// `start` and ending at the target; consecutive configurations are joined by collision-free
/// joint-space segments.
pub fn linear_path<C: CollisionChecker + ?Sized>(
    robot: &RobotModel,
    checker: &C,
    start: &[f64],
    offset: [f64; 3],
    settings: &LinearSettings,
) -> Result<Vec<Vec<f64>>, LinearError> {
    if !checker.config_valid(&to_f32(start)) {
        return Err(LinearError::InvalidStart);
    }
    let from = robot.tcp_pose(start);
    let len = offset.iter().map(|v| v * v).sum::<f64>().sqrt();
    let n = (len / settings.step).ceil().max(1.0) as usize;
    let ik = IkSettings {
        restarts: 0,
        ..settings.ik.clone()
    };
    let mut path = vec![start.to_vec()];
    for i in 1..=n {
        let t = i as f64 / n as f64;
        let target = Pose {
            rot: from.rot,
            trans: [0, 1, 2].map(|k| from.trans[k] + offset[k] * t),
        };
        let prev = path.last().unwrap();
        let q = solve(robot, &target, Some(prev), &ik)
            .ok_or(LinearError::NoIk(i))?
            .q;
        if q.iter()
            .zip(prev)
            .any(|(a, b)| (a - b).abs() > settings.max_joint_step)
        {
            return Err(LinearError::JointJump(i));
        }
        let (a, b) = (to_f32(prev), to_f32(&q));
        if !checker.config_valid(&b) || !checker.motion_valid(&a, &b) {
            return Err(LinearError::Collision(i));
        }
        path.push(q);
    }
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::checker::SimdChecker;
    use crate::environment::{Cuboid, Environment};
    use crate::robot::PANDA_READY;

    #[test]
    fn follows_a_straight_line() {
        let robot = RobotModel::panda();
        let checker = SimdChecker::new(&robot, &Environment::new(), 32.0);
        let offset = [0.05, -0.08, -0.15];
        let path = linear_path(
            &robot,
            &checker,
            &PANDA_READY,
            offset,
            &LinearSettings::default(),
        )
        .unwrap();
        let p0 = robot.tcp_pose(&PANDA_READY);
        let len = offset.iter().map(|v| v * v).sum::<f64>().sqrt();
        for q in &path {
            let p = robot.tcp_pose(q);
            let d = [0, 1, 2].map(|k| p.trans[k] - p0.trans[k]);
            // Distance from the line through p0 along `offset`.
            let along = (0..3).map(|k| d[k] * offset[k]).sum::<f64>() / len;
            let off2 = d.iter().map(|v| v * v).sum::<f64>() - along * along;
            assert!(off2.max(0.0).sqrt() < 1e-3);
            for r in 0..3 {
                for c in 0..3 {
                    assert!((p.rot[r][c] - p0.rot[r][c]).abs() < 2e-3);
                }
            }
        }
    }

    #[test]
    fn reports_collisions_on_the_line() {
        let robot = RobotModel::panda();
        let tcp = robot.tcp_pose(&PANDA_READY).trans;
        let mut env = Environment::new();
        env.add_cuboid(Cuboid::aabb(
            [tcp[0] as f32, tcp[1] as f32, (tcp[2] - 0.15) as f32],
            [0.3, 0.3, 0.02],
        ));
        let checker = SimdChecker::new(&robot, &env, 32.0);
        let r = linear_path(
            &robot,
            &checker,
            &PANDA_READY,
            [0.0, 0.0, -0.3],
            &LinearSettings::default(),
        );
        assert!(matches!(r, Err(LinearError::Collision(_))), "{r:?}");
    }
}
