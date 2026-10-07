//! Inverse kinematics with damped least squares (Levenberg-Marquardt style).
//!
//! The solver iterates `dq = J^T (J J^T + lambda^2 I)^-1 e` on the full 6D pose error, clamps the
//! step size and the joint limits, and restarts from random configurations (seeded, so results are
//! reproducible) when it gets stuck. An optional acceptance test, for example a collision check,
//! filters solutions.
//!
//! ```
//! use motionamigo::ik::{solve, IkSettings};
//! use motionamigo::{RobotModel, PANDA_READY};
//! let robot = RobotModel::panda();
//! let target = robot.tcp_pose(&[0.3, -0.2, 0.1, -2.0, 0.1, 2.0, 0.5]);
//! let sol = solve(&robot, &target, Some(&PANDA_READY), &IkSettings::default()).unwrap();
//! let reached = robot.tcp_pose(&sol.q);
//! assert!((reached.trans[0] - target.trans[0]).abs() < 1e-3);
//! ```

use crate::math::Pose;
use crate::rng::Rng;
use crate::robot::{DhConvention, RobotModel};

/// Settings of the IK solver.
#[derive(Debug, Clone, PartialEq)]
pub struct IkSettings {
    /// Iterations per attempt.
    pub max_iterations: usize,
    /// Damping factor `lambda`.
    pub damping: f64,
    /// Largest joint step per iteration (L2 norm, radians).
    pub max_step: f64,
    /// Required position accuracy in meters.
    pub position_tolerance: f64,
    /// Required orientation accuracy in radians.
    pub orientation_tolerance: f64,
    /// Number of random restarts after the initial guess.
    pub restarts: usize,
    /// Seed of the random restarts.
    pub seed: u64,
}

impl Default for IkSettings {
    fn default() -> Self {
        IkSettings {
            max_iterations: 150,
            damping: 0.05,
            max_step: 0.3,
            position_tolerance: 1e-4,
            orientation_tolerance: 1e-3,
            restarts: 32,
            seed: 0,
        }
    }
}

/// A converged IK solution.
#[derive(Debug, Clone, PartialEq)]
pub struct IkSolution {
    /// Joint configuration within the limits.
    pub q: Vec<f64>,
    /// Remaining position error in meters.
    pub position_error: f64,
    /// Remaining orientation error in radians.
    pub orientation_error: f64,
    /// Attempt that converged (0 is the initial guess).
    pub attempt: usize,
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Rotation vector (axis times angle) of `r`.
fn log_so3(r: &[[f64; 3]; 3]) -> [f64; 3] {
    let tr = r[0][0] + r[1][1] + r[2][2];
    let cos = ((tr - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos.acos();
    let v = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    if angle < 1e-9 {
        return [0.5 * v[0], 0.5 * v[1], 0.5 * v[2]];
    }
    if angle > core::f64::consts::PI - 1e-6 {
        // Near pi: axis from the diagonal of (R + I) / 2.
        let k = (0..3)
            .max_by(|&a, &b| r[a][a].partial_cmp(&r[b][b]).unwrap())
            .unwrap();
        let mut axis = [0.0; 3];
        axis[k] = ((r[k][k] + 1.0) / 2.0).max(0.0).sqrt();
        for j in 0..3 {
            if j != k {
                axis[j] = (r[j][k] + r[k][j]) / (4.0 * axis[k]);
            }
        }
        return axis.map(|a| a * angle);
    }
    let s = angle / (2.0 * angle.sin());
    v.map(|x| x * s)
}

/// 6D error `[position; rotation vector]` from `current` to `target`, in the world frame.
pub fn pose_error(current: &Pose, target: &Pose) -> [f64; 6] {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = (0..3).map(|k| target.rot[i][k] * current.rot[j][k]).sum();
        }
    }
    let w = log_so3(&r);
    [
        target.trans[0] - current.trans[0],
        target.trans[1] - current.trans[1],
        target.trans[2] - current.trans[2],
        w[0],
        w[1],
        w[2],
    ]
}

/// Geometric Jacobian (6 x dof, linear rows first) of the TCP and the TCP pose.
pub fn jacobian(robot: &RobotModel, q: &[f64]) -> (Vec<[f64; 6]>, Pose) {
    let frames = robot.frames(q);
    let tcp = *frames.last().unwrap() * robot.tcp;
    let cols = (0..robot.dof())
        .map(|i| {
            // The axis of joint i is the z axis of the frame in which its rotation is applied.
            let f = match robot.convention {
                DhConvention::ModifiedDh => frames[i + 1],
                DhConvention::Dh => frames[i],
            };
            let z = f.axis(2);
            let d = [
                tcp.trans[0] - f.trans[0],
                tcp.trans[1] - f.trans[1],
                tcp.trans[2] - f.trans[2],
            ];
            let v = cross(z, d);
            [v[0], v[1], v[2], z[0], z[1], z[2]]
        })
        .collect();
    (cols, tcp)
}

/// Solves `a x = b` for a symmetric positive definite 6x6 matrix (Cholesky).
fn solve6(mut a: [[f64; 6]; 6], b: [f64; 6]) -> [f64; 6] {
    for j in 0..6 {
        let mut d = a[j][j];
        for k in 0..j {
            d -= a[j][k] * a[j][k];
        }
        let d = d.max(1e-18).sqrt();
        a[j][j] = d;
        for i in j + 1..6 {
            let mut s = a[i][j];
            for k in 0..j {
                s -= a[i][k] * a[j][k];
            }
            a[i][j] = s / d;
        }
    }
    let mut y = [0.0; 6];
    for i in 0..6 {
        let mut s = b[i];
        for k in 0..i {
            s -= a[i][k] * y[k];
        }
        y[i] = s / a[i][i];
    }
    let mut x = [0.0; 6];
    for i in (0..6).rev() {
        let mut s = y[i];
        for k in i + 1..6 {
            s -= a[k][i] * x[k];
        }
        x[i] = s / a[i][i];
    }
    x
}

fn norm3(v: &[f64]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// One damped least squares descent from `q`. Returns the final configuration and errors.
fn descend(
    robot: &RobotModel,
    target: &Pose,
    mut q: Vec<f64>,
    s: &IkSettings,
) -> (Vec<f64>, f64, f64) {
    let (lo, hi) = (robot.lower_limits(), robot.upper_limits());
    let lambda2 = s.damping * s.damping;
    let mut err = (f64::INFINITY, f64::INFINITY);
    for _ in 0..s.max_iterations {
        let (jac, tcp) = jacobian(robot, &q);
        let e = pose_error(&tcp, target);
        err = (norm3(&e[..3]), norm3(&e[3..]));
        if err.0 < s.position_tolerance && err.1 < s.orientation_tolerance {
            break;
        }
        // (J J^T + lambda^2 I) y = e, dq = J^T y
        let mut a = [[0.0; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                a[r][c] = jac.iter().map(|col| col[r] * col[c]).sum();
            }
            a[r][r] += lambda2;
        }
        let y = solve6(a, e);
        let mut dq: Vec<f64> = jac
            .iter()
            .map(|col| (0..6).map(|r| col[r] * y[r]).sum())
            .collect();
        let n = dq.iter().map(|v| v * v).sum::<f64>().sqrt();
        if n > s.max_step {
            dq.iter_mut().for_each(|v| *v *= s.max_step / n);
        }
        for k in 0..q.len() {
            q[k] = (q[k] + dq[k]).clamp(lo[k], hi[k]);
        }
    }
    let tcp = robot.tcp_pose(&q);
    let e = pose_error(&tcp, target);
    if norm3(&e[..3]) < err.0 || !err.0.is_finite() {
        err = (norm3(&e[..3]), norm3(&e[3..]));
    }
    (q, err.0, err.1)
}

/// Solves IK, starting from `initial` (if given) and then from random restarts. Returns the first
/// converged solution for which `accept` returns true.
pub fn solve_with<F: FnMut(&[f64]) -> bool>(
    robot: &RobotModel,
    target: &Pose,
    initial: Option<&[f64]>,
    settings: &IkSettings,
    mut accept: F,
) -> Option<IkSolution> {
    let mut rng = Rng::new(settings.seed);
    let (lo, hi) = (robot.lower_limits(), robot.upper_limits());
    for attempt in 0..=settings.restarts {
        let q0: Vec<f64> = match (attempt, initial) {
            (0, Some(q)) => q
                .iter()
                .zip(lo.iter().zip(&hi))
                .map(|(&v, (&l, &h))| v.clamp(l, h))
                .collect(),
            _ => (0..robot.dof())
                .map(|k| rng.uniform(lo[k] as f32, hi[k] as f32) as f64)
                .collect(),
        };
        let (q, pe, oe) = descend(robot, target, q0, settings);
        if pe < settings.position_tolerance
            && oe < settings.orientation_tolerance
            && robot.within_limits(&q)
            && accept(&q)
        {
            return Some(IkSolution {
                q,
                position_error: pe,
                orientation_error: oe,
                attempt,
            });
        }
    }
    None
}

/// Solves IK without additional acceptance test (see [`solve_with`]).
pub fn solve(
    robot: &RobotModel,
    target: &Pose,
    initial: Option<&[f64]>,
    settings: &IkSettings,
) -> Option<IkSolution> {
    solve_with(robot, target, initial, settings, |_| true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::PANDA_READY;

    #[test]
    fn jacobian_matches_finite_differences() {
        let robot = RobotModel::panda();
        let q = [0.3, -0.4, 0.2, -2.1, 0.3, 1.9, 0.4];
        let (jac, tcp) = jacobian(&robot, &q);
        let h = 1e-6;
        for k in 0..7 {
            let mut qp = q;
            qp[k] += h;
            let e = pose_error(&tcp, &robot.tcp_pose(&qp));
            for r in 0..6 {
                assert!((e[r] / h - jac[k][r]).abs() < 1e-4, "joint {k} row {r}");
            }
        }
    }

    #[test]
    fn log_map_handles_small_and_large_angles() {
        for angle in [1e-12, 0.3, 2.0, core::f64::consts::PI - 1e-9] {
            let r = Pose::rot_z(angle) * Pose::rot_x(0.0);
            let w = log_so3(&r.rot);
            assert!((w[2] - angle).abs() < 1e-6, "{angle} -> {w:?}");
        }
    }

    #[test]
    fn solves_reachable_poses() {
        let robot = RobotModel::panda();
        let mut rng = Rng::new(42);
        let (lo, hi) = (robot.lower_limits(), robot.upper_limits());
        let mut solved = 0;
        let n = 50;
        for i in 0..n {
            let q: Vec<f64> = (0..7)
                .map(|k| rng.uniform(lo[k] as f32, hi[k] as f32) as f64)
                .collect();
            let target = robot.tcp_pose(&q);
            let settings = IkSettings {
                seed: i,
                ..IkSettings::default()
            };
            if let Some(sol) = solve(&robot, &target, Some(&PANDA_READY), &settings) {
                let e = pose_error(&robot.tcp_pose(&sol.q), &target);
                assert!(norm3(&e[..3]) < 1e-4 && norm3(&e[3..]) < 1e-3);
                assert!(robot.within_limits(&sol.q));
                solved += 1;
            }
        }
        assert!(solved >= 48, "solved {solved} of {n}");
    }

    #[test]
    fn unreachable_pose_returns_none() {
        let robot = RobotModel::panda();
        let target = Pose::from_translation([2.0, 0.0, 0.5]);
        let settings = IkSettings {
            restarts: 3,
            ..IkSettings::default()
        };
        assert!(solve(&robot, &target, None, &settings).is_none());
    }
}
