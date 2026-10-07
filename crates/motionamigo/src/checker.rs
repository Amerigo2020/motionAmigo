//! Configuration and motion validity checkers.
//!
//! Two implementations of [`CollisionChecker`] exist:
//!
//! * [`ScalarChecker`] is the reference: it checks one configuration at a time.
//! * `SimdChecker` (milestone M3) checks an edge eight configurations at a time ("raked" checking): the eight
//!   lanes start spread evenly along the edge and then step backwards together, so a collision
//!   anywhere on the edge is found after few iterations.
//!
//! Both discretize an edge into exactly the same set of configurations and evaluate them with
//! the same generic kernel, so they always agree, bit for bit.

use crate::collision::{fkcc, EnvBlock};
use crate::environment::Environment;
use crate::kinematics::CompiledRobot;
use crate::robot::{RobotModel, MAX_DOF};
use crate::simd::{Real, LANES};
use std::sync::Arc;

/// Default edge resolution: checked configurations per radian of joint-space distance.
pub const DEFAULT_RESOLUTION: f32 = 32.0;

/// Validity queries used by the planners.
pub trait CollisionChecker {
    /// Number of joints.
    fn dof(&self) -> usize;
    /// Lower joint limits.
    fn lower(&self) -> &[f32];
    /// Upper joint limits.
    fn upper(&self) -> &[f32];
    /// True if `q` is within the joint limits and collision-free.
    fn config_valid(&self, q: &[f32]) -> bool;
    /// True if every configuration on the straight segment from `a` (exclusive) to `b`
    /// (inclusive) is collision-free, discretized with the checker's resolution.
    fn motion_valid(&self, a: &[f32], b: &[f32]) -> bool;
    /// Short name of the implementation, e.g. `"scalar"` or `"simd-avx2"`.
    fn name(&self) -> String;
}

/// Number of passes of the rake along an edge of length `dist`.
#[inline(always)]
pub(crate) fn edge_passes(dist: f32, resolution: f32) -> usize {
    let n = (dist * resolution / LANES as f32).ceil();
    if n >= 1.0 {
        n as usize
    } else {
        1
    }
}

/// Index (1 to `n * LANES`) of the edge point checked by rake lane `lane` in pass `pass`.
///
/// Lane `i` owns the points `i * n + 1 ..= (i + 1) * n` and walks them backwards, so the first
/// pass samples the edge evenly at eight places.
#[inline(always)]
pub(crate) fn rake_index(lane: usize, pass: usize, n: usize) -> usize {
    lane * n + (n - pass)
}

/// How an edge point is computed. See [`edge_point`].
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    NearA,
    Middle,
    NearB,
}

/// Parameters of edge point `k` of `total`: which end it is computed from and the fraction.
#[inline(always)]
fn point_params(k: usize, total: usize) -> (Side, f32) {
    match (2 * k).cmp(&total) {
        core::cmp::Ordering::Less => (Side::NearA, k as f32 / total as f32),
        core::cmp::Ordering::Equal => (Side::Middle, 0.5),
        core::cmp::Ordering::Greater => (Side::NearB, (total - k) as f32 / total as f32),
    }
}

/// Edge point `k` (1 to `total`) of the segment from `a` to `b`, joint value only.
///
/// Points in the first half are computed from `a`, points in the second half from `b`, and the
/// midpoint symmetrically. Because `a - b == -(b - a)` exactly in IEEE arithmetic, the segment
/// from `b` to `a` then produces bit-identical points: edge validity does not depend on the
/// direction in which an edge is checked (apart from which end point is included).
#[inline(always)]
fn edge_point(a: f32, b: f32, d: f32, side: Side, t: f32) -> f32 {
    match side {
        Side::NearA => a + d * t,
        Side::Middle => (a + b) * 0.5,
        Side::NearB => b - d * t,
    }
}

#[inline(always)]
fn edge_setup(a: &[f32], b: &[f32]) -> ([f32; MAX_DOF], f32) {
    let mut d = [0.0f32; MAX_DOF];
    let mut dist2 = 0.0f32;
    for j in 0..a.len() {
        d[j] = b[j] - a[j];
        dist2 += d[j] * d[j];
    }
    (d, dist2.sqrt())
}

/// Raked edge validation with lane type `R` (eight lanes).
#[allow(dead_code)] // used by the SIMD checker
#[inline(always)]
pub(crate) fn motion_valid_rake<R: Real>(
    robot: &CompiledRobot,
    env: &EnvBlock<R>,
    a: &[f32],
    b: &[f32],
    resolution: f32,
) -> bool {
    debug_assert_eq!(R::LANES, LANES);
    let dof = robot.dof;
    let (d, dist) = edge_setup(a, b);
    let n = edge_passes(dist, resolution);
    let total = n * LANES;
    let mut q = [R::splat(0.0); MAX_DOF];
    let mut t = [0.0f32; LANES];
    let mut side = [0.0f32; LANES];
    for pass in 0..n {
        for lane in 0..LANES {
            let (s, tl) = point_params(rake_index(lane, pass, n), total);
            t[lane] = tl;
            side[lane] = match s {
                Side::NearA => -1.0,
                Side::Middle => 0.0,
                Side::NearB => 1.0,
            };
        }
        let tv = R::load(&t);
        let sv = R::load(&side);
        let near_a = sv.lt(R::splat(0.0));
        let near_b = sv.gt(R::splat(0.0));
        for j in 0..dof {
            let (av, bv, dv) = (R::splat(a[j]), R::splat(b[j]), R::splat(d[j]));
            let mid = R::splat((a[j] + b[j]) * 0.5);
            q[j] = R::select(near_a, av + dv * tv, R::select(near_b, bv - dv * tv, mid));
        }
        if fkcc(robot, env, &q[..dof]) {
            return false;
        }
    }
    true
}

/// The same discretization as [`motion_valid_rake`], one configuration at a time.
fn motion_valid_scalar(
    robot: &CompiledRobot,
    env: &EnvBlock<f32>,
    a: &[f32],
    b: &[f32],
    resolution: f32,
) -> bool {
    let dof = robot.dof;
    let (d, dist) = edge_setup(a, b);
    let n = edge_passes(dist, resolution);
    let total = n * LANES;
    let mut q = [0.0f32; MAX_DOF];
    for pass in 0..n {
        for lane in 0..LANES {
            let (side, t) = point_params(rake_index(lane, pass, n), total);
            for j in 0..dof {
                q[j] = edge_point(a[j], b[j], d[j], side, t);
            }
            if fkcc(robot, env, &q[..dof]) {
                return false;
            }
        }
    }
    true
}

/// All configurations checked on the edge from `a` to `b`, in the order the rake visits them.
pub fn edge_configurations(a: &[f32], b: &[f32], resolution: f32) -> Vec<Vec<f32>> {
    let (d, dist) = edge_setup(a, b);
    let n = edge_passes(dist, resolution);
    let total = n * LANES;
    let mut out = Vec::with_capacity(total);
    for pass in 0..n {
        for lane in 0..LANES {
            let (side, t) = point_params(rake_index(lane, pass, n), total);
            out.push(
                (0..a.len())
                    .map(|j| edge_point(a[j], b[j], d[j], side, t))
                    .collect(),
            );
        }
    }
    out
}

fn within(q: &[f32], lo: &[f32], hi: &[f32]) -> bool {
    q.len() == lo.len()
        && q.iter()
            .zip(lo)
            .zip(hi)
            .all(|((&v, &l), &h)| v >= l && v <= h)
}

/// Scalar reference checker.
#[derive(Debug, Clone)]
pub struct ScalarChecker {
    robot: CompiledRobot,
    env: EnvBlock<f32>,
    resolution: f32,
}

impl ScalarChecker {
    /// Creates a checker for `robot` in `env` with the given edge resolution.
    pub fn new(robot: &RobotModel, env: &Environment, resolution: f32) -> ScalarChecker {
        let pcs: Arc<[_]> = env.pointclouds.clone().into();
        ScalarChecker {
            robot: CompiledRobot::new(robot),
            env: EnvBlock::new(env, pcs),
            resolution,
        }
    }

    /// True if `q` is in collision (ignores joint limits).
    pub fn in_collision(&self, q: &[f32]) -> bool {
        fkcc(&self.robot, &self.env, q)
    }
}

impl CollisionChecker for ScalarChecker {
    fn dof(&self) -> usize {
        self.robot.dof
    }
    fn lower(&self) -> &[f32] {
        &self.robot.lower
    }
    fn upper(&self) -> &[f32] {
        &self.robot.upper
    }
    fn config_valid(&self, q: &[f32]) -> bool {
        within(q, &self.robot.lower, &self.robot.upper) && !fkcc(&self.robot, &self.env, q)
    }
    fn motion_valid(&self, a: &[f32], b: &[f32]) -> bool {
        motion_valid_scalar(&self.robot, &self.env, a, b, self.resolution)
    }
    fn name(&self) -> String {
        "scalar".into()
    }
}
