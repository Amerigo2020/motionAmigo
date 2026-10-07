//! Collision kernel shared by the scalar reference and the vectorized checker.
//!
//! [`fkcc`] ("forward kinematics plus collision checking") evaluates the kinematic chain
//! lazily, link by link, and tests each link against the environment as soon as its frame is
//! known. Every link is first tested with its bounding sphere; only if that sphere touches an
//! obstacle in at least one lane are the link's individual spheres tested. The function returns
//! as soon as any lane is in collision. Self-collision is tested at the end for the configured
//! link pairs, again bounding spheres first.

use crate::environment::Environment;
use crate::kinematics::{CompiledRobot, Fk};
use crate::pointcloud::PointCloud;
use crate::robot::MAX_SPHERES;
use crate::simd::{Mask, Real};
use std::sync::Arc;

#[derive(Debug, Clone, Copy)]
pub(crate) struct SphereB<R: Real> {
    c: [R; 3],
    r: R,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct CapsuleB<R: Real> {
    a: [R; 3],
    v: [R; 3],
    inv_vv: R,
    r: R,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct CuboidB<R: Real> {
    c: [R; 3],
    axes: [[R; 3]; 3],
    half: [R; 3],
}

/// The environment with every constant broadcast to the lane type `R`.
#[derive(Debug, Clone)]
pub(crate) struct EnvBlock<R: Real> {
    spheres: Vec<SphereB<R>>,
    capsules: Vec<CapsuleB<R>>,
    cuboids: Vec<CuboidB<R>>,
    pointclouds: Arc<[PointCloud]>,
}

impl<R: Real> EnvBlock<R> {
    pub fn new(env: &Environment, pointclouds: Arc<[PointCloud]>) -> Self {
        let s3 = |v: [f32; 3]| v.map(R::splat);
        EnvBlock {
            spheres: env
                .spheres
                .iter()
                .map(|s| SphereB {
                    c: s3(s.center),
                    r: R::splat(s.radius),
                })
                .collect(),
            capsules: env
                .capsules
                .iter()
                .map(|c| {
                    let v = [c.b[0] - c.a[0], c.b[1] - c.a[1], c.b[2] - c.a[2]];
                    let vv = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                    CapsuleB {
                        a: s3(c.a),
                        v: s3(v),
                        inv_vv: R::splat(1.0 / vv),
                        r: R::splat(c.radius),
                    }
                })
                .collect(),
            cuboids: env
                .cuboids
                .iter()
                .map(|b| CuboidB {
                    c: s3(b.center),
                    axes: b.axes.map(s3),
                    half: s3(b.half_extents),
                })
                .collect(),
            pointclouds,
        }
    }
}

#[inline(always)]
fn dot<R: Real>(a: [R; 3], b: [R; 3]) -> R {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline(always)]
fn sub<R: Real>(a: [R; 3], b: [R; 3]) -> [R; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Lanes in which the sphere `(p, r)` overlaps the sphere obstacle.
#[inline(always)]
fn sphere_sphere<R: Real>(p: [R; 3], r: R, s: &SphereB<R>) -> R::Mask {
    let d = sub(p, s.c);
    let rs = r + s.r;
    dot(d, d).lt(rs * rs)
}

#[inline(always)]
fn sphere_capsule<R: Real>(p: [R; 3], r: R, c: &CapsuleB<R>) -> R::Mask {
    let d = sub(p, c.a);
    let t = (dot(d, c.v) * c.inv_vv).clamp(R::splat(0.0), R::splat(1.0));
    let e = [d[0] - c.v[0] * t, d[1] - c.v[1] * t, d[2] - c.v[2] * t];
    let rs = r + c.r;
    dot(e, e).lt(rs * rs)
}

#[inline(always)]
fn sphere_cuboid<R: Real>(p: [R; 3], r: R, b: &CuboidB<R>) -> R::Mask {
    let d = sub(p, b.c);
    let zero = R::splat(0.0);
    let e0 = (dot(d, b.axes[0]).abs() - b.half[0]).max(zero);
    let e1 = (dot(d, b.axes[1]).abs() - b.half[1]).max(zero);
    let e2 = (dot(d, b.axes[2]).abs() - b.half[2]).max(zero);
    (e0 * e0 + e1 * e1 + e2 * e2).lt(r * r)
}

/// True if the sphere `(p, r)` collides with the environment in any lane.
#[inline(always)]
pub(crate) fn sphere_env_any<R: Real>(env: &EnvBlock<R>, p: [R; 3], r: R) -> bool {
    for s in &env.spheres {
        if sphere_sphere(p, r, s).any() {
            return true;
        }
    }
    for c in &env.capsules {
        if sphere_capsule(p, r, c).any() {
            return true;
        }
    }
    for b in &env.cuboids {
        if sphere_cuboid(p, r, b).any() {
            return true;
        }
    }
    if !env.pointclouds.is_empty() {
        for lane in 0..R::LANES {
            let c = [p[0].lane(lane), p[1].lane(lane), p[2].lane(lane)];
            let rl = r.lane(lane);
            if env.pointclouds.iter().any(|pc| pc.collides(c, rl)) {
                return true;
            }
        }
    }
    false
}

/// Forward kinematics plus collision check for a block of configurations.
///
/// `q[k]` holds joint `k` for all lanes. Returns `true` if at least one lane is in collision
/// (with the environment or with the robot itself). Joint limits are not checked here.
#[inline(always)]
pub(crate) fn fkcc<R: Real>(robot: &CompiledRobot, env: &EnvBlock<R>, q: &[R]) -> bool {
    let mut fk = Fk::new(robot, q);
    let zero = R::splat(0.0);
    let mut centers = [[zero; 3]; MAX_SPHERES];
    let mut bounds = [[zero; 3]; crate::robot::MAX_LINKS];
    for (li, l) in robot.links.iter().enumerate() {
        let f = *fk.frame(l.frame);
        let spheres = &robot.spheres[l.start..l.start + l.len];
        for (k, s) in spheres.iter().enumerate() {
            centers[l.start + k] = f.point([s[0], s[1], s[2]]);
        }
        let single = l.len == 1;
        if single {
            bounds[li] = centers[l.start];
        } else {
            bounds[li] = f.point([l.bound[0], l.bound[1], l.bound[2]]);
        }
        let bound_r = R::splat(if single { spheres[0][3] } else { l.bound[3] });
        if !sphere_env_any(env, bounds[li], bound_r) {
            continue;
        }
        if single {
            return true;
        }
        for (k, s) in spheres.iter().enumerate() {
            if sphere_env_any(env, centers[l.start + k], R::splat(s[3])) {
                return true;
            }
        }
    }
    for &(a, b) in &robot.self_pairs {
        let (la, lb) = (&robot.links[a], &robot.links[b]);
        let ra = if la.len == 1 {
            robot.spheres[la.start][3]
        } else {
            la.bound[3]
        };
        let rb = if lb.len == 1 {
            robot.spheres[lb.start][3]
        } else {
            lb.bound[3]
        };
        let d = sub(bounds[a], bounds[b]);
        let rs = R::splat(ra + rb);
        if !dot(d, d).lt(rs * rs).any() {
            continue;
        }
        for i in la.start..la.start + la.len {
            for j in lb.start..lb.start + lb.len {
                let d = sub(centers[i], centers[j]);
                let rs = R::splat(robot.spheres[i][3] + robot.spheres[j][3]);
                if dot(d, d).lt(rs * rs).any() {
                    return true;
                }
            }
        }
    }
    false
}
