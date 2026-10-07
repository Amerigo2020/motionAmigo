//! Collision kernel shared by the scalar reference and the vectorized checker.
//!
//! [`fkcc`] ("forward kinematics plus collision checking") evaluates the kinematic chain
//! lazily, link by link, and tests each link against the environment as soon as its frame is
//! known. Every link is first tested with its bounding sphere; only if that sphere touches an
//! obstacle in at least one lane are the link's individual spheres tested. The function returns
//! as soon as any lane is in collision. Self-collision is tested at the end for the configured
//! link pairs, again bounding spheres first.

use crate::environment::Environment;
use crate::kinematics::{CompiledRobot, Fk, Frame};
use crate::pointcloud::PointCloud;
use crate::robot::MAX_LINKS;
use crate::simd::stack::StackVec;

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
    (0..env.pointclouds.len()).any(|i| pointcloud_any(env, i, p, r))
}

#[inline(always)]
fn pointcloud_any<R: Real>(env: &EnvBlock<R>, i: usize, p: [R; 3], r: R) -> bool {
    let pc = &env.pointclouds[i];
    (0..R::LANES).any(|lane| {
        let c = [p[0].lane(lane), p[1].lane(lane), p[2].lane(lane)];
        pc.collides(c, r.lane(lane))
    })
}

const MAX_HITS: usize = 16;

/// Obstacles touched by a link's bounding sphere. The link's own spheres only need to be tested
/// against these (a sphere inside the bounding sphere cannot touch anything else).
pub(crate) struct Hits {
    len: usize,
    overflow: bool,
    items: [(u8, u32); MAX_HITS],
}

const SPHERE: u8 = 0;
const CAPSULE: u8 = 1;
const CUBOID: u8 = 2;
const CLOUD: u8 = 3;

impl Hits {
    #[inline(always)]
    fn new() -> Hits {
        Hits {
            len: 0,
            overflow: false,
            items: [(0, 0); MAX_HITS],
        }
    }

    #[inline(always)]
    fn push(&mut self, kind: u8, i: usize) {
        if self.len < MAX_HITS {
            self.items[self.len] = (kind, i as u32);
            self.len += 1;
        } else {
            self.overflow = true;
        }
    }
}

/// Records every obstacle the sphere `(p, r)` touches in any lane. Returns true if there is one.
#[inline(always)]
fn sphere_env_hits<R: Real>(env: &EnvBlock<R>, p: [R; 3], r: R, hits: &mut Hits) -> bool {
    for (i, s) in env.spheres.iter().enumerate() {
        if sphere_sphere(p, r, s).any() {
            hits.push(SPHERE, i);
        }
    }
    for (i, c) in env.capsules.iter().enumerate() {
        if sphere_capsule(p, r, c).any() {
            hits.push(CAPSULE, i);
        }
    }
    for (i, b) in env.cuboids.iter().enumerate() {
        if sphere_cuboid(p, r, b).any() {
            hits.push(CUBOID, i);
        }
    }
    for i in 0..env.pointclouds.len() {
        if pointcloud_any(env, i, p, r) {
            hits.push(CLOUD, i);
        }
    }
    hits.len > 0 || hits.overflow
}

/// Like [`sphere_env_any`] but restricted to the recorded obstacles.
#[inline(always)]
fn sphere_hits_any<R: Real>(env: &EnvBlock<R>, p: [R; 3], r: R, hits: &Hits) -> bool {
    if hits.overflow {
        return sphere_env_any(env, p, r);
    }
    for &(kind, i) in &hits.items[..hits.len] {
        let i = i as usize;
        let hit = match kind {
            SPHERE => sphere_sphere(p, r, &env.spheres[i]).any(),
            CAPSULE => sphere_capsule(p, r, &env.capsules[i]).any(),
            CUBOID => sphere_cuboid(p, r, &env.cuboids[i]).any(),
            _ => pointcloud_any(env, i, p, r),
        };
        if hit {
            return true;
        }
    }
    false
}

/// Center and radius of the sphere used for the first-level test of a link: its bounding
/// sphere, or the link's only sphere.
#[inline(always)]
fn link_bound<R: Real>(robot: &CompiledRobot, li: usize, f: &Frame<R>) -> ([R; 3], f32) {
    let l = &robot.links[li];
    if l.len == 1 {
        let s = robot.spheres[l.start];
        (f.point([s[0], s[1], s[2]]), s[3])
    } else {
        (f.point([l.bound[0], l.bound[1], l.bound[2]]), l.bound[3])
    }
}

/// Forward kinematics plus collision check for a block of configurations.
///
/// `q[k]` holds joint `k` for all lanes. Returns `true` if at least one lane is in collision
/// (with the environment or with the robot itself). Joint limits are not checked here.
#[inline(always)]
pub(crate) fn fkcc<R: Real>(robot: &CompiledRobot, env: &EnvBlock<R>, q: &[R]) -> bool {
    let mut fk = Fk::new(robot, q);
    let mut bounds = StackVec::<[R; 3], MAX_LINKS>::new();
    for (li, l) in robot.links.iter().enumerate() {
        let f = *fk.frame(l.frame);
        let (bc, br) = link_bound(robot, li, &f);
        bounds.push(bc);
        if l.len == 1 {
            if sphere_env_any(env, bc, R::splat(br)) {
                return true;
            }
            continue;
        }
        let mut hits = Hits::new();
        if !sphere_env_hits(env, bc, R::splat(br), &mut hits) {
            continue;
        }
        for s in &robot.spheres[l.start..l.start + l.len] {
            if sphere_hits_any(env, f.point([s[0], s[1], s[2]]), R::splat(s[3]), &hits) {
                return true;
            }
        }
    }
    // All frames are computed at this point (every link was visited).
    for &(a, b) in &robot.self_pairs {
        let (la, lb) = (&robot.links[a], &robot.links[b]);
        let (ra, rb) = (robot.links[a].bound[3], robot.links[b].bound[3]);
        let (ca, cb) = (*bounds.get(a), *bounds.get(b));
        let d = sub(ca, cb);
        let rs = R::splat(ra + rb);
        if !dot(d, d).lt(rs * rs).any() {
            continue;
        }
        // Second level: the spheres of each link against the bounding sphere of the other.
        let fa = *fk.frame(la.frame);
        let fb = *fk.frame(lb.frame);
        let near_a = spheres_near(robot, a, &fa, cb, rb);
        if near_a == 0 {
            continue;
        }
        let near_b = spheres_near(robot, b, &fb, ca, ra);
        if near_b == 0 {
            continue;
        }
        // Third level: the remaining sphere pairs.
        let mut ma = near_a;
        while ma != 0 {
            let i = la.start + ma.trailing_zeros() as usize;
            ma &= ma - 1;
            let sa = robot.spheres[i];
            let pa = fa.point([sa[0], sa[1], sa[2]]);
            let mut mb = near_b;
            while mb != 0 {
                let j = lb.start + mb.trailing_zeros() as usize;
                mb &= mb - 1;
                let sb = robot.spheres[j];
                let d = sub(pa, fb.point([sb[0], sb[1], sb[2]]));
                let rs = R::splat(sa[3] + sb[3]);
                if dot(d, d).lt(rs * rs).any() {
                    return true;
                }
            }
        }
    }
    false
}

/// Bit `k` is set if sphere `k` of link `li` touches the sphere `(c, r)` in any lane.
#[inline(always)]
fn spheres_near<R: Real>(robot: &CompiledRobot, li: usize, f: &Frame<R>, c: [R; 3], r: f32) -> u64 {
    let l = &robot.links[li];
    let mut mask = 0u64;
    for (k, s) in robot.spheres[l.start..l.start + l.len].iter().enumerate() {
        let d = sub(f.point([s[0], s[1], s[2]]), c);
        let rs = R::splat(s[3] + r);
        if dot(d, d).lt(rs * rs).any() {
            mask |= 1 << k;
        }
    }
    mask
}
