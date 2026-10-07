//! Forward kinematics kernel in `f32`, generic over [`Real`].
//!
//! The robot description is "compiled" into a compact `f32` form once. The kernel then computes
//! link frames and sphere centers for one configuration (`R = f32`) or for a block of eight
//! configurations in structure-of-arrays layout (`R` = an eight-lane vector), using exactly the
//! same sequence of floating-point operations in both cases.

use crate::math::{sin_cos, Pose};
use crate::robot::{DhConvention, RobotModel, MAX_DOF};
use crate::simd::Real;

/// How the constant twist `alpha` of a joint is applied. The common multiples of `pi/2` are
/// special-cased to avoid multiplications by zero and one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Twist {
    Zero,
    PlusHalfPi,
    MinusHalfPi,
    Pi,
    General { c: f32, s: f32 },
}

impl Twist {
    fn from_alpha(alpha: f64) -> Twist {
        use core::f64::consts::{FRAC_PI_2, PI};
        let close = |x: f64| (alpha - x).abs() < 1e-9;
        if close(0.0) {
            Twist::Zero
        } else if close(FRAC_PI_2) {
            Twist::PlusHalfPi
        } else if close(-FRAC_PI_2) {
            Twist::MinusHalfPi
        } else if close(PI) || close(-PI) {
            Twist::Pi
        } else {
            Twist::General {
                c: alpha.cos() as f32,
                s: alpha.sin() as f32,
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct CompiledJoint {
    pub a: f32,
    pub d: f32,
    pub twist: Twist,
    pub theta_offset: f32,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // `bound` is consumed by the collision checker
pub(crate) struct CompiledLink {
    pub frame: usize,
    pub start: usize,
    pub len: usize,
    pub bound: [f32; 4],
}

/// `f32` representation of a [`RobotModel`] used by the collision kernels.
#[derive(Debug, Clone)]
#[allow(dead_code)] // limits and self-collision pairs are consumed by the collision checker
pub struct CompiledRobot {
    pub(crate) dof: usize,
    pub(crate) convention: DhConvention,
    pub(crate) joints: Vec<CompiledJoint>,
    pub(crate) base: [[f32; 3]; 4],
    pub(crate) links: Vec<CompiledLink>,
    pub(crate) spheres: Vec<[f32; 4]>,
    pub(crate) self_pairs: Vec<(usize, usize)>,
    pub(crate) lower: Vec<f32>,
    pub(crate) upper: Vec<f32>,
    pub(crate) min_radius: f32,
    pub(crate) max_radius: f32,
}

/// Rounds an `f64` to the nearest `f32` that is not smaller.
fn round_up(v: f64) -> f32 {
    let f = v as f32;
    if (f as f64) < v {
        f32::from_bits(f.to_bits() + 1)
    } else {
        f
    }
}

impl CompiledRobot {
    /// Compiles a robot model.
    pub fn new(robot: &RobotModel) -> CompiledRobot {
        let joints = robot
            .joints
            .iter()
            .map(|j| CompiledJoint {
                a: j.a as f32,
                d: j.d as f32,
                twist: Twist::from_alpha(j.alpha),
                theta_offset: j.theta_offset as f32,
            })
            .collect();
        let b = robot.base;
        let base = [
            [b.rot[0][0], b.rot[1][0], b.rot[2][0]].map(|v| v as f32),
            [b.rot[0][1], b.rot[1][1], b.rot[2][1]].map(|v| v as f32),
            [b.rot[0][2], b.rot[1][2], b.rot[2][2]].map(|v| v as f32),
            b.trans.map(|v| v as f32),
        ];
        let mut links = Vec::new();
        let mut spheres = Vec::new();
        // Check links in kinematic order so collisions near the base exit early with few frames.
        let mut order: Vec<usize> = (0..robot.links.len()).collect();
        order.sort_by_key(|&i| robot.links[i].frame);
        let mut remap = vec![0; robot.links.len()];
        for (new, &old) in order.iter().enumerate() {
            remap[old] = new;
            let l = &robot.links[old];
            links.push(CompiledLink {
                frame: l.frame,
                start: spheres.len(),
                len: l.spheres.len(),
                bound: [
                    l.bounding[0] as f32,
                    l.bounding[1] as f32,
                    l.bounding[2] as f32,
                    round_up(l.bounding[3]),
                ],
            });
            spheres.extend(l.spheres.iter().map(|s| s.map(|v| v as f32)));
        }
        let self_pairs = robot
            .self_collision
            .iter()
            .map(|&(a, b)| (remap[a], remap[b]))
            .collect();
        let radii = spheres.iter().map(|s| s[3]);
        CompiledRobot {
            dof: robot.dof(),
            convention: robot.convention,
            joints,
            base,
            links,
            min_radius: radii.clone().fold(f32::INFINITY, f32::min),
            max_radius: radii.fold(0.0, f32::max),
            spheres,
            self_pairs,
            lower: robot.joints.iter().map(|j| j.lower as f32).collect(),
            upper: robot.joints.iter().map(|j| j.upper as f32).collect(),
        }
    }

    /// Number of joints.
    pub fn dof(&self) -> usize {
        self.dof
    }

    /// Number of collision spheres.
    pub fn num_spheres(&self) -> usize {
        self.spheres.len()
    }
}

/// A rigid frame with lane-wise entries: `c[k]` is local axis `k`, `p` the origin.
#[derive(Debug, Clone, Copy)]
pub struct Frame<R: Real> {
    /// Axes of the frame expressed in the world.
    pub c: [[R; 3]; 3],
    /// Origin of the frame in the world.
    pub p: [R; 3],
}

#[inline(always)]
fn axpy<R: Real>(acc: [R; 3], v: [R; 3], s: f32) -> [R; 3] {
    if s == 0.0 {
        return acc;
    }
    let s = R::splat(s);
    [acc[0] + v[0] * s, acc[1] + v[1] * s, acc[2] + v[2] * s]
}

#[inline(always)]
fn lin2<R: Real>(u: [R; 3], cu: R, v: [R; 3], cv: R) -> [R; 3] {
    [
        u[0] * cu + v[0] * cv,
        u[1] * cu + v[1] * cv,
        u[2] * cu + v[2] * cv,
    ]
}

#[inline(always)]
fn neg3<R: Real>(v: [R; 3]) -> [R; 3] {
    [-v[0], -v[1], -v[2]]
}

impl<R: Real> Frame<R> {
    #[inline(always)]
    fn from_rows(m: &[[f32; 3]; 4]) -> Frame<R> {
        Frame {
            c: [m[0].map(R::splat), m[1].map(R::splat), m[2].map(R::splat)],
            p: m[3].map(R::splat),
        }
    }

    #[inline(always)]
    fn twist(self, t: Twist) -> Frame<R> {
        let [x, y, z] = self.c;
        let (y, z) = match t {
            Twist::Zero => (y, z),
            Twist::PlusHalfPi => (z, neg3(y)),
            Twist::MinusHalfPi => (neg3(z), y),
            Twist::Pi => (neg3(y), neg3(z)),
            Twist::General { c, s } => {
                let (c, s) = (R::splat(c), R::splat(s));
                (lin2(y, c, z, s), lin2(z, c, y, -s))
            }
        };
        Frame {
            c: [x, y, z],
            p: self.p,
        }
    }

    #[inline(always)]
    fn rot_z(self, s: R, c: R) -> Frame<R> {
        let [x, y, z] = self.c;
        Frame {
            c: [lin2(x, c, y, s), lin2(y, c, x, -s), z],
            p: self.p,
        }
    }

    /// Applies joint `j` with joint angle sine `s` and cosine `c`.
    #[inline(always)]
    pub(crate) fn apply_joint(self, conv: DhConvention, j: &CompiledJoint, s: R, c: R) -> Self {
        match conv {
            DhConvention::ModifiedDh => {
                let f = self.twist(j.twist);
                let f = Frame {
                    p: axpy(f.p, f.c[0], j.a),
                    ..f
                };
                let f = f.rot_z(s, c);
                Frame {
                    p: axpy(f.p, f.c[2], j.d),
                    ..f
                }
            }
            DhConvention::Dh => {
                let f = self.rot_z(s, c);
                let f = Frame {
                    p: axpy(f.p, f.c[2], j.d),
                    ..f
                };
                let f = Frame {
                    p: axpy(f.p, f.c[0], j.a),
                    ..f
                };
                f.twist(j.twist)
            }
        }
    }

    /// Transforms a constant local point into the world.
    #[inline(always)]
    pub fn point(&self, local: [f32; 3]) -> [R; 3] {
        let acc = axpy(self.p, self.c[0], local[0]);
        let acc = axpy(acc, self.c[1], local[1]);
        axpy(acc, self.c[2], local[2])
    }
}

/// Lazily evaluated forward kinematics for one configuration block.
pub(crate) struct Fk<'a, R: Real> {
    robot: &'a CompiledRobot,
    q: &'a [R],
    frames: [Frame<R>; MAX_DOF + 1],
    computed: usize,
}

impl<'a, R: Real> Fk<'a, R> {
    #[inline(always)]
    pub fn new(robot: &'a CompiledRobot, q: &'a [R]) -> Self {
        let base = Frame::from_rows(&robot.base);
        Fk {
            robot,
            q,
            frames: [base; MAX_DOF + 1],
            computed: 0,
        }
    }

    /// Frame `i` (0 = base), computing intermediate frames on demand.
    #[inline(always)]
    pub fn frame(&mut self, i: usize) -> &Frame<R> {
        while self.computed < i {
            let k = self.computed;
            let j = &self.robot.joints[k];
            let theta = if j.theta_offset == 0.0 {
                self.q[k]
            } else {
                self.q[k] + R::splat(j.theta_offset)
            };
            let (s, c) = sin_cos(theta);
            self.frames[k + 1] = self.frames[k].apply_joint(self.robot.convention, j, s, c);
            self.computed += 1;
        }
        &self.frames[i]
    }
}

/// Computes the world centers of all spheres (in compiled link order) for a block of
/// configurations. `q[k]` holds joint `k` for every lane; `out[i]` receives `[x, y, z]` of
/// sphere `i`.
pub fn sphere_centers<R: Real>(robot: &CompiledRobot, q: &[R], out: &mut [[R; 3]]) {
    let mut fk = Fk::new(robot, q);
    for l in &robot.links {
        let f = *fk.frame(l.frame);
        for (k, s) in robot.spheres[l.start..l.start + l.len].iter().enumerate() {
            out[l.start + k] = f.point([s[0], s[1], s[2]]);
        }
    }
}

/// Convenience: `f32` sphere centers and radii for one configuration, in compiled link order.
pub fn spheres_f32(robot: &CompiledRobot, q: &[f32]) -> Vec<[f32; 4]> {
    let mut centers = vec![[0.0f32; 3]; robot.num_spheres()];
    sphere_centers::<f32>(robot, q, &mut centers);
    centers
        .iter()
        .zip(&robot.spheres)
        .map(|(c, s)| [c[0], c[1], c[2], s[3]])
        .collect()
}

/// Converts the world pose of frame `frame_index` from the `f32` kernel into a [`Pose`].
pub fn frame_pose_f32(robot: &CompiledRobot, q: &[f32], frame_index: usize) -> Pose {
    let mut fk = Fk::<f32>::new(robot, q);
    let f = *fk.frame(frame_index);
    let mut rot = [[0.0; 3]; 3];
    for (i, row) in rot.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = f.c[j][i] as f64;
        }
    }
    Pose {
        rot,
        trans: f.p.map(|v| v as f64),
    }
}
