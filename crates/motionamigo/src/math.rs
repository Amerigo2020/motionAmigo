//! Small math utilities: rigid transforms in `f64` and a portable vectorized sine and cosine.

use crate::simd::Real;

/// A rigid transform (rotation plus translation) in `f64`.
///
/// `rot` is row-major: `rot[i][j]` is row `i`, column `j`. Column `j` is the image of the local
/// axis `j` expressed in the parent frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// Rotation matrix, row-major.
    pub rot: [[f64; 3]; 3],
    /// Translation in meters.
    pub trans: [f64; 3],
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose {
    /// The identity transform.
    pub const IDENTITY: Pose = Pose {
        rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        trans: [0.0; 3],
    };

    /// Pure translation.
    pub fn from_translation(t: [f64; 3]) -> Pose {
        Pose {
            trans: t,
            ..Pose::IDENTITY
        }
    }

    /// Rotation about x by `a` radians.
    pub fn rot_x(a: f64) -> Pose {
        let (s, c) = a.sin_cos();
        Pose {
            rot: [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
            trans: [0.0; 3],
        }
    }

    /// Rotation about y by `a` radians.
    pub fn rot_y(a: f64) -> Pose {
        let (s, c) = a.sin_cos();
        Pose {
            rot: [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
            trans: [0.0; 3],
        }
    }

    /// Rotation about z by `a` radians.
    pub fn rot_z(a: f64) -> Pose {
        let (s, c) = a.sin_cos();
        Pose {
            rot: [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            trans: [0.0; 3],
        }
    }

    /// Transform from URDF style `xyz` and fixed-axis `rpy` (roll about x, then pitch about y,
    /// then yaw about z), i.e. `R = Rz(yaw) * Ry(pitch) * Rx(roll)`.
    pub fn from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Pose {
        let r = Pose::rot_z(rpy[2]) * Pose::rot_y(rpy[1]) * Pose::rot_x(rpy[0]);
        Pose { trans: xyz, ..r }
    }

    /// Transforms a point.
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        let mut out = self.trans;
        for (i, o) in out.iter_mut().enumerate() {
            *o += self.rot[i][0] * p[0] + self.rot[i][1] * p[1] + self.rot[i][2] * p[2];
        }
        out
    }

    /// Rotates a vector (no translation).
    pub fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let mut out = [0.0; 3];
        for (i, o) in out.iter_mut().enumerate() {
            *o = self.rot[i][0] * v[0] + self.rot[i][1] * v[1] + self.rot[i][2] * v[2];
        }
        out
    }

    /// Inverse transform.
    pub fn inverse(&self) -> Pose {
        let mut rot = [[0.0; 3]; 3];
        for (i, row) in rot.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = self.rot[j][i];
            }
        }
        let t = self.trans;
        let mut trans = [0.0; 3];
        for (i, o) in trans.iter_mut().enumerate() {
            *o = -(rot[i][0] * t[0] + rot[i][1] * t[1] + rot[i][2] * t[2]);
        }
        Pose { rot, trans }
    }

    /// Local axis `j` (0 = x, 1 = y, 2 = z) expressed in the parent frame.
    pub fn axis(&self, j: usize) -> [f64; 3] {
        [self.rot[0][j], self.rot[1][j], self.rot[2][j]]
    }

    /// Unit quaternion `[x, y, z, w]` of the rotation.
    pub fn quaternion_xyzw(&self) -> [f64; 4] {
        let m = &self.rot;
        let tr = m[0][0] + m[1][1] + m[2][2];
        let q = if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0;
            [
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
                0.25 * s,
            ]
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
            [
                0.25 * s,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s,
                (m[2][1] - m[1][2]) / s,
            ]
        } else if m[1][1] > m[2][2] {
            let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
            [
                (m[0][1] + m[1][0]) / s,
                0.25 * s,
                (m[1][2] + m[2][1]) / s,
                (m[0][2] - m[2][0]) / s,
            ]
        } else {
            let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
            [
                (m[0][2] + m[2][0]) / s,
                (m[1][2] + m[2][1]) / s,
                0.25 * s,
                (m[1][0] - m[0][1]) / s,
            ]
        };
        let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        q.map(|v| v / n)
    }

    /// Row-major 4x4 homogeneous matrix.
    pub fn to_matrix(&self) -> [[f64; 4]; 4] {
        let mut m = [[0.0; 4]; 4];
        for i in 0..3 {
            m[i][..3].copy_from_slice(&self.rot[i]);
            m[i][3] = self.trans[i];
        }
        m[3][3] = 1.0;
        m
    }
}

impl core::ops::Mul for Pose {
    type Output = Pose;
    fn mul(self, o: Pose) -> Pose {
        let mut rot = [[0.0; 3]; 3];
        for (i, row) in rot.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = (0..3).map(|k| self.rot[i][k] * o.rot[k][j]).sum();
            }
        }
        Pose {
            rot,
            trans: self.transform_point(o.trans),
        }
    }
}

// Constants of the portable sine and cosine.
const INV_TWO_PI: f32 = 0.159_154_94;
// 2*pi split into three parts (Cody-Waite). TWO_PI_HI has only 8 significant bits, so `k * HI`
// is exact for every |k| < 2^16.
const TWO_PI_HI: f32 = 6.281_25;
const TWO_PI_MID: f32 = 1.935_307_2e-3;
const TWO_PI_LO: f32 = 1.025_313_2e-11;
const PI: f32 = core::f32::consts::PI;
const FRAC_PI_2: f32 = core::f32::consts::FRAC_PI_2;

/// Lane-wise sine and cosine, identical on every backend.
///
/// The argument is reduced to `[-pi, pi]`, reflected into `[-pi/2, pi/2]` and evaluated with
/// Taylor polynomials of degree 11 (sine) and 12 (cosine). The absolute error is below `5e-7`
/// for `|x| < 40`, plenty for joint angles. It only uses exactly rounded operations, so all
/// [`Real`] backends return bit-identical results.
#[inline(always)]
pub fn sin_cos<R: Real>(x: R) -> (R, R) {
    let k = (x * R::splat(INV_TWO_PI)).round();
    let y = ((x - k * R::splat(TWO_PI_HI)) - k * R::splat(TWO_PI_MID)) - k * R::splat(TWO_PI_LO);
    let hi = y.gt(R::splat(FRAC_PI_2));
    let lo = y.lt(R::splat(-FRAC_PI_2));
    let y = R::select(hi, R::splat(PI) - y, R::select(lo, R::splat(-PI) - y, y));
    let cos_sign = R::select(hi | lo, R::splat(-1.0), R::splat(1.0));
    let y2 = y * y;
    let s = R::splat(-2.505_210_8e-8);
    let s = s * y2 + R::splat(2.755_731_9e-6);
    let s = s * y2 + R::splat(-1.984_127e-4);
    let s = s * y2 + R::splat(8.333_333e-3);
    let s = s * y2 + R::splat(-0.166_666_67);
    let s = s * y2 + R::splat(1.0);
    let sin = y * s;
    let c = R::splat(2.087_675_7e-9);
    let c = c * y2 + R::splat(-2.755_732e-7);
    let c = c * y2 + R::splat(2.480_158_7e-5);
    let c = c * y2 + R::splat(-1.388_888_9e-3);
    let c = c * y2 + R::splat(4.166_666_8e-2);
    let c = c * y2 + R::splat(-0.5);
    let c = c * y2 + R::splat(1.0);
    (sin, c * cos_sign)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simd::portable::F32x8;

    #[test]
    fn sin_cos_is_accurate() {
        let mut worst: f64 = 0.0;
        for i in -40_000..=40_000 {
            let x = i as f32 * 1e-4 * 10.0;
            let (s, c) = sin_cos(x);
            let xd = x as f64;
            worst = worst
                .max((s as f64 - xd.sin()).abs())
                .max((c as f64 - xd.cos()).abs());
        }
        assert!(worst < 5e-7, "worst error {worst}");
    }

    #[test]
    fn sin_cos_vector_matches_scalar_bitwise() {
        for i in 0..2000 {
            let base = -12.0 + i as f32 * 0.0123;
            let xs: [f32; 8] = core::array::from_fn(|k| base + k as f32 * 0.37);
            let (vs, vc) = sin_cos(F32x8(xs));
            for k in 0..8 {
                let (s, c) = sin_cos(xs[k]);
                assert_eq!(vs.0[k].to_bits(), s.to_bits());
                assert_eq!(vc.0[k].to_bits(), c.to_bits());
            }
        }
    }

    #[test]
    fn pose_algebra() {
        let p = Pose::from_xyz_rpy([0.1, -0.2, 0.3], [0.4, -0.5, 0.6]);
        let id = p * p.inverse();
        for i in 0..3 {
            for j in 0..3 {
                let e = if i == j { 1.0 } else { 0.0 };
                assert!((id.rot[i][j] - e).abs() < 1e-12);
            }
            assert!(id.trans[i].abs() < 1e-12);
        }
        let q = p.quaternion_xyzw();
        assert!((q.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-12);
    }
}
