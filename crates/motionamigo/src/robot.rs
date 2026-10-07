//! Robot models: kinematic chain, joint limits and collision spheres.
//!
//! Robots are described by TOML files. The bundled Franka Emika Panda lives in
//! `crates/motionamigo/robots/panda.toml` and is available via [`RobotModel::panda`].
//!
//! ```toml
//! name = "my_arm"
//! convention = "modified_dh"      # or "dh" (standard Denavit-Hartenberg)
//!
//! [[joints]]                       # one entry per revolute joint
//! name = "joint1"
//! a = 0.0                          # meters
//! d = 0.333                        # meters
//! alpha = 0.0                      # radians
//! theta_offset = 0.0               # optional, radians
//! lower = -2.8973                  # joint limits, radians
//! upper = 2.8973
//!
//! [tcp]                            # tool center point relative to the last joint frame
//! xyz = [0.0, 0.0, 0.2104]
//! rpy = [0.0, 0.0, -0.785398]
//!
//! [[links]]                        # collision geometry
//! name = "link1"
//! frame = 1                        # 0 = robot base, i = frame of joint i
//! spheres = [[0.0, -0.08, 0.0, 0.06]]   # x, y, z, radius in that frame
//!
//! self_collision = [["link0", "link5"]]  # link pairs checked for self-collision
//! ```

use crate::math::Pose;
use serde::Deserialize;

/// Maximum number of joints supported by the vectorized kernels.
pub const MAX_DOF: usize = 8;
/// Maximum number of collision spheres per robot.
pub const MAX_SPHERES: usize = 128;
/// Maximum number of collision links per robot.
pub const MAX_LINKS: usize = 24;
/// Maximum number of spheres per collision link.
pub const MAX_LINK_SPHERES: usize = 64;

/// Errors raised while loading a robot description.
#[derive(Debug, thiserror::Error)]
pub enum RobotError {
    /// The TOML is malformed or does not match the expected structure.
    #[error("invalid robot TOML: {0}")]
    Toml(#[from] toml::de::Error),
    /// The description is well-formed but inconsistent.
    #[error("invalid robot description: {0}")]
    Invalid(String),
}

/// Denavit-Hartenberg convention used by a robot description.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DhConvention {
    /// Modified (Craig) convention: `T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)`.
    ModifiedDh,
    /// Standard convention: `T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`.
    Dh,
}

/// A revolute joint with its DH parameters and limits.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Joint {
    /// Joint name.
    pub name: String,
    /// Link length `a` in meters.
    pub a: f64,
    /// Link offset `d` in meters.
    pub d: f64,
    /// Link twist `alpha` in radians.
    pub alpha: f64,
    /// Constant offset added to the joint angle.
    #[serde(default)]
    pub theta_offset: f64,
    /// Lower joint limit in radians.
    pub lower: f64,
    /// Upper joint limit in radians.
    pub upper: f64,
}

/// A collision link: a set of spheres rigidly attached to one kinematic frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Link {
    /// Link name.
    pub name: String,
    /// Frame index: 0 is the robot base, `i` is the frame after joint `i`.
    pub frame: usize,
    /// Spheres `[x, y, z, r]` in the link frame.
    pub spheres: Vec<[f64; 4]>,
    /// A sphere that encloses all spheres of the link, used for hierarchical checks.
    pub bounding: [f64; 4],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RobotToml {
    name: String,
    convention: DhConvention,
    joints: Vec<Joint>,
    #[serde(default)]
    tcp: Option<TcpToml>,
    links: Vec<LinkToml>,
    #[serde(default)]
    self_collision: Vec<[String; 2]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TcpToml {
    #[serde(default)]
    xyz: [f64; 3],
    #[serde(default)]
    rpy: [f64; 3],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LinkToml {
    name: String,
    frame: usize,
    spheres: Vec<[f64; 4]>,
}

/// A serial robot arm with revolute joints and a sphere-based collision model.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotModel {
    /// Robot name.
    pub name: String,
    /// DH convention of [`RobotModel::joints`].
    pub convention: DhConvention,
    /// Revolute joints from base to tip.
    pub joints: Vec<Joint>,
    /// Tool center point relative to the frame of the last joint.
    pub tcp: Pose,
    /// Pose of the robot base in the world. Defaults to the identity (base at the origin, z up).
    pub base: Pose,
    /// Collision links.
    pub links: Vec<Link>,
    /// Pairs of link indices checked for self-collision.
    pub self_collision: Vec<(usize, usize)>,
}

const PANDA_TOML: &str = include_str!("../robots/panda.toml");

impl RobotModel {
    /// The bundled Franka Emika Panda (7 DoF) with 59 collision spheres.
    pub fn panda() -> RobotModel {
        RobotModel::from_toml(PANDA_TOML).expect("bundled panda.toml is valid")
    }

    /// Parses a robot description (see the module docs for the format).
    pub fn from_toml(text: &str) -> Result<RobotModel, RobotError> {
        let raw: RobotToml = toml::from_str(text)?;
        let dof = raw.joints.len();
        if dof == 0 || dof > MAX_DOF {
            return Err(RobotError::Invalid(format!(
                "robot must have between 1 and {MAX_DOF} joints, found {dof}"
            )));
        }
        for j in &raw.joints {
            if !(j.lower.is_finite() && j.upper.is_finite()) || j.lower >= j.upper {
                return Err(RobotError::Invalid(format!(
                    "joint {} has lower limit >= upper limit",
                    j.name
                )));
            }
        }
        if raw.links.len() > MAX_LINKS {
            return Err(RobotError::Invalid(format!(
                "at most {MAX_LINKS} links are supported"
            )));
        }
        let mut n_spheres = 0;
        let mut links = Vec::with_capacity(raw.links.len());
        for l in raw.links {
            if l.frame > dof {
                return Err(RobotError::Invalid(format!(
                    "link {} refers to frame {} but the robot has {dof} joints",
                    l.name, l.frame
                )));
            }
            if l.spheres.is_empty()
                || l.spheres
                    .iter()
                    .any(|s| s.iter().any(|v| !v.is_finite()) || s[3] <= 0.0)
            {
                return Err(RobotError::Invalid(format!(
                    "link {} needs at least one sphere with positive radius",
                    l.name
                )));
            }
            n_spheres += l.spheres.len();
            let bounding = bounding_sphere(&l.spheres);
            links.push(Link {
                name: l.name,
                frame: l.frame,
                spheres: l.spheres,
                bounding,
            });
        }
        if n_spheres > MAX_SPHERES {
            return Err(RobotError::Invalid(format!(
                "at most {MAX_SPHERES} spheres are supported, found {n_spheres}"
            )));
        }
        let index = |name: &str| {
            links.iter().position(|l| l.name == name).ok_or_else(|| {
                RobotError::Invalid(format!("unknown link {name:?} in self_collision"))
            })
        };
        let mut self_collision = Vec::new();
        for [a, b] in &raw.self_collision {
            let (ia, ib) = (index(a)?, index(b)?);
            if ia == ib {
                return Err(RobotError::Invalid(format!(
                    "self-collision pair {a:?} with itself"
                )));
            }
            self_collision.push((ia.min(ib), ia.max(ib)));
        }
        let tcp = raw
            .tcp
            .map(|t| Pose::from_xyz_rpy(t.xyz, t.rpy))
            .unwrap_or(Pose::IDENTITY);
        Ok(RobotModel {
            name: raw.name,
            convention: raw.convention,
            joints: raw.joints,
            tcp,
            base: Pose::IDENTITY,
            links,
            self_collision,
        })
    }

    /// Returns a copy of the robot mounted at `base` in the world.
    pub fn with_base(mut self, base: Pose) -> RobotModel {
        self.base = base;
        self
    }

    /// Number of joints.
    pub fn dof(&self) -> usize {
        self.joints.len()
    }

    /// Total number of collision spheres.
    pub fn num_spheres(&self) -> usize {
        self.links.iter().map(|l| l.spheres.len()).sum()
    }

    /// Lower joint limits.
    pub fn lower_limits(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.lower).collect()
    }

    /// Upper joint limits.
    pub fn upper_limits(&self) -> Vec<f64> {
        self.joints.iter().map(|j| j.upper).collect()
    }

    /// True if `q` has the right length and lies within the joint limits.
    pub fn within_limits(&self, q: &[f64]) -> bool {
        q.len() == self.dof()
            && q.iter()
                .zip(&self.joints)
                .all(|(&v, j)| v >= j.lower && v <= j.upper)
    }

    /// Transform of joint `i` (from frame `i` to frame `i + 1`) for joint angle `q`.
    pub fn joint_transform(&self, i: usize, q: f64) -> Pose {
        let j = &self.joints[i];
        let theta = q + j.theta_offset;
        match self.convention {
            DhConvention::ModifiedDh => {
                Pose::rot_x(j.alpha)
                    * Pose::from_translation([j.a, 0.0, 0.0])
                    * Pose::rot_z(theta)
                    * Pose::from_translation([0.0, 0.0, j.d])
            }
            DhConvention::Dh => {
                Pose::rot_z(theta)
                    * Pose::from_translation([0.0, 0.0, j.d])
                    * Pose::from_translation([j.a, 0.0, 0.0])
                    * Pose::rot_x(j.alpha)
            }
        }
    }

    /// Forward kinematics in `f64`: world poses of frames `0..=dof` (frame 0 is the base).
    ///
    /// # Panics
    /// Panics if `q.len() != self.dof()`.
    pub fn frames(&self, q: &[f64]) -> Vec<Pose> {
        assert_eq!(q.len(), self.dof(), "configuration has wrong length");
        let mut frames = Vec::with_capacity(self.dof() + 1);
        frames.push(self.base);
        for (i, &qi) in q.iter().enumerate() {
            let next = *frames.last().unwrap() * self.joint_transform(i, qi);
            frames.push(next);
        }
        frames
    }

    /// World pose of the tool center point.
    pub fn tcp_pose(&self, q: &[f64]) -> Pose {
        *self.frames(q).last().unwrap() * self.tcp
    }

    /// World positions and radii `[x, y, z, r]` of all collision spheres, link by link.
    pub fn spheres_world(&self, q: &[f64]) -> Vec<[f64; 4]> {
        let frames = self.frames(q);
        self.links
            .iter()
            .flat_map(|l| {
                let f = frames[l.frame];
                l.spheres.iter().map(move |s| {
                    let p = f.transform_point([s[0], s[1], s[2]]);
                    [p[0], p[1], p[2], s[3]]
                })
            })
            .collect()
    }

    /// A configuration in the middle of the joint range.
    pub fn mid_configuration(&self) -> Vec<f64> {
        self.joints
            .iter()
            .map(|j| 0.5 * (j.lower + j.upper))
            .collect()
    }
}

/// Conservative enclosing sphere of a set of spheres (center of the bounding box).
fn bounding_sphere(spheres: &[[f64; 4]]) -> [f64; 4] {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for s in spheres {
        for k in 0..3 {
            lo[k] = lo[k].min(s[k] - s[3]);
            hi[k] = hi[k].max(s[k] + s[3]);
        }
    }
    let c = [0, 1, 2].map(|k| 0.5 * (lo[k] + hi[k]));
    let r = spheres
        .iter()
        .map(|s| {
            let d = ((s[0] - c[0]).powi(2) + (s[1] - c[1]).powi(2) + (s[2] - c[2]).powi(2)).sqrt();
            d + s[3]
        })
        .fold(0.0, f64::max);
    // A small margin keeps the hierarchy conservative after rounding to f32.
    [c[0], c[1], c[2], r + 1e-4]
}

/// The "ready" configuration commonly used for the Panda.
pub const PANDA_READY: [f64; 7] = [
    0.0,
    -core::f64::consts::FRAC_PI_4,
    0.0,
    -3.0 * core::f64::consts::FRAC_PI_4,
    0.0,
    core::f64::consts::FRAC_PI_2,
    core::f64::consts::FRAC_PI_4,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn panda_loads() {
        let r = RobotModel::panda();
        assert_eq!(r.dof(), 7);
        assert_eq!(r.num_spheres(), 59);
        assert_eq!(r.links.len(), 11);
        assert_eq!(r.self_collision.len(), 21);
        assert!(r.within_limits(&PANDA_READY));
        assert!(
            !r.within_limits(&[0.0; 7]),
            "q4 = 0 violates the Panda limits"
        );
    }

    #[test]
    fn bounding_spheres_enclose_link_spheres() {
        let r = RobotModel::panda();
        for l in &r.links {
            let b = l.bounding;
            for s in &l.spheres {
                let d =
                    ((s[0] - b[0]).powi(2) + (s[1] - b[1]).powi(2) + (s[2] - b[2]).powi(2)).sqrt();
                assert!(d + s[3] <= b[3], "{}", l.name);
            }
        }
    }

    #[test]
    fn rejects_bad_descriptions() {
        let bad_frame = r#"name = "x"
convention = "dh"
[[joints]]
name = "j"
a = 0.0
d = 0.0
alpha = 0.0
lower = -1.0
upper = 1.0
[[links]]
name = "l"
frame = 2
spheres = [[0.0, 0.0, 0.0, 0.1]]
"#;
        assert!(RobotModel::from_toml(bad_frame).is_err());
        let bad_limits = bad_frame.replace("upper = 1.0", "upper = -2.0");
        assert!(RobotModel::from_toml(&bad_limits).is_err());
    }

    #[test]
    fn standard_dh_planar_two_link_arm() {
        let toml = r#"name = "planar"
convention = "dh"
[[joints]]
name = "j1"
a = 1.0
d = 0.0
alpha = 0.0
lower = -3.0
upper = 3.0
[[joints]]
name = "j2"
a = 0.5
d = 0.0
alpha = 0.0
lower = -3.0
upper = 3.0
[[links]]
name = "tip"
frame = 2
spheres = [[0.0, 0.0, 0.0, 0.05]]
"#;
        let r = RobotModel::from_toml(toml).unwrap();
        let (q1, q2) = (0.3_f64, -0.7_f64);
        let p = r.tcp_pose(&[q1, q2]).trans;
        let expect = [
            q1.cos() + 0.5 * (q1 + q2).cos(),
            q1.sin() + 0.5 * (q1 + q2).sin(),
            0.0,
        ];
        for k in 0..3 {
            assert!((p[k] - expect[k]).abs() < 1e-12);
        }
    }
}
