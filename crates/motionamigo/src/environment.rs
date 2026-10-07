//! Collision environment: spheres, capsules, oriented boxes and point clouds.

use crate::pointcloud::PointCloud;
use crate::scene::Scene;

/// A sphere obstacle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    /// Center in meters.
    pub center: [f32; 3],
    /// Radius in meters.
    pub radius: f32,
}

/// A capsule obstacle: all points within `radius` of the segment `a`-`b`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Capsule {
    /// First segment end point.
    pub a: [f32; 3],
    /// Second segment end point.
    pub b: [f32; 3],
    /// Radius in meters.
    pub radius: f32,
}

/// An oriented box (cuboid) obstacle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cuboid {
    /// Box center.
    pub center: [f32; 3],
    /// Orthonormal box axes expressed in the world (`axes[k]` is local axis `k`).
    pub axes: [[f32; 3]; 3],
    /// Half extents along the three axes.
    pub half_extents: [f32; 3],
}

impl Cuboid {
    /// Axis-aligned box from center and full sizes.
    pub fn aabb(center: [f32; 3], size: [f32; 3]) -> Cuboid {
        Cuboid::with_yaw(center, size, 0.0)
    }

    /// Box rotated by `yaw` about the world z axis, from center and full sizes.
    pub fn with_yaw(center: [f32; 3], size: [f32; 3], yaw: f32) -> Cuboid {
        let (s, c) = (yaw as f64).sin_cos();
        let (s, c) = (s as f32, c as f32);
        Cuboid {
            center,
            axes: [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]],
            half_extents: size.map(|v| 0.5 * v),
        }
    }

    /// Box from center, rotation matrix (row-major, columns are the box axes) and half extents.
    pub fn from_rotation(center: [f32; 3], rot: [[f32; 3]; 3], half_extents: [f32; 3]) -> Cuboid {
        Cuboid {
            center,
            axes: [0, 1, 2].map(|j| [rot[0][j], rot[1][j], rot[2][j]]),
            half_extents,
        }
    }
}

/// A named obstacle, for display and lookup purposes.
#[derive(Debug, Clone, PartialEq)]
pub enum Obstacle {
    /// Sphere obstacle.
    Sphere(Sphere),
    /// Capsule obstacle.
    Capsule(Capsule),
    /// Oriented box obstacle.
    Cuboid(Cuboid),
}

/// The set of obstacles a robot must avoid.
#[derive(Debug, Clone, Default)]
pub struct Environment {
    /// Sphere obstacles.
    pub spheres: Vec<Sphere>,
    /// Capsule obstacles.
    pub capsules: Vec<Capsule>,
    /// Oriented box obstacles.
    pub cuboids: Vec<Cuboid>,
    /// Point clouds.
    pub pointclouds: Vec<PointCloud>,
    /// Optional names of the cuboids (same order as `cuboids`), e.g. scene object ids.
    pub cuboid_names: Vec<String>,
}

impl Environment {
    /// An empty environment.
    pub fn new() -> Environment {
        Environment::default()
    }

    /// Builds an environment from a v0.1 scene: every object becomes an oriented box.
    pub fn from_scene(scene: &Scene) -> Environment {
        let mut env = Environment::new();
        for o in &scene.objects {
            env.add_named_cuboid(
                &o.id,
                Cuboid::with_yaw(
                    o.center.map(|v| v as f32),
                    o.size.map(|v| v as f32),
                    o.yaw as f32,
                ),
            );
        }
        env
    }

    /// Like [`Environment::from_scene`] but leaves out the objects whose id is in `skip`
    /// (useful when the robot is about to touch an object, e.g. grasping).
    pub fn from_scene_without(scene: &Scene, skip: &[&str]) -> Environment {
        let mut filtered = scene.clone();
        filtered.objects.retain(|o| !skip.contains(&o.id.as_str()));
        Environment::from_scene(&filtered)
    }

    /// Adds a sphere.
    pub fn add_sphere(&mut self, center: [f32; 3], radius: f32) -> &mut Self {
        self.spheres.push(Sphere { center, radius });
        self
    }

    /// Adds a capsule. A capsule of zero length is stored as a sphere.
    pub fn add_capsule(&mut self, a: [f32; 3], b: [f32; 3], radius: f32) -> &mut Self {
        if a == b {
            return self.add_sphere(a, radius);
        }
        self.capsules.push(Capsule { a, b, radius });
        self
    }

    /// Adds an oriented box.
    pub fn add_cuboid(&mut self, cuboid: Cuboid) -> &mut Self {
        self.add_named_cuboid("", cuboid)
    }

    /// Adds an oriented box with a name.
    pub fn add_named_cuboid(&mut self, name: &str, cuboid: Cuboid) -> &mut Self {
        self.cuboids.push(cuboid);
        self.cuboid_names.push(name.to_string());
        self
    }

    /// Adds a point cloud.
    pub fn add_pointcloud(&mut self, cloud: PointCloud) -> &mut Self {
        self.pointclouds.push(cloud);
        self
    }

    /// Total number of primitive obstacles (point clouds count once).
    pub fn len(&self) -> usize {
        self.spheres.len() + self.capsules.len() + self.cuboids.len() + self.pointclouds.len()
    }

    /// True if there are no obstacles.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
