//! Python bindings for motionAmigo.
//!
//! The Python package `motionamigo` re-exports everything defined here. Arrays go in and out as
//! NumPy arrays (`float64`); any array-like input (lists, tuples, arrays of other dtypes) is
//! accepted and converted.

use motionamigo as ma;
use numpy::{AllowTypeChange, PyArray1, PyArray2, PyArray3, PyArrayLike1, PyArrayLike2};
use pyo3::create_exception;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use std::sync::Arc;

create_exception!(
    _motionamigo,
    PlanningError,
    PyRuntimeError,
    "Raised when no path is found or the query is invalid."
);

type ArrayIn1<'py> = PyArrayLike1<'py, f64, AllowTypeChange>;
type ArrayIn2<'py> = PyArrayLike2<'py, f64, AllowTypeChange>;

fn vec1(a: &ArrayIn1<'_>) -> Vec<f64> {
    a.as_array().iter().copied().collect()
}

fn vec3(a: &ArrayIn1<'_>, what: &str) -> PyResult<[f32; 3]> {
    let v = vec1(a);
    if v.len() != 3 {
        return Err(PyValueError::new_err(format!(
            "{what} must have 3 elements, got {}",
            v.len()
        )));
    }
    Ok([v[0] as f32, v[1] as f32, v[2] as f32])
}

fn pose_to_numpy<'py>(py: Python<'py>, p: &ma::math::Pose) -> Bound<'py, PyArray2<f64>> {
    let m = p.to_matrix();
    PyArray2::from_vec2(py, &m.iter().map(|r| r.to_vec()).collect::<Vec<_>>()).expect("4x4 matrix")
}

/// A robot model: kinematic chain, joint limits and collision spheres.
#[pyclass(module = "motionamigo", name = "Robot", frozen, skip_from_py_object)]
#[derive(Clone)]
struct Robot {
    inner: Arc<ma::RobotModel>,
}

#[pymethods]
impl Robot {
    /// The bundled Franka Emika Panda (7 DoF, 59 collision spheres).
    #[staticmethod]
    fn panda() -> Robot {
        Robot {
            inner: Arc::new(ma::RobotModel::panda()),
        }
    }

    /// The bundled Universal Robots UR5 with a Robotiq 2F-85 gripper (6 DoF, 40 collision spheres).
    #[staticmethod]
    fn ur5() -> Robot {
        Robot {
            inner: Arc::new(ma::RobotModel::ur5()),
        }
    }

    /// Loads a robot from a TOML description (see the Rust docs for the format).
    #[staticmethod]
    fn from_toml(text: &str) -> PyResult<Robot> {
        ma::RobotModel::from_toml(text)
            .map(|r| Robot { inner: Arc::new(r) })
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Robot name.
    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }

    /// Number of joints.
    #[getter]
    fn dof(&self) -> usize {
        self.inner.dof()
    }

    /// Number of collision spheres.
    #[getter]
    fn num_spheres(&self) -> usize {
        self.inner.num_spheres()
    }

    /// Joint names.
    #[getter]
    fn joint_names(&self) -> Vec<String> {
        self.inner.joints.iter().map(|j| j.name.clone()).collect()
    }

    /// Lower joint limits in radians.
    #[getter]
    fn lower_limits<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.inner.lower_limits())
    }

    /// Upper joint limits in radians.
    #[getter]
    fn upper_limits<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.inner.upper_limits())
    }

    /// True if the configuration lies within the joint limits.
    fn within_limits(&self, q: ArrayIn1<'_>) -> bool {
        self.inner.within_limits(&vec1(&q))
    }

    /// 4x4 homogeneous transform of the tool center point.
    fn fk<'py>(&self, py: Python<'py>, q: ArrayIn1<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let q = self.check_q(&q)?;
        Ok(pose_to_numpy(py, &self.inner.tcp_pose(&q)))
    }

    /// Transforms of frames 0 (base) to dof, shape (dof + 1, 4, 4).
    fn frames<'py>(
        &self,
        py: Python<'py>,
        q: ArrayIn1<'py>,
    ) -> PyResult<Bound<'py, PyArray3<f64>>> {
        let q = self.check_q(&q)?;
        let frames: Vec<Vec<Vec<f64>>> = self
            .inner
            .frames(&q)
            .iter()
            .map(|f| f.to_matrix().iter().map(|r| r.to_vec()).collect())
            .collect();
        PyArray3::from_vec3(py, &frames).map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// World positions and radii of all collision spheres, shape (num_spheres, 4).
    fn spheres<'py>(
        &self,
        py: Python<'py>,
        q: ArrayIn1<'py>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let q = self.check_q(&q)?;
        let s: Vec<Vec<f64>> = self
            .inner
            .spheres_world(&q)
            .iter()
            .map(|s| s.to_vec())
            .collect();
        PyArray2::from_vec2(py, &s).map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Inverse kinematics for a 4x4 target pose of the tool center point (damped least squares
    /// with random restarts). Returns None if no solution within the limits was found.
    #[pyo3(signature = (target, seed_q=None, *, seed=0, restarts=32, position_tolerance=1e-4, orientation_tolerance=1e-3))]
    #[allow(clippy::too_many_arguments)]
    fn ik<'py>(
        &self,
        py: Python<'py>,
        target: ArrayIn2<'py>,
        seed_q: Option<ArrayIn1<'py>>,
        seed: u64,
        restarts: usize,
        position_tolerance: f64,
        orientation_tolerance: f64,
    ) -> PyResult<Option<Bound<'py, PyArray1<f64>>>> {
        let pose = matrix_to_pose(&target)?;
        let settings = ma::ik::IkSettings {
            restarts,
            position_tolerance,
            orientation_tolerance,
            seed,
            ..ma::ik::IkSettings::default()
        };
        let initial = match seed_q {
            Some(q) => Some(self.check_q(&q)?),
            None => None,
        };
        let robot = self.inner.clone();
        let sol = py.detach(move || ma::ik::solve(&robot, &pose, initial.as_deref(), &settings));
        Ok(sol.map(|s| PyArray1::from_vec(py, s.q)))
    }

    fn __repr__(&self) -> String {
        format!(
            "Robot(name={:?}, dof={}, spheres={})",
            self.inner.name,
            self.inner.dof(),
            self.inner.num_spheres()
        )
    }
}

impl Robot {
    fn check_q(&self, q: &ArrayIn1<'_>) -> PyResult<Vec<f64>> {
        let q = vec1(q);
        if q.len() != self.inner.dof() {
            return Err(PyValueError::new_err(format!(
                "expected {} joint values, got {}",
                self.inner.dof(),
                q.len()
            )));
        }
        Ok(q)
    }
}

fn matrix_to_pose(m: &ArrayIn2<'_>) -> PyResult<ma::math::Pose> {
    let a = m.as_array();
    if a.shape() != [4, 4] && a.shape() != [3, 4] {
        return Err(PyValueError::new_err("pose must be a 4x4 (or 3x4) matrix"));
    }
    let mut pose = ma::math::Pose::IDENTITY;
    for i in 0..3 {
        for j in 0..3 {
            pose.rot[i][j] = a[[i, j]];
        }
        pose.trans[i] = a[[i, 3]];
    }
    Ok(pose)
}

/// Obstacles: spheres, capsules, oriented boxes and point clouds.
#[pyclass(module = "motionamigo", name = "Environment", skip_from_py_object)]
#[derive(Clone, Default)]
struct Environment {
    inner: ma::Environment,
}

#[pymethods]
impl Environment {
    #[new]
    fn new() -> Environment {
        Environment::default()
    }

    /// Builds an environment from a scene in the shared v0.1 format (a JSON string, a path, or a
    /// dict). Every object becomes an oriented box; objects listed in `skip` are left out.
    #[staticmethod]
    #[pyo3(signature = (scene, skip=Vec::new()))]
    fn from_scene(py: Python<'_>, scene: &Bound<'_, PyAny>, skip: Vec<String>) -> PyResult<Self> {
        let scene = parse_scene(py, scene)?;
        let skip: Vec<&str> = skip.iter().map(|s| s.as_str()).collect();
        Ok(Environment {
            inner: ma::Environment::from_scene_without(&scene, &skip),
        })
    }

    /// Adds a sphere.
    fn add_sphere(&mut self, center: ArrayIn1<'_>, radius: f64) -> PyResult<()> {
        self.inner
            .add_sphere(vec3(&center, "center")?, radius as f32);
        Ok(())
    }

    /// Adds a capsule (all points within `radius` of the segment from `a` to `b`).
    fn add_capsule(&mut self, a: ArrayIn1<'_>, b: ArrayIn1<'_>, radius: f64) -> PyResult<()> {
        self.inner
            .add_capsule(vec3(&a, "a")?, vec3(&b, "b")?, radius as f32);
        Ok(())
    }

    /// Adds a box with full edge lengths `size`, rotated by `yaw` about z (scene convention).
    #[pyo3(signature = (center, size, yaw=0.0, name=""))]
    fn add_box(
        &mut self,
        center: ArrayIn1<'_>,
        size: ArrayIn1<'_>,
        yaw: f64,
        name: &str,
    ) -> PyResult<()> {
        let c = ma::Cuboid::with_yaw(vec3(&center, "center")?, vec3(&size, "size")?, yaw as f32);
        self.inner.add_named_cuboid(name, c);
        Ok(())
    }

    /// Adds an oriented box from half extents and a 3x3 rotation matrix (columns are the box axes).
    #[pyo3(signature = (center, half_extents, rotation=None))]
    fn add_cuboid(
        &mut self,
        center: ArrayIn1<'_>,
        half_extents: ArrayIn1<'_>,
        rotation: Option<ArrayIn2<'_>>,
    ) -> PyResult<()> {
        let mut rot = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        if let Some(r) = rotation {
            let a = r.as_array();
            if a.shape() != [3, 3] {
                return Err(PyValueError::new_err("rotation must be a 3x3 matrix"));
            }
            for i in 0..3 {
                for j in 0..3 {
                    rot[i][j] = a[[i, j]] as f32;
                }
            }
        }
        self.inner.add_cuboid(ma::Cuboid::from_rotation(
            vec3(&center, "center")?,
            rot,
            vec3(&half_extents, "half_extents")?,
        ));
        Ok(())
    }

    /// Adds a point cloud of shape (N, 3). Each point is inflated by `point_radius`.
    #[pyo3(signature = (points, point_radius=0.0, cell_size=0.05))]
    fn add_pointcloud(
        &mut self,
        points: ArrayIn2<'_>,
        point_radius: f64,
        cell_size: f64,
    ) -> PyResult<()> {
        let a = points.as_array();
        if a.ncols() != 3 {
            return Err(PyValueError::new_err("points must have shape (N, 3)"));
        }
        if cell_size <= 0.0 || point_radius < 0.0 {
            return Err(PyValueError::new_err(
                "cell_size must be positive and point_radius non-negative",
            ));
        }
        let pts: Vec<[f32; 3]> = a
            .rows()
            .into_iter()
            .map(|r| [r[0] as f32, r[1] as f32, r[2] as f32])
            .collect();
        self.inner.add_pointcloud(ma::PointCloud::new(
            &pts,
            point_radius as f32,
            cell_size as f32,
        ));
        Ok(())
    }

    /// Number of primitive obstacles (each point cloud counts once).
    fn __len__(&self) -> usize {
        self.inner.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "Environment(spheres={}, capsules={}, boxes={}, pointclouds={})",
            self.inner.spheres.len(),
            self.inner.capsules.len(),
            self.inner.cuboids.len(),
            self.inner.pointclouds.len()
        )
    }
}

fn parse_scene(py: Python<'_>, scene: &Bound<'_, PyAny>) -> PyResult<ma::Scene> {
    let text: String = if let Ok(s) = scene.extract::<String>() {
        if s.trim_start().starts_with('{') {
            s
        } else {
            std::fs::read_to_string(&s)
                .map_err(|e| PyValueError::new_err(format!("cannot read {s}: {e}")))?
        }
    } else if scene.hasattr("__fspath__")? {
        let p: std::path::PathBuf = scene.extract()?;
        std::fs::read_to_string(&p)
            .map_err(|e| PyValueError::new_err(format!("cannot read {}: {e}", p.display())))?
    } else {
        let json = py.import("json")?;
        json.call_method1("dumps", (scene,))?.extract()?
    };
    ma::Scene::from_json(&text).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Result of a successful planning query.
#[pyclass(module = "motionamigo", name = "PlanResult", frozen)]
struct PlanResult {
    path: Vec<Vec<f64>>,
    #[pyo3(get)]
    length: f64,
    #[pyo3(get)]
    initial_length: f64,
    #[pyo3(get)]
    iterations: usize,
    #[pyo3(get)]
    tree_sizes: (usize, usize),
    #[pyo3(get)]
    planning_time: f64,
    #[pyo3(get)]
    simplify_time: f64,
    #[pyo3(get)]
    checker: String,
}

#[pymethods]
impl PlanResult {
    /// Waypoints, shape (K, dof). Straight joint-space segments between them are collision-free.
    #[getter]
    fn path<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        PyArray2::from_vec2(py, &self.path).map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Total time in seconds (planning plus simplification).
    #[getter]
    fn total_time(&self) -> f64 {
        self.planning_time + self.simplify_time
    }

    /// The path resampled so that consecutive configurations are at most `step` radians apart
    /// (L2 norm). Useful for animation and execution.
    #[pyo3(signature = (step=0.05))]
    fn interpolate<'py>(&self, py: Python<'py>, step: f64) -> PyResult<Bound<'py, PyArray2<f64>>> {
        if step <= 0.0 {
            return Err(PyValueError::new_err("step must be positive"));
        }
        let mut out: Vec<Vec<f64>> = vec![self.path[0].clone()];
        for w in self.path.windows(2) {
            let d: f64 = w[0]
                .iter()
                .zip(&w[1])
                .map(|(a, b)| (b - a) * (b - a))
                .sum::<f64>()
                .sqrt();
            let n = (d / step).ceil().max(1.0) as usize;
            for k in 1..=n {
                let t = k as f64 / n as f64;
                out.push(
                    w[0].iter()
                        .zip(&w[1])
                        .map(|(a, b)| a + (b - a) * t)
                        .collect(),
                );
            }
        }
        PyArray2::from_vec2(py, &out).map_err(|e| PyValueError::new_err(e.to_string()))
    }

    fn __len__(&self) -> usize {
        self.path.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "PlanResult(waypoints={}, length={:.3}, planning_time={:.1} us, simplify_time={:.1} us, checker={:?})",
            self.path.len(),
            self.length,
            self.planning_time * 1e6,
            self.simplify_time * 1e6,
            self.checker
        )
    }
}

/// Collision checker plus planner for one robot in one environment.
#[pyclass(module = "motionamigo", name = "Planner", frozen)]
struct Planner {
    checker: Arc<dyn ma::CollisionChecker + Send + Sync>,
    dof: usize,
}

fn make_checker(
    robot: &ma::RobotModel,
    env: &ma::Environment,
    checker: &str,
    resolution: f32,
) -> PyResult<Arc<dyn ma::CollisionChecker + Send + Sync>> {
    Ok(match checker {
        "simd" => Arc::new(ma::SimdChecker::new(robot, env, resolution)),
        "portable" => Arc::new(ma::SimdChecker::with_backend(
            robot,
            env,
            resolution,
            ma::Backend::Portable,
        )),
        "scalar" => Arc::new(ma::ScalarChecker::new(robot, env, resolution)),
        other => {
            return Err(PyValueError::new_err(format!(
                "checker must be 'simd', 'portable' or 'scalar', not {other:?}"
            )))
        }
    })
}

#[pymethods]
impl Planner {
    /// Creates a planner. `checker` is "simd" (best available SIMD backend), "portable" or
    /// "scalar"; `resolution` is the number of checked configurations per radian on edges.
    #[new]
    #[pyo3(signature = (robot, environment=None, checker="simd", resolution=32.0))]
    fn new(
        robot: &Robot,
        environment: Option<&Environment>,
        checker: &str,
        resolution: f64,
    ) -> PyResult<Planner> {
        if resolution <= 0.0 {
            return Err(PyValueError::new_err("resolution must be positive"));
        }
        let empty = ma::Environment::new();
        let env = environment.map(|e| &e.inner).unwrap_or(&empty);
        Ok(Planner {
            checker: make_checker(&robot.inner, env, checker, resolution as f32)?,
            dof: robot.inner.dof(),
        })
    }

    /// Name of the collision checker, e.g. "simd-avx2".
    #[getter]
    fn checker(&self) -> String {
        self.checker.name()
    }

    /// True if `q` is within the joint limits and collision-free.
    fn config_valid(&self, q: ArrayIn1<'_>) -> PyResult<bool> {
        let q = self.q32(&q)?;
        Ok(self.checker.config_valid(&q))
    }

    /// Validity of many configurations at once, shape (N, dof) in, boolean array of length N out.
    fn configs_valid<'py>(
        &self,
        py: Python<'py>,
        qs: ArrayIn2<'py>,
    ) -> PyResult<Bound<'py, PyArray1<bool>>> {
        let a = qs.as_array();
        if a.ncols() != self.dof {
            return Err(PyValueError::new_err(format!(
                "expected shape (N, {})",
                self.dof
            )));
        }
        let rows: Vec<Vec<f32>> = a
            .rows()
            .into_iter()
            .map(|r| r.iter().map(|&v| v as f32).collect())
            .collect();
        let checker = self.checker.clone();
        let out: Vec<bool> =
            py.detach(move || rows.iter().map(|q| checker.config_valid(q)).collect());
        Ok(PyArray1::from_vec(py, out))
    }

    /// True if the straight joint-space motion from `a` to `b` is collision-free.
    fn motion_valid(&self, a: ArrayIn1<'_>, b: ArrayIn1<'_>) -> PyResult<bool> {
        let (a, b) = (self.q32(&a)?, self.q32(&b)?);
        Ok(self.checker.motion_valid(&a, &b))
    }

    /// Plans from `start` to `goal` (or to the nearest reachable of several goals, if `goal` has
    /// shape (G, dof)). Raises PlanningError if the query is invalid or no path is found.
    #[pyo3(signature = (start, goal, *, seed=0, range=1.0, max_iterations=100_000, simplify=true))]
    #[allow(clippy::too_many_arguments)]
    fn plan(
        &self,
        py: Python<'_>,
        start: ArrayIn1<'_>,
        goal: &Bound<'_, PyAny>,
        seed: u64,
        range: f64,
        max_iterations: usize,
        simplify: bool,
    ) -> PyResult<PlanResult> {
        let start = vec1(&start);
        let goals: Vec<Vec<f64>> = if let Ok(g) = goal.extract::<ArrayIn2<'_>>() {
            g.as_array()
                .rows()
                .into_iter()
                .map(|r| r.to_vec())
                .collect()
        } else {
            let g: ArrayIn1<'_> = goal.extract()?;
            vec![vec1(&g)]
        };
        let mut settings = ma::PlanSettings {
            seed,
            ..ma::PlanSettings::default()
        };
        settings.rrtc.range = range as f32;
        settings.rrtc.max_iterations = max_iterations;
        if !simplify {
            settings.simplify = ma::planner::simplify::SimplifySettings::disabled();
        }
        let checker = self.checker.clone();
        let result = py.detach(move || ma::plan_with(checker.as_ref(), &start, &goals, &settings));
        match result {
            Ok(p) => Ok(PlanResult {
                length: p.length,
                initial_length: p.initial_length,
                iterations: p.iterations,
                tree_sizes: (p.tree_sizes[0], p.tree_sizes[1]),
                planning_time: p.planning_time.as_secs_f64(),
                simplify_time: p.simplify_time.as_secs_f64(),
                checker: p.checker,
                path: p.path,
            }),
            Err(e) => Err(PlanningError::new_err(e.to_string())),
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "Planner(dof={}, checker={:?})",
            self.dof,
            self.checker.name()
        )
    }
}

impl Planner {
    fn q32(&self, q: &ArrayIn1<'_>) -> PyResult<Vec<f32>> {
        let q = vec1(q);
        if q.len() != self.dof {
            return Err(PyValueError::new_err(format!(
                "expected {} joint values, got {}",
                self.dof,
                q.len()
            )));
        }
        Ok(q.iter().map(|&v| v as f32).collect())
    }
}

/// Result of :func:`plan_to_pregrasp`.
#[pyclass(module = "motionamigo", name = "PregraspResult", frozen)]
struct PregraspResult {
    pose: ma::math::Pose,
    goal: Vec<f64>,
    #[pyo3(get)]
    grasp_width: f64,
    #[pyo3(get)]
    plan: Py<PlanResult>,
}

#[pymethods]
impl PregraspResult {
    /// 4x4 pre-grasp pose of the tool center point.
    #[getter]
    fn pose<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        pose_to_numpy(py, &self.pose)
    }

    /// Pre-grasp joint configuration.
    #[getter]
    fn goal<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        PyArray1::from_vec(py, self.goal.clone())
    }

    fn __repr__(&self) -> String {
        format!(
            "PregraspResult(tcp={:.3?}, grasp_width={:.3})",
            self.pose.trans, self.grasp_width
        )
    }
}

/// Plans a motion from `start` to a collision-free pre-grasp pose `clearance` meters above the
/// object `object_id` of `scene` (path, JSON string or dict), approaching from above.
#[pyfunction]
#[pyo3(signature = (robot, scene, object_id, start, *, clearance=0.1, seed=0))]
fn plan_to_pregrasp(
    py: Python<'_>,
    robot: &Robot,
    scene: &Bound<'_, PyAny>,
    object_id: &str,
    start: ArrayIn1<'_>,
    clearance: f64,
    seed: u64,
) -> PyResult<PregraspResult> {
    let scene = parse_scene(py, scene)?;
    let start = robot.check_q(&start)?;
    let mut settings = ma::grasp::PregraspSettings {
        clearance,
        ..ma::grasp::PregraspSettings::default()
    };
    settings.plan.seed = seed;
    settings.ik.seed = seed;
    let model = robot.inner.clone();
    let id = object_id.to_string();
    let r = py
        .detach(move || ma::grasp::plan_to_pregrasp(&model, &scene, &id, &start, &settings))
        .map_err(|e| PlanningError::new_err(e.to_string()))?;
    let p = r.plan;
    let plan = Py::new(
        py,
        PlanResult {
            length: p.length,
            initial_length: p.initial_length,
            iterations: p.iterations,
            tree_sizes: (p.tree_sizes[0], p.tree_sizes[1]),
            planning_time: p.planning_time.as_secs_f64(),
            simplify_time: p.simplify_time.as_secs_f64(),
            checker: p.checker,
            path: p.path,
        },
    )?;
    Ok(PregraspResult {
        pose: r.pose,
        goal: r.goal,
        grasp_width: r.grasp_width,
        plan,
    })
}

/// The best SIMD backend on this machine ("avx2", "neon", "wasm-simd128" or "portable").
#[pyfunction]
fn simd_backend() -> &'static str {
    ma::Backend::detect().name()
}

/// Native extension module, re-exported by the `motionamigo` Python package.
#[pymodule]
fn _motionamigo(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("PANDA_READY", ma::PANDA_READY.to_vec())?;
    m.add("UR5_HOME", ma::UR5_HOME.to_vec())?;
    m.add("PlanningError", m.py().get_type::<PlanningError>())?;
    m.add_class::<Robot>()?;
    m.add_class::<Environment>()?;
    m.add_class::<Planner>()?;
    m.add_class::<PlanResult>()?;
    m.add_class::<PregraspResult>()?;
    m.add_function(wrap_pyfunction!(plan_to_pregrasp, m)?)?;
    m.add_function(wrap_pyfunction!(simd_backend, m)?)?;
    Ok(())
}
