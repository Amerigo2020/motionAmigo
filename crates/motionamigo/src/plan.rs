//! High-level planning API.

use crate::checker::{CollisionChecker, ScalarChecker, DEFAULT_RESOLUTION};
use crate::environment::Environment;
use crate::planner::path_length;
use crate::planner::rrtc::{rrt_connect, RrtcSettings};
use crate::planner::simplify::{simplify, SimplifySettings};
use crate::rng::Rng;
use crate::robot::RobotModel;
use crate::time::Stopwatch;
use core::time::Duration;

/// Which collision checker implementation to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CheckerKind {
    /// Scalar reference implementation.
    #[default]
    Scalar,
}

/// All settings of a planning query.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanSettings {
    /// RRT-Connect settings.
    pub rrtc: RrtcSettings,
    /// Path simplification settings.
    pub simplify: SimplifySettings,
    /// Random seed. The same seed, problem and settings always give the same path.
    pub seed: u64,
    /// Edge resolution in checked configurations per radian.
    pub resolution: f32,
    /// Collision checker implementation.
    pub checker: CheckerKind,
}

impl Default for PlanSettings {
    fn default() -> Self {
        PlanSettings {
            rrtc: RrtcSettings::default(),
            simplify: SimplifySettings::default(),
            seed: 0,
            resolution: DEFAULT_RESOLUTION,
            checker: CheckerKind::default(),
        }
    }
}

/// Reasons a planning query can fail.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PlanError {
    /// A configuration has the wrong number of joints.
    #[error("expected {expected} joint values, got {got}")]
    Dimension {
        /// Robot DoF.
        expected: usize,
        /// Length of the offending configuration.
        got: usize,
    },
    /// The start configuration violates the joint limits or is in collision.
    #[error("start configuration is invalid (joint limits or collision)")]
    InvalidStart,
    /// A goal configuration violates the joint limits or is in collision.
    #[error("goal configuration {0} is invalid (joint limits or collision)")]
    InvalidGoal(usize),
    /// No goals were given.
    #[error("no goal configuration given")]
    NoGoal,
    /// The planner hit its iteration limit.
    #[error("no path found within {iterations} iterations")]
    NoSolution {
        /// Iterations used.
        iterations: usize,
    },
}

/// A successful planning result.
#[derive(Debug, Clone, PartialEq)]
pub struct Plan {
    /// Waypoints from start to goal (joint angles in radians). Straight lines in joint space
    /// between consecutive waypoints are collision-free.
    pub path: Vec<Vec<f64>>,
    /// Joint-space length of the path before simplification.
    pub initial_length: f64,
    /// Joint-space length of the returned path.
    pub length: f64,
    /// RRT-Connect iterations.
    pub iterations: usize,
    /// Sizes of the start and goal trees.
    pub tree_sizes: [usize; 2],
    /// Time spent in RRT-Connect (zero on wasm32).
    pub planning_time: Duration,
    /// Time spent simplifying (zero on wasm32).
    pub simplify_time: Duration,
    /// Name of the collision checker used.
    pub checker: String,
}

impl Plan {
    /// Total time (planning plus simplification).
    pub fn total_time(&self) -> Duration {
        self.planning_time + self.simplify_time
    }
}

fn to_f32(q: &[f64]) -> Vec<f32> {
    q.iter().map(|&v| v as f32).collect()
}

/// Plans with an existing checker. This is the most efficient entry point when many queries
/// share one environment, because the checker is built only once.
pub fn plan_with<C: CollisionChecker + ?Sized>(
    checker: &C,
    start: &[f64],
    goals: &[Vec<f64>],
    settings: &PlanSettings,
) -> Result<Plan, PlanError> {
    let dof = checker.dof();
    if goals.is_empty() {
        return Err(PlanError::NoGoal);
    }
    for q in std::iter::once(start).chain(goals.iter().map(|g| g.as_slice())) {
        if q.len() != dof {
            return Err(PlanError::Dimension {
                expected: dof,
                got: q.len(),
            });
        }
    }
    let start = to_f32(start);
    let goals: Vec<Vec<f32>> = goals.iter().map(|g| to_f32(g)).collect();
    if !checker.config_valid(&start) {
        return Err(PlanError::InvalidStart);
    }
    if let Some(i) = goals.iter().position(|g| !checker.config_valid(g)) {
        return Err(PlanError::InvalidGoal(i));
    }
    let mut rng = Rng::new(settings.seed);
    let sw = Stopwatch::start();
    let result = rrt_connect(checker, &start, &goals, &settings.rrtc, &mut rng);
    let planning_time = sw.elapsed();
    let Some(mut path) = result.path else {
        return Err(PlanError::NoSolution {
            iterations: result.iterations,
        });
    };
    let initial_length = path_length(&path) as f64;
    let sw = Stopwatch::start();
    simplify(&mut path, checker, &settings.simplify, &mut rng);
    let simplify_time = sw.elapsed();
    Ok(Plan {
        length: path_length(&path) as f64,
        path: path
            .into_iter()
            .map(|q| q.into_iter().map(f64::from).collect())
            .collect(),
        initial_length,
        iterations: result.iterations,
        tree_sizes: result.tree_sizes,
        planning_time,
        simplify_time,
        checker: checker.name(),
    })
}

/// Builds the checker selected in `settings`.
pub fn make_checker(
    robot: &RobotModel,
    env: &Environment,
    settings: &PlanSettings,
) -> Box<dyn CollisionChecker + Send + Sync> {
    match settings.checker {
        CheckerKind::Scalar => Box::new(ScalarChecker::new(robot, env, settings.resolution)),
    }
}

/// Plans a collision-free path for `robot` in `env` from `start` to `goal`.
///
/// ```
/// use motionamigo::{plan, Environment, PlanSettings, RobotModel, PANDA_READY};
/// let robot = RobotModel::panda();
/// let mut env = Environment::new();
/// env.add_sphere([0.45, 0.0, 0.45], 0.1);
/// let start = PANDA_READY.to_vec();
/// let goal = vec![1.2, 0.3, 0.0, -1.8, 0.0, 2.2, 0.8];
/// let plan = plan(&robot, &env, &start, &goal, &PlanSettings::default()).unwrap();
/// assert_eq!(plan.path.first().unwrap(), &start.iter().map(|&v| v as f32 as f64).collect::<Vec<_>>());
/// ```
pub fn plan(
    robot: &RobotModel,
    env: &Environment,
    start: &[f64],
    goal: &[f64],
    settings: &PlanSettings,
) -> Result<Plan, PlanError> {
    let checker = make_checker(robot, env, settings);
    plan_with(checker.as_ref(), start, &[goal.to_vec()], settings)
}
