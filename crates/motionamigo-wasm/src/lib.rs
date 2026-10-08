//! WebAssembly bindings for the motionAmigo browser demo.
//!
//! Built with `wasm-pack build crates/motionamigo-wasm --release --target web --out-dir ../../web/pkg`.
//! `.cargo/config.toml` enables the `simd128` target feature, so the collision checker uses the
//! WebAssembly SIMD backend. Configurations are passed as `Float64Array`s.

use motionamigo::grasp::{plan_pick, pregrasp_configuration, PickSettings, PregraspSettings};
use motionamigo::planner::path_length;
use motionamigo::planner::rrtc::rrt_connect;
use motionamigo::planner::simplify::{simplify, SimplifySettings};
use motionamigo::rng::Rng;
use motionamigo::{
    Backend, CollisionChecker, Environment, PlanSettings, RobotModel, Scene, SimdChecker,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = performance, js_name = now)]
    fn performance_now() -> f64;
}

/// Returns the crate version.
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

/// Name of the SIMD backend compiled into this module (normally "wasm-simd128").
#[wasm_bindgen(js_name = simdBackend)]
pub fn simd_backend() -> String {
    Backend::detect().name().to_string()
}

/// The Panda "ready" configuration.
#[wasm_bindgen(js_name = pandaReady)]
pub fn panda_ready() -> Vec<f64> {
    motionamigo::PANDA_READY.to_vec()
}

/// Result of a planning query.
#[wasm_bindgen]
pub struct PlanOutput {
    path: Vec<f64>,
    dof: usize,
    /// Whether a path was found.
    pub solved: bool,
    /// RRT-Connect time in milliseconds.
    #[wasm_bindgen(js_name = planningMs)]
    pub planning_ms: f64,
    /// Shortcutting time in milliseconds.
    #[wasm_bindgen(js_name = simplifyMs)]
    pub simplify_ms: f64,
    /// RRT-Connect iterations.
    pub iterations: usize,
    /// Joint-space length of the path before shortcutting.
    #[wasm_bindgen(js_name = initialLength)]
    pub initial_length: f64,
    /// Joint-space length of the returned path.
    pub length: f64,
    message: String,
}

#[wasm_bindgen]
impl PlanOutput {
    /// Waypoints, flattened row by row (`waypoints * dof` values).
    #[wasm_bindgen(getter)]
    pub fn path(&self) -> Vec<f64> {
        self.path.clone()
    }

    /// Number of waypoints.
    #[wasm_bindgen(getter)]
    pub fn waypoints(&self) -> usize {
        self.path.len().checked_div(self.dof).unwrap_or(0)
    }

    /// Error message if the query failed.
    #[wasm_bindgen(getter)]
    pub fn message(&self) -> String {
        self.message.clone()
    }
}

/// Result of a pick query: all segments joined into one path.
#[wasm_bindgen]
pub struct PickOutput {
    path: Vec<f64>,
    object_in_tcp: Vec<f64>,
    /// Whether a pick was found.
    pub solved: bool,
    /// Index of the waypoint at which the object is grasped (end of the approach). From here on
    /// the object moves with the hand.
    #[wasm_bindgen(js_name = graspIndex)]
    pub grasp_index: usize,
    /// Total planning time (IK, linear segments and RRT-Connect) in milliseconds.
    pub ms: f64,
    message: String,
}

#[wasm_bindgen]
impl PickOutput {
    /// Waypoints, flattened row by row: to pre-grasp, approach, retreat (object held), home.
    #[wasm_bindgen(getter)]
    pub fn path(&self) -> Vec<f64> {
        self.path.clone()
    }

    /// Pose of the object in the TCP frame while held, column-major 4x4 (three.js order).
    #[wasm_bindgen(getter, js_name = objectInTcp)]
    pub fn object_in_tcp(&self) -> Vec<f64> {
        self.object_in_tcp.clone()
    }

    /// Error message if the query failed.
    #[wasm_bindgen(getter)]
    pub fn message(&self) -> String {
        self.message.clone()
    }
}

fn column_major(p: &motionamigo::math::Pose) -> Vec<f64> {
    let m = p.to_matrix();
    (0..4).flat_map(|c| m.iter().map(move |r| r[c])).collect()
}

/// Robot, scene and collision checker of the demo.
#[wasm_bindgen]
pub struct Demo {
    robot: RobotModel,
    scene: Scene,
    checker: SimdChecker,
}

fn js_err(e: impl std::fmt::Display) -> JsError {
    JsError::new(&e.to_string())
}

#[wasm_bindgen]
impl Demo {
    /// Creates the demo for the Panda in a scene given as v0.1 JSON.
    #[wasm_bindgen(constructor)]
    pub fn new(scene_json: &str) -> Result<Demo, JsError> {
        let robot = RobotModel::panda();
        let scene = Scene::from_json(scene_json).map_err(js_err)?;
        let checker = SimdChecker::new(&robot, &Environment::from_scene(&scene), 32.0);
        Ok(Demo {
            robot,
            scene,
            checker,
        })
    }

    /// Replaces the scene.
    #[wasm_bindgen(js_name = setScene)]
    pub fn set_scene(&mut self, scene_json: &str) -> Result<(), JsError> {
        self.scene = Scene::from_json(scene_json).map_err(js_err)?;
        self.rebuild();
        Ok(())
    }

    /// The current scene as JSON (including moved objects).
    #[wasm_bindgen(js_name = sceneJson)]
    pub fn scene_json(&self) -> String {
        self.scene.to_json()
    }

    /// Moves a scene object to a new center and yaw and rebuilds the collision environment.
    #[wasm_bindgen(js_name = moveObject)]
    pub fn move_object(
        &mut self,
        id: &str,
        x: f64,
        y: f64,
        z: f64,
        yaw: f64,
    ) -> Result<(), JsError> {
        let obj = self
            .scene
            .objects
            .iter_mut()
            .find(|o| o.id == id)
            .ok_or_else(|| JsError::new(&format!("no object {id:?}")))?;
        obj.center = [x, y, z];
        obj.yaw = yaw;
        self.rebuild();
        Ok(())
    }

    fn rebuild(&mut self) {
        self.checker = SimdChecker::new(&self.robot, &Environment::from_scene(&self.scene), 32.0);
    }

    /// Number of joints.
    #[wasm_bindgen(getter)]
    pub fn dof(&self) -> usize {
        self.robot.dof()
    }

    /// Lower joint limits.
    #[wasm_bindgen(js_name = lowerLimits)]
    pub fn lower_limits(&self) -> Vec<f64> {
        self.robot.lower_limits()
    }

    /// Upper joint limits.
    #[wasm_bindgen(js_name = upperLimits)]
    pub fn upper_limits(&self) -> Vec<f64> {
        self.robot.upper_limits()
    }

    /// True if `q` is within the joint limits and collision-free.
    #[wasm_bindgen(js_name = configValid)]
    pub fn config_valid(&self, q: &[f64]) -> bool {
        let q: Vec<f32> = q.iter().map(|&v| v as f32).collect();
        self.checker.config_valid(&q)
    }

    /// Collision spheres in the world, flattened `[x, y, z, r, ...]`.
    pub fn spheres(&self, q: &[f64]) -> Vec<f64> {
        self.robot.spheres_world(q).into_iter().flatten().collect()
    }

    /// Frames 0 (base) to dof as column-major 4x4 matrices (three.js order), flattened.
    pub fn frames(&self, q: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity((self.robot.dof() + 2) * 16);
        let mut frames = self.robot.frames(q);
        frames.push(*frames.last().unwrap() * self.robot.tcp);
        for f in frames {
            out.extend(column_major(&f));
        }
        out
    }

    /// Plans from `start` to `goal`. Never throws: failures are reported in the output.
    pub fn plan(&self, start: &[f64], goal: &[f64], seed: u64) -> PlanOutput {
        let dof = self.robot.dof();
        let fail = |message: String| PlanOutput {
            path: Vec::new(),
            dof,
            solved: false,
            planning_ms: 0.0,
            simplify_ms: 0.0,
            iterations: 0,
            initial_length: 0.0,
            length: 0.0,
            message,
        };
        if start.len() != dof || goal.len() != dof {
            return fail(format!("configurations need {dof} values"));
        }
        let s: Vec<f32> = start.iter().map(|&v| v as f32).collect();
        let g: Vec<f32> = goal.iter().map(|&v| v as f32).collect();
        if !self.checker.config_valid(&s) {
            return fail("start configuration is in collision".into());
        }
        if !self.checker.config_valid(&g) {
            return fail("goal configuration is in collision".into());
        }
        let settings = PlanSettings::default();
        let mut rng = Rng::new(seed);
        let t0 = performance_now();
        let result = rrt_connect(&self.checker, &s, &[g], &settings.rrtc, &mut rng);
        let t1 = performance_now();
        let Some(mut path) = result.path else {
            return fail(format!("no path found in {} iterations", result.iterations));
        };
        let initial_length = path_length(&path) as f64;
        simplify(
            &mut path,
            &self.checker,
            &SimplifySettings::default(),
            &mut rng,
        );
        let t2 = performance_now();
        PlanOutput {
            length: path_length(&path) as f64,
            path: path.into_iter().flatten().map(f64::from).collect(),
            dof,
            solved: true,
            planning_ms: t1 - t0,
            simplify_ms: t2 - t1,
            iterations: result.iterations,
            initial_length,
            message: String::new(),
        }
    }

    /// A collision-free configuration with the hand 10 cm above the object `id`, pointing down
    /// (or tilted away from the base if needed). Returns an empty array if none was found.
    #[wasm_bindgen(js_name = pregraspGoal)]
    pub fn pregrasp_goal(&self, id: &str, start: &[f64]) -> Vec<f64> {
        let Some(object) = self.scene.object(id) else {
            return Vec::new();
        };
        let settings = PregraspSettings::default();
        pregrasp_configuration(&self.robot, &self.checker, object, start, &settings)
            .map(|(_, _, q)| q)
            .unwrap_or_default()
    }

    /// Plans a pick of object `id` from `start`: to the pre-grasp pose, linear approach, grasp,
    /// linear lift with the object attached and back to the ready pose holding it. Never throws.
    pub fn pick(&self, id: &str, start: &[f64], seed: u64) -> PickOutput {
        let mut settings = PickSettings {
            place: Some(motionamigo::PANDA_READY.to_vec()),
            ..PickSettings::default()
        };
        settings.pregrasp.plan.seed = seed;
        let t0 = performance_now();
        let result = plan_pick(&self.robot, &self.scene, id, start, &settings);
        let ms = performance_now() - t0;
        match result {
            Ok(p) => {
                let pre = p.to_pregrasp.path.len() + p.approach.len() - 1;
                let place = p.place.map(|pl| pl.path).unwrap_or_default();
                let path = p
                    .to_pregrasp
                    .path
                    .iter()
                    .chain(&p.approach[1..])
                    .chain(&p.retreat[1..])
                    .chain(place.iter().skip(1))
                    .flatten()
                    .copied()
                    .collect();
                PickOutput {
                    path,
                    object_in_tcp: column_major(&p.object_in_tcp),
                    solved: true,
                    grasp_index: pre - 1,
                    ms,
                    message: String::new(),
                }
            }
            Err(e) => PickOutput {
                path: Vec::new(),
                object_in_tcp: Vec::new(),
                solved: false,
                grasp_index: 0,
                ms,
                message: e.to_string(),
            },
        }
    }

    /// Samples a random collision-free configuration whose tool points down and lies above the
    /// height `min_z`. Returns an empty array if none was found within `attempts` samples.
    #[wasm_bindgen(js_name = randomGoal)]
    pub fn random_goal(&self, seed: u64, min_z: f64, attempts: usize) -> Vec<f64> {
        let mut rng = Rng::new(seed);
        let (lo, hi) = (self.robot.lower_limits(), self.robot.upper_limits());
        for _ in 0..attempts {
            let q: Vec<f64> = (0..self.robot.dof())
                .map(|k| rng.uniform(lo[k] as f32, hi[k] as f32) as f64)
                .collect();
            let tcp = self.robot.tcp_pose(&q);
            if tcp.trans[2] < min_z || tcp.rot[2][2] > -0.8 || tcp.trans[0] < 0.2 {
                continue;
            }
            if self.config_valid(&q) {
                return q;
            }
        }
        Vec::new()
    }
}
