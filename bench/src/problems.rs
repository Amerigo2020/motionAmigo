//! Benchmark problems for the tabletop, shelf and cage scenes.
//!
//! Each problem connects two task configurations in different regions of a scene (for example
//! two shelf compartments). Task configurations are found by a seeded stochastic search over
//! joint space that places the tool center point in a target box with a given approach direction,
//! and only collision-free configurations are kept. Problems whose start and goal can be connected
//! by a straight line are rejected, so every problem requires search. The generated files are
//! committed, so the benchmark does not depend on this generator staying the same.

use motionamigo::rng::Rng;
use motionamigo::{CollisionChecker, Environment, RobotModel, Scene, SimdChecker};
use serde::{Deserialize, Serialize};

use crate::root;

pub const SCENES: [&str; 3] = ["tabletop", "shelf", "cage"];

#[derive(Serialize, Deserialize)]
pub struct ProblemSet {
    pub scene: String,
    pub robot: String,
    pub generator: String,
    pub problems: Vec<Problem>,
}

#[derive(Serialize, Deserialize)]
pub struct Problem {
    pub name: String,
    pub start_region: String,
    pub goal_region: String,
    pub start: Vec<f64>,
    pub goal: Vec<f64>,
}

/// A box for the tool center point plus the desired direction of the tool's z axis.
struct Region {
    name: &'static str,
    lo: [f64; 3],
    hi: [f64; 3],
    approach: [f64; 3],
}

const DOWN: [f64; 3] = [0.0, 0.0, -1.0];
const FORWARD: [f64; 3] = [1.0, 0.0, 0.0];

fn regions(scene: &str) -> Vec<Region> {
    let r = |name, lo, hi, approach| Region {
        name,
        lo,
        hi,
        approach,
    };
    match scene {
        // Low above the cluttered table, between the objects.
        "tabletop" => vec![
            r(
                "right_front",
                [0.35, -0.42, 0.50],
                [0.55, -0.12, 0.56],
                DOWN,
            ),
            r("left_front", [0.35, 0.12, 0.50], [0.55, 0.42, 0.56], DOWN),
            r("right_back", [0.62, -0.42, 0.50], [0.78, -0.12, 0.58], DOWN),
            r("left_back", [0.62, 0.16, 0.50], [0.78, 0.42, 0.58], DOWN),
        ],
        // Inside the compartments of the bookshelf, reaching in horizontally.
        "shelf" => vec![
            r(
                "middle_left",
                [0.56, 0.08, 0.42],
                [0.68, 0.36, 0.58],
                FORWARD,
            ),
            r(
                "middle_right",
                [0.56, -0.36, 0.42],
                [0.68, -0.08, 0.58],
                FORWARD,
            ),
            r("upper", [0.56, -0.3, 0.75], [0.66, 0.15, 0.90], FORWARD),
            r("lower", [0.56, -0.3, 0.10], [0.70, 0.3, 0.26], FORWARD),
        ],
        // Inside the cage (below and above the front bar) and outside above the table.
        "cage" => vec![
            r(
                "inside_low",
                [0.48, -0.18, 0.38],
                [0.62, 0.18, 0.46],
                FORWARD,
            ),
            r(
                "inside_high",
                [0.48, -0.18, 0.60],
                [0.62, 0.18, 0.68],
                FORWARD,
            ),
            r("outside_left", [0.32, 0.4, 0.55], [0.5, 0.55, 0.7], DOWN),
            r("outside_right", [0.32, -0.55, 0.55], [0.5, -0.4, 0.7], DOWN),
        ],
        _ => panic!("unknown scene {scene}"),
    }
}

fn cost(robot: &RobotModel, q: &[f64], target: [f64; 3], approach: [f64; 3]) -> (f64, f64, f64) {
    let p = robot.tcp_pose(q);
    let d = (0..3)
        .map(|k| (p.trans[k] - target[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let z = p.axis(2);
    let cos = z[0] * approach[0] + z[1] * approach[1] + z[2] * approach[2];
    (d + 0.2 * (1.0 - cos), d, cos)
}

/// Seeded stochastic search for a configuration with the TCP at `target` along `approach`.
fn solve(
    robot: &RobotModel,
    checker: &dyn CollisionChecker,
    rng: &mut Rng,
    target: [f64; 3],
    approach: [f64; 3],
) -> Option<Vec<f64>> {
    let lo = robot.lower_limits();
    let hi = robot.upper_limits();
    let mut q: Vec<f64> = (0..7)
        .map(|k| rng.uniform(lo[k] as f32, hi[k] as f32) as f64)
        .collect();
    let mut best = cost(robot, &q, target, approach).0;
    let steps = 3000;
    for i in 0..steps {
        let sigma = 0.4 * (0.005f64 / 0.4).powf(i as f64 / steps as f64);
        let mut cand = q.clone();
        for k in 0..7 {
            if rng.next_f32() < 0.5 {
                let g = (rng.next_f32() as f64 - 0.5) * 2.0 * sigma;
                cand[k] = (cand[k] + g).clamp(lo[k], hi[k]);
            }
        }
        let c = cost(robot, &cand, target, approach).0;
        if c < best {
            best = c;
            q = cand;
        }
    }
    let (_, d, cos) = cost(robot, &q, target, approach);
    let q32: Vec<f32> = q.iter().map(|&v| v as f32).collect();
    // 5 mm and 15 degrees.
    (d < 0.005 && cos > 0.966 && checker.config_valid(&q32)).then_some(q)
}

fn sample_in(rng: &mut Rng, r: &Region) -> [f64; 3] {
    [0, 1, 2].map(|k| r.lo[k] + (r.hi[k] - r.lo[k]) * rng.next_f32() as f64)
}

/// Rounds to 6 decimals, then checks that the rounded configuration is still valid.
fn round6(q: &[f64]) -> Vec<f64> {
    q.iter().map(|v| (v * 1e6).round() / 1e6).collect()
}

pub fn generate(scene_name: &str, count: usize) -> ProblemSet {
    let robot = RobotModel::panda();
    let scene_rel = format!("examples/scenes/{scene_name}.json");
    let scene = Scene::from_path(root().join(&scene_rel)).unwrap();
    let env = Environment::from_scene(&scene);
    let checker = SimdChecker::new(&robot, &env, 32.0);
    let regions = regions(scene_name);
    let mut rng = Rng::new(0xBE7C_4000 + scene_name.len() as u64);

    // A pool of task configurations per region.
    let per_region = count / 2 + 10;
    let mut pools: Vec<Vec<Vec<f64>>> = Vec::new();
    for r in &regions {
        let mut pool = Vec::new();
        let mut attempts = 0;
        while pool.len() < per_region {
            attempts += 1;
            assert!(attempts < 200 * per_region, "region {} unreachable", r.name);
            let t = sample_in(&mut rng, r);
            if let Some(q) = solve(&robot, &checker, &mut rng, t, r.approach) {
                let q = round6(&q);
                let q32: Vec<f32> = q.iter().map(|&v| v as f32).collect();
                if checker.config_valid(&q32) {
                    pool.push(q);
                }
            }
        }
        eprintln!(
            "  {scene_name}/{}: {} configurations ({} attempts)",
            r.name,
            pool.len(),
            attempts
        );
        pools.push(pool);
    }

    let mut problems = Vec::new();
    let mut trivial = 0;
    while problems.len() < count {
        let a = rng.below(regions.len());
        let b = (a + 1 + rng.below(regions.len() - 1)) % regions.len();
        let start = pools[a][rng.below(pools[a].len())].clone();
        let goal = pools[b][rng.below(pools[b].len())].clone();
        let s32: Vec<f32> = start.iter().map(|&v| v as f32).collect();
        let g32: Vec<f32> = goal.iter().map(|&v| v as f32).collect();
        if checker.motion_valid(&s32, &g32) {
            trivial += 1;
            continue;
        }
        problems.push(Problem {
            name: format!("{scene_name}_{:03}", problems.len()),
            start_region: regions[a].name.into(),
            goal_region: regions[b].name.into(),
            start,
            goal,
        });
    }
    eprintln!("  {scene_name}: rejected {trivial} straight-line problems");
    ProblemSet {
        scene: scene_rel,
        robot: "panda".into(),
        generator:
            "motionamigo-bench gen (seeded stochastic TCP search, straight-line problems rejected)"
                .into(),
        problems,
    }
}

pub fn generate_all(count: usize) {
    for s in SCENES {
        let set = generate(s, count);
        let path = root().join(format!("bench/problems/{s}.json"));
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, serde_json::to_string_pretty(&set).unwrap()).unwrap();
        eprintln!(
            "wrote {} problems to {}",
            set.problems.len(),
            path.display()
        );
    }
}

pub fn load(scene: &str) -> ProblemSet {
    let path = root().join(format!("bench/problems/{scene}.json"));
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    serde_json::from_str(&text).unwrap()
}
