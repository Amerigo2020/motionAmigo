//! Reproducible benchmark suite for motionAmigo.
//!
//! ```text
//! motionamigo-bench gen [--count N]                 generate bench/problems/*.json
//! motionamigo-bench run [--seeds N] [--checker C]   plan on tabletop, shelf and cage
//! motionamigo-bench mbm FILE [--checker C]          plan on exported MotionBenchMaker problems
//!                    [--robot panda|ur5]
//! ```
//!
//! `C` is `simd` (default, best backend), `portable` or `scalar`. Results are printed as
//! Markdown tables and written as JSON to `bench/results/`.

mod problems;
mod stats;

use motionamigo::math::Pose;
use motionamigo::{
    plan_with, Backend, CollisionChecker, Cuboid, Environment, PlanSettings, RobotModel,
    ScalarChecker, Scene, SimdChecker,
};
use serde::{Deserialize, Serialize};
use stats::Summary;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

struct Args {
    cmd: String,
    positional: Vec<String>,
    count: usize,
    seeds: u64,
    checker: String,
    limits: String,
    robot: String,
    rounds: Option<usize>,
    attempts: Option<usize>,
}

fn parse_args() -> Args {
    let mut it = std::env::args().skip(1);
    let mut a = Args {
        cmd: it.next().unwrap_or_else(|| "help".into()),
        positional: vec![],
        count: 100,
        seeds: 10,
        checker: "simd".into(),
        limits: "vamp".into(),
        robot: "panda".into(),
        rounds: None,
        attempts: None,
    };
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--count" => a.count = it.next().unwrap().parse().unwrap(),
            "--seeds" => a.seeds = it.next().unwrap().parse().unwrap(),
            "--checker" => a.checker = it.next().unwrap(),
            "--limits" => a.limits = it.next().unwrap(),
            "--robot" => a.robot = it.next().unwrap(),
            "--simplify-rounds" => a.rounds = Some(it.next().unwrap().parse().unwrap()),
            "--simplify-attempts" => a.attempts = Some(it.next().unwrap().parse().unwrap()),
            _ => a.positional.push(arg),
        }
    }
    a
}

fn make_checker(
    kind: &str,
    robot: &RobotModel,
    env: &Environment,
) -> Box<dyn CollisionChecker + Send + Sync> {
    match kind {
        "scalar" => Box::new(ScalarChecker::new(robot, env, 32.0)),
        "portable" => Box::new(SimdChecker::with_backend(
            robot,
            env,
            32.0,
            Backend::Portable,
        )),
        "simd" => Box::new(SimdChecker::new(robot, env, 32.0)),
        other => panic!("unknown checker {other:?}"),
    }
}

/// Machine description for result tables.
fn hardware() -> String {
    let cpu = std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("model name"))
                .map(|l| l.split(':').nth(1).unwrap_or("").trim().to_string())
        })
        .unwrap_or_else(|| std::env::consts::ARCH.to_string());
    let threads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1);
    format!(
        "{cpu}, {threads} hardware threads, {} {}",
        std::env::consts::OS,
        std::env::consts::ARCH
    )
}

#[derive(Serialize)]
struct RunRecord {
    problem: String,
    seed: u64,
    solved: bool,
    planning_us: f64,
    simplify_us: f64,
    total_us: f64,
    initial_length: f64,
    length: f64,
    iterations: usize,
}

static SETTINGS: std::sync::OnceLock<PlanSettings> = std::sync::OnceLock::new();

fn base_settings() -> PlanSettings {
    SETTINGS.get().cloned().unwrap_or_default()
}

fn run_problem(
    checker: &dyn CollisionChecker,
    name: &str,
    start: &[f64],
    goal: &[f64],
    seed: u64,
) -> RunRecord {
    let settings = PlanSettings {
        seed,
        ..base_settings()
    };
    match plan_with(checker, start, &[goal.to_vec()], &settings) {
        Ok(p) => RunRecord {
            problem: name.into(),
            seed,
            solved: true,
            planning_us: p.planning_time.as_secs_f64() * 1e6,
            simplify_us: p.simplify_time.as_secs_f64() * 1e6,
            total_us: p.total_time().as_secs_f64() * 1e6,
            initial_length: p.initial_length,
            length: p.length,
            iterations: p.iterations,
        },
        Err(e) => {
            if !matches!(e, motionamigo::PlanError::NoSolution { .. }) {
                eprintln!("{name}: {e}");
            }
            RunRecord {
                problem: name.into(),
                seed,
                solved: false,
                planning_us: f64::NAN,
                simplify_us: f64::NAN,
                total_us: f64::NAN,
                initial_length: f64::NAN,
                length: f64::NAN,
                iterations: 0,
            }
        }
    }
}

fn cmd_run(args: &Args) {
    let robot = RobotModel::panda();
    let mut rows = Vec::new();
    let mut all = serde_json::Map::new();
    for scene_name in problems::SCENES {
        let set = problems::load(scene_name);
        let scene = Scene::from_path(root().join(&set.scene)).unwrap();
        let env = Environment::from_scene(&scene);
        let checker = make_checker(&args.checker, &robot, &env);
        let mut records = Vec::new();
        for p in &set.problems {
            for seed in 0..args.seeds {
                records.push(run_problem(
                    checker.as_ref(),
                    &p.name,
                    &p.start,
                    &p.goal,
                    seed,
                ));
            }
        }
        let s = Summary::from_records(
            records
                .iter()
                .map(|r| (r.solved, r.planning_us, r.simplify_us, r.total_us, r.length)),
        );
        rows.push((scene_name.to_string(), set.problems.len(), s));
        all.insert(
            scene_name.to_string(),
            serde_json::to_value(&records).unwrap(),
        );
    }
    println!(
        "\nmotionAmigo, checker `{}` ({}), {} seeds per problem\nHardware: {}\n",
        args.checker,
        Backend::detect().name(),
        args.seeds,
        hardware()
    );
    println!("| scene | problems | runs | success | median time | P95 time | median planning | median path length |");
    println!("|---|---:|---:|---:|---:|---:|---:|---:|");
    for (name, n, s) in &rows {
        println!(
            "| {name} | {n} | {} | {:.1}% | {} | {} | {} | {:.2} rad |",
            s.runs,
            100.0 * s.success,
            stats::fmt_us(s.total_median),
            stats::fmt_us(s.total_p95),
            stats::fmt_us(s.planning_median),
            s.length_median
        );
    }
    let out = root().join(format!("bench/results/scenes-{}.json", args.checker));
    std::fs::create_dir_all(out.parent().unwrap()).unwrap();
    std::fs::write(&out, serde_json::to_string_pretty(&all).unwrap()).unwrap();
    eprintln!("\nraw results written to {}", out.display());
}

/// A MotionBenchMaker problem as exported by `bench/vamp/export_mbm.py`.
#[derive(Deserialize)]
struct MbmProblem {
    valid: bool,
    start: Vec<f64>,
    goal: Vec<f64>,
    spheres: Vec<MbmSphere>,
    capsules: Vec<MbmCapsule>,
    cuboids: Vec<MbmCuboid>,
}

#[derive(Deserialize)]
struct MbmSphere {
    center: [f32; 3],
    radius: f32,
}

#[derive(Deserialize)]
struct MbmCapsule {
    a: [f32; 3],
    b: [f32; 3],
    radius: f32,
}

#[derive(Deserialize)]
struct MbmCuboid {
    center: [f32; 3],
    rotation: [[f32; 3]; 3],
    half_extents: [f32; 3],
}

impl MbmProblem {
    fn environment(&self) -> Environment {
        let mut env = Environment::new();
        for s in &self.spheres {
            env.add_sphere(s.center, s.radius);
        }
        for c in &self.capsules {
            env.add_capsule(c.a, c.b, c.radius);
        }
        for b in &self.cuboids {
            env.add_cuboid(Cuboid::from_rotation(b.center, b.rotation, b.half_extents));
        }
        env
    }
}

/// The Panda with the joint limits of the URDF used by VAMP and MotionBenchMaker. Some MBM start
/// and goal configurations lie outside the tighter limits of the Franka datasheet.
#[allow(clippy::approx_constant)] // literal values from the URDF
fn panda_with_vamp_limits() -> RobotModel {
    let mut robot = RobotModel::panda();
    let limits = [
        (-2.9671, 2.9671),
        (-1.8326, 1.8326),
        (-2.9671, 2.9671),
        (-3.1416, 0.0873),
        (-2.9671, 2.9671),
        (-0.0873, 3.8223),
        (-2.9671, 2.9671),
    ];
    for (j, (lo, hi)) in robot.joints.iter_mut().zip(limits) {
        j.lower = lo;
        j.upper = hi;
    }
    robot
}

/// The UR5 placed like in VAMP's ur5_spherized.urdf: base_link sits on a 0.9144 m pedestal,
/// rotated by 1.57 rad, and the DH base frame is base_link rotated by pi.
fn ur5_on_mbm_pedestal() -> RobotModel {
    let base = Pose::from_translation([0.0, 0.0, 0.9144])
        * Pose::rot_z(1.57)
        * Pose::rot_z(std::f64::consts::PI);
    RobotModel::ur5().with_base(base)
}

fn cmd_mbm(args: &Args) {
    let path = args.positional.first().cloned().unwrap_or_else(|| {
        root()
            .join(format!("bench/data/mbm/{}_mbm.json", args.robot))
            .display()
            .to_string()
    });
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {path}: {e}. Run bench/vamp/run_vamp.sh first."));
    let all: std::collections::BTreeMap<String, Vec<MbmProblem>> =
        serde_json::from_str(&text).unwrap();
    // The UR5 limits (+-pi) already are those of VAMP's URDF.
    let robot = match (args.robot.as_str(), args.limits.as_str()) {
        ("ur5", _) => ur5_on_mbm_pedestal(),
        ("panda", "vamp") => panda_with_vamp_limits(),
        ("panda", _) => RobotModel::panda(),
        (other, _) => panic!("unknown robot {other:?}"),
    };
    let prefix = match args.robot.as_str() {
        "panda" => "mbm".to_string(),
        other => format!("mbm-{other}"),
    };
    let mut rows = Vec::new();
    let mut json = serde_json::Map::new();
    for (scenario, problems) in &all {
        let mut records = Vec::new();
        for (i, p) in problems.iter().enumerate().filter(|(_, p)| p.valid) {
            let env = p.environment();
            let checker = make_checker(&args.checker, &robot, &env);
            records.push(run_problem(
                checker.as_ref(),
                &format!("{scenario}/{i}"),
                &p.start,
                &p.goal,
                i as u64,
            ));
        }
        let s = Summary::from_records(
            records
                .iter()
                .map(|r| (r.solved, r.planning_us, r.simplify_us, r.total_us, r.length)),
        );
        rows.push((scenario.clone(), s));
        json.insert(scenario.clone(), serde_json::to_value(&records).unwrap());
    }
    println!(
        "\nmotionAmigo on MotionBenchMaker ({}), checker `{}` ({})\nHardware: {}\n",
        args.robot,
        args.checker,
        Backend::detect().name(),
        hardware()
    );
    stats::print_mbm_table(&rows);
    let out = root().join(format!(
        "bench/results/{prefix}-motionamigo-{}.json",
        args.checker
    ));
    std::fs::create_dir_all(out.parent().unwrap()).unwrap();
    std::fs::write(&out, serde_json::to_string_pretty(&json).unwrap()).unwrap();
    eprintln!("\nraw results written to {}", out.display());
}

fn main() {
    let args = parse_args();
    let mut settings = PlanSettings::default();
    if let Some(r) = args.rounds {
        settings.simplify.max_rounds = r;
    }
    if let Some(a) = args.attempts {
        settings.simplify.random_attempts = a;
    }
    SETTINGS.set(settings).unwrap();
    match args.cmd.as_str() {
        "gen" => problems::generate_all(args.count),
        "run" => cmd_run(&args),
        "mbm" => cmd_mbm(&args),
        _ => {
            eprintln!("usage: motionamigo-bench gen|run|mbm [options], see bench/README.md");
            std::process::exit(2);
        }
    }
}
