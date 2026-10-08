import json
import threading
from pathlib import Path

import numpy as np
import pytest

import motionamigo as ma

ROOT = Path(__file__).resolve().parents[3]
TABLETOP = ROOT / "examples" / "scenes" / "tabletop.json"
ABOVE_MUG_1 = [0.06, 0.41, -1.16, -1.02, 0.55, 1.36, 0.52]
ABOVE_MUG_2 = [-0.18, 0.22, 0.89, -1.21, -0.08, 1.54, 1.31]


@pytest.fixture(scope="module")
def robot():
    return ma.Robot.panda()


@pytest.fixture(scope="module")
def cluttered():
    env = ma.Environment()
    env.add_box([0.6, 0.0, 0.2], [0.5, 1.2, 0.4])
    env.add_box([0.55, 0.0, 0.75], [0.3, 0.04, 0.5], yaw=0.1)
    env.add_sphere([0.0, 0.55, 0.6], 0.1)
    env.add_capsule([-0.5, -0.5, 0.0], [-0.5, 0.5, 1.0], 0.05)
    return env


def test_version_and_backend():
    assert ma.__version__.count(".") == 2
    assert ma.simd_backend() in {"avx2", "neon", "portable", "wasm-simd128"}


def test_robot_model(robot):
    assert robot.dof == 7
    assert robot.num_spheres == 59
    assert robot.lower_limits.shape == (7,)
    assert robot.within_limits(ma.PANDA_READY)
    assert not robot.within_limits(np.zeros(7))
    assert len(robot.joint_names) == 7


def test_forward_kinematics_reference(robot):
    pose = robot.fk(ma.PANDA_READY)
    assert pose.shape == (4, 4)
    np.testing.assert_allclose(pose[:3, 3], [0.3068905666, 0.0, 0.4868820523], atol=1e-9)
    np.testing.assert_allclose(pose[:3, :3], np.diag([1.0, -1.0, -1.0]), atol=1e-9)
    frames = robot.frames(ma.PANDA_READY)
    assert frames.shape == (8, 4, 4)
    spheres = robot.spheres(list(ma.PANDA_READY))
    assert spheres.shape == (59, 4)


def test_wrong_dimension_raises(robot):
    with pytest.raises(ValueError):
        robot.fk([0.0, 1.0])


def test_environment_from_scene_variants():
    env = ma.Environment.from_scene(str(TABLETOP))
    assert len(env) == 7
    assert len(ma.Environment.from_scene(TABLETOP)) == 7
    scene = json.loads(TABLETOP.read_text())
    assert len(ma.Environment.from_scene(scene)) == 7
    assert len(ma.Environment.from_scene(TABLETOP.read_text(), skip=["mug_1"])) == 6
    with pytest.raises(ValueError):
        ma.Environment.from_scene({"version": "9.9", "frame": "world", "objects": []})


def test_ur5_model_and_planning():
    ur5 = ma.Robot.ur5()
    assert ur5.name == "ur5"
    assert ur5.dof == 6
    assert ur5.num_spheres == 40
    assert ur5.lower_limits.shape == (6,)
    assert ur5.within_limits(ma.UR5_HOME)
    np.testing.assert_allclose(ur5.fk(np.zeros(6))[:3, 3], [-0.81725, -0.19145, -0.005491], atol=1e-9)
    results = {}
    for checker in ("simd", "scalar"):
        planner = ma.Planner(ur5, ma.Environment(), checker=checker)
        assert planner.config_valid(ma.UR5_HOME)
        goal = np.array(ma.UR5_HOME) + [1.5, 0.3, -0.4, 0.2, 0.5, -1.0]
        results[checker] = planner.plan(ma.UR5_HOME, goal, seed=3).path
        assert results[checker].shape[1] == 6
    np.testing.assert_array_equal(results["simd"], results["scalar"])


def test_plan_in_tabletop(robot):
    env = ma.Environment.from_scene(TABLETOP)
    planner = ma.Planner(robot, env)
    assert planner.checker.startswith("simd-")
    result = planner.plan(ma.PANDA_READY, ABOVE_MUG_1)
    path = result.path
    assert path.shape[1] == 7
    np.testing.assert_allclose(path[0], ma.PANDA_READY, atol=1e-6)
    np.testing.assert_allclose(path[-1], ABOVE_MUG_1, atol=1e-6)
    assert result.length <= result.initial_length + 1e-6
    assert result.total_time >= 0.0
    dense = result.interpolate(0.05)
    assert np.all(np.linalg.norm(np.diff(dense, axis=0), axis=1) <= 0.05 + 1e-9)
    assert planner.configs_valid(dense).all()


def test_planned_paths_are_valid_and_deterministic(robot, cluttered):
    planner = ma.Planner(robot, cluttered)
    rng = np.random.default_rng(3)
    lo, hi = robot.lower_limits, robot.upper_limits
    samples = rng.uniform(lo, hi, size=(4000, 7))
    valid = samples[planner.configs_valid(samples)]
    solved = 0
    for i in range(8):
        start, goal = valid[2 * i], valid[2 * i + 1]
        try:
            a = planner.plan(start, goal, seed=i)
        except ma.PlanningError:
            continue
        b = planner.plan(start, goal, seed=i)
        np.testing.assert_array_equal(a.path, b.path)
        for q in a.path:
            assert planner.config_valid(q)
        for q0, q1 in zip(a.path[:-1], a.path[1:]):
            assert planner.motion_valid(q0, q1)
        solved += 1
    assert solved >= 6


def test_scalar_and_simd_give_identical_plans(robot, cluttered):
    simd = ma.Planner(robot, cluttered, checker="simd")
    scalar = ma.Planner(robot, cluttered, checker="scalar")
    portable = ma.Planner(robot, cluttered, checker="portable")
    rng = np.random.default_rng(7)
    samples = rng.uniform(robot.lower_limits, robot.upper_limits, size=(2000, 7))
    valid = samples[scalar.configs_valid(samples)]
    np.testing.assert_array_equal(scalar.configs_valid(samples), simd.configs_valid(samples))
    for i in range(4):
        start, goal = valid[2 * i], valid[2 * i + 1]
        ref = scalar.plan(start, goal, seed=i)
        for other in (simd, portable):
            np.testing.assert_array_equal(ref.path, other.plan(start, goal, seed=i).path)


def test_multiple_goals(robot):
    planner = ma.Planner(robot)
    goals = np.array([ABOVE_MUG_1, ABOVE_MUG_2])
    result = planner.plan(ma.PANDA_READY, goals)
    assert any(np.allclose(result.path[-1], g, atol=1e-6) for g in goals)


def test_invalid_queries_raise(robot, cluttered):
    planner = ma.Planner(robot, cluttered)
    with pytest.raises(ma.PlanningError, match="start"):
        planner.plan(np.zeros(7), ma.PANDA_READY)
    with pytest.raises(ValueError):
        ma.Planner(robot, cluttered, checker="gpu")


def test_pointcloud_blocks_motion(robot):
    # A dense wall of points in front of the robot.
    ys, zs = np.meshgrid(np.linspace(-0.6, 0.6, 61), np.linspace(0.0, 1.2, 61))
    points = np.stack([np.full(ys.size, 0.45), ys.ravel(), zs.ravel()], axis=1)
    env = ma.Environment()
    env.add_pointcloud(points, point_radius=0.01)
    planner = ma.Planner(robot, env)
    reach_through = [0.0, 0.9, 0.0, -0.6, 0.0, 1.5, 0.8]
    assert planner.config_valid(ma.PANDA_READY)
    assert not planner.config_valid(reach_through)


def test_planning_releases_the_gil(robot, cluttered):
    planner = ma.Planner(robot, cluttered)
    results = []

    def work(seed):
        results.append(planner.plan(ma.PANDA_READY, ABOVE_MUG_2, seed=seed).length)

    threads = [threading.Thread(target=work, args=(s,)) for s in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(results) == 4


def test_inverse_kinematics_roundtrip(robot):
    rng = np.random.default_rng(1)
    for _ in range(10):
        q = rng.uniform(robot.lower_limits, robot.upper_limits)
        target = robot.fk(q)
        sol = robot.ik(target, ma.PANDA_READY)
        assert sol is not None
        np.testing.assert_allclose(robot.fk(sol), target, atol=2e-3)
        assert robot.within_limits(sol)
    unreachable = np.eye(4)
    unreachable[0, 3] = 3.0
    assert robot.ik(unreachable, restarts=2) is None


def test_plan_to_pregrasp(robot):
    result = ma.plan_to_pregrasp(robot, TABLETOP, "mug_1", ma.PANDA_READY)
    tcp = robot.fk(result.goal)
    np.testing.assert_allclose(tcp[:3, 3], [0.4, -0.2, 0.6], atol=1e-3)
    assert tcp[2, 2] < -0.99  # tool points down
    np.testing.assert_allclose(result.plan.path[-1], result.goal, atol=1e-6)
    with pytest.raises(ma.PlanningError):
        ma.plan_to_pregrasp(robot, TABLETOP, "unicorn", ma.PANDA_READY)


def test_attached_object(robot):
    held = robot.with_attached("box", [[0.0, 0.0, 0.05, 0.03], [0.0, 0.0, 0.1, 0.03]])
    assert held.attached == ["box"]
    assert robot.attached == []
    assert held.num_spheres == robot.num_spheres + 2
    tcp = held.fk(ma.PANDA_READY)
    expected = tcp[:3, :3] @ [0.0, 0.0, 0.1] + tcp[:3, 3]
    np.testing.assert_allclose(held.spheres(ma.PANDA_READY)[-1, :3], expected, atol=1e-9)
    # A box hanging below the hand into an obstacle makes the configuration invalid.
    env = ma.Environment()
    env.add_sphere(expected, 0.02)
    assert ma.Planner(robot, env).config_valid(ma.PANDA_READY)
    assert not ma.Planner(held, env).config_valid(ma.PANDA_READY)
    assert held.without_attached("box").num_spheres == robot.num_spheres
    with pytest.raises(KeyError):
        robot.without_attached("box")
    with pytest.raises(ValueError):
        robot.with_attached("bad", [[0.0, 0.0, 0.0]])


def test_plan_pick(robot):
    result = ma.plan_pick(robot, TABLETOP, "mug_1", ma.PANDA_READY, place=ma.PANDA_READY)
    np.testing.assert_allclose(result.grasp_pose[:3, 3], [0.4, -0.2, 0.48], atol=1e-3)
    np.testing.assert_allclose(result.retreat[0], result.approach[-1])
    np.testing.assert_allclose(result.retreat[-1], result.approach[0])
    assert result.attached_spheres.shape[1] == 4
    assert result.place is not None
    np.testing.assert_allclose(result.place.path[-1], ma.PANDA_READY, atol=1e-6)
    with pytest.raises(ma.PlanningError):
        ma.plan_pick(robot, TABLETOP, "mug_2", ma.PANDA_READY)
