//! Forward kinematics of the Panda and the UR5 against independent reference values.
#![allow(clippy::approx_constant, clippy::type_complexity)]

use motionamigo::kinematics::{frame_pose_f32, spheres_f32, CompiledRobot};
use motionamigo::math::Pose;
use motionamigo::robot::{RobotModel, PANDA_READY, UR5_HOME};
use proptest::prelude::*;
use std::f64::consts::{FRAC_PI_2, PI};

/// Joint origins (xyz, rpy) of the official franka_description URDF. Each joint rotates about
/// its local z axis. This chain is built independently of the DH parameters in panda.toml.
const URDF_JOINTS: [([f64; 3], [f64; 3]); 7] = [
    ([0.0, 0.0, 0.333], [0.0, 0.0, 0.0]),
    ([0.0, 0.0, 0.0], [-FRAC_PI_2, 0.0, 0.0]),
    ([0.0, -0.316, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([0.0825, 0.0, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([-0.0825, 0.384, 0.0], [-FRAC_PI_2, 0.0, 0.0]),
    ([0.0, 0.0, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([0.088, 0.0, 0.0], [FRAC_PI_2, 0.0, 0.0]),
];

fn urdf_frames(q: &[f64]) -> Vec<Pose> {
    let mut out = vec![Pose::IDENTITY];
    for (i, (xyz, rpy)) in URDF_JOINTS.iter().enumerate() {
        let f = *out.last().unwrap() * Pose::from_xyz_rpy(*xyz, *rpy) * Pose::rot_z(q[i]);
        out.push(f);
    }
    out
}

fn assert_pose_close(a: &Pose, b: &Pose, tol: f64) {
    for i in 0..3 {
        assert!((a.trans[i] - b.trans[i]).abs() < tol, "{a:?} vs {b:?}");
        for j in 0..3 {
            assert!((a.rot[i][j] - b.rot[i][j]).abs() < tol, "{a:?} vs {b:?}");
        }
    }
}

#[test]
fn tcp_reference_values() {
    let r = RobotModel::panda();
    // Values computed with an independent numpy implementation of the URDF chain.
    let cases: [([f64; 7], [f64; 3], [[f64; 3]; 3]); 3] = [
        (
            [0.0; 7],
            [0.088, 0.0, 0.8226],
            [
                [0.7071067812, 0.7071067812, 0.0],
                [0.7071067812, -0.7071067812, 0.0],
                [0.0, 0.0, -1.0],
            ],
        ),
        (
            PANDA_READY,
            [0.3068905666, 0.0, 0.4868820523],
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
        ),
        (
            [0.3, -0.5, 0.7, -1.9, 0.4, 1.8, -0.6],
            [0.1599727838, 0.4738530916, 0.5677468001],
            [
                [-0.6990908112, 0.7082772729, 0.0980578524],
                [0.7018572191, 0.6535205653, 0.2833854524],
                [0.1366326523, 0.2669347774, -0.9539797393],
            ],
        ),
    ];
    for (q, t, rot) in cases {
        let p = r.tcp_pose(&q);
        assert_pose_close(&p, &Pose { rot, trans: t }, 1e-9);
    }
}

#[test]
fn flange_at_zero_configuration() {
    // The classic sanity check: with all joints at zero the flange is at (0.088, 0, 0.926).
    let r = RobotModel::panda();
    let flange = r.frames(&[0.0; 7])[7] * Pose::from_translation([0.0, 0.0, 0.107]);
    assert_pose_close(
        &Pose {
            rot: flange.rot,
            trans: [0.088, 0.0, 0.926],
        },
        &flange,
        1e-12,
    );
}

proptest! {
    #[test]
    fn dh_frames_match_urdf_chain(q in proptest::collection::vec(-3.2f64..3.8, 7)) {
        let r = RobotModel::panda();
        let dh = r.frames(&q);
        let urdf = urdf_frames(&q);
        for (a, b) in dh.iter().zip(&urdf) {
            assert_pose_close(a, b, 1e-9);
        }
    }

    #[test]
    fn f32_kernel_matches_f64(q in proptest::collection::vec(-3.2f64..3.8, 7)) {
        let r = RobotModel::panda();
        let c = CompiledRobot::new(&r);
        let q32: Vec<f32> = q.iter().map(|&v| v as f32).collect();
        let q_rounded: Vec<f64> = q32.iter().map(|&v| v as f64).collect();
        let frames = r.frames(&q_rounded);
        for (i, f) in frames.iter().enumerate() {
            assert_pose_close(&frame_pose_f32(&c, &q32, i), f, 2e-5);
        }
        // Sphere centers: the kernel orders links by frame, the panda.toml already is.
        let s64 = r.spheres_world(&q_rounded);
        let s32 = spheres_f32(&c, &q32);
        prop_assert_eq!(s64.len(), s32.len());
        for (a, b) in s64.iter().zip(&s32) {
            for k in 0..4 {
                prop_assert!((a[k] - b[k] as f64).abs() < 2e-5);
            }
        }
    }
}

/// Revolute joints of VAMP's ur5_spherized.urdf (classic ROS-Industrial ur_description):
/// origin xyz, origin rpy and whether the joint axis is local y (otherwise local z). The URDF
/// writes pi/2 as 1.570796325. Built independently of the DH parameters in ur5.toml.
const UR5_URDF_JOINTS: [([f64; 3], [f64; 3], bool); 6] = [
    ([0.0, 0.0, 0.089159], [0.0, 0.0, 0.0], false),
    ([0.0, 0.13585, 0.0], [0.0, 1.570796325, 0.0], true),
    ([0.0, -0.1197, 0.425], [0.0, 0.0, 0.0], true),
    ([0.0, 0.0, 0.39225], [0.0, 1.570796325, 0.0], true),
    ([0.0, 0.093, 0.0], [0.0, 0.0, 0.0], false),
    ([0.0, 0.0, 0.09465], [0.0, 0.0, 0.0], true),
];
/// Fixed joint from wrist_3_link to the tool0 flange frame.
const UR5_TOOL0: ([f64; 3], [f64; 3]) = ([0.0, 0.0823, 0.0], [-1.570796325, 0.0, 0.0]);

/// URDF link frames base_link, shoulder_link, ..., wrist_3_link, tool0 relative to base_link.
fn ur5_urdf_frames(q: &[f64]) -> Vec<Pose> {
    let mut out = vec![Pose::IDENTITY];
    for (i, (xyz, rpy, about_y)) in UR5_URDF_JOINTS.iter().enumerate() {
        let r = if *about_y {
            Pose::rot_y(q[i])
        } else {
            Pose::rot_z(q[i])
        };
        let f = *out.last().unwrap() * Pose::from_xyz_rpy(*xyz, *rpy) * r;
        out.push(f);
    }
    let tool0 = *out.last().unwrap() * Pose::from_xyz_rpy(UR5_TOOL0.0, UR5_TOOL0.1);
    out.push(tool0);
    out
}

/// DH frames of the model expressed in base_link (the DH base is base_link rotated by pi).
fn ur5_dh_in_base_link(r: &RobotModel, q: &[f64]) -> Vec<Pose> {
    r.frames(q).iter().map(|f| Pose::rot_z(PI) * *f).collect()
}

#[test]
fn ur5_tcp_reference_values() {
    let r = RobotModel::ur5();
    // Values computed with an independent Python implementation of the standard DH chain.
    let cases: [([f64; 6], [f64; 3], [[f64; 3]; 3]); 3] = [
        (
            [0.0; 6],
            [-0.81725, -0.19145, -0.005491],
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        ),
        (
            UR5_HOME,
            [-0.4869, -0.10915, 0.431859],
            [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
        ),
        (
            [0.4, -1.1, 1.3, -0.6, 0.9, -2.0],
            [-0.5578584101, -0.4099064016, 0.3279206056],
            [
                [-0.6725401443, 0.6076244128, -0.4224716882],
                [0.0695714483, -0.5164229674, -0.8535028602],
                [-0.7367832571, -0.6034069039, 0.3050418666],
            ],
        ),
    ];
    for (q, t, rot) in cases {
        assert_pose_close(&r.tcp_pose(&q), &Pose { rot, trans: t }, 1e-9);
    }
}

proptest! {
    /// Every URDF link frame is rigidly attached to its DH frame: the offset between them does
    /// not depend on the configuration, and the TCP coincides with the URDF tool0 frame. This is
    /// what tools/gen_ur5_toml.py relies on when it moves the spheres into the DH frames.
    #[test]
    fn ur5_dh_frames_match_urdf_chain(q in proptest::collection::vec(-PI..PI, 6)) {
        let r = RobotModel::ur5();
        let dh = ur5_dh_in_base_link(&r, &q);
        let dh0 = ur5_dh_in_base_link(&r, &[0.0; 6]);
        let urdf = ur5_urdf_frames(&q);
        let urdf0 = ur5_urdf_frames(&[0.0; 6]);
        for i in 0..=6 {
            let offset = dh[i].inverse() * urdf[i];
            let offset0 = dh0[i].inverse() * urdf0[i];
            assert_pose_close(&offset, &offset0, 1e-7);
        }
        let tcp = Pose::rot_z(PI) * r.tcp_pose(&q);
        assert_pose_close(&tcp, &urdf[7], 1e-7);
    }

    #[test]
    fn ur5_f32_kernel_matches_f64(q in proptest::collection::vec(-PI..PI, 6)) {
        let r = RobotModel::ur5();
        let c = CompiledRobot::new(&r);
        let q32: Vec<f32> = q.iter().map(|&v| v as f32).collect();
        let q_rounded: Vec<f64> = q32.iter().map(|&v| v as f64).collect();
        for (i, f) in r.frames(&q_rounded).iter().enumerate() {
            assert_pose_close(&frame_pose_f32(&c, &q32, i), f, 2e-5);
        }
        let s64 = r.spheres_world(&q_rounded);
        let s32 = spheres_f32(&c, &q32);
        prop_assert_eq!(s64.len(), s32.len());
        for (a, b) in s64.iter().zip(&s32) {
            for k in 0..4 {
                prop_assert!((a[k] - b[k] as f64).abs() < 2e-5);
            }
        }
    }
}

#[test]
fn ur5_spheres_follow_urdf_links() {
    // The first upper_arm_link sphere sits at (0, 0, 0.105) in the URDF link frame.
    let r = RobotModel::ur5();
    let q = [0.4, -1.1, 1.3, -0.6, 0.9, -2.0];
    let expected = ur5_urdf_frames(&q)[2].transform_point([0.0, 0.0, 0.105]);
    let base_link = Pose::rot_z(PI);
    let found = r.spheres_world(&q).iter().any(|s| {
        let p = base_link.transform_point([s[0], s[1], s[2]]);
        (0..3).all(|k| (p[k] - expected[k]).abs() < 1e-6)
    });
    assert!(found, "no sphere at {expected:?}");
}
