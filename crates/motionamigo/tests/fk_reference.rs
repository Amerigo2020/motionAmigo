//! Forward kinematics of the Panda against independent reference values.
#![allow(clippy::approx_constant, clippy::type_complexity)]

use motionamigo::kinematics::{frame_pose_f32, spheres_f32, CompiledRobot};
use motionamigo::math::Pose;
use motionamigo::robot::{RobotModel, PANDA_READY};
use proptest::prelude::*;
use std::f64::consts::FRAC_PI_2;

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
