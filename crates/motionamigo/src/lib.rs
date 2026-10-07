//! motionAmigo: fast sampling-based motion planning for robot arms.
#![forbid(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]
// Lane loops index several arrays in lockstep; iterators would obscure that.
#![allow(clippy::needless_range_loop)]

pub mod kinematics;
pub mod math;
pub mod robot;
pub mod scene;
pub mod simd;
