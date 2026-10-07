//! AVX2 backend for x86_64 (eight `f32` lanes in one 256-bit register).
//!
//! The backend is selected at runtime. Values of [`F32x8`] must only be created and used after
//! [`available`] returned true; the type is crate-private, and the only entry points are
//! [`motion_valid`] and [`with_avx2`], which are compiled with AVX2 enabled so that the generic
//! kernels inline into AVX2 code.
//!
//! This module is one of the two places in the crate that contain `unsafe` (the other being the
//! NEON backend): calling a `#[target_feature]` intrinsic from code that is not itself compiled
//! with that feature is unsafe, because executing it on a CPU without the feature is undefined
//! behaviour. Every such call is guarded by the runtime check described above.

use super::{Mask, Real, LANES};
use crate::checker::{block_in_collision as block_generic, motion_valid_rake};
use crate::collision::EnvBlock;
use crate::kinematics::CompiledRobot;
use core::arch::x86_64::*;
use core::ops::{Add, BitAnd, BitOr, Mul, Neg, Sub};

/// True if the CPU supports AVX2.
pub fn available() -> bool {
    std::is_x86_feature_detected!("avx2")
}

/// Eight `f32` lanes in an AVX register.
#[derive(Clone, Copy, Debug)]
#[repr(transparent)]
pub struct F32x8(__m256);

/// Lane mask of [`F32x8`] (all bits set in a lane means true).
#[derive(Clone, Copy, Debug)]
#[repr(transparent)]
pub struct M8(__m256);

// SAFETY (for every `unsafe` block below): the intrinsics require AVX, which is implied by AVX2.
// Values of these types only exist after `available()` returned true (see the module docs).

impl Add for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn add(self, o: Self) -> Self {
        F32x8(unsafe { _mm256_add_ps(self.0, o.0) })
    }
}

impl Sub for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn sub(self, o: Self) -> Self {
        F32x8(unsafe { _mm256_sub_ps(self.0, o.0) })
    }
}

impl Mul for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, o: Self) -> Self {
        F32x8(unsafe { _mm256_mul_ps(self.0, o.0) })
    }
}

impl Neg for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self {
        F32x8(unsafe { _mm256_xor_ps(self.0, _mm256_set1_ps(-0.0)) })
    }
}

impl BitOr for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitor(self, o: Self) -> Self {
        M8(unsafe { _mm256_or_ps(self.0, o.0) })
    }
}

impl BitAnd for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitand(self, o: Self) -> Self {
        M8(unsafe { _mm256_and_ps(self.0, o.0) })
    }
}

impl Mask for M8 {
    #[inline(always)]
    fn any(self) -> bool {
        unsafe { _mm256_movemask_ps(self.0) != 0 }
    }
    #[inline(always)]
    fn all(self) -> bool {
        unsafe { _mm256_movemask_ps(self.0) == 0xff }
    }
    #[inline(always)]
    fn none() -> Self {
        M8(unsafe { _mm256_setzero_ps() })
    }
    #[inline(always)]
    fn bitmask(self) -> u32 {
        unsafe { _mm256_movemask_ps(self.0) as u32 }
    }
}

impl Real for F32x8 {
    type Mask = M8;
    const LANES: usize = LANES;

    #[inline(always)]
    fn splat(v: f32) -> Self {
        F32x8(unsafe { _mm256_set1_ps(v) })
    }
    #[inline(always)]
    fn load(src: &[f32]) -> Self {
        assert!(src.len() >= LANES);
        // SAFETY: the slice holds at least eight floats; the load is unaligned.
        F32x8(unsafe { _mm256_loadu_ps(src.as_ptr()) })
    }
    #[inline(always)]
    fn store(self, dst: &mut [f32]) {
        assert!(dst.len() >= LANES);
        // SAFETY: the slice has room for eight floats; the store is unaligned.
        unsafe { _mm256_storeu_ps(dst.as_mut_ptr(), self.0) }
    }
    #[inline(always)]
    fn lane(self, i: usize) -> f32 {
        let mut out = [0.0f32; LANES];
        self.store(&mut out);
        out[i]
    }
    #[inline(always)]
    fn min(self, o: Self) -> Self {
        // `minps` returns the second operand unless `self < o`, matching the scalar reference.
        F32x8(unsafe { _mm256_min_ps(self.0, o.0) })
    }
    #[inline(always)]
    fn max(self, o: Self) -> Self {
        F32x8(unsafe { _mm256_max_ps(self.0, o.0) })
    }
    #[inline(always)]
    fn abs(self) -> Self {
        F32x8(unsafe { _mm256_andnot_ps(_mm256_set1_ps(-0.0), self.0) })
    }
    #[inline(always)]
    fn round(self) -> Self {
        F32x8(unsafe {
            _mm256_round_ps::<{ _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC }>(self.0)
        })
    }
    #[inline(always)]
    fn lt(self, o: Self) -> M8 {
        M8(unsafe { _mm256_cmp_ps::<_CMP_LT_OQ>(self.0, o.0) })
    }
    #[inline(always)]
    fn le(self, o: Self) -> M8 {
        M8(unsafe { _mm256_cmp_ps::<_CMP_LE_OQ>(self.0, o.0) })
    }
    #[inline(always)]
    fn gt(self, o: Self) -> M8 {
        M8(unsafe { _mm256_cmp_ps::<_CMP_GT_OQ>(self.0, o.0) })
    }
    #[inline(always)]
    fn select(m: M8, a: Self, b: Self) -> Self {
        F32x8(unsafe { _mm256_blendv_ps(b.0, a.0, m.0) })
    }
}

#[target_feature(enable = "avx2")]
fn motion_valid_avx2(
    robot: &CompiledRobot,
    env: &EnvBlock<F32x8>,
    a: &[f32],
    b: &[f32],
    resolution: f32,
) -> bool {
    motion_valid_rake(robot, env, a, b, resolution)
}

/// Raked edge validation compiled for AVX2.
///
/// # Panics
/// Panics if the CPU does not support AVX2.
#[inline]
pub(crate) fn motion_valid(
    robot: &CompiledRobot,
    env: &EnvBlock<F32x8>,
    a: &[f32],
    b: &[f32],
    resolution: f32,
) -> bool {
    assert!(available());
    // SAFETY: AVX2 support was just verified.
    unsafe { motion_valid_avx2(robot, env, a, b, resolution) }
}

#[target_feature(enable = "avx2")]
fn block_in_collision_avx2(
    robot: &CompiledRobot,
    env: &EnvBlock<F32x8>,
    block: &[[f32; LANES]],
) -> bool {
    block_generic(robot, env, block)
}

/// Block collision check compiled for AVX2.
///
/// # Panics
/// Panics if the CPU does not support AVX2.
#[inline]
pub(crate) fn block_in_collision(
    robot: &CompiledRobot,
    env: &EnvBlock<F32x8>,
    block: &[[f32; LANES]],
) -> bool {
    assert!(available());
    // SAFETY: AVX2 support was just verified.
    unsafe { block_in_collision_avx2(robot, env, block) }
}

#[target_feature(enable = "avx2")]
fn call_avx2<T>(f: impl FnOnce() -> T) -> T {
    f()
}

/// Runs `f` in a function compiled with AVX2 enabled.
///
/// # Panics
/// Panics if the CPU does not support AVX2.
pub fn with_avx2<T>(f: impl FnOnce() -> T) -> T {
    assert!(available());
    // SAFETY: AVX2 support was just verified.
    unsafe { call_avx2(f) }
}
