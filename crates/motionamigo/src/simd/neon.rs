//! NEON backend for aarch64 (eight `f32` lanes in two 128-bit registers).
//!
//! NEON is part of the aarch64 baseline, so no runtime detection is needed. Minimum and maximum
//! are implemented as compare plus select (not `fmin`/`fmax`), so that signed zeros and NaNs are
//! handled exactly like the scalar reference.

use super::{Mask, Real, LANES};
use core::arch::aarch64::*;
use core::ops::{Add, BitAnd, BitOr, Mul, Neg, Sub};

/// Eight `f32` lanes in two NEON registers.
#[derive(Clone, Copy, Debug)]
pub struct F32x8(float32x4_t, float32x4_t);

/// Lane mask of [`F32x8`].
#[derive(Clone, Copy, Debug)]
pub struct M8(uint32x4_t, uint32x4_t);

// SAFETY (for every `unsafe` block below): NEON is always available on aarch64.

macro_rules! lanewise {
    ($a:expr, $b:expr, $f:ident) => {
        unsafe { F32x8($f($a.0, $b.0), $f($a.1, $b.1)) }
    };
}

impl Add for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn add(self, o: Self) -> Self {
        lanewise!(self, o, vaddq_f32)
    }
}

impl Sub for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn sub(self, o: Self) -> Self {
        lanewise!(self, o, vsubq_f32)
    }
}

impl Mul for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, o: Self) -> Self {
        lanewise!(self, o, vmulq_f32)
    }
}

impl Neg for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self {
        unsafe { F32x8(vnegq_f32(self.0), vnegq_f32(self.1)) }
    }
}

impl BitOr for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitor(self, o: Self) -> Self {
        unsafe { M8(vorrq_u32(self.0, o.0), vorrq_u32(self.1, o.1)) }
    }
}

impl BitAnd for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitand(self, o: Self) -> Self {
        unsafe { M8(vandq_u32(self.0, o.0), vandq_u32(self.1, o.1)) }
    }
}

impl Mask for M8 {
    #[inline(always)]
    fn any(self) -> bool {
        unsafe { vmaxvq_u32(vorrq_u32(self.0, self.1)) != 0 }
    }
    #[inline(always)]
    fn all(self) -> bool {
        unsafe { vminvq_u32(vandq_u32(self.0, self.1)) == u32::MAX }
    }
    #[inline(always)]
    fn none() -> Self {
        unsafe { M8(vdupq_n_u32(0), vdupq_n_u32(0)) }
    }
    #[inline(always)]
    fn bitmask(self) -> u32 {
        let mut lanes = [0u32; LANES];
        unsafe {
            vst1q_u32(lanes.as_mut_ptr(), self.0);
            vst1q_u32(lanes.as_mut_ptr().add(4), self.1);
        }
        lanes
            .iter()
            .enumerate()
            .fold(0, |acc, (i, &v)| acc | (((v != 0) as u32) << i))
    }
}

impl Real for F32x8 {
    type Mask = M8;
    const LANES: usize = LANES;

    #[inline(always)]
    fn splat(v: f32) -> Self {
        unsafe { F32x8(vdupq_n_f32(v), vdupq_n_f32(v)) }
    }
    #[inline(always)]
    fn load(src: &[f32]) -> Self {
        assert!(src.len() >= LANES);
        // SAFETY: the slice holds at least eight floats.
        unsafe { F32x8(vld1q_f32(src.as_ptr()), vld1q_f32(src.as_ptr().add(4))) }
    }
    #[inline(always)]
    fn store(self, dst: &mut [f32]) {
        assert!(dst.len() >= LANES);
        // SAFETY: the slice has room for eight floats.
        unsafe {
            vst1q_f32(dst.as_mut_ptr(), self.0);
            vst1q_f32(dst.as_mut_ptr().add(4), self.1);
        }
    }
    #[inline(always)]
    fn lane(self, i: usize) -> f32 {
        let mut out = [0.0f32; LANES];
        self.store(&mut out);
        out[i]
    }
    #[inline(always)]
    fn min(self, o: Self) -> Self {
        Self::select(self.lt(o), self, o)
    }
    #[inline(always)]
    fn max(self, o: Self) -> Self {
        Self::select(self.gt(o), self, o)
    }
    #[inline(always)]
    fn abs(self) -> Self {
        unsafe { F32x8(vabsq_f32(self.0), vabsq_f32(self.1)) }
    }
    #[inline(always)]
    fn round(self) -> Self {
        unsafe { F32x8(vrndnq_f32(self.0), vrndnq_f32(self.1)) }
    }
    #[inline(always)]
    fn lt(self, o: Self) -> M8 {
        unsafe { M8(vcltq_f32(self.0, o.0), vcltq_f32(self.1, o.1)) }
    }
    #[inline(always)]
    fn le(self, o: Self) -> M8 {
        unsafe { M8(vcleq_f32(self.0, o.0), vcleq_f32(self.1, o.1)) }
    }
    #[inline(always)]
    fn gt(self, o: Self) -> M8 {
        unsafe { M8(vcgtq_f32(self.0, o.0), vcgtq_f32(self.1, o.1)) }
    }
    #[inline(always)]
    fn select(m: M8, a: Self, b: Self) -> Self {
        unsafe { F32x8(vbslq_f32(m.0, a.0, b.0), vbslq_f32(m.1, a.1, b.1)) }
    }
}
