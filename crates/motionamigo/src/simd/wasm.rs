//! WebAssembly SIMD backend (eight `f32` lanes in two `v128` values).
//!
//! Requires building with `-C target-feature=+simd128` (set in `.cargo/config.toml`). Minimum and
//! maximum are compare plus select, matching the scalar reference bit for bit.

use super::{Mask, Real, LANES};
use core::arch::wasm32::*;
use core::ops::{Add, BitAnd, BitOr, Mul, Neg, Sub};

/// Eight `f32` lanes in two `v128` values.
#[derive(Clone, Copy, Debug)]
pub struct F32x8(v128, v128);

/// Lane mask of [`F32x8`].
#[derive(Clone, Copy, Debug)]
pub struct M8(v128, v128);

impl Add for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn add(self, o: Self) -> Self {
        F32x8(f32x4_add(self.0, o.0), f32x4_add(self.1, o.1))
    }
}

impl Sub for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn sub(self, o: Self) -> Self {
        F32x8(f32x4_sub(self.0, o.0), f32x4_sub(self.1, o.1))
    }
}

impl Mul for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, o: Self) -> Self {
        F32x8(f32x4_mul(self.0, o.0), f32x4_mul(self.1, o.1))
    }
}

impl Neg for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self {
        F32x8(f32x4_neg(self.0), f32x4_neg(self.1))
    }
}

impl BitOr for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitor(self, o: Self) -> Self {
        M8(v128_or(self.0, o.0), v128_or(self.1, o.1))
    }
}

impl BitAnd for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitand(self, o: Self) -> Self {
        M8(v128_and(self.0, o.0), v128_and(self.1, o.1))
    }
}

impl Mask for M8 {
    #[inline(always)]
    fn any(self) -> bool {
        v128_any_true(v128_or(self.0, self.1))
    }
    #[inline(always)]
    fn all(self) -> bool {
        i32x4_all_true(v128_and(self.0, self.1))
    }
    #[inline(always)]
    fn none() -> Self {
        M8(i32x4_splat(0), i32x4_splat(0))
    }
    #[inline(always)]
    fn bitmask(self) -> u32 {
        (i32x4_bitmask(self.0) as u32) | ((i32x4_bitmask(self.1) as u32) << 4)
    }
}

impl Real for F32x8 {
    type Mask = M8;
    const LANES: usize = LANES;

    #[inline(always)]
    fn splat(v: f32) -> Self {
        F32x8(f32x4_splat(v), f32x4_splat(v))
    }
    #[inline(always)]
    fn load(src: &[f32]) -> Self {
        F32x8(
            f32x4(src[0], src[1], src[2], src[3]),
            f32x4(src[4], src[5], src[6], src[7]),
        )
    }
    #[inline(always)]
    fn store(self, dst: &mut [f32]) {
        dst[0] = f32x4_extract_lane::<0>(self.0);
        dst[1] = f32x4_extract_lane::<1>(self.0);
        dst[2] = f32x4_extract_lane::<2>(self.0);
        dst[3] = f32x4_extract_lane::<3>(self.0);
        dst[4] = f32x4_extract_lane::<0>(self.1);
        dst[5] = f32x4_extract_lane::<1>(self.1);
        dst[6] = f32x4_extract_lane::<2>(self.1);
        dst[7] = f32x4_extract_lane::<3>(self.1);
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
        F32x8(f32x4_abs(self.0), f32x4_abs(self.1))
    }
    #[inline(always)]
    fn round(self) -> Self {
        F32x8(f32x4_nearest(self.0), f32x4_nearest(self.1))
    }
    #[inline(always)]
    fn lt(self, o: Self) -> M8 {
        M8(f32x4_lt(self.0, o.0), f32x4_lt(self.1, o.1))
    }
    #[inline(always)]
    fn le(self, o: Self) -> M8 {
        M8(f32x4_le(self.0, o.0), f32x4_le(self.1, o.1))
    }
    #[inline(always)]
    fn gt(self, o: Self) -> M8 {
        M8(f32x4_gt(self.0, o.0), f32x4_gt(self.1, o.1))
    }
    #[inline(always)]
    fn select(m: M8, a: Self, b: Self) -> Self {
        F32x8(v128_bitselect(a.0, b.0, m.0), v128_bitselect(a.1, b.1, m.1))
    }
}
