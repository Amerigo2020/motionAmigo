//! Portable eight-lane vector built on `[f32; 8]`.
//!
//! Each operation is a plain loop over the lanes; the compiler is free to autovectorize it with
//! whatever instruction set the build targets. It is the fallback on every platform and the
//! baseline the architecture-specific backends are compared against.

use super::{Mask, Real, LANES};
use core::ops::{Add, BitAnd, BitOr, Mul, Neg, Sub};

/// Eight `f32` lanes stored as a plain array.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C, align(32))]
pub struct F32x8(pub [f32; LANES]);

/// Lane mask of [`F32x8`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct M8(pub [bool; LANES]);

#[inline(always)]
fn map2(a: [f32; LANES], b: [f32; LANES], f: impl Fn(f32, f32) -> f32) -> [f32; LANES] {
    let mut out = [0.0; LANES];
    for i in 0..LANES {
        out[i] = f(a[i], b[i]);
    }
    out
}

#[inline(always)]
fn cmp(a: [f32; LANES], b: [f32; LANES], f: impl Fn(f32, f32) -> bool) -> M8 {
    let mut out = [false; LANES];
    for i in 0..LANES {
        out[i] = f(a[i], b[i]);
    }
    M8(out)
}

impl Add for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn add(self, o: Self) -> Self {
        F32x8(map2(self.0, o.0, |a, b| a + b))
    }
}

impl Sub for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn sub(self, o: Self) -> Self {
        F32x8(map2(self.0, o.0, |a, b| a - b))
    }
}

impl Mul for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn mul(self, o: Self) -> Self {
        F32x8(map2(self.0, o.0, |a, b| a * b))
    }
}

impl Neg for F32x8 {
    type Output = Self;
    #[inline(always)]
    fn neg(self) -> Self {
        F32x8(self.0.map(|a| -a))
    }
}

impl BitOr for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitor(self, o: Self) -> Self {
        let mut out = self.0;
        for i in 0..LANES {
            out[i] |= o.0[i];
        }
        M8(out)
    }
}

impl BitAnd for M8 {
    type Output = Self;
    #[inline(always)]
    fn bitand(self, o: Self) -> Self {
        let mut out = self.0;
        for i in 0..LANES {
            out[i] &= o.0[i];
        }
        M8(out)
    }
}

impl Mask for M8 {
    #[inline(always)]
    fn any(self) -> bool {
        self.0.iter().any(|&b| b)
    }
    #[inline(always)]
    fn all(self) -> bool {
        self.0.iter().all(|&b| b)
    }
    #[inline(always)]
    fn none() -> Self {
        M8([false; LANES])
    }
    #[inline(always)]
    fn bitmask(self) -> u32 {
        self.0
            .iter()
            .enumerate()
            .fold(0, |acc, (i, &b)| acc | ((b as u32) << i))
    }
}

impl Real for F32x8 {
    type Mask = M8;
    const LANES: usize = LANES;

    #[inline(always)]
    fn splat(v: f32) -> Self {
        F32x8([v; LANES])
    }
    #[inline(always)]
    fn load(src: &[f32]) -> Self {
        let mut out = [0.0; LANES];
        out.copy_from_slice(&src[..LANES]);
        F32x8(out)
    }
    #[inline(always)]
    fn store(self, dst: &mut [f32]) {
        dst[..LANES].copy_from_slice(&self.0);
    }
    #[inline(always)]
    fn lane(self, i: usize) -> f32 {
        self.0[i]
    }
    #[inline(always)]
    fn min(self, o: Self) -> Self {
        F32x8(map2(self.0, o.0, |a, b| if a < b { a } else { b }))
    }
    #[inline(always)]
    fn max(self, o: Self) -> Self {
        F32x8(map2(self.0, o.0, |a, b| if a > b { a } else { b }))
    }
    #[inline(always)]
    fn abs(self) -> Self {
        F32x8(self.0.map(f32::abs))
    }
    #[inline(always)]
    fn round(self) -> Self {
        F32x8(self.0.map(f32::round_ties_even))
    }
    #[inline(always)]
    fn lt(self, o: Self) -> M8 {
        cmp(self.0, o.0, |a, b| a < b)
    }
    #[inline(always)]
    fn le(self, o: Self) -> M8 {
        cmp(self.0, o.0, |a, b| a <= b)
    }
    #[inline(always)]
    fn gt(self, o: Self) -> M8 {
        cmp(self.0, o.0, |a, b| a > b)
    }
    #[inline(always)]
    fn select(m: M8, a: Self, b: Self) -> Self {
        let mut out = b.0;
        for i in 0..LANES {
            if m.0[i] {
                out[i] = a.0[i];
            }
        }
        F32x8(out)
    }
}
