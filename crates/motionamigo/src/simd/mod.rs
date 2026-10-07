//! A small SIMD abstraction.
//!
//! All geometric kernels in this crate (forward kinematics, sphere tests) are written once,
//! generically over the [`Real`] trait. Instantiating them with plain `f32` gives the scalar
//! reference implementation that checks one configuration at a time; instantiating them with an
//! eight-lane vector type gives the vectorized implementation that checks eight configurations at
//! once in structure-of-arrays layout.
//!
//! Every backend only uses IEEE-754 operations that are exactly rounded (add, sub, mul, min, max,
//! abs, compare, round-to-nearest-even). Fused multiply-add is deliberately not used. As a
//! consequence the scalar reference and all SIMD backends produce bit-identical results, which the
//! equivalence tests verify, and a plan computed in the browser equals the plan computed natively.
//!
//! Backends:
//!
//! | type                   | target                    | instructions          |
//! |------------------------|---------------------------|-----------------------|
//! | `f32`                  | any                       | scalar reference      |
//! | [`portable::F32x8`]    | any                       | `[f32; 8]`, autovectorized |
//! | `x86::F32x8` (internal) | x86_64, runtime detected | AVX2 (256 bit)        |
//! | `neon::F32x8`          | aarch64                   | NEON (2 x 128 bit)    |
//! | `wasm::F32x8`          | wasm32 with `simd128`     | WASM SIMD (2 x 128 bit) |

use core::ops::{Add, BitAnd, BitOr, Mul, Neg, Sub};

pub mod portable;
pub(crate) mod stack;

#[cfg(target_arch = "x86_64")]
pub(crate) mod x86;

#[cfg(target_arch = "aarch64")]
pub mod neon;

#[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
pub mod wasm;

/// Number of lanes of the vectorized backends (and therefore the rake width).
pub const LANES: usize = 8;

/// A lane-wise boolean mask produced by comparisons on a [`Real`].
pub trait Mask: Copy + BitOr<Output = Self> + BitAnd<Output = Self> {
    /// True if at least one lane is set.
    fn any(self) -> bool;
    /// True if every lane is set.
    fn all(self) -> bool;
    /// Mask with every lane cleared.
    fn none() -> Self;
    /// Bit `i` of the result is lane `i` of the mask.
    fn bitmask(self) -> u32;
}

/// A vector of `f32` lanes (or a single `f32`) supporting the operations needed by the kernels.
///
/// Implementations must be exactly rounded and must not fuse operations, so that every
/// implementation computes bit-identical lane values.
pub trait Real:
    Copy
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Neg<Output = Self>
    + core::fmt::Debug
{
    /// Comparison mask type.
    type Mask: Mask;
    /// Number of lanes.
    const LANES: usize;

    /// Broadcasts a scalar to all lanes.
    fn splat(v: f32) -> Self;
    /// Loads `LANES` values from the start of `src`.
    fn load(src: &[f32]) -> Self;
    /// Stores all lanes to the start of `dst`.
    fn store(self, dst: &mut [f32]);
    /// Returns lane `i`.
    fn lane(self, i: usize) -> f32;
    /// Lane-wise minimum with x86 semantics: `if self < o { self } else { o }`.
    fn min(self, o: Self) -> Self;
    /// Lane-wise maximum with x86 semantics: `if self > o { self } else { o }`.
    fn max(self, o: Self) -> Self;
    /// Lane-wise absolute value.
    fn abs(self) -> Self;
    /// Lane-wise rounding to the nearest integer, ties to even.
    fn round(self) -> Self;
    /// Lane-wise `self < o`.
    fn lt(self, o: Self) -> Self::Mask;
    /// Lane-wise `self <= o`.
    fn le(self, o: Self) -> Self::Mask;
    /// Lane-wise `self > o`.
    fn gt(self, o: Self) -> Self::Mask;
    /// Lane-wise `if m { a } else { b }`.
    fn select(m: Self::Mask, a: Self, b: Self) -> Self;

    /// Lane-wise clamp to `[lo, hi]`.
    #[inline(always)]
    fn clamp(self, lo: Self, hi: Self) -> Self {
        self.max(lo).min(hi)
    }
}

impl Mask for bool {
    #[inline(always)]
    fn any(self) -> bool {
        self
    }
    #[inline(always)]
    fn all(self) -> bool {
        self
    }
    #[inline(always)]
    fn none() -> Self {
        false
    }
    #[inline(always)]
    fn bitmask(self) -> u32 {
        self as u32
    }
}

/// The scalar reference "vector" with a single lane.
impl Real for f32 {
    type Mask = bool;
    const LANES: usize = 1;

    #[inline(always)]
    fn splat(v: f32) -> Self {
        v
    }
    #[inline(always)]
    fn load(src: &[f32]) -> Self {
        src[0]
    }
    #[inline(always)]
    fn store(self, dst: &mut [f32]) {
        dst[0] = self;
    }
    #[inline(always)]
    fn lane(self, _i: usize) -> f32 {
        self
    }
    #[inline(always)]
    fn min(self, o: Self) -> Self {
        if self < o {
            self
        } else {
            o
        }
    }
    #[inline(always)]
    fn max(self, o: Self) -> Self {
        if self > o {
            self
        } else {
            o
        }
    }
    #[inline(always)]
    fn abs(self) -> Self {
        f32::abs(self)
    }
    #[inline(always)]
    fn round(self) -> Self {
        self.round_ties_even()
    }
    #[inline(always)]
    fn lt(self, o: Self) -> bool {
        self < o
    }
    #[inline(always)]
    fn le(self, o: Self) -> bool {
        self <= o
    }
    #[inline(always)]
    fn gt(self, o: Self) -> bool {
        self > o
    }
    #[inline(always)]
    fn select(m: bool, a: Self, b: Self) -> Self {
        if m {
            a
        } else {
            b
        }
    }
}

/// Which vectorized backend is used by the SIMD collision checker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Backend {
    /// AVX2 on x86_64 (selected at runtime if the CPU supports it).
    Avx2,
    /// NEON on aarch64.
    Neon,
    /// WebAssembly 128-bit SIMD.
    Wasm128,
    /// Portable `[f32; 8]` arrays, left to the compiler's autovectorizer.
    Portable,
}

impl Backend {
    /// The best backend available on this machine.
    ///
    /// Setting the environment variable `MOTIONAMIGO_SIMD=portable` forces the portable backend
    /// (used by CI to test it on every platform).
    pub fn detect() -> Backend {
        #[cfg(not(target_arch = "wasm32"))]
        if std::env::var("MOTIONAMIGO_SIMD").is_ok_and(|v| v == "portable") {
            return Backend::Portable;
        }
        Self::detect_native()
    }

    #[allow(unreachable_code)]
    fn detect_native() -> Backend {
        #[cfg(target_arch = "x86_64")]
        if x86::available() {
            return Backend::Avx2;
        }
        #[cfg(target_arch = "aarch64")]
        return Backend::Neon;
        #[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
        return Backend::Wasm128;
        Backend::Portable
    }

    /// All backends usable on this machine, best first.
    pub fn available() -> Vec<Backend> {
        let mut out = Vec::new();
        let native = Self::detect_native();
        if native != Backend::Portable {
            out.push(native);
        }
        out.push(Backend::Portable);
        out
    }

    /// Human-readable backend name.
    pub fn name(self) -> &'static str {
        match self {
            Backend::Avx2 => "avx2",
            Backend::Neon => "neon",
            Backend::Wasm128 => "wasm-simd128",
            Backend::Portable => "portable",
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Runs a battery of lane-wise checks against the scalar `f32` implementation.
    pub fn check_backend<R: Real>() {
        let a: Vec<f32> = (0..R::LANES)
            .map(|i| (i as f32 - 3.3) * 1.7 + 0.5)
            .collect();
        let b: Vec<f32> = (0..R::LANES).map(|i| 2.5 - i as f32 * 0.9).collect();
        let va = R::load(&a);
        let vb = R::load(&b);
        for i in 0..R::LANES {
            let (x, y) = (a[i], b[i]);
            assert_eq!((va + vb).lane(i), x + y);
            assert_eq!((va - vb).lane(i), x - y);
            assert_eq!((va * vb).lane(i), x * y);
            assert_eq!((-va).lane(i), -x);
            assert_eq!(va.min(vb).lane(i), Real::min(x, y));
            assert_eq!(va.max(vb).lane(i), Real::max(x, y));
            assert_eq!(va.abs().lane(i), x.abs());
            assert_eq!(va.round().lane(i), x.round_ties_even());
            assert_eq!((va.lt(vb).bitmask() >> i) & 1 == 1, x < y, "lt lane {i}");
            assert_eq!((va.le(vb).bitmask() >> i) & 1 == 1, x <= y);
            assert_eq!((va.gt(vb).bitmask() >> i) & 1 == 1, x > y);
            let s = R::select(va.lt(vb), va, vb);
            assert_eq!(s.lane(i), if x < y { x } else { y });
        }
        // Ties round to even.
        let halves: Vec<f32> = (0..R::LANES).map(|i| i as f32 + 0.5).collect();
        let r = R::load(&halves).round();
        for (i, h) in halves.iter().enumerate() {
            assert_eq!(r.lane(i), h.round_ties_even());
        }
        let mut out = vec![0.0; R::LANES];
        va.store(&mut out);
        assert_eq!(out, a);
        assert!(!R::splat(1.0).lt(R::splat(0.0)).any());
        assert!(R::splat(0.0).lt(R::splat(1.0)).all());
        assert!(!<R::Mask as Mask>::none().any());
    }

    #[test]
    fn scalar_backend() {
        check_backend::<f32>();
    }

    #[test]
    fn portable_backend() {
        check_backend::<portable::F32x8>();
    }

    #[cfg(target_arch = "x86_64")]
    #[test]
    fn avx2_backend() {
        if x86::available() {
            x86::with_avx2(check_backend::<x86::F32x8>);
        }
    }

    #[cfg(target_arch = "aarch64")]
    #[test]
    fn neon_backend() {
        check_backend::<neon::F32x8>();
    }

    #[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
    #[test]
    fn wasm_backend() {
        check_backend::<wasm::F32x8>();
    }
}
