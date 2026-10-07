//! Wall-clock timing that compiles on every target.
//!
//! `std::time::Instant` panics on `wasm32-unknown-unknown`, so on that target the stopwatch
//! reports zero and the WASM bindings measure time with `performance.now()` instead.

use core::time::Duration;

/// A started stopwatch.
#[derive(Debug, Clone, Copy)]
pub struct Stopwatch {
    #[cfg(not(target_arch = "wasm32"))]
    start: std::time::Instant,
}

impl Stopwatch {
    /// Starts a stopwatch.
    pub fn start() -> Stopwatch {
        Stopwatch {
            #[cfg(not(target_arch = "wasm32"))]
            start: std::time::Instant::now(),
        }
    }

    /// Time since [`Stopwatch::start`] (always zero on wasm32).
    pub fn elapsed(&self) -> Duration {
        #[cfg(not(target_arch = "wasm32"))]
        return self.start.elapsed();
        #[cfg(target_arch = "wasm32")]
        Duration::ZERO
    }
}
