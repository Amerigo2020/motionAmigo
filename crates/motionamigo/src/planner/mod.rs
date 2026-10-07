//! Sampling-based planning: RRT-Connect and path simplification.

pub mod rrtc;
pub mod simplify;

/// Euclidean (L2) distance between two configurations.
#[inline]
pub fn distance(a: &[f32], b: &[f32]) -> f32 {
    let mut s = 0.0f32;
    for (x, y) in a.iter().zip(b) {
        let d = x - y;
        s += d * d;
    }
    s.sqrt()
}

/// Joint-space (L2) length of a path.
pub fn path_length(path: &[Vec<f32>]) -> f32 {
    path.windows(2).map(|w| distance(&w[0], &w[1])).sum()
}
