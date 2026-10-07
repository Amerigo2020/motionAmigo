//! Path simplification: greedy vertex shortcutting and randomized partial shortcuts.

use super::{distance, path_length};
use crate::checker::CollisionChecker;
use crate::rng::Rng;

/// Settings of [`simplify`].
#[derive(Debug, Clone, PartialEq)]
pub struct SimplifySettings {
    /// Maximum number of rounds (greedy pass plus random pass).
    pub max_rounds: usize,
    /// Random partial shortcut attempts per round.
    pub random_attempts: usize,
    /// Stop when a round shortens the path by less than this fraction.
    pub min_improvement: f32,
}

impl Default for SimplifySettings {
    fn default() -> Self {
        SimplifySettings {
            max_rounds: 3,
            random_attempts: 32,
            min_improvement: 1e-3,
        }
    }
}

impl SimplifySettings {
    /// Settings that leave the path untouched.
    pub fn disabled() -> Self {
        SimplifySettings {
            max_rounds: 0,
            random_attempts: 0,
            min_improvement: 0.0,
        }
    }
}

/// Greedy shortcutting: from each vertex, connect directly to the farthest later vertex that is
/// reachable in a straight line. Returns true if the path changed.
pub fn shortcut_greedy<C: CollisionChecker + ?Sized>(
    path: &mut Vec<Vec<f32>>,
    checker: &C,
) -> bool {
    if path.len() < 3 {
        return false;
    }
    let mut changed = false;
    let mut i = 0;
    while i + 2 < path.len() {
        let mut j = path.len() - 1;
        while j > i + 1 {
            if checker.motion_valid(&path[i], &path[j]) {
                path.drain(i + 1..j);
                changed = true;
                break;
            }
            j -= 1;
        }
        i += 1;
    }
    changed
}

/// Point at arc-length parameter `s` along the path: (segment index, configuration).
fn point_at(path: &[Vec<f32>], cumulative: &[f32], s: f32) -> (usize, Vec<f32>) {
    let seg = cumulative
        .windows(2)
        .position(|w| s <= w[1])
        .unwrap_or(path.len() - 2);
    let len = cumulative[seg + 1] - cumulative[seg];
    let t = if len > 0.0 {
        ((s - cumulative[seg]) / len).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (a, b) = (&path[seg], &path[seg + 1]);
    (seg, a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect())
}

/// Randomized partial shortcuts: pick two random points on the path and replace the part in
/// between by a straight segment if that is valid and shorter. Returns true if the path changed.
pub fn shortcut_random<C: CollisionChecker + ?Sized>(
    path: &mut Vec<Vec<f32>>,
    checker: &C,
    rng: &mut Rng,
    attempts: usize,
) -> bool {
    let mut changed = false;
    for _ in 0..attempts {
        if path.len() < 3 {
            break;
        }
        let mut cumulative = Vec::with_capacity(path.len());
        cumulative.push(0.0f32);
        for w in path.windows(2) {
            let last = *cumulative.last().unwrap();
            cumulative.push(last + distance(&w[0], &w[1]));
        }
        let total = *cumulative.last().unwrap();
        let (mut s1, mut s2) = (rng.next_f32() * total, rng.next_f32() * total);
        if s1 > s2 {
            core::mem::swap(&mut s1, &mut s2);
        }
        let (i, p1) = point_at(path, &cumulative, s1);
        let (j, p2) = point_at(path, &cumulative, s2);
        if i == j {
            continue;
        }
        let old =
            (cumulative[i + 1] - s1) + (cumulative[j] - cumulative[i + 1]) + (s2 - cumulative[j]);
        let new = distance(&p1, &p2);
        if new >= old * 0.999 {
            continue;
        }
        // The two cut points and the partial edges to them are new and must be checked too
        // (`p2` is the end point of the shortcut edge and therefore already checked).
        // The shortcut itself is the check most likely to fail, so it goes first.
        if !(checker.motion_valid(&p1, &p2)
            && checker.config_valid(&p1)
            && checker.motion_valid(&path[i], &p1)
            && checker.motion_valid(&p2, &path[j + 1]))
        {
            continue;
        }
        let tail = path.split_off(j + 1);
        path.truncate(i + 1);
        path.push(p1);
        path.push(p2);
        path.extend(tail);
        path.dedup();
        changed = true;
    }
    changed
}

/// Simplifies a valid path in place. The result is valid as well and never longer.
pub fn simplify<C: CollisionChecker + ?Sized>(
    path: &mut Vec<Vec<f32>>,
    checker: &C,
    settings: &SimplifySettings,
    rng: &mut Rng,
) {
    for _ in 0..settings.max_rounds {
        let before = path_length(path);
        let a = shortcut_greedy(path, checker);
        let b = shortcut_random(path, checker, rng, settings.random_attempts);
        let c = shortcut_greedy(path, checker);
        let after = path_length(path);
        if !(a || b || c) || after > before * (1.0 - settings.min_improvement) {
            break;
        }
    }
}
