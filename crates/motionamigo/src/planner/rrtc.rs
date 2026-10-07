//! RRT-Connect (Kuffner and LaValle, ICRA 2000) with balanced trees.

use super::distance;
use crate::checker::CollisionChecker;
use crate::rng::Rng;

/// Settings of [`rrt_connect`].
#[derive(Debug, Clone, PartialEq)]
pub struct RrtcSettings {
    /// Maximum extension distance per step (radians, L2).
    pub range: f32,
    /// Maximum number of iterations (samples).
    pub max_iterations: usize,
    /// Extend the smaller tree first.
    pub balance: bool,
    /// Size ratio that triggers swapping trees when balancing.
    pub tree_ratio: f32,
    /// Start by extending the start tree.
    pub start_tree_first: bool,
}

impl Default for RrtcSettings {
    fn default() -> Self {
        RrtcSettings {
            range: 1.0,
            max_iterations: 100_000,
            balance: true,
            tree_ratio: 1.0,
            start_tree_first: true,
        }
    }
}

/// Outcome of [`rrt_connect`].
#[derive(Debug, Clone, PartialEq)]
pub struct RrtcResult {
    /// The path from start to one of the goals, if found.
    pub path: Option<Vec<Vec<f32>>>,
    /// Iterations used.
    pub iterations: usize,
    /// Number of nodes in the start and goal trees.
    pub tree_sizes: [usize; 2],
}

/// A tree of configurations stored in one flat array.
#[derive(Debug, Clone)]
struct Tree {
    dof: usize,
    nodes: Vec<f32>,
    parents: Vec<u32>,
}

const ROOT: u32 = u32::MAX;

impl Tree {
    fn new(dof: usize) -> Tree {
        Tree {
            dof,
            nodes: Vec::with_capacity(dof * 1024),
            parents: Vec::with_capacity(1024),
        }
    }

    fn len(&self) -> usize {
        self.parents.len()
    }

    fn get(&self, i: usize) -> &[f32] {
        &self.nodes[i * self.dof..(i + 1) * self.dof]
    }

    fn add(&mut self, q: &[f32], parent: u32) -> usize {
        self.nodes.extend_from_slice(q);
        self.parents.push(parent);
        self.len() - 1
    }

    /// Index of the node nearest to `q` (lowest index on ties).
    fn nearest(&self, q: &[f32]) -> usize {
        let mut best = 0;
        let mut best_d = f32::INFINITY;
        for (i, node) in self.nodes.chunks_exact(self.dof).enumerate() {
            let mut d = 0.0f32;
            for (a, b) in node.iter().zip(q) {
                let e = a - b;
                d += e * e;
            }
            if d < best_d {
                best_d = d;
                best = i;
            }
        }
        best
    }

    /// Configurations from node `i` up to its root.
    fn branch(&self, mut i: usize) -> Vec<Vec<f32>> {
        let mut out = vec![self.get(i).to_vec()];
        while self.parents[i] != ROOT {
            i = self.parents[i] as usize;
            out.push(self.get(i).to_vec());
        }
        out
    }
}

fn steer(from: &[f32], to: &[f32], range: f32, out: &mut [f32]) -> bool {
    let d = distance(from, to);
    if d <= range {
        out.copy_from_slice(to);
        true
    } else {
        let s = range / d;
        for k in 0..out.len() {
            out[k] = from[k] + (to[k] - from[k]) * s;
        }
        false
    }
}

/// Plans from `start` to any of `goals` with RRT-Connect.
///
/// `start` and `goals` must be valid configurations; this is not re-checked here.
pub fn rrt_connect<C: CollisionChecker + ?Sized>(
    checker: &C,
    start: &[f32],
    goals: &[Vec<f32>],
    settings: &RrtcSettings,
    rng: &mut Rng,
) -> RrtcResult {
    let dof = checker.dof();
    for g in goals {
        if checker.motion_valid(start, g) {
            return RrtcResult {
                path: Some(vec![start.to_vec(), g.clone()]),
                iterations: 0,
                tree_sizes: [1, 1],
            };
        }
    }
    let mut trees = [Tree::new(dof), Tree::new(dof)];
    trees[0].add(start, ROOT);
    for g in goals {
        trees[1].add(g, ROOT);
    }
    let (lo, hi) = (checker.lower().to_vec(), checker.upper().to_vec());
    let mut sample = vec![0.0f32; dof];
    let mut new = vec![0.0f32; dof];
    let mut step = vec![0.0f32; dof];
    let mut current = if settings.start_tree_first { 0 } else { 1 };

    for iteration in 1..=settings.max_iterations {
        if settings.balance {
            let (s0, s1) = (trees[0].len() as f32, trees[1].len() as f32);
            current = if s0 * settings.tree_ratio <= s1 { 0 } else { 1 };
        }
        let other = 1 - current;
        for k in 0..dof {
            sample[k] = rng.uniform(lo[k], hi[k]);
        }

        // Extend the current tree towards the sample.
        let near = trees[current].nearest(&sample);
        steer(trees[current].get(near), &sample, settings.range, &mut new);
        if checker.motion_valid(trees[current].get(near), &new) {
            let new_idx = trees[current].add(&new, near as u32);

            // Greedily connect the other tree to the new node.
            let mut cur = trees[other].nearest(&new);
            loop {
                let reached = steer(trees[other].get(cur), &new, settings.range, &mut step);
                if !checker.motion_valid(trees[other].get(cur), &step) {
                    break;
                }
                if reached {
                    let mut a = trees[current].branch(new_idx);
                    let b = trees[other].branch(cur);
                    a.reverse();
                    a.extend(b);
                    if current == 1 {
                        a.reverse();
                    }
                    return RrtcResult {
                        path: Some(a),
                        iterations: iteration,
                        tree_sizes: [trees[0].len(), trees[1].len()],
                    };
                }
                cur = trees[other].add(&step, cur as u32);
            }
        }
        if !settings.balance {
            current = other;
        }
    }
    RrtcResult {
        path: None,
        iterations: settings.max_iterations,
        tree_sizes: [trees[0].len(), trees[1].len()],
    }
}
