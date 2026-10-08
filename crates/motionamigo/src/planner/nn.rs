//! Exact nearest-neighbour search over a growing set of configurations.
//!
//! [`KdTree`] is an incremental, unbalanced kd-tree whose nodes are the points themselves, split on
//! dimension `depth % dof`. Queries return exactly what [`nearest_linear`] returns: the same f32
//! squared distance (same summation order) and the lowest index on ties. Pruning is exact because
//! f32 subtraction, squaring and addition of non-negative values are monotone, so the computed
//! distance of any point beyond a splitting plane is never below the computed squared distance to
//! that plane.
//!
//! Public only for the `nn` benchmark; not part of the stable API.

const NONE: u32 = u32::MAX;

/// Squared L2 distance, summed in dimension order (the reference computation).
#[inline]
fn dist2(node: &[f32], q: &[f32]) -> f32 {
    let mut d = 0.0f32;
    for (a, b) in node.iter().zip(q) {
        let e = a - b;
        d += e * e;
    }
    d
}

/// Index of the point in `points` (flat, `dof` per point) nearest to `q`, lowest index on ties.
pub fn nearest_linear(points: &[f32], dof: usize, q: &[f32]) -> usize {
    let mut best = 0;
    let mut best_d = f32::INFINITY;
    for (i, node) in points.chunks_exact(dof).enumerate() {
        let d = dist2(node, q);
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best
}

/// Kd-tree index over a flat point array owned by the caller.
#[derive(Debug, Clone, Default)]
pub struct KdTree {
    children: Vec<[u32; 2]>,
    stack: Vec<(u32, u32)>,
    offsets: Vec<f32>,
}

impl KdTree {
    /// Indexes point `children.len()`, which must be the last point of `points`.
    pub fn insert(&mut self, points: &[f32], dof: usize) {
        let i = self.children.len() as u32;
        self.children.push([NONE; 2]);
        if i == 0 {
            return;
        }
        let p = &points[i as usize * dof..(i as usize + 1) * dof];
        let (mut n, mut depth) = (0usize, 0usize);
        loop {
            let k = depth % dof;
            let side = usize::from(p[k] >= points[n * dof + k]);
            match self.children[n][side] {
                NONE => {
                    self.children[n][side] = i;
                    return;
                }
                c => {
                    n = c as usize;
                    depth += 1;
                }
            }
        }
    }

    /// Same result as [`nearest_linear`] over the indexed points.
    pub fn nearest(&mut self, points: &[f32], dof: usize, q: &[f32]) -> usize {
        let mut best = 0usize;
        let mut best_d = f32::INFINITY;
        if self.children.is_empty() {
            return best;
        }
        // Each stack entry (node, depth) owns `dof` squared per-axis offsets from `q` to the cell
        // of its subtree in `offsets`; their sum in dimension order bounds the subtree from below.
        self.stack.clear();
        self.offsets.clear();
        self.stack.push((0, 0));
        self.offsets.resize(dof, 0.0);
        while let Some((n, depth)) = self.stack.pop() {
            let top = self.stack.len() * dof;
            let mut bound = 0.0f32;
            for &o in &self.offsets[top..] {
                bound += o;
            }
            if bound > best_d {
                self.offsets.truncate(top);
                continue;
            }
            let n = n as usize;
            let node = &points[n * dof..(n + 1) * dof];
            let d = dist2(node, q);
            if d < best_d || (d == best_d && n < best) {
                best_d = d;
                best = n;
            }
            let k = depth as usize % dof;
            let e = node[k] - q[k];
            // Points with p[k] >= node[k] are on side 1.
            let near = usize::from(q[k] >= node[k]);
            let [c_near, c_far] = [self.children[n][near], self.children[n][1 - near]];
            // The popped entry's offsets are at `top`; reuse them for the children.
            match (c_far != NONE, c_near != NONE) {
                (false, false) => self.offsets.truncate(top),
                (false, true) => self.stack.push((c_near, depth + 1)),
                (true, false) => {
                    self.offsets[top + k] = e * e;
                    self.stack.push((c_far, depth + 1));
                }
                (true, true) => {
                    self.offsets.extend_from_within(top..top + dof);
                    self.offsets[top + k] = e * e;
                    self.stack.push((c_far, depth + 1));
                    self.stack.push((c_near, depth + 1));
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    /// Coordinates from a tiny grid so that ties and duplicate points are common.
    fn coord() -> impl Strategy<Value = f32> {
        prop_oneof![(-3i32..=3).prop_map(|v| v as f32 * 0.5), -4.0f32..4.0]
    }

    proptest! {
        #[test]
        fn kd_tree_matches_linear_scan(
            dof in 1usize..8,
            raw in prop::collection::vec(coord(), 1..400),
            queries in prop::collection::vec(coord(), 8..64),
        ) {
            let points = &raw[..raw.len() / dof * dof];
            prop_assume!(!points.is_empty());
            let mut kd = KdTree::default();
            for n in 1..=points.len() / dof {
                kd.insert(&points[..n * dof], dof);
                for q in queries.chunks_exact(dof) {
                    prop_assert_eq!(
                        kd.nearest(&points[..n * dof], dof, q),
                        nearest_linear(&points[..n * dof], dof, q)
                    );
                }
            }
        }
    }

    #[test]
    fn duplicates_return_lowest_index() {
        let points = [1.0f32, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
        let mut kd = KdTree::default();
        for n in 1..=4 {
            kd.insert(&points[..n * 2], 2);
        }
        assert_eq!(kd.nearest(&points, 2, &[0.0, 0.0]), 1);
        assert_eq!(kd.nearest(&points, 2, &[1.0, 1.0]), 0);
        // Equidistant from both distinct points: lowest index wins.
        assert_eq!(kd.nearest(&points, 2, &[0.5, 0.5]), 0);
    }
}
