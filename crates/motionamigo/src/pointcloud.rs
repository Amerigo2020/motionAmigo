//! Point cloud obstacles stored in a uniform grid.
//!
//! Points are bucketed into cubic cells; each cell stores its points contiguously in
//! structure-of-arrays form. A sphere query visits only the cells overlapped by the sphere's
//! bounding box, and the inner loop over a cell's points is a branch-free distance test that the
//! compiler vectorizes. Queries answer "is any point closer than `r + point_radius`?" exactly.

/// A static point cloud with a uniform grid index.
#[derive(Debug, Clone)]
pub struct PointCloud {
    point_radius: f32,
    origin: [f32; 3],
    inv_cell: f32,
    cell: f32,
    dims: [usize; 3],
    cell_start: Vec<u32>,
    xs: Vec<f32>,
    ys: Vec<f32>,
    zs: Vec<f32>,
    lo: [f32; 3],
    hi: [f32; 3],
}

/// Upper bound on the number of grid cells; larger clouds get coarser cells.
const MAX_CELLS: usize = 1 << 22;

impl PointCloud {
    /// Builds the grid. `point_radius` inflates every point (e.g. sensor noise or a safety
    /// margin), `cell_size` is the edge length of the grid cells in meters (about the radius of
    /// the robot spheres is a good choice).
    ///
    /// # Panics
    /// Panics if `cell_size` is not positive or `point_radius` is negative.
    pub fn new(points: &[[f32; 3]], point_radius: f32, cell_size: f32) -> PointCloud {
        assert!(cell_size > 0.0, "cell_size must be positive");
        assert!(point_radius >= 0.0, "point_radius must not be negative");
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for p in points {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        if points.is_empty() {
            lo = [0.0; 3];
            hi = [0.0; 3];
        }
        let mut cell = cell_size;
        let mut dims;
        loop {
            dims = [0, 1, 2].map(|k| (((hi[k] - lo[k]) / cell).floor() as usize) + 1);
            if dims[0] * dims[1] * dims[2] <= MAX_CELLS {
                break;
            }
            cell *= 2.0;
        }
        let inv_cell = 1.0 / cell;
        let n_cells = dims[0] * dims[1] * dims[2];
        let index_of = |p: &[f32; 3]| {
            let i = [0, 1, 2].map(|k| (((p[k] - lo[k]) * inv_cell) as usize).min(dims[k] - 1));
            (i[2] * dims[1] + i[1]) * dims[0] + i[0]
        };
        let mut counts = vec![0u32; n_cells + 1];
        for p in points {
            counts[index_of(p) + 1] += 1;
        }
        for i in 0..n_cells {
            counts[i + 1] += counts[i];
        }
        let mut fill = counts.clone();
        let mut xs = vec![0.0; points.len()];
        let mut ys = vec![0.0; points.len()];
        let mut zs = vec![0.0; points.len()];
        for p in points {
            let c = index_of(p);
            let at = fill[c] as usize;
            fill[c] += 1;
            xs[at] = p[0];
            ys[at] = p[1];
            zs[at] = p[2];
        }
        PointCloud {
            point_radius,
            origin: lo,
            inv_cell,
            cell,
            dims,
            cell_start: counts,
            xs,
            ys,
            zs,
            lo: lo.map(|v| v - point_radius),
            hi: hi.map(|v| v + point_radius),
        }
    }

    /// Number of points.
    pub fn len(&self) -> usize {
        self.xs.len()
    }

    /// True if the cloud has no points.
    pub fn is_empty(&self) -> bool {
        self.xs.is_empty()
    }

    /// Radius by which every point is inflated.
    pub fn point_radius(&self) -> f32 {
        self.point_radius
    }

    /// Edge length of the grid cells.
    pub fn cell_size(&self) -> f32 {
        self.cell
    }

    /// The `i`-th point (in grid order).
    pub fn point(&self, i: usize) -> [f32; 3] {
        [self.xs[i], self.ys[i], self.zs[i]]
    }

    /// True if any point lies strictly closer than `r + point_radius` to `c`.
    #[inline]
    pub fn collides(&self, c: [f32; 3], r: f32) -> bool {
        if self.xs.is_empty() {
            return false;
        }
        for k in 0..3 {
            if c[k] + r <= self.lo[k] || c[k] - r >= self.hi[k] {
                return false;
            }
        }
        let rr = r + self.point_radius;
        let rr2 = rr * rr;
        let mut first = [0usize; 3];
        let mut last = [0usize; 3];
        for k in 0..3 {
            let a = ((c[k] - rr - self.origin[k]) * self.inv_cell).floor();
            let b = ((c[k] + rr - self.origin[k]) * self.inv_cell).floor();
            let max = (self.dims[k] - 1) as f32;
            first[k] = a.clamp(0.0, max) as usize;
            last[k] = b.clamp(0.0, max) as usize;
        }
        for z in first[2]..=last[2] {
            for y in first[1]..=last[1] {
                let row = (z * self.dims[1] + y) * self.dims[0];
                let s = self.cell_start[row + first[0]] as usize;
                let e = self.cell_start[row + last[0] + 1] as usize;
                if self.any_within(s, e, c, rr2) {
                    return true;
                }
            }
        }
        false
    }

    #[inline]
    fn any_within(&self, s: usize, e: usize, c: [f32; 3], rr2: f32) -> bool {
        let (xs, ys, zs) = (&self.xs[s..e], &self.ys[s..e], &self.zs[s..e]);
        // Process fixed-size chunks without early exit so the loop vectorizes.
        let mut hit = false;
        let chunks = xs.len() / 8 * 8;
        let mut i = 0;
        while i < chunks {
            let mut any = false;
            for k in i..i + 8 {
                let dx = xs[k] - c[0];
                let dy = ys[k] - c[1];
                let dz = zs[k] - c[2];
                any |= dx * dx + dy * dy + dz * dz < rr2;
            }
            if any {
                return true;
            }
            i += 8;
        }
        for k in chunks..xs.len() {
            let dx = xs[k] - c[0];
            let dy = ys[k] - c[1];
            let dz = zs[k] - c[2];
            hit |= dx * dx + dy * dy + dz * dz < rr2;
        }
        hit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg(seed: &mut u64) -> f32 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 40) as f32) / (1u64 << 24) as f32
    }

    #[test]
    fn grid_query_matches_brute_force() {
        let mut seed = 7;
        let pts: Vec<[f32; 3]> = (0..3000)
            .map(|_| {
                [
                    lcg(&mut seed),
                    lcg(&mut seed) * 0.5,
                    lcg(&mut seed) * 2.0 - 1.0,
                ]
            })
            .collect();
        let pc = PointCloud::new(&pts, 0.005, 0.05);
        for _ in 0..2000 {
            let c = [
                lcg(&mut seed) * 1.4 - 0.2,
                lcg(&mut seed) * 0.9 - 0.2,
                lcg(&mut seed) * 2.4 - 1.2,
            ];
            let r = lcg(&mut seed) * 0.08;
            let rr = r + 0.005;
            let brute = pts.iter().any(|p| {
                let (dx, dy, dz) = (p[0] - c[0], p[1] - c[1], p[2] - c[2]);
                dx * dx + dy * dy + dz * dz < rr * rr
            });
            assert_eq!(pc.collides(c, r), brute, "{c:?} {r}");
        }
    }

    #[test]
    fn empty_cloud_never_collides() {
        let pc = PointCloud::new(&[], 0.0, 0.1);
        assert!(pc.is_empty());
        assert!(!pc.collides([0.0; 3], 1.0));
    }
}
