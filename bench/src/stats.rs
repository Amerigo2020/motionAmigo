//! Summary statistics.

/// Aggregated results of a set of runs. Times are in microseconds and computed over the solved
/// runs only (failures are reported through `success`).
#[derive(Debug, Clone)]
pub struct Summary {
    pub runs: usize,
    pub success: f64,
    pub planning_median: f64,
    pub planning_p95: f64,
    pub simplify_median: f64,
    pub total_median: f64,
    pub total_p95: f64,
    pub length_median: f64,
}

/// Percentile with linear interpolation (like numpy's default).
pub fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let pos = p / 100.0 * (sorted.len() - 1) as f64;
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    sorted[lo] + (sorted[hi] - sorted[lo]) * (pos - lo as f64)
}

fn sorted(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v
}

impl Summary {
    /// Builds a summary from `(solved, planning_us, simplify_us, total_us, length)` tuples.
    pub fn from_records(records: impl Iterator<Item = (bool, f64, f64, f64, f64)>) -> Summary {
        let all: Vec<_> = records.collect();
        let ok: Vec<_> = all.iter().filter(|r| r.0).collect();
        let planning = sorted(ok.iter().map(|r| r.1).collect());
        let simplify = sorted(ok.iter().map(|r| r.2).collect());
        let total = sorted(ok.iter().map(|r| r.3).collect());
        let length = sorted(ok.iter().map(|r| r.4).collect());
        Summary {
            runs: all.len(),
            success: ok.len() as f64 / all.len().max(1) as f64,
            planning_median: percentile(&planning, 50.0),
            planning_p95: percentile(&planning, 95.0),
            simplify_median: percentile(&simplify, 50.0),
            total_median: percentile(&total, 50.0),
            total_p95: percentile(&total, 95.0),
            length_median: percentile(&length, 50.0),
        }
    }
}

/// Formats microseconds with a sensible unit.
pub fn fmt_us(us: f64) -> String {
    if us.is_nan() {
        "n/a".into()
    } else if us < 1000.0 {
        format!("{us:.0} µs")
    } else if us < 1e6 {
        format!("{:.2} ms", us / 1000.0)
    } else {
        format!("{:.2} s", us / 1e6)
    }
}

pub fn print_mbm_table(rows: &[(String, Summary)]) {
    println!("| scenario | problems | success | planning median | planning P95 | simplification median | total median | total P95 | path length median |");
    println!("|---|---:|---:|---:|---:|---:|---:|---:|---:|");
    for (name, s) in rows {
        println!(
            "| {name} | {} | {:.0}% | {} | {} | {} | {} | {} | {:.2} |",
            s.runs,
            100.0 * s.success,
            fmt_us(s.planning_median),
            fmt_us(s.planning_p95),
            fmt_us(s.simplify_median),
            fmt_us(s.total_median),
            fmt_us(s.total_p95),
            s.length_median
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentile_matches_numpy() {
        let v = [1.0, 2.0, 3.0, 4.0, 10.0];
        assert_eq!(percentile(&v, 50.0), 3.0);
        assert!((percentile(&v, 95.0) - 8.8).abs() < 1e-12);
    }
}
