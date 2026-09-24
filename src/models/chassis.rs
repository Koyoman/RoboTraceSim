//! Nonnegative, minimum-norm support loads subject to vertical force/moment balance.
use crate::math::Vec2;
/// Enumerate active supports (3..8). Infeasible equilibrium is an error, not clamping.
pub fn support_loads(points: &[Vec2], total: f64, center: Vec2) -> Result<Vec<f64>, String> {
    if points.len() < 3
        || points.len() > 8
        || !total.is_finite()
        || total < 0.
        || !center.x.is_finite()
        || !center.y.is_finite()
    {
        return Err("invalid support equilibrium inputs".into());
    }
    if total == 0. {
        return Ok(vec![0.; points.len()]);
    }
    let mut best: Option<(f64, Vec<f64>)> = None;
    for mask in 0u32..(1 << points.len()) {
        if mask.count_ones() < 3 {
            continue;
        }
        let mut a = [[0.; 4]; 3];
        for (i, p) in points.iter().enumerate() {
            if mask & (1 << i) == 0 {
                continue;
            }
            let col = [1., p.x, p.y];
            for r in 0..3 {
                for c in 0..3 {
                    a[r][c] += col[r] * col[c];
                }
            }
        }
        a[0][3] = total;
        a[1][3] = total * center.x;
        a[2][3] = total * center.y;
        let mut singular = false;
        for c in 0..3 {
            let pivot = (c..3)
                .max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))
                .unwrap();
            if a[pivot][c].abs() < 1e-14 {
                singular = true;
                break;
            }
            a.swap(c, pivot);
            let d = a[c][c];
            for k in c..4 {
                a[c][k] /= d;
            }
            for r in 0..3 {
                if r != c {
                    let f = a[r][c];
                    for k in c..4 {
                        a[r][k] -= f * a[c][k];
                    }
                }
            }
        }
        if singular {
            continue;
        }
        let loads: Vec<_> = points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                if mask & (1 << i) == 0 {
                    0.
                } else {
                    a[0][3] + a[1][3] * p.x + a[2][3] * p.y
                }
            })
            .collect();
        if loads.iter().any(|v| !v.is_finite() || *v < -1e-9) {
            continue;
        }
        let score = loads.iter().map(|v| v * v).sum::<f64>();
        if best.as_ref().is_none_or(|(s, _)| score < *s) {
            best = Some((score, loads.into_iter().map(|v| v.max(0.)).collect()));
        }
    }
    best.map(|(_, n)| n)
        .ok_or_else(|| "support equilibrium impossible: tipping/vertical dynamics required".into())
}
/// Ground resultant moves opposite acceleration; external vertical loads are already in total/center.
pub fn load_center(static_center: Vec2, mass: f64, height: f64, total: f64, accel: Vec2) -> Vec2 {
    if total <= 0. {
        static_center
    } else {
        static_center - accel * (mass * height / total)
    }
}
