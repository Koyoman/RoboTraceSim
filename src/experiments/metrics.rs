//! Strict scientific series: missing values are never substituted with zero.
use crate::{json::JsonValue as J, math::wrap_angle};
use std::{collections::BTreeMap, path::Path};
#[derive(Clone, Debug)]
pub struct Row {
    pub t_us: i64,
    pub values: BTreeMap<String, f64>,
}
#[derive(Clone, Debug, Default)]
pub struct Series {
    pub rows: Vec<Row>,
}
pub fn object(fields: impl IntoIterator<Item = (&'static str, J)>) -> J {
    J::Object(fields.into_iter().map(|(k, v)| (k.into(), v)).collect())
}
pub fn number(n: Option<f64>) -> J {
    n.map(J::Number).unwrap_or(J::Null)
}
impl Series {
    pub fn read(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::parse(&text)
    }
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let headers: Vec<_> = lines
            .next()
            .ok_or("empty CSV")?
            .split(',')
            .map(str::trim)
            .collect();
        if headers.first() != Some(&"t_us") {
            return Err("scientific CSV requires first column t_us (integer microseconds)".into());
        }
        let mut unique = std::collections::BTreeSet::new();
        if headers.iter().any(|s| s.is_empty() || !unique.insert(*s)) {
            return Err("empty/duplicate column".into());
        }
        let mut rows = Vec::new();
        for (i, line) in lines.enumerate() {
            super::jobs::check_cancelled()?;
            if line.trim().is_empty() {
                continue;
            }
            let fields: Vec<_> = line.split(',').map(str::trim).collect();
            if fields.len() != headers.len() {
                return Err(format!("CSV row {} column count mismatch", i + 2));
            }
            let t = fields[0]
                .parse::<i64>()
                .map_err(|_| format!("invalid timestamp row {}", i + 2))?;
            if t.unsigned_abs() > 9007199254740991 {
                return Err("timestamp outside exact supported microsecond range".into());
            }
            if rows.last().is_some_and(|r: &Row| r.t_us >= t) {
                return Err("timestamps must be strictly increasing; duplicates/reordering require explicit preprocessing".into());
            }
            let mut values = BTreeMap::new();
            for (k, v) in headers.iter().zip(fields.iter()).skip(1) {
                if !v.is_empty() {
                    let n = v
                        .parse::<f64>()
                        .map_err(|_| format!("invalid {k} at row {}", i + 2))?;
                    if !n.is_finite() {
                        return Err(format!("nonfinite {k}"));
                    }
                    values.insert(k.to_string(), n);
                }
            }
            rows.push(Row { t_us: t, values });
        }
        if rows.is_empty() {
            return Err("no observations".into());
        }
        Ok(Self { rows })
    }
    pub fn align(
        &mut self,
        origin_us: i64,
        offset_us: i64,
        translation: [f64; 2],
        angle: f64,
    ) -> Result<(), String> {
        if !angle.is_finite() || translation.iter().any(|v| !v.is_finite()) {
            return Err("invalid coordinate transform".into());
        }
        for row in &mut self.rows {
            row.t_us = row
                .t_us
                .checked_sub(origin_us)
                .and_then(|t| t.checked_add(offset_us))
                .ok_or("timestamp overflow")?;
            for (key, value) in &mut row.values {
                if key.ends_with("_acquired_us") || key.ends_with("_available_us") {
                    *value = *value - origin_us as f64 + offset_us as f64;
                }
            }
            let (sin, cos) = angle.sin_cos();
            if let (Some(x), Some(y)) = (
                row.values.get("x_m").copied(),
                row.values.get("y_m").copied(),
            ) {
                row.values
                    .insert("x_m".into(), cos * x - sin * y + translation[0]);
                row.values
                    .insert("y_m".into(), sin * x + cos * y + translation[1]);
            }
            if let Some(yaw) = row.values.get_mut("yaw_rad") {
                *yaw = wrap_angle(*yaw + angle);
            }
        }
        Ok(())
    }
    pub fn at(&self, t: i64, key: &str, max_gap_us: i64) -> Option<f64> {
        let idx = self.rows.partition_point(|r| r.t_us < t);
        if let Some(r) = self.rows.get(idx) {
            if r.t_us == t {
                return valid_value(r, key);
            }
        }
        if idx == 0 || idx == self.rows.len() {
            return None;
        }
        let a = &self.rows[idx - 1];
        let b = &self.rows[idx];
        if b.t_us - a.t_us > max_gap_us {
            return None;
        }
        let av = valid_value(a, key)?;
        if held(key) {
            return Some(av);
        }
        let bv = valid_value(b, key)?;
        let alpha = (t - a.t_us) as f64 / (b.t_us - a.t_us) as f64;
        Some(if key == "yaw_rad" {
            wrap_angle(av + wrap_angle(bv - av) * alpha)
        } else {
            av + (bv - av) * alpha
        })
    }
}
fn valid_value(row: &Row, key: &str) -> Option<f64> {
    if let Some(prefix) = key.strip_suffix("_adc") {
        if row
            .values
            .get(&format!("{prefix}_valid"))
            .is_some_and(|v| *v == 0.)
            || row
                .values
                .get(&format!("{prefix}_available_us"))
                .is_some_and(|t| *t > row.t_us as f64)
        {
            return None;
        }
    }
    row.values.get(key).copied()
}
fn held(key: &str) -> bool {
    key.ends_with("_adc")
        || key.starts_with("pwm_")
        || key.ends_with("_ticks")
        || key.ends_with("_valid")
        || matches!(key, "line_visible" | "saturated")
}
#[derive(Clone, Debug)]
pub struct Metric {
    pub signal: String,
    pub requested: usize,
    pub observed: usize,
    pub matched: usize,
    pub coverage: f64,
    pub rms: Option<f64>,
    pub max_abs: Option<f64>,
    pub mean_abs: Option<f64>,
}
impl Metric {
    pub fn json(&self) -> J {
        object([
            ("signal", J::String(self.signal.clone())),
            ("requested", J::Number(self.requested as f64)),
            ("observed", J::Number(self.observed as f64)),
            ("matched", J::Number(self.matched as f64)),
            ("coverage", J::Number(self.coverage)),
            ("rms", number(self.rms)),
            ("max_abs", number(self.max_abs)),
            ("mean_abs", number(self.mean_abs)),
        ])
    }
}
pub fn compare(sim: &Series, measured: &Series, signal: &str, max_gap_us: i64) -> Metric {
    let mut errors = Vec::new();
    let mut observed = 0;
    for row in &measured.rows {
        if let Some(v) = valid_value(row, signal) {
            observed += 1;
            if let Some(p) = sim.at(row.t_us, signal, max_gap_us) {
                errors.push(if signal == "yaw_rad" {
                    wrap_angle(p - v)
                } else {
                    p - v
                });
            }
        }
    }
    let n = errors.len();
    Metric {
        signal: signal.into(),
        requested: measured.rows.len(),
        observed,
        matched: n,
        coverage: n as f64 / measured.rows.len().max(1) as f64,
        rms: (n > 0).then(|| (errors.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt()),
        mean_abs: (n > 0).then(|| errors.iter().map(|x| x.abs()).sum::<f64>() / n as f64),
        max_abs: errors.iter().map(|x| x.abs()).reduce(f64::max),
    }
}
/// Fixed finite search; never use this on held-out validation data to improve a score.
pub fn estimate_offset(
    sim: &Series,
    measured: &Series,
    signal: &str,
    candidates: &[i64],
    max_gap_us: i64,
    min_coverage: f64,
) -> Result<i64, String> {
    if candidates.is_empty() || candidates.len() > 10001 {
        return Err("offset search must have 1..10001 candidates".into());
    }
    let observed: Vec<_> = measured
        .rows
        .iter()
        .filter_map(|r| valid_value(r, signal))
        .collect();
    if observed.len() < 3
        || observed.iter().copied().reduce(f64::max).unwrap()
            - observed.iter().copied().reduce(f64::min).unwrap()
            < 1e-12
    {
        return Err("offset unidentifiable from constant/insufficient signal".into());
    }
    if !(0.0..=1.).contains(&min_coverage) || min_coverage == 0. || max_gap_us <= 0 {
        return Err("invalid offset coverage/gap".into());
    }
    let mut best = None;
    for &offset in candidates {
        let mut shifted = measured.clone();
        shifted.align(0, offset, [0.; 2], 0.)?;
        let m = compare(sim, &shifted, signal, max_gap_us);
        if m.coverage >= min_coverage {
            if let Some(rms) = m.rms {
                if best.is_none_or(|(_, score)| rms < score) {
                    best = Some((offset, rms));
                }
            }
        }
    }
    best.map(|(t, _)| t)
        .ok_or("offset has insufficient overlap/coverage".into())
}
/// Derived metrics require contiguous observations. Duration coverage accompanies integrals.
pub fn derived(series: &Series, max_gap_us: i64) -> J {
    let mut energy = 0.;
    let mut energy_time = 0.;
    let mut loss_time = 0.;
    let mut line_time = 0.;
    let mut saturation_time = 0.;
    let mut saturation_coverage = 0.;
    for w in series.rows.windows(2) {
        let dt = w[1].t_us - w[0].t_us;
        if dt <= 0 || dt > max_gap_us {
            continue;
        }
        let seconds = dt as f64 * 1e-6;
        let power =
            |r: &Row| Some(r.values.get("battery_voltage_v")? * r.values.get("battery_current_a")?);
        if let (Some(a), Some(b)) = (power(&w[0]), power(&w[1])) {
            energy += (a + b) * 0.5 * seconds;
            energy_time += seconds;
        }
        if let Some(v) = w[0]
            .values
            .get("line_visible")
            .filter(|_| w[1].values.contains_key("line_visible"))
        {
            line_time += seconds;
            if *v == 0. {
                loss_time += seconds;
            }
        }
        if let Some(v) = w[0]
            .values
            .get("saturated")
            .filter(|_| w[1].values.contains_key("saturated"))
        {
            saturation_coverage += seconds;
            if *v != 0. {
                saturation_time += seconds;
            }
        }
    }
    // Braking distance is supported only with an explicit brake marker and continuous pose/speed.
    let mut braking = false;
    let mut braking_distance = 0.;
    let mut measured_braking = None;
    for w in series.rows.windows(2) {
        if !braking && w[0].values.get("brake_active").is_some_and(|v| *v == 1.) {
            braking = true;
            braking_distance = 0.;
        }
        if braking {
            let position = |r: &Row| Some((*r.values.get("x_m")?, *r.values.get("y_m")?));
            if w[1].t_us - w[0].t_us > max_gap_us
                || w[0].values.get("brake_active") != Some(&1.)
                || w[1].values.get("brake_active") != Some(&1.)
            {
                break;
            }
            if let (Some((x, y)), Some((xx, yy)), Some(speed)) = (
                position(&w[0]),
                position(&w[1]),
                w[1].values.get("vx_body_m_s"),
            ) {
                braking_distance += (xx - x).hypot(yy - y);
                if speed.abs() <= 0.01 {
                    measured_braking = Some(braking_distance);
                    break;
                }
            } else {
                break;
            }
        }
    }
    let last = series.rows.last();
    object([
        ("energy_j", number((energy_time > 0.).then_some(energy))),
        ("energy_covered_s", J::Number(energy_time)),
        ("line_loss_s", number((line_time > 0.).then_some(loss_time))),
        ("line_covered_s", J::Number(line_time)),
        (
            "saturation_s",
            number((saturation_coverage > 0.).then_some(saturation_time)),
        ),
        ("saturation_covered_s", J::Number(saturation_coverage)),
        (
            "lap_time_s",
            number(last.and_then(|r| r.values.get("lap_time_s").copied())),
        ),
        (
            "braking_distance_m",
            number(
                last.and_then(|r| r.values.get("braking_distance_m").copied())
                    .or(measured_braking),
            ),
        ),
    ])
}

/// Euclidean planar position error, requiring both measured coordinates.
pub fn trajectory(sim: &Series, measured: &Series, max_gap_us: i64) -> Metric {
    let mut observed = 0;
    let mut errors = Vec::new();
    for row in &measured.rows {
        if let (Some(x), Some(y)) = (row.values.get("x_m"), row.values.get("y_m")) {
            observed += 1;
            if let (Some(px), Some(py)) = (
                sim.at(row.t_us, "x_m", max_gap_us),
                sim.at(row.t_us, "y_m", max_gap_us),
            ) {
                errors.push((px - x).hypot(py - y));
            }
        }
    }
    let n = errors.len();
    Metric {
        signal: "trajectory_m".into(),
        requested: measured.rows.len(),
        observed,
        matched: n,
        coverage: n as f64 / measured.rows.len().max(1) as f64,
        rms: (n > 0).then(|| (errors.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt()),
        mean_abs: (n > 0).then(|| errors.iter().sum::<f64>() / n as f64),
        max_abs: errors.iter().copied().reduce(f64::max),
    }
}
