//! Versioned studies: calibration only chooses parameters; holdout is evaluated afterwards.
use super::metrics::{self, number, object, Row, Series};
use crate::{
    config::{load_project, LoadedConfig},
    json::{parse_json, JsonValue as J},
    sim::SimulationCore,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
    time::Instant,
};
pub fn text<'a>(v: &'a J, key: &str) -> Result<&'a str, String> {
    v.get(key)
        .and_then(J::as_str)
        .filter(|v| !v.trim().is_empty())
        .ok_or(format!("required string {key}"))
}
pub fn num(v: &J, key: &str) -> Result<f64, String> {
    v.get(key)
        .and_then(J::as_f64)
        .filter(|x| x.is_finite())
        .ok_or(format!("required finite number {key}"))
}
pub fn integer(v: &J, key: &str) -> Result<i64, String> {
    let n = num(v, key)?;
    if n.fract() != 0. || n.abs() > 9007199254740991. {
        return Err(format!("{key} must be exact integer"));
    }
    Ok(n as i64)
}
fn array<'a>(v: &'a J, key: &str) -> Result<&'a [J], String> {
    v.get(key)
        .and_then(J::as_array)
        .ok_or(format!("required array {key}"))
}
#[derive(Clone)]
pub struct Objective {
    pub signal: String,
    pub scale: f64,
    pub weight: f64,
    pub max_rms: f64,
    pub min_coverage: f64,
    pub unit: String,
}
#[derive(Clone)]
pub struct Parameter {
    pub name: String,
    pub min: f64,
    pub max: f64,
    pub steps: usize,
}
#[derive(Clone)]
pub struct Dataset {
    pub id: String,
    pub split: String,
    pub kind: String,
    pub group: String,
    pub series: Series,
    pub cfg: LoadedConfig,
    pub max_gap: i64,
    pub fingerprint: String,
    pub protocol: J,
}
pub struct Study {
    pub source: J,
    pub fingerprint: String,
    pub datasets: Vec<Dataset>,
    pub objectives: Vec<Objective>,
    pub parameters: Vec<Parameter>,
    pub duration_us: u64,
    pub preset: String,
}
impl Study {
    pub fn read(path: &Path) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let source = parse_json(std::str::from_utf8(&bytes).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        if text(&source, "schema")? != "rtsim-study-v1" {
            return Err("unsupported study schema".into());
        }
        let _ = text(&source, "tolerance_basis")?;
        let preset = text(&source, "preset")?.to_string();
        crate::models::fidelity::FidelityConfig::preset(&preset)?;
        let duration = integer(&source, "duration_us")?;
        if duration <= 0 {
            return Err("duration must be positive".into());
        }
        let mut objectives = Vec::new();
        let mut names = BTreeSet::new();
        for o in array(&source, "objectives")? {
            let signal = text(o, "signal")?.to_string();
            if !names.insert(signal.clone()) {
                return Err("duplicate objective".into());
            }
            let scale = num(o, "scale")?;
            let weight = num(o, "weight")?;
            let max_rms = num(o, "max_rms")?;
            let min_coverage = num(o, "min_coverage")?;
            if scale <= 0.
                || weight <= 0.
                || max_rms < 0.
                || !(0.0..=1.).contains(&min_coverage)
                || min_coverage == 0.
            {
                return Err("invalid objective bounds".into());
            }
            let unit = text(o, "unit")?;
            let expected = match signal.as_str() {
                "x_m" | "y_m" | "line_error_m" => "m",
                "yaw_rad" => "rad",
                "vx_body_m_s" | "vy_body_m_s" => "m/s",
                "yaw_rate_rad_s" => "rad/s",
                "battery_voltage_v" => "V",
                "battery_current_a" | "motor_current_left_a" | "motor_current_right_a" => "A",
                "line_visible" | "saturated" => "boolean",
                "pwm_left" | "pwm_right" => "1",
                v if v.starts_with("sensor_") && v.ends_with("_adc") => "ADC",
                _ => {
                    return Err(format!(
                        "unsupported objective {signal}; use sampled equivalent signals"
                    ))
                }
            };
            if unit != expected {
                return Err(format!(
                    "{signal} requires unit {expected}; convert explicitly before import"
                ));
            }
            objectives.push(Objective {
                signal,
                scale,
                weight,
                max_rms,
                min_coverage,
                unit: text(o, "unit")?.into(),
            });
        }
        if objectives.is_empty() {
            return Err("no objectives".into());
        }
        let mut parameters = Vec::new();
        let mut names = BTreeSet::new();
        let mut grid = 1usize;
        for p in array(&source, "parameters")? {
            let name = text(p, "name")?.to_string();
            if !names.insert(name.clone()) || !PARAMETERS.contains(&name.as_str()) {
                return Err(format!("unknown/duplicate parameter {name}"));
            }
            let min = num(p, "min")?;
            let max = num(p, "max")?;
            let steps = integer(p, "steps")?;
            if min <= 0. || max <= min || !(2..=21).contains(&steps) {
                return Err(
                    "parameter requires positive increasing bounds and 2..21 grid points".into(),
                );
            }
            grid = grid.checked_mul(steps as usize).ok_or("grid overflow")?;
            if grid > 256 {
                return Err("grid limited to 256 candidates".into());
            }
            parameters.push(Parameter {
                name,
                min,
                max,
                steps: steps as usize,
            });
        }
        if parameters.len() > 3 {
            return Err("at most 3 parameters per identifiable study".into());
        }
        let mut datasets = Vec::new();
        let mut ids = BTreeSet::new();
        let mut hashes = BTreeSet::new();
        let parent = path.parent().unwrap_or(Path::new("."));
        for d in array(&source, "datasets")? {
            let id = text(d, "id")?.to_string();
            if !ids.insert(id.clone()) {
                return Err("duplicate dataset id".into());
            }
            let split = text(d, "split")?.to_string();
            if !["calibration", "validation"].contains(&split.as_str()) {
                return Err("invalid data split".into());
            }
            let kind = text(d, "kind")?.to_string();
            if !["synthetic", "measured", "unknown"].contains(&kind.as_str()) {
                return Err("invalid provenance kind".into());
            }
            if text(d, "time_basis")? != "delivery" {
                return Err("study compares delivered readings; acquisition logs need an explicit acquisition-to-delivery conversion, preserving timestamps".into());
            }
            let protocol = d.get("protocol").cloned().ok_or("missing protocol")?;
            for key in [
                "robot_components",
                "instrument",
                "instrument_accuracy",
                "mounting",
                "surface",
                "lighting",
                "temperature",
                "battery_state",
                "units",
                "acquisition_rate",
                "date",
                "operator",
            ] {
                text(&protocol, key)?;
            }
            let csv = parent.join(text(d, "csv")?);
            let raw = std::fs::read(&csv).map_err(|e| e.to_string())?;
            let fingerprint = crate::io::experiment::fingerprint(&raw);
            if !hashes.insert(fingerprint.clone()) {
                return Err("duplicate data content; calibration/validation leakage".into());
            }
            let mut series = Series::parse(std::str::from_utf8(&raw).map_err(|e| e.to_string())?)?;
            let translation = if let Some(f) = d.get("frame") {
                [num(f, "x_m")?, num(f, "y_m")?, num(f, "yaw_rad")?]
            } else {
                [0.; 3]
            };
            series.align(
                integer(d, "origin_us")?,
                integer(d, "offset_us")?,
                [translation[0], translation[1]],
                translation[2],
            )?;
            let max_gap = integer(d, "max_gap_us")?;
            if max_gap <= 0 {
                return Err("max_gap_us must be positive".into());
            }
            let cfg = load_project(parent.join(text(d, "project")?)).map_err(|e| e.to_string())?;
            datasets.push(Dataset {
                id,
                split,
                kind,
                group: text(d, "condition_group")?.into(),
                series,
                cfg,
                max_gap,
                fingerprint,
                protocol,
            });
        }
        if !datasets.iter().any(|d| d.split == "calibration")
            || !datasets.iter().any(|d| d.split == "validation")
        {
            return Err("independent calibration and validation datasets required".into());
        }
        if datasets.len() > 16 {
            return Err("at most 16 datasets".into());
        }
        let training: BTreeSet<_> = datasets
            .iter()
            .filter(|d| d.split == "calibration")
            .map(|d| d.group.as_str())
            .collect();
        if datasets
            .iter()
            .any(|d| d.split == "validation" && training.contains(d.group.as_str()))
        {
            return Err("validation condition_group was used in calibration".into());
        }
        Ok(Self {
            source,
            fingerprint: crate::io::experiment::fingerprint(&bytes),
            datasets,
            objectives,
            parameters,
            duration_us: duration as u64,
            preset,
        })
    }
}
pub const PARAMETERS: &[&str] = &[
    "motor_torque_scale",
    "motor_resistance_scale",
    "mu_longitudinal_scale",
    "mu_lateral_scale",
    "rolling_resistance_scale",
    "battery_resistance_scale",
];
pub fn apply(cfg: &mut LoadedConfig, values: &BTreeMap<String, f64>) -> Result<(), String> {
    for (key, &v) in values {
        if !v.is_finite() || v <= 0. {
            return Err("parameter scale must be positive finite".into());
        }
        match key.as_str() {
            "motor_torque_scale" => {
                if cfg.robot.powertrain.is_some() {
                    return Err("stall torque scale requires simple motor; use motor_resistance_scale for electrical model".into());
                }
                cfg.robot.motor_left.stall_torque_nm *= v;
                cfg.robot.motor_right.stall_torque_nm *= v;
            }
            "motor_resistance_scale" => {
                for m in &mut cfg
                    .robot
                    .powertrain
                    .as_mut()
                    .ok_or("motor_resistance_scale requires powertrain")?
                    .motors
                {
                    m.resistance_ohm *= v;
                }
            }
            "battery_resistance_scale" => {
                cfg.robot.battery.internal_resistance_ohm *= v;
                if let Some(p) = &mut cfg.robot.powertrain {
                    for (_, r) in &mut p.resistance_curve {
                        *r *= v;
                    }
                }
            }
            "mu_longitudinal_scale" | "mu_lateral_scale" | "rolling_resistance_scale" => {
                let change = |t: &mut crate::config::TireConfig| match key.as_str() {
                    "mu_longitudinal_scale" => t.mu_longitudinal *= v,
                    "mu_lateral_scale" => t.mu_lateral *= v,
                    _ => t.rolling_resistance *= v,
                };
                change(&mut cfg.robot.tire);
                if let Some(a) = &mut cfg.robot.assembly {
                    for w in &mut a.wheels {
                        change(&mut w.tire);
                    }
                }
            }
            _ => return Err(format!("unsupported parameter {key}")),
        }
    }
    Ok(())
}
pub struct Prediction {
    pub series: Series,
    pub wall_s: f64,
    pub snapshot: J,
    pub termination: String,
}
pub fn simulate(cfg: LoadedConfig, duration: u64) -> Result<Prediction, String> {
    let start = Instant::now();
    let mut core = SimulationCore::new(cfg, Some(duration))?;
    let snapshot = parse_json(&core.resolved_experiment().to_json()).map_err(|e| e.to_string())?;
    let mut rows = Vec::new();
    loop {
        if core.should_log() {
            let s = core.sample();
            let mut v: BTreeMap<String, f64> = [
                ("x_m", s.x_m),
                ("y_m", s.y_m),
                ("yaw_rad", s.yaw_rad),
                ("vx_body_m_s", s.vx_body_m_s),
                ("vy_body_m_s", s.vy_body_m_s),
                ("yaw_rate_rad_s", s.yaw_rate_rad_s),
                ("line_visible", s.line_visible as u8 as f64),
                ("line_error_m", s.line_error_m),
                ("battery_voltage_v", s.battery_voltage_v),
                ("battery_current_a", s.battery_current_a),
                ("motor_current_left_a", s.motor_current_left_a),
                ("motor_current_right_a", s.motor_current_right_a),
                ("pwm_left", s.pwm_left),
                ("pwm_right", s.pwm_right),
            ]
            .into_iter()
            .map(|(k, v)| (k.into(), v))
            .collect();
            if let Some(p) = core.power_state() {
                v.insert(
                    "brake_active".into(),
                    p.applied
                        .modes
                        .iter()
                        .all(|m| *m == crate::models::power::ActuatorMode::Brake)
                        as u8 as f64,
                );
                v.insert(
                    "saturated".into(),
                    p.motors.iter().any(|m| m.current_limited) as u8 as f64,
                );
            }
            for (i, c) in core.sensor_readings().channels.iter().enumerate() {
                v.insert(format!("sensor_{i:02}_adc"), c.adc as f64);
                v.insert(format!("sensor_{i:02}_valid"), c.valid as u8 as f64);
                v.insert(format!("sensor_{i:02}_acquired_us"), c.acquired_us as f64);
                v.insert(format!("sensor_{i:02}_available_us"), c.available_us as f64);
            }
            let mut last = None;
            let mut lap = None;
            for e in core.race_state().events() {
                use crate::track::events::RaceEventKind as K;
                if e.kind == K::Started {
                    last = Some(e.t_us);
                }
                if e.kind == K::Lap {
                    if let Some(t) = last {
                        lap = Some((e.t_us - t) as f64 * 1e-6);
                    }
                    last = Some(e.t_us);
                }
            }
            if let Some(t) = lap {
                v.insert("lap_time_s".into(), t);
            }
            rows.push(Row {
                t_us: s.t_us as i64,
                values: v,
            });
        }
        if !core.try_step()? {
            break;
        }
    }
    Ok(Prediction {
        series: Series { rows },
        wall_s: start.elapsed().as_secs_f64(),
        snapshot,
        termination: core.termination_reason().into(),
    })
}
fn prediction(
    study: &Study,
    d: &Dataset,
    values: &BTreeMap<String, f64>,
    preset: &str,
) -> Result<Prediction, String> {
    let mut cfg = d.cfg.clone();
    cfg.robot.physics = Some(crate::models::fidelity::FidelityConfig::preset(preset)?);
    apply(&mut cfg, values)?;
    simulate(cfg, study.duration_us)
}
fn residuals(study: &Study, values: &BTreeMap<String, f64>) -> Result<Vec<f64>, String> {
    let mut residuals = Vec::new();
    for d in study.datasets.iter().filter(|d| d.split == "calibration") {
        super::jobs::check_cancelled()?;
        let p = prediction(study, d, values, &study.preset)?;
        for o in &study.objectives {
            let m = metrics::compare(&p.series, &d.series, &o.signal, d.max_gap);
            if m.coverage < o.min_coverage || m.matched == 0 {
                return Err(format!(
                    "insufficient training coverage {} / {}",
                    d.id, o.signal
                ));
            }
            let normalization = (o.weight / m.matched as f64).sqrt() / o.scale;
            for r in &d.series.rows {
                if let (Some(a), Some(b)) = (
                    p.series.at(r.t_us, &o.signal, d.max_gap),
                    d.series.at(r.t_us, &o.signal, d.max_gap),
                ) {
                    residuals.push(
                        (if o.signal == "yaw_rad" {
                            crate::math::wrap_angle(a - b)
                        } else {
                            a - b
                        }) * normalization,
                    );
                }
            }
        }
    }
    Ok(residuals)
}
fn score(residuals: &[f64]) -> f64 {
    residuals.iter().map(|v| v * v).sum()
}
/// Finite-difference excitation and pairwise collinearity check, not a proof of global identifiability.
pub fn sensitivity(study: &Study) -> Result<(J, bool), String> {
    let mid: BTreeMap<_, _> = study
        .parameters
        .iter()
        .map(|p| (p.name.clone(), (p.min + p.max) * 0.5))
        .collect();
    let mut columns = Vec::new();
    let mut report = Vec::new();
    let mut identifiable = true;
    for p in &study.parameters {
        let mut a = mid.clone();
        let mut b = mid.clone();
        a.insert(p.name.clone(), p.min);
        b.insert(p.name.clone(), p.max);
        let low = residuals(study, &a)?;
        let high = residuals(study, &b)?;
        if low.len() != high.len() {
            return Err("parameter changes observation coverage; sensitivity undefined".into());
        }
        let column: Vec<_> = high.iter().zip(low).map(|(b, a)| b - a).collect();
        let norm = column.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm < 1e-8 {
            identifiable = false;
        }
        report.push(object([
            ("parameter", J::String(p.name.clone())),
            ("normalized_excitation", J::Number(norm)),
        ]));
        columns.push(column);
    }
    let mut correlations = Vec::new();
    for i in 0..columns.len() {
        for j in i + 1..columns.len() {
            let dot = columns[i]
                .iter()
                .zip(&columns[j])
                .map(|(a, b)| a * b)
                .sum::<f64>();
            let norm = |v: &Vec<f64>| v.iter().map(|x| x * x).sum::<f64>().sqrt();
            let denominator = norm(&columns[i]) * norm(&columns[j]);
            let cosine = if denominator > 0. {
                dot / denominator
            } else {
                1.
            };
            if cosine.abs() > 0.98 {
                identifiable = false;
            }
            correlations.push(object([
                ("a", J::String(study.parameters[i].name.clone())),
                ("b", J::String(study.parameters[j].name.clone())),
                ("cosine", J::Number(cosine)),
            ]));
        }
    }
    Ok((
        object([
            ("parameters", J::Array(report)),
            ("pairwise_collinearity", J::Array(correlations)),
            ("identifiable_screen", J::Bool(identifiable)),
        ]),
        identifiable,
    ))
}
pub fn qualify(path: &Path, output: &Path) -> Result<J, String> {
    let study = Study::read(path)?;
    let (sensitivity, identifiable) = sensitivity(&study)?;
    let mut candidates = vec![BTreeMap::new()];
    for p in &study.parameters {
        let mut next = Vec::new();
        for c in candidates {
            for i in 0..p.steps {
                let mut row = c.clone();
                row.insert(
                    p.name.clone(),
                    p.min + (p.max - p.min) * i as f64 / (p.steps - 1) as f64,
                );
                next.push(row);
            }
        }
        candidates = next;
    }
    let mut scores = Vec::new();
    if identifiable {
        for candidate in candidates {
            super::jobs::check_cancelled()?;
            match residuals(&study, &candidate) {
                Ok(r) => scores.push((candidate, Some(score(&r)), None)),
                Err(e) => scores.push((candidate, None, Some(e))),
            }
        }
    }
    let best = scores
        .iter()
        .filter_map(|(p, s, _)| s.map(|s| (p, s)))
        .min_by(|a, b| a.1.total_cmp(&b.1));
    let mut evaluation = Vec::new();
    let mut validation_passed = best.is_some();
    if let Some((chosen, _)) = best {
        for d in &study.datasets {
            for preset in [&study.preset, "simplified"] {
                let result = prediction(&study, d, chosen, preset);
                match result {
                    Ok(p) => {
                        let metrics: Vec<_> = study
                            .objectives
                            .iter()
                            .map(|o| {
                                let m =
                                    metrics::compare(&p.series, &d.series, &o.signal, d.max_gap);
                                let pass = m.coverage >= o.min_coverage
                                    && m.rms.is_some_and(|v| v <= o.max_rms);
                                if d.split == "validation" && preset == &study.preset && !pass {
                                    validation_passed = false;
                                }
                                object([
                                    ("metric", m.json()),
                                    ("unit", J::String(o.unit.clone())),
                                    ("max_rms", J::Number(o.max_rms)),
                                    ("passed", J::Bool(pass)),
                                ])
                            })
                            .collect();
                        let mut diagnostic_signals: std::collections::BTreeSet<String> = [
                            "x_m",
                            "y_m",
                            "yaw_rad",
                            "vx_body_m_s",
                            "vy_body_m_s",
                            "yaw_rate_rad_s",
                            "line_error_m",
                            "battery_current_a",
                            "battery_voltage_v",
                            "motor_current_left_a",
                            "motor_current_right_a",
                            "line_visible",
                            "saturated",
                        ]
                        .into_iter()
                        .map(str::to_owned)
                        .collect();
                        for row in &d.series.rows {
                            for key in row.values.keys() {
                                if key.ends_with("_adc") {
                                    diagnostic_signals.insert(key.clone());
                                }
                            }
                        }
                        let mut diagnostics: Vec<J> = diagnostic_signals
                            .iter()
                            .map(|signal| {
                                metrics::compare(&p.series, &d.series, signal, d.max_gap).json()
                            })
                            .collect();
                        diagnostics
                            .push(metrics::trajectory(&p.series, &d.series, d.max_gap).json());
                        evaluation.push(object([
                            ("dataset", J::String(d.id.clone())),
                            ("split", J::String(d.split.clone())),
                            ("kind", J::String(d.kind.clone())),
                            ("preset", J::String(preset.to_string())),
                            ("wall_s", J::Number(p.wall_s)),
                            ("termination", J::String(p.termination)),
                            ("metrics", J::Array(metrics)),
                            ("diagnostics", J::Array(diagnostics)),
                            (
                                "saturation_definition",
                                J::String(
                                    "electrical driver current limiting; absent without powertrain"
                                        .into(),
                                ),
                            ),
                            ("measured_derived", metrics::derived(&d.series, d.max_gap)),
                            ("simulated_derived", metrics::derived(&p.series, d.max_gap)),
                            ("snapshot", p.snapshot),
                        ]));
                    }
                    Err(e) => {
                        if d.split == "validation" && preset == &study.preset {
                            validation_passed = false;
                        }
                        evaluation.push(object([
                            ("dataset", J::String(d.id.clone())),
                            ("preset", J::String(preset.to_string())),
                            ("error", J::String(e)),
                        ]));
                    }
                }
            }
        }
    }
    let at_bounds = best
        .map(|(p, _)| {
            study
                .parameters
                .iter()
                .filter(|parameter| {
                    p.get(&parameter.name).is_some_and(|v| {
                        (*v - parameter.min).abs() < 1e-12 || (*v - parameter.max).abs() < 1e-12
                    })
                })
                .map(|p| J::String(p.name.clone()))
                .collect()
        })
        .unwrap_or_default();
    let fitted = best
        .map(|(p, _)| J::Object(p.iter().map(|(k, v)| (k.clone(), J::Number(*v))).collect()))
        .unwrap_or(J::Null);
    let status = if !identifiable {
        "identifiability_screen_failed"
    } else if !validation_passed {
        "validation_criteria_failed"
    } else if study.datasets.iter().all(|d| d.kind == "measured") {
        "measured_criteria_passed_review_required"
    } else {
        "synthetic_or_unknown_not_physically_qualified"
    };
    let data = study
        .datasets
        .iter()
        .map(|d| {
            object([
                ("id", J::String(d.id.clone())),
                ("kind", J::String(d.kind.clone())),
                ("split", J::String(d.split.clone())),
                ("condition_group", J::String(d.group.clone())),
                ("data_fnv1a64", J::String(d.fingerprint.clone())),
                ("protocol", d.protocol.clone()),
            ])
        })
        .collect();
    let report = object([
        ("schema", J::String("rtsim-qualification-v1".into())),
        ("app_version", J::String(env!("CARGO_PKG_VERSION").into())),
        ("study_fnv1a64", J::String(study.fingerprint)),
        ("status", J::String(status.into())),
        ("physically_qualified", J::Bool(false)),
        ("manifest", study.source),
        ("datasets", J::Array(data)),
        ("sensitivity", sensitivity),
        ("parameters", fitted),
        ("parameters_at_bounds", J::Array(at_bounds)),
        (
            "parameter_uncertainty",
            J::String("not estimated; bounded grid is not a confidence interval".into()),
        ),
        (
            "candidates",
            J::Array(
                scores
                    .iter()
                    .map(|(p, s, e)| {
                        object([
                            (
                                "parameters",
                                J::Object(
                                    p.iter().map(|(k, v)| (k.clone(), J::Number(*v))).collect(),
                                ),
                            ),
                            ("training_score", number(*s)),
                            ("error", e.clone().map(J::String).unwrap_or(J::Null)),
                        ])
                    })
                    .collect(),
            ),
        ),
        ("evaluation", J::Array(evaluation)),
    ]);
    super::jobs::write_output_atomic(output, |p| {
        std::fs::write(p, report.to_json()?).map_err(|e| e.to_string())
    })?;
    Ok(report)
}
