//! Numerical qualification is separate from experimental validation.
use super::{
    calibration::{apply, simulate},
    metrics::{self, number, object},
};
use crate::{config::load_project, json::JsonValue as J};
use std::{collections::BTreeMap, path::Path};
pub fn run(project: &Path, output: &Path, duration_us: u64) -> Result<J, String> {
    if duration_us == 0 || duration_us % 100 != 0 {
        return Err("robustness duration must be positive multiple of 100 us".into());
    }
    let base = load_project(project).map_err(|e| e.to_string())?;
    run_config(base, output, duration_us)
}
pub fn run_config(
    base: crate::config::LoadedConfig,
    output: &Path,
    duration_us: u64,
) -> Result<J, String> {
    if duration_us == 0 || duration_us % 100 != 0 {
        return Err("robustness duration must be positive multiple of 100 us".into());
    }
    let mut attempts = Vec::new();
    let mut successful = Vec::new();
    for seed in [1371u64, 1372, 1373] {
        for scale in [0.95, 1., 1.05] {
            for dt in [100u64, 50, 25] {
                super::jobs::check_cancelled()?;
                let mut cfg = base.clone();
                cfg.project.time.physics_dt_us = dt;
                for (i, s) in cfg.robot.sensors.iter_mut().enumerate() {
                    s.acquisition.seed = seed + i as u64 * 97;
                }
                cfg.robot.gyro.seed = seed + 300;
                cfg.robot.sensing.encoder.seed = seed + 400;
                cfg.robot.sensing.imu.seed = seed + 500;
                apply(
                    &mut cfg,
                    &[("mu_longitudinal_scale".into(), scale)]
                        .into_iter()
                        .collect(),
                )?;
                let values = [
                    ("physics_dt_us", J::Number(dt as f64)),
                    ("seed", J::Number(seed as f64)),
                    ("mu_longitudinal_scale", J::Number(scale)),
                ];
                let mut row = match object(values) {
                    J::Object(v) => v,
                    _ => unreachable!(),
                };
                match simulate(cfg, duration_us) {
                    Ok(p) => {
                        row.insert("wall_s".into(), J::Number(p.wall_s));
                        row.insert("termination".into(), J::String(p.termination));
                        row.insert("snapshot".into(), p.snapshot);
                        row.insert(
                            "derived".into(),
                            metrics::derived(&p.series, base.project.time.log_period_us as i64),
                        );
                        if let Some(last) = p.series.rows.last() {
                            row.insert(
                                "final".into(),
                                J::Object(
                                    last.values
                                        .iter()
                                        .map(|(k, v)| (k.clone(), J::Number(*v)))
                                        .collect(),
                                ),
                            );
                        }
                        successful.push((seed, scale, dt, p.series));
                    }
                    Err(e) => {
                        row.insert("error".into(), J::String(e));
                    }
                }
                attempts.push(J::Object(row));
            }
        }
    }
    let mut comparisons = Vec::new();
    for (seed, scale, dt, series) in &successful {
        if *dt == 25 {
            continue;
        }
        if let Some((_, _, _, fine)) = successful
            .iter()
            .find(|(s, p, d, _)| s == seed && p == scale && *d == 25)
        {
            comparisons.push(object([
                ("seed", J::Number(*seed as f64)),
                ("mu_longitudinal_scale", J::Number(*scale)),
                ("physics_dt_us", J::Number(*dt as f64)),
                ("reference_dt_us", J::Number(25.)),
                (
                    "metrics",
                    J::Array(
                        ["x_m", "y_m", "yaw_rad", "vx_body_m_s", "battery_current_a"]
                            .iter()
                            .map(|s| {
                                metrics::compare(
                                    series,
                                    fine,
                                    s,
                                    base.project.time.log_period_us as i64,
                                )
                                .json()
                            })
                            .collect(),
                    ),
                ),
            ]));
        }
    }
    let mut dispersion = BTreeMap::new();
    for signal in ["x_m", "y_m", "yaw_rad", "vx_body_m_s"] {
        let values: Vec<_> = successful
            .iter()
            .filter(|(_, _, dt, _)| *dt == 25)
            .filter_map(|(_, _, _, s)| s.rows.last()?.values.get(signal).copied())
            .collect();
        let mean = (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64);
        let sd = mean.map(|m| {
            (values.iter().map(|x| (x - m).powi(2)).sum::<f64>() / values.len() as f64).sqrt()
        });
        dispersion.insert(
            signal.into(),
            object([
                ("count", J::Number(values.len() as f64)),
                ("mean", number(mean)),
                ("population_stddev", number(sd)),
                ("min", number(values.iter().copied().reduce(f64::min))),
                ("max", number(values.iter().copied().reduce(f64::max))),
            ]),
        );
    }
    let report = object([
        ("schema", J::String("rtsim-robustness-v1".into())),
        ("duration_us", J::Number(duration_us as f64)),
        ("claim", J::String("numerical sensitivity only; 25 us is a reference, not physical ground truth; logical/sensor/log periods unchanged".into())),
        ("attempts", J::Array(attempts)),
        ("refinement", J::Array(comparisons)),
        ("finest_step_dispersion", J::Object(dispersion)),
    ]);
    super::jobs::write_output_atomic(output, |p| {
        std::fs::write(p, report.to_json()?).map_err(|e| e.to_string())
    })?;
    Ok(report)
}
