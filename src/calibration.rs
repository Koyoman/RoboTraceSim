use crate::config::LoadedConfig;
use crate::math::wrap_angle;
use crate::sim::SimulationSession;
use crate::telemetry::TelemetrySample;
use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::Path;

#[derive(Debug, Clone)]
pub struct RealLog {
    pub source: String,
    pub sensor_count: usize,
    pub samples: Vec<RealLogSample>,
}

#[derive(Debug, Clone)]
pub struct RealLogSample {
    pub t_us: u64,
    pub x_m: Option<f64>,
    pub y_m: Option<f64>,
    pub yaw_rad: Option<f64>,
    pub vx_body_m_s: Option<f64>,
    pub yaw_rate_rad_s: Option<f64>,
    pub line_position_m: Option<f64>,
    pub line_error_m: Option<f64>,
    pub sensor_adc: Vec<Option<f64>>,
}

impl RealLogSample {
    fn empty(t_us: u64, sensor_count: usize) -> Self {
        Self {
            t_us,
            x_m: None,
            y_m: None,
            yaw_rad: None,
            vx_body_m_s: None,
            yaw_rate_rad_s: None,
            line_position_m: None,
            line_error_m: None,
            sensor_adc: vec![None; sensor_count],
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MetricStats {
    pub count: usize,
    pub mean_abs: f64,
    pub rms: f64,
    pub max_abs: f64,
}

impl MetricStats {
    fn from_values(values: &[f64]) -> Self {
        if values.is_empty() {
            return Self::default();
        }
        let count = values.len();
        let mut sum_abs = 0.0;
        let mut sum_sq = 0.0;
        let mut max_abs = 0.0;
        for value in values {
            let abs = value.abs();
            sum_abs += abs;
            sum_sq += value * value;
            if abs > max_abs {
                max_abs = abs;
            }
        }
        Self {
            count,
            mean_abs: sum_abs / count as f64,
            rms: (sum_sq / count as f64).sqrt(),
            max_abs,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ComparisonMetrics {
    pub aligned_samples: usize,
    pub real_samples: usize,
    pub sim_samples: usize,
    pub duration_us: u64,
    pub trajectory_error_m: MetricStats,
    pub yaw_error_rad: MetricStats,
    pub speed_error_m_s: MetricStats,
    pub yaw_rate_error_rad_s: MetricStats,
    pub line_position_error_m: MetricStats,
    pub line_error_m: MetricStats,
    pub sensor_error_adc: MetricStats,
    pub score: f64,
}

#[derive(Debug, Clone)]
pub struct ComparisonReport {
    pub metrics: ComparisonMetrics,
    pub rows: Vec<ComparisonRow>,
}

#[derive(Debug, Clone)]
pub struct ComparisonRow {
    pub t_us: u64,
    pub sim_x_m: f64,
    pub real_x_m: Option<f64>,
    pub sim_y_m: f64,
    pub real_y_m: Option<f64>,
    pub sim_yaw_rad: f64,
    pub real_yaw_rad: Option<f64>,
    pub trajectory_error_m: Option<f64>,
    pub yaw_error_rad: Option<f64>,
    pub speed_error_m_s: Option<f64>,
    pub line_position_error_m: Option<f64>,
    pub line_error_m: Option<f64>,
    pub sensor_rmse_adc: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct TuningCandidate {
    pub mu_longitudinal: f64,
    pub stall_torque_scale: f64,
    pub stall_torque_left_nm: f64,
    pub stall_torque_right_nm: f64,
    pub metrics: ComparisonMetrics,
}

#[derive(Debug, Clone)]
pub struct TuningReport {
    pub baseline: ComparisonMetrics,
    pub best: TuningCandidate,
    pub evaluated_candidates: usize,
}

#[derive(Debug, Clone, Copy)]
struct CsvMapping {
    t_us: Option<usize>,
    t_s: Option<usize>,
    t_ms: Option<usize>,
    x_m: Option<usize>,
    y_m: Option<usize>,
    yaw_rad: Option<usize>,
    vx_body_m_s: Option<usize>,
    yaw_rate_rad_s: Option<usize>,
    line_position_m: Option<usize>,
    line_error_m: Option<usize>,
}

pub fn import_real_log(input: &Path) -> Result<RealLog, String> {
    let file = File::open(input)
        .map_err(|e| format!("failed to open real log {}: {e}", input.display()))?;
    let mut reader = BufReader::new(file);
    let mut header_line = String::new();
    reader
        .read_line(&mut header_line)
        .map_err(|e| format!("failed to read CSV header: {e}"))?;
    if header_line.trim().is_empty() {
        return Err("real log CSV is empty".to_string());
    }

    let headers = split_csv_line(header_line.trim_end_matches(&['\r', '\n'][..]))
        .into_iter()
        .map(|h| normalize_header(&h))
        .collect::<Vec<_>>();
    let mapping = map_headers(&headers);
    if mapping.t_us.is_none() && mapping.t_s.is_none() && mapping.t_ms.is_none() {
        return Err(
            "real log needs a time column: t_us, time_us, t_s, time_s, t_ms or time_ms".to_string(),
        );
    }
    let sensor_columns = map_sensor_columns(&headers);
    let sensor_count = sensor_columns
        .iter()
        .filter_map(|(idx, _col)| idx.checked_add(1))
        .max()
        .unwrap_or(0);

    let mut samples = Vec::new();
    for (line_idx, line) in reader.lines().enumerate() {
        let line = line.map_err(|e| format!("failed to read CSV line {}: {e}", line_idx + 2))?;
        if line.trim().is_empty() {
            continue;
        }
        let fields = split_csv_line(&line);
        let Some(t_us) = parse_time_us(&fields, mapping) else {
            continue;
        };
        let mut sample = RealLogSample::empty(t_us, sensor_count);
        sample.x_m = get_num(&fields, mapping.x_m);
        sample.y_m = get_num(&fields, mapping.y_m);
        sample.yaw_rad = get_num(&fields, mapping.yaw_rad);
        sample.vx_body_m_s = get_num(&fields, mapping.vx_body_m_s);
        sample.yaw_rate_rad_s = get_num(&fields, mapping.yaw_rate_rad_s);
        sample.line_position_m = get_num(&fields, mapping.line_position_m);
        sample.line_error_m = get_num(&fields, mapping.line_error_m);
        for (sensor_idx, col_idx) in &sensor_columns {
            if *sensor_idx < sample.sensor_adc.len() {
                sample.sensor_adc[*sensor_idx] = get_num(&fields, Some(*col_idx));
            }
        }
        samples.push(sample);
    }

    samples.sort_by_key(|s| s.t_us);
    samples.dedup_by_key(|s| s.t_us);
    if samples.is_empty() {
        return Err("real log contains no usable samples".to_string());
    }
    if let Some(t0) = samples.first().map(|s| s.t_us) {
        for sample in &mut samples {
            sample.t_us = sample.t_us.saturating_sub(t0);
        }
    }

    Ok(RealLog {
        source: input.display().to_string(),
        sensor_count,
        samples,
    })
}

pub fn write_normalized_real_log(log: &RealLog, output: &Path) -> io::Result<()> {
    let mut writer = BufWriter::new(File::create(output)?);
    write!(
        writer,
        "t_us,t_s,x_m,y_m,yaw_rad,vx_body_m_s,yaw_rate_rad_s,line_position_m,line_error_m"
    )?;
    for i in 0..log.sensor_count {
        write!(writer, ",sensor_{:02}_adc", i)?;
    }
    writeln!(writer)?;
    for sample in &log.samples {
        write!(
            writer,
            "{},{}",
            sample.t_us,
            sample.t_us as f64 / 1_000_000.0
        )?;
        write_optional(&mut writer, sample.x_m)?;
        write_optional(&mut writer, sample.y_m)?;
        write_optional(&mut writer, sample.yaw_rad)?;
        write_optional(&mut writer, sample.vx_body_m_s)?;
        write_optional(&mut writer, sample.yaw_rate_rad_s)?;
        write_optional(&mut writer, sample.line_position_m)?;
        write_optional(&mut writer, sample.line_error_m)?;
        for i in 0..log.sensor_count {
            write_optional(&mut writer, sample.sensor_adc.get(i).and_then(|v| *v))?;
        }
        writeln!(writer)?;
    }
    writer.flush()
}

pub fn run_simulation_samples(
    cfg: LoadedConfig,
    duration_us: Option<u64>,
) -> Result<Vec<TelemetrySample>, String> {
    let log_period_us = cfg
        .project
        .time
        .log_period_us
        .max(cfg.project.time.physics_dt_us);
    let mut session = SimulationSession::new(cfg, duration_us)?;
    let mut samples = Vec::new();
    let mut next_log_us = 0u64;

    loop {
        let t_us = session.time_us();
        if t_us >= next_log_us {
            samples.push(session.sample());
            next_log_us = next_log_us.saturating_add(log_period_us);
        }
        if session.is_finished() {
            break;
        }
        session.step_once();
    }
    Ok(samples)
}

pub fn compare_project_with_real(
    cfg: LoadedConfig,
    real: &RealLog,
    duration_us: Option<u64>,
) -> Result<ComparisonReport, String> {
    let duration = duration_us.unwrap_or_else(|| real.samples.last().map(|s| s.t_us).unwrap_or(0));
    let sim_samples = run_simulation_samples(cfg, Some(duration))?;
    Ok(compare_samples(&sim_samples, real))
}

pub fn compare_samples(sim_samples: &[TelemetrySample], real: &RealLog) -> ComparisonReport {
    let mut rows = Vec::new();
    let mut trajectory_errors = Vec::new();
    let mut yaw_errors = Vec::new();
    let mut speed_errors = Vec::new();
    let mut yaw_rate_errors = Vec::new();
    let mut line_position_errors = Vec::new();
    let mut line_errors = Vec::new();
    let mut sensor_errors = Vec::new();

    if sim_samples.is_empty() || real.samples.is_empty() {
        return ComparisonReport {
            metrics: ComparisonMetrics::default(),
            rows,
        };
    }

    for real_sample in &real.samples {
        let Some(sim) = interpolated_sim_sample(sim_samples, real_sample.t_us) else {
            continue;
        };
        let trajectory_error_m = match (real_sample.x_m, real_sample.y_m) {
            (Some(x), Some(y)) => {
                let e = ((sim.x_m - x).powi(2) + (sim.y_m - y).powi(2)).sqrt();
                trajectory_errors.push(e);
                Some(e)
            }
            _ => None,
        };
        let yaw_error_rad = real_sample.yaw_rad.map(|yaw| {
            let e = wrap_angle(sim.yaw_rad - yaw);
            yaw_errors.push(e);
            e
        });
        let speed_error_m_s = real_sample.vx_body_m_s.map(|v| {
            let e = sim.vx_body_m_s - v;
            speed_errors.push(e);
            e
        });
        let _yaw_rate_error_rad_s = real_sample.yaw_rate_rad_s.map(|v| {
            let e = sim.yaw_rate_rad_s - v;
            yaw_rate_errors.push(e);
            e
        });
        let line_position_error_m = real_sample.line_position_m.map(|v| {
            let e = sim.line_position_m - v;
            line_position_errors.push(e);
            e
        });
        let line_error_m = real_sample.line_error_m.map(|v| {
            let e = sim.line_error_m - v;
            line_errors.push(e);
            e
        });

        let mut sensor_sq = 0.0;
        let mut sensor_count = 0usize;
        for (idx, real_adc) in real_sample.sensor_adc.iter().enumerate() {
            if let Some(real_adc) = real_adc {
                if let Some(sim_adc) = sim.sensor_adc.get(idx) {
                    let e = *sim_adc as f64 - *real_adc;
                    sensor_errors.push(e);
                    sensor_sq += e * e;
                    sensor_count += 1;
                }
            }
        }
        let sensor_rmse_adc = if sensor_count > 0 {
            Some((sensor_sq / sensor_count as f64).sqrt())
        } else {
            None
        };

        rows.push(ComparisonRow {
            t_us: real_sample.t_us,
            sim_x_m: sim.x_m,
            real_x_m: real_sample.x_m,
            sim_y_m: sim.y_m,
            real_y_m: real_sample.y_m,
            sim_yaw_rad: sim.yaw_rad,
            real_yaw_rad: real_sample.yaw_rad,
            trajectory_error_m,
            yaw_error_rad,
            speed_error_m_s,
            line_position_error_m,
            line_error_m,
            sensor_rmse_adc,
        });
    }

    let metrics = ComparisonMetrics {
        aligned_samples: rows.len(),
        real_samples: real.samples.len(),
        sim_samples: sim_samples.len(),
        duration_us: rows.last().map(|r| r.t_us).unwrap_or(0),
        trajectory_error_m: MetricStats::from_values(&trajectory_errors),
        yaw_error_rad: MetricStats::from_values(&yaw_errors),
        speed_error_m_s: MetricStats::from_values(&speed_errors),
        yaw_rate_error_rad_s: MetricStats::from_values(&yaw_rate_errors),
        line_position_error_m: MetricStats::from_values(&line_position_errors),
        line_error_m: MetricStats::from_values(&line_errors),
        sensor_error_adc: MetricStats::from_values(&sensor_errors),
        score: score_from_errors(
            &trajectory_errors,
            &yaw_errors,
            &speed_errors,
            &line_position_errors,
            &line_errors,
            &sensor_errors,
        ),
    };

    ComparisonReport { metrics, rows }
}

pub fn write_comparison_csv(report: &ComparisonReport, output: &Path) -> io::Result<()> {
    let mut writer = BufWriter::new(File::create(output)?);
    writeln!(writer, "t_us,t_s,sim_x_m,real_x_m,sim_y_m,real_y_m,sim_yaw_rad,real_yaw_rad,trajectory_error_m,yaw_error_rad,speed_error_m_s,line_position_error_m,line_error_m,sensor_rmse_adc")?;
    for row in &report.rows {
        write!(writer, "{},{}", row.t_us, row.t_us as f64 / 1_000_000.0)?;
        write!(writer, ",{:.9}", row.sim_x_m)?;
        write_optional(&mut writer, row.real_x_m)?;
        write!(writer, ",{:.9}", row.sim_y_m)?;
        write_optional(&mut writer, row.real_y_m)?;
        write!(writer, ",{:.9}", row.sim_yaw_rad)?;
        write_optional(&mut writer, row.real_yaw_rad)?;
        write_optional(&mut writer, row.trajectory_error_m)?;
        write_optional(&mut writer, row.yaw_error_rad)?;
        write_optional(&mut writer, row.speed_error_m_s)?;
        write_optional(&mut writer, row.line_position_error_m)?;
        write_optional(&mut writer, row.line_error_m)?;
        write_optional(&mut writer, row.sensor_rmse_adc)?;
        writeln!(writer)?;
    }
    writer.flush()
}

pub fn write_comparison_report(report: &ComparisonReport, output: &Path) -> io::Result<()> {
    let mut writer = BufWriter::new(File::create(output)?);
    writeln!(
        writer,
        "Robotrace Sim v0.08 - comparação simulação vs robô real"
    )?;
    writeln!(writer, "amostras reais: {}", report.metrics.real_samples)?;
    writeln!(writer, "amostras simuladas: {}", report.metrics.sim_samples)?;
    writeln!(
        writer,
        "amostras alinhadas: {}",
        report.metrics.aligned_samples
    )?;
    writeln!(
        writer,
        "duração comparada [s]: {:.6}",
        report.metrics.duration_us as f64 / 1_000_000.0
    )?;
    write_metric(
        &mut writer,
        "erro de trajetória [m]",
        &report.metrics.trajectory_error_m,
    )?;
    write_metric(
        &mut writer,
        "erro de yaw [rad]",
        &report.metrics.yaw_error_rad,
    )?;
    write_metric(
        &mut writer,
        "erro de velocidade [m/s]",
        &report.metrics.speed_error_m_s,
    )?;
    write_metric(
        &mut writer,
        "erro de yaw rate [rad/s]",
        &report.metrics.yaw_rate_error_rad_s,
    )?;
    write_metric(
        &mut writer,
        "erro de posição da linha [m]",
        &report.metrics.line_position_error_m,
    )?;
    write_metric(
        &mut writer,
        "erro de linha/controlador [m]",
        &report.metrics.line_error_m,
    )?;
    write_metric(
        &mut writer,
        "erro de sensores [ADC]",
        &report.metrics.sensor_error_adc,
    )?;
    writeln!(writer, "score calibrável: {:.9}", report.metrics.score)?;
    writer.flush()
}

pub fn tune_project_against_real(
    cfg: LoadedConfig,
    real: &RealLog,
    duration_us: Option<u64>,
) -> Result<TuningReport, String> {
    let baseline_report = compare_project_with_real(cfg.clone(), real, duration_us)?;
    let baseline_metrics = baseline_report.metrics.clone();
    let base_left_torque = cfg.robot.motor_left.stall_torque_nm.max(1e-9);
    let base_right_torque = cfg.robot.motor_right.stall_torque_nm.max(1e-9);
    let base_left_current = cfg.robot.motor_left.stall_current_a.max(1e-9);
    let base_right_current = cfg.robot.motor_right.stall_current_a.max(1e-9);
    let base_mu = cfg.robot.tire.mu_longitudinal.max(1e-9);

    let mu_scales = [0.70, 0.85, 1.0, 1.15, 1.35, 1.60];
    let torque_scales = [0.70, 0.85, 1.0, 1.15, 1.35];
    let mut best: Option<TuningCandidate> = None;
    let mut evaluated = 0usize;

    for mu_scale in mu_scales {
        for torque_scale in torque_scales {
            let mut candidate_cfg = cfg.clone();
            candidate_cfg.robot.tire.mu_longitudinal = base_mu * mu_scale;
            candidate_cfg.robot.motor_left.stall_torque_nm = base_left_torque * torque_scale;
            candidate_cfg.robot.motor_right.stall_torque_nm = base_right_torque * torque_scale;
            candidate_cfg.robot.motor_left.stall_current_a = base_left_current * torque_scale;
            candidate_cfg.robot.motor_right.stall_current_a = base_right_current * torque_scale;
            let report = compare_project_with_real(candidate_cfg, real, duration_us)?;
            evaluated += 1;
            let candidate = TuningCandidate {
                mu_longitudinal: base_mu * mu_scale,
                stall_torque_scale: torque_scale,
                stall_torque_left_nm: base_left_torque * torque_scale,
                stall_torque_right_nm: base_right_torque * torque_scale,
                metrics: report.metrics,
            };
            let replace = best
                .as_ref()
                .map(|b| candidate.metrics.score < b.metrics.score)
                .unwrap_or(true);
            if replace {
                best = Some(candidate);
            }
        }
    }

    let best = best.ok_or_else(|| "no tuning candidates were evaluated".to_string())?;
    Ok(TuningReport {
        baseline: baseline_metrics,
        best,
        evaluated_candidates: evaluated,
    })
}

pub fn write_tuning_report(report: &TuningReport, output: &Path) -> io::Result<()> {
    let mut writer = BufWriter::new(File::create(output)?);
    writeln!(writer, "{{")?;
    writeln!(writer, "  \"schema\": \"rtsim-calibration-v1\",")?;
    writeln!(
        writer,
        "  \"evaluated_candidates\": {},",
        report.evaluated_candidates
    )?;
    writeln!(writer, "  \"baseline\": {{")?;
    write_metrics_json(&mut writer, &report.baseline, 4)?;
    writeln!(writer, "  }},")?;
    writeln!(writer, "  \"best_parameters\": {{")?;
    writeln!(
        writer,
        "    \"tire.mu_longitudinal\": {:.9},",
        report.best.mu_longitudinal
    )?;
    writeln!(
        writer,
        "    \"motor.stall_torque_scale\": {:.9},",
        report.best.stall_torque_scale
    )?;
    writeln!(
        writer,
        "    \"motors.left.stall_torque_mnm\": {:.9},",
        report.best.stall_torque_left_nm * 1000.0
    )?;
    writeln!(
        writer,
        "    \"motors.right.stall_torque_mnm\": {:.9}",
        report.best.stall_torque_right_nm * 1000.0
    )?;
    writeln!(writer, "  }},")?;
    writeln!(writer, "  \"best_metrics\": {{")?;
    write_metrics_json(&mut writer, &report.best.metrics, 4)?;
    writeln!(writer, "  }}")?;
    writeln!(writer, "}}")?;
    writer.flush()
}

pub fn print_metrics(metrics: &ComparisonMetrics) {
    println!("amostras reais:     {}", metrics.real_samples);
    println!("amostras simuladas: {}", metrics.sim_samples);
    println!("amostras alinhadas: {}", metrics.aligned_samples);
    println!(
        "duração comparada:  {:.6} s",
        metrics.duration_us as f64 / 1_000_000.0
    );
    print_metric_line("trajetória [m]", &metrics.trajectory_error_m);
    print_metric_line("yaw [rad]", &metrics.yaw_error_rad);
    print_metric_line("velocidade [m/s]", &metrics.speed_error_m_s);
    print_metric_line("sensores [ADC]", &metrics.sensor_error_adc);
    print_metric_line("linha [m]", &metrics.line_error_m);
    println!("score:              {:.9}", metrics.score);
}

fn parse_time_us(fields: &[String], mapping: CsvMapping) -> Option<u64> {
    if let Some(v) = get_num(fields, mapping.t_us) {
        return Some(v.round().max(0.0) as u64);
    }
    if let Some(v) = get_num(fields, mapping.t_s) {
        return Some((v * 1_000_000.0).round().max(0.0) as u64);
    }
    if let Some(v) = get_num(fields, mapping.t_ms) {
        return Some((v * 1_000.0).round().max(0.0) as u64);
    }
    None
}

fn map_headers(headers: &[String]) -> CsvMapping {
    CsvMapping {
        t_us: find_any(headers, &["t_us", "time_us", "timestamp_us", "timestampus"]),
        t_s: find_any(
            headers,
            &["t_s", "time_s", "time", "timestamp_s", "timestamps"],
        ),
        t_ms: find_any(headers, &["t_ms", "time_ms", "timestamp_ms", "timestampms"]),
        x_m: find_any(headers, &["x_m", "x", "pos_x_m", "pose_x_m"]),
        y_m: find_any(headers, &["y_m", "y", "pos_y_m", "pose_y_m"]),
        yaw_rad: find_any(headers, &["yaw_rad", "yaw", "theta_rad", "heading_rad"]),
        vx_body_m_s: find_any(
            headers,
            &[
                "vx_body_m_s",
                "speed_m_s",
                "v_m_s",
                "velocity_m_s",
                "vel_m_s",
            ],
        ),
        yaw_rate_rad_s: find_any(
            headers,
            &[
                "yaw_rate_rad_s",
                "gyro_yaw_rate_rad_s",
                "wz_rad_s",
                "omega_rad_s",
            ],
        ),
        line_position_m: find_any(
            headers,
            &["line_position_m", "line_pos_m", "line_position", "line_pos"],
        ),
        line_error_m: find_any(
            headers,
            &[
                "line_error_m",
                "error_m",
                "line_error",
                "controller_error_m",
            ],
        ),
    }
}

fn map_sensor_columns(headers: &[String]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for (col, header) in headers.iter().enumerate() {
        if let Some(idx) = parse_sensor_index(header) {
            out.push((idx, col));
        }
    }
    out.sort_by_key(|(idx, _)| *idx);
    out
}

fn parse_sensor_index(header: &str) -> Option<usize> {
    for prefix in ["sensor_", "sensor", "adc_", "adc"] {
        if let Some(rest) = header.strip_prefix(prefix) {
            let digits = rest
                .trim_start_matches('_')
                .trim_end_matches("_adc")
                .trim_end_matches("adc");
            if !digits.is_empty() && digits.chars().all(|c| c.is_ascii_digit()) {
                return digits.parse::<usize>().ok();
            }
        }
    }
    None
}

fn find_any(headers: &[String], names: &[&str]) -> Option<usize> {
    headers
        .iter()
        .position(|header| names.iter().any(|name| header == name))
}

fn normalize_header(header: &str) -> String {
    header
        .trim()
        .trim_matches('\u{feff}')
        .trim_matches('"')
        .to_ascii_lowercase()
        .replace(' ', "_")
        .replace('-', "_")
}

fn get_num(fields: &[String], idx: Option<usize>) -> Option<f64> {
    let idx = idx?;
    fields
        .get(idx)
        .map(|s| s.trim().trim_matches('"').replace(',', "."))
        .and_then(|s| {
            if s.is_empty() {
                None
            } else {
                s.parse::<f64>().ok().filter(|v| v.is_finite())
            }
        })
}

fn split_csv_line(line: &str) -> Vec<String> {
    let mut fields = Vec::new();
    let mut current = String::new();
    let mut chars = line.chars().peekable();
    let mut in_quotes = false;
    while let Some(ch) = chars.next() {
        match ch {
            '"' => {
                if in_quotes && chars.peek() == Some(&'"') {
                    current.push('"');
                    chars.next();
                } else {
                    in_quotes = !in_quotes;
                }
            }
            ',' if !in_quotes => {
                fields.push(current.trim().to_string());
                current.clear();
            }
            ';' if !in_quotes => {
                fields.push(current.trim().to_string());
                current.clear();
            }
            _ => current.push(ch),
        }
    }
    fields.push(current.trim().to_string());
    fields
}

fn interpolated_sim_sample(samples: &[TelemetrySample], t_us: u64) -> Option<TelemetrySample> {
    if samples.is_empty() || t_us < samples.first()?.t_us || t_us > samples.last()?.t_us {
        return None;
    }
    match samples.binary_search_by_key(&t_us, |s| s.t_us) {
        Ok(idx) => Some(samples[idx].clone()),
        Err(idx) => {
            if idx == 0 || idx >= samples.len() {
                return None;
            }
            let a = &samples[idx - 1];
            let b = &samples[idx];
            let denom = (b.t_us - a.t_us) as f64;
            let alpha = if denom <= 0.0 {
                0.0
            } else {
                (t_us - a.t_us) as f64 / denom
            };
            Some(lerp_sample(a, b, t_us, alpha))
        }
    }
}

fn lerp_sample(a: &TelemetrySample, b: &TelemetrySample, t_us: u64, alpha: f64) -> TelemetrySample {
    let sensor_count = a.sensor_adc.len().max(b.sensor_adc.len());
    let mut sensor_adc = Vec::with_capacity(sensor_count);
    for i in 0..sensor_count {
        let av = a.sensor_adc.get(i).copied().unwrap_or(0) as f64;
        let bv = b.sensor_adc.get(i).copied().unwrap_or(0) as f64;
        sensor_adc.push(lerp(av, bv, alpha).round().max(0.0) as u32);
    }
    TelemetrySample {
        t_us,
        x_m: lerp(a.x_m, b.x_m, alpha),
        y_m: lerp(a.y_m, b.y_m, alpha),
        yaw_rad: wrap_angle(a.yaw_rad + wrap_angle(b.yaw_rad - a.yaw_rad) * alpha),
        vx_body_m_s: lerp(a.vx_body_m_s, b.vx_body_m_s, alpha),
        vy_body_m_s: lerp(a.vy_body_m_s, b.vy_body_m_s, alpha),
        yaw_rate_rad_s: lerp(a.yaw_rate_rad_s, b.yaw_rate_rad_s, alpha),
        line_position_m: lerp(a.line_position_m, b.line_position_m, alpha),
        line_error_m: lerp(a.line_error_m, b.line_error_m, alpha),
        line_visible: if alpha < 0.5 {
            a.line_visible
        } else {
            b.line_visible
        },
        line_confidence: lerp(a.line_confidence, b.line_confidence, alpha),
        pwm_left: lerp(a.pwm_left, b.pwm_left, alpha),
        pwm_right: lerp(a.pwm_right, b.pwm_right, alpha),
        pwm_downforce: lerp(a.pwm_downforce, b.pwm_downforce, alpha),
        motor_current_left_a: lerp(a.motor_current_left_a, b.motor_current_left_a, alpha),
        motor_current_right_a: lerp(a.motor_current_right_a, b.motor_current_right_a, alpha),
        motor_torque_left_nm: lerp(a.motor_torque_left_nm, b.motor_torque_left_nm, alpha),
        motor_torque_right_nm: lerp(a.motor_torque_right_nm, b.motor_torque_right_nm, alpha),
        motor_voltage_left_v: lerp(a.motor_voltage_left_v, b.motor_voltage_left_v, alpha),
        motor_voltage_right_v: lerp(a.motor_voltage_right_v, b.motor_voltage_right_v, alpha),
        wheel_force_left_n: lerp(a.wheel_force_left_n, b.wheel_force_left_n, alpha),
        wheel_force_right_n: lerp(a.wheel_force_right_n, b.wheel_force_right_n, alpha),
        desired_wheel_force_left_n: lerp(
            a.desired_wheel_force_left_n,
            b.desired_wheel_force_left_n,
            alpha,
        ),
        desired_wheel_force_right_n: lerp(
            a.desired_wheel_force_right_n,
            b.desired_wheel_force_right_n,
            alpha,
        ),
        slip_left: lerp(a.slip_left, b.slip_left, alpha),
        slip_right: lerp(a.slip_right, b.slip_right, alpha),
        wheel_surface_speed_left_m_s: lerp(
            a.wheel_surface_speed_left_m_s,
            b.wheel_surface_speed_left_m_s,
            alpha,
        ),
        wheel_surface_speed_right_m_s: lerp(
            a.wheel_surface_speed_right_m_s,
            b.wheel_surface_speed_right_m_s,
            alpha,
        ),
        normal_left_n: lerp(a.normal_left_n, b.normal_left_n, alpha),
        normal_right_n: lerp(a.normal_right_n, b.normal_right_n, alpha),
        normal_front_left_n: lerp(a.normal_front_left_n, b.normal_front_left_n, alpha),
        normal_front_right_n: lerp(a.normal_front_right_n, b.normal_front_right_n, alpha),
        normal_rear_left_n: lerp(a.normal_rear_left_n, b.normal_rear_left_n, alpha),
        normal_rear_right_n: lerp(a.normal_rear_right_n, b.normal_rear_right_n, alpha),
        downforce_extra_n: lerp(a.downforce_extra_n, b.downforce_extra_n, alpha),
        downforce_fan_n: lerp(a.downforce_fan_n, b.downforce_fan_n, alpha),
        downforce_suction_n: lerp(a.downforce_suction_n, b.downforce_suction_n, alpha),
        downforce_current_a: lerp(a.downforce_current_a, b.downforce_current_a, alpha),
        battery_voltage_v: lerp(a.battery_voltage_v, b.battery_voltage_v, alpha),
        battery_current_a: lerp(a.battery_current_a, b.battery_current_a, alpha),
        encoder_left_ticks: lerp(
            a.encoder_left_ticks as f64,
            b.encoder_left_ticks as f64,
            alpha,
        )
        .round() as i64,
        encoder_right_ticks: lerp(
            a.encoder_right_ticks as f64,
            b.encoder_right_ticks as f64,
            alpha,
        )
        .round() as i64,
        encoder_left_velocity_rad_s: lerp(
            a.encoder_left_velocity_rad_s,
            b.encoder_left_velocity_rad_s,
            alpha,
        ),
        encoder_right_velocity_rad_s: lerp(
            a.encoder_right_velocity_rad_s,
            b.encoder_right_velocity_rad_s,
            alpha,
        ),
        gyro_yaw_rate_rad_s: lerp(a.gyro_yaw_rate_rad_s, b.gyro_yaw_rate_rad_s, alpha),
        gyro_bias_rad_s: lerp(a.gyro_bias_rad_s, b.gyro_bias_rad_s, alpha),
        sensor_adc,
    }
}

fn lerp(a: f64, b: f64, alpha: f64) -> f64 {
    a + (b - a) * alpha
}

fn score_from_errors(
    trajectory: &[f64],
    yaw: &[f64],
    speed: &[f64],
    line_position: &[f64],
    line_error: &[f64],
    sensors: &[f64],
) -> f64 {
    let items = [
        (trajectory, 3.0, 0.05),
        (yaw, 1.0, 0.35),
        (speed, 1.0, 1.0),
        (line_position, 2.0, 0.02),
        (line_error, 2.0, 0.02),
        (sensors, 1.0, 4096.0),
    ];
    let mut sum = 0.0;
    let mut weight = 0.0;
    for (values, w, scale) in items {
        if !values.is_empty() {
            let value = MetricStats::from_values(values).rms / scale;
            if value.is_finite() {
                sum += value * w;
                weight += w;
            }
        }
    }
    if weight > 0.0 {
        sum / weight
    } else {
        f64::INFINITY
    }
}

fn write_optional<W: Write>(writer: &mut W, value: Option<f64>) -> io::Result<()> {
    match value {
        Some(value) => write!(writer, ",{:.9}", value),
        None => write!(writer, ","),
    }
}

fn write_metric<W: Write>(writer: &mut W, label: &str, stats: &MetricStats) -> io::Result<()> {
    writeln!(
        writer,
        "{}: count={} mean_abs={:.9} rms={:.9} max_abs={:.9}",
        label, stats.count, stats.mean_abs, stats.rms, stats.max_abs
    )
}

fn print_metric_line(label: &str, stats: &MetricStats) {
    if stats.count == 0 {
        println!("{label:<18} sem dados");
    } else {
        println!(
            "{label:<18} rms={:.6} mean_abs={:.6} max={:.6} n={}",
            stats.rms, stats.mean_abs, stats.max_abs, stats.count
        );
    }
}

fn write_metrics_json<W: Write>(
    writer: &mut W,
    metrics: &ComparisonMetrics,
    indent: usize,
) -> io::Result<()> {
    let pad = " ".repeat(indent);
    writeln!(
        writer,
        "{}\"aligned_samples\": {},",
        pad, metrics.aligned_samples
    )?;
    writeln!(writer, "{}\"score\": {:.9},", pad, metrics.score)?;
    writeln!(
        writer,
        "{}\"trajectory_error_rms_m\": {:.9},",
        pad, metrics.trajectory_error_m.rms
    )?;
    writeln!(
        writer,
        "{}\"yaw_error_rms_rad\": {:.9},",
        pad, metrics.yaw_error_rad.rms
    )?;
    writeln!(
        writer,
        "{}\"speed_error_rms_m_s\": {:.9},",
        pad, metrics.speed_error_m_s.rms
    )?;
    writeln!(
        writer,
        "{}\"line_error_rms_m\": {:.9},",
        pad, metrics.line_error_m.rms
    )?;
    writeln!(
        writer,
        "{}\"sensor_error_rms_adc\": {:.9}",
        pad, metrics.sensor_error_adc.rms
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_quoted_csv_line() {
        let fields = split_csv_line("t_us,\"x,m\",sensor_00_adc");
        assert_eq!(fields, vec!["t_us", "x,m", "sensor_00_adc"]);
    }

    #[test]
    fn maps_sensor_headers() {
        let headers = vec!["sensor_00_adc".to_string(), "sensor_15_adc".to_string()];
        let sensors = map_sensor_columns(&headers);
        assert_eq!(sensors, vec![(0, 0), (15, 1)]);
    }
}
