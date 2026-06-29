use crate::calibration::{
    compare_project_with_real, import_real_log, print_metrics, tune_project_against_real,
    write_comparison_csv, write_comparison_report, write_normalized_real_log, write_tuning_report,
};
use crate::config::load_project;
use crate::replay::export_replay_to_csv;
use crate::sim::{run_simulation, RunOptions, RunSummary};
use std::path::{Path, PathBuf};

pub fn run_cli(args: Vec<String>) -> Result<(), String> {
    if args.len() < 2 {
        print_help();
        return Ok(());
    }

    match args[1].as_str() {
        "run" => run_command(&args[2..]),
        "benchmark" => benchmark_command(&args[2..]),
        "export" => export_command(&args[2..]),
        "import-log" => import_log_command(&args[2..]),
        "compare" => compare_command(&args[2..]),
        "tune" | "calibrate" => tune_command(&args[2..]),
        "batch" => {
            Err("batch is planned after v0.08; use repeated 'run' commands for now".to_string())
        }
        "help" | "--help" | "-h" => {
            print_help();
            Ok(())
        }
        other => Err(format!("unknown command '{other}'")),
    }
}

fn run_command(args: &[String]) -> Result<(), String> {
    let mut project: Option<PathBuf> = None;
    let mut headless = false;
    let mut output_csv: Option<PathBuf> = None;
    let mut output_replay: Option<PathBuf> = None;
    let mut duration_us: Option<u64> = None;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--headless" => headless = true,
            "--duration" => {
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| "--duration needs a value".to_string())?;
                duration_us = Some(parse_duration_us(value)?);
            }
            flag if flag.starts_with("--duration=") => {
                duration_us = Some(parse_duration_us(flag.trim_start_matches("--duration="))?);
            }
            "--output" | "-o" => {
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| "--output needs a file path".to_string())?;
                assign_output_path(PathBuf::from(value), &mut output_csv, &mut output_replay);
            }
            flag if flag.starts_with("--output=") => {
                assign_output_path(
                    PathBuf::from(flag.trim_start_matches("--output=")),
                    &mut output_csv,
                    &mut output_replay,
                );
            }
            "--csv" => {
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| "--csv needs a file path".to_string())?;
                output_csv = Some(PathBuf::from(value));
            }
            flag if flag.starts_with("--csv=") => {
                output_csv = Some(PathBuf::from(flag.trim_start_matches("--csv=")));
            }
            "--replay" | "--rtlog" => {
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| "--replay needs a file path".to_string())?;
                output_replay = Some(PathBuf::from(value));
            }
            flag if flag.starts_with("--replay=") => {
                output_replay = Some(PathBuf::from(flag.trim_start_matches("--replay=")));
            }
            "--help" | "-h" => {
                print_run_help();
                return Ok(());
            }
            value if value.starts_with('-') => return Err(format!("unknown run option '{value}'")),
            value => {
                if project.is_some() {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
                project = Some(PathBuf::from(value));
            }
        }
        i += 1;
    }

    let project = project.ok_or_else(|| "missing project file. Example: robotrace-sim run projeto.rtsim --headless --duration 10s".to_string())?;
    let cfg = load_project(&project).map_err(|e| e.to_string())?;
    let summary = run_simulation(
        cfg,
        RunOptions {
            duration_us,
            output_csv,
            output_replay,
            headless,
            benchmark: false,
            physics_dt_override_us: None,
        },
    )?;
    print_run_summary(&summary);
    Ok(())
}

fn benchmark_command(args: &[String]) -> Result<(), String> {
    let mut project: Option<PathBuf> = None;
    let mut duration_us: Option<u64> = None;
    let mut physics_dt_override_us: Option<u64> = None;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--duration" => {
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| "--duration needs a value".to_string())?;
                duration_us = Some(parse_duration_us(value)?);
            }
            flag if flag.starts_with("--duration=") => {
                duration_us = Some(parse_duration_us(flag.trim_start_matches("--duration="))?);
            }
            "--physics-dt-us" => {
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| "--physics-dt-us needs a value".to_string())?;
                physics_dt_override_us = Some(
                    value
                        .parse::<u64>()
                        .map_err(|_| "invalid --physics-dt-us".to_string())?,
                );
            }
            flag if flag.starts_with("--physics-dt-us=") => {
                physics_dt_override_us = Some(
                    flag.trim_start_matches("--physics-dt-us=")
                        .parse::<u64>()
                        .map_err(|_| "invalid --physics-dt-us".to_string())?,
                );
            }
            "--help" | "-h" => {
                print_benchmark_help();
                return Ok(());
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown benchmark option '{value}'"))
            }
            value => {
                if project.is_some() {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
                project = Some(PathBuf::from(value));
            }
        }
        i += 1;
    }

    let project = project.ok_or_else(|| "missing project file. Example: robotrace-sim benchmark projeto.rtsim --physics-dt-us 500".to_string())?;
    let cfg = load_project(&project).map_err(|e| e.to_string())?;
    let summary = run_simulation(
        cfg,
        RunOptions {
            duration_us,
            output_csv: None,
            output_replay: None,
            headless: true,
            benchmark: true,
            physics_dt_override_us,
        },
    )?;
    print_benchmark_summary(&summary);
    Ok(())
}

fn export_command(args: &[String]) -> Result<(), String> {
    let mut input: Option<PathBuf> = None;
    let mut output: Option<PathBuf> = None;
    let mut format = "csv".to_string();

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--format" => {
                i += 1;
                format = args
                    .get(i)
                    .ok_or_else(|| "--format needs a value".to_string())?
                    .to_string();
            }
            flag if flag.starts_with("--format=") => {
                format = flag.trim_start_matches("--format=").to_string();
            }
            "--output" | "-o" => {
                i += 1;
                output = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--output needs a file path".to_string())?,
                ));
            }
            flag if flag.starts_with("--output=") => {
                output = Some(PathBuf::from(flag.trim_start_matches("--output=")));
            }
            "--help" | "-h" => {
                print_export_help();
                return Ok(());
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown export option '{value}'"))
            }
            value => {
                if input.is_some() {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
                input = Some(PathBuf::from(value));
            }
        }
        i += 1;
    }

    if !format.eq_ignore_ascii_case("csv") {
        return Err("v0.08 export supports only --format csv".to_string());
    }
    let input = input.ok_or_else(|| "missing replay input. Example: robotrace-sim export resultado.rtlog --format csv --output resultado.csv".to_string())?;
    let output = output.unwrap_or_else(|| default_export_path(&input));
    let rows = export_replay_to_csv(&input, &output)
        .map_err(|e| format!("failed to export {}: {e}", input.display()))?;
    println!("exported {rows} replay samples to {}", output.display());
    Ok(())
}

pub fn parse_duration_us(text: &str) -> Result<u64, String> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Err("empty duration".to_string());
    }
    let (number, multiplier) = if let Some(n) = trimmed.strip_suffix("us") {
        (n, 1.0)
    } else if let Some(n) = trimmed.strip_suffix("ms") {
        (n, 1_000.0)
    } else if let Some(n) = trimmed.strip_suffix('s') {
        (n, 1_000_000.0)
    } else {
        (trimmed, 1_000_000.0)
    };
    let value = number
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("invalid duration '{text}'"))?;
    if !value.is_finite() || value < 0.0 {
        return Err("duration must be a finite non-negative value".to_string());
    }
    Ok((value * multiplier).round() as u64)
}

fn assign_output_path(
    path: PathBuf,
    output_csv: &mut Option<PathBuf>,
    output_replay: &mut Option<PathBuf>,
) {
    match path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase()
        .as_str()
    {
        "rtlog" | "rtsr" => *output_replay = Some(path),
        _ => *output_csv = Some(path),
    }
}

fn import_log_command(args: &[String]) -> Result<(), String> {
    let mut input: Option<PathBuf> = None;
    let mut output: Option<PathBuf> = None;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--output" | "-o" => {
                i += 1;
                output = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--output needs a file path".to_string())?,
                ));
            }
            flag if flag.starts_with("--output=") => {
                output = Some(PathBuf::from(flag.trim_start_matches("--output=")));
            }
            "--help" | "-h" => {
                print_import_log_help();
                return Ok(());
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown import-log option '{value}'"))
            }
            value => {
                if input.is_some() {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
                input = Some(PathBuf::from(value));
            }
        }
        i += 1;
    }

    let input = input.ok_or_else(|| "missing real CSV log. Example: robotrace-sim import-log real.csv --output real_normalized.csv".to_string())?;
    let output = output.unwrap_or_else(|| {
        let mut out = input.clone();
        out.set_extension("normalized.csv");
        out
    });
    let real = import_real_log(&input)?;
    write_normalized_real_log(&real, &output)
        .map_err(|e| format!("failed to write {}: {e}", output.display()))?;
    println!(
        "imported {} real samples from {}",
        real.samples.len(),
        input.display()
    );
    println!("normalized CSV: {}", output.display());
    println!("detected sensors: {}", real.sensor_count);
    Ok(())
}

fn compare_command(args: &[String]) -> Result<(), String> {
    let mut project: Option<PathBuf> = None;
    let mut real_path: Option<PathBuf> = None;
    let mut output_csv: Option<PathBuf> = None;
    let mut report_path: Option<PathBuf> = None;
    let mut duration_us: Option<u64> = None;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--real" | "--real-log" => {
                i += 1;
                real_path = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--real needs a CSV path".to_string())?,
                ));
            }
            flag if flag.starts_with("--real=") => {
                real_path = Some(PathBuf::from(flag.trim_start_matches("--real=")));
            }
            "--duration" => {
                i += 1;
                duration_us = Some(parse_duration_us(
                    args.get(i)
                        .ok_or_else(|| "--duration needs a value".to_string())?,
                )?);
            }
            flag if flag.starts_with("--duration=") => {
                duration_us = Some(parse_duration_us(flag.trim_start_matches("--duration="))?);
            }
            "--output" | "--csv" | "-o" => {
                i += 1;
                output_csv = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--output needs a file path".to_string())?,
                ));
            }
            flag if flag.starts_with("--output=") => {
                output_csv = Some(PathBuf::from(flag.trim_start_matches("--output=")));
            }
            flag if flag.starts_with("--csv=") => {
                output_csv = Some(PathBuf::from(flag.trim_start_matches("--csv=")));
            }
            "--report" => {
                i += 1;
                report_path = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--report needs a file path".to_string())?,
                ));
            }
            flag if flag.starts_with("--report=") => {
                report_path = Some(PathBuf::from(flag.trim_start_matches("--report=")));
            }
            "--help" | "-h" => {
                print_compare_help();
                return Ok(());
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown compare option '{value}'"))
            }
            value => {
                if project.is_some() {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
                project = Some(PathBuf::from(value));
            }
        }
        i += 1;
    }

    let project = project.ok_or_else(|| {
        "missing project file. Example: robotrace-sim compare projeto.rtsim --real real.csv"
            .to_string()
    })?;
    let real_path = real_path.ok_or_else(|| "missing --real <real.csv>".to_string())?;
    let cfg = load_project(&project).map_err(|e| e.to_string())?;
    let real = import_real_log(&real_path)?;
    let report = compare_project_with_real(cfg, &real, duration_us)?;

    println!("Robotrace Sim v0.08 comparison");
    println!("real log: {}", real.source);
    print_metrics(&report.metrics);

    if let Some(path) = output_csv {
        write_comparison_csv(&report, &path)
            .map_err(|e| format!("failed to write {}: {e}", path.display()))?;
        println!("comparison CSV: {}", path.display());
    }
    if let Some(path) = report_path {
        write_comparison_report(&report, &path)
            .map_err(|e| format!("failed to write {}: {e}", path.display()))?;
        println!("comparison report: {}", path.display());
    }
    Ok(())
}

fn tune_command(args: &[String]) -> Result<(), String> {
    let mut project: Option<PathBuf> = None;
    let mut real_path: Option<PathBuf> = None;
    let mut output: Option<PathBuf> = None;
    let mut duration_us: Option<u64> = None;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--real" | "--real-log" => {
                i += 1;
                real_path = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--real needs a CSV path".to_string())?,
                ));
            }
            flag if flag.starts_with("--real=") => {
                real_path = Some(PathBuf::from(flag.trim_start_matches("--real=")));
            }
            "--duration" => {
                i += 1;
                duration_us = Some(parse_duration_us(
                    args.get(i)
                        .ok_or_else(|| "--duration needs a value".to_string())?,
                )?);
            }
            flag if flag.starts_with("--duration=") => {
                duration_us = Some(parse_duration_us(flag.trim_start_matches("--duration="))?);
            }
            "--output" | "-o" => {
                i += 1;
                output = Some(PathBuf::from(
                    args.get(i)
                        .ok_or_else(|| "--output needs a file path".to_string())?,
                ));
            }
            flag if flag.starts_with("--output=") => {
                output = Some(PathBuf::from(flag.trim_start_matches("--output=")));
            }
            "--help" | "-h" => {
                print_tune_help();
                return Ok(());
            }
            value if value.starts_with('-') => {
                return Err(format!("unknown tune option '{value}'"))
            }
            value => {
                if project.is_some() {
                    return Err(format!("unexpected positional argument '{value}'"));
                }
                project = Some(PathBuf::from(value));
            }
        }
        i += 1;
    }

    let project = project.ok_or_else(|| "missing project file. Example: robotrace-sim tune projeto.rtsim --real real.csv --output ajuste.json".to_string())?;
    let real_path = real_path.ok_or_else(|| "missing --real <real.csv>".to_string())?;
    let output = output.unwrap_or_else(|| PathBuf::from("calibration_result.json"));
    let cfg = load_project(&project).map_err(|e| e.to_string())?;
    let real = import_real_log(&real_path)?;
    let report = tune_project_against_real(cfg, &real, duration_us)?;
    write_tuning_report(&report, &output)
        .map_err(|e| format!("failed to write {}: {e}", output.display()))?;

    println!("Robotrace Sim v0.08 parameter tuning");
    println!("evaluated candidates: {}", report.evaluated_candidates);
    println!("baseline score: {:.9}", report.baseline.score);
    println!("best score:     {:.9}", report.best.metrics.score);
    println!(
        "best tire.mu_longitudinal: {:.6}",
        report.best.mu_longitudinal
    );
    println!(
        "best motor torque scale:   {:.6}",
        report.best.stall_torque_scale
    );
    println!("tuning JSON: {}", output.display());
    Ok(())
}

fn default_export_path(input: &Path) -> PathBuf {
    let mut out = input.to_path_buf();
    out.set_extension("csv");
    out
}

fn print_run_summary(summary: &RunSummary) {
    println!("Robotrace Sim v0.08 headless run complete");
    println!("project: {}", summary.project_name);
    println!("robot:   {}", summary.robot_name);
    println!("track:   {}", summary.track_name);
    println!(
        "sim:     {:.6} s, {} fixed steps",
        summary.simulated_time_s, summary.steps
    );
    println!(
        "final:   x={:.4} m y={:.4} m yaw={:.4} rad",
        summary.final_pose.x, summary.final_pose.y, summary.final_pose.yaw
    );
    if let Some(path) = summary.csv_path.as_ref() {
        println!("csv:     {}", path.display());
    }
    if let Some(path) = summary.replay_path.as_ref() {
        println!("replay:  {}", path.display());
    }
}

fn print_benchmark_summary(summary: &RunSummary) {
    println!("Robotrace Sim v0.08 benchmark");
    println!("project:          {}", summary.project_name);
    println!("simulated time:   {:.6} s", summary.simulated_time_s);
    println!("steps:            {}", summary.steps);
    println!("wall time:        {:.6} s", summary.wall_time.as_secs_f64());
    println!("steps/s:          {:.0}", summary.steps_per_second);
    println!("real-time factor: {:.2}x", summary.realtime_factor);
}

fn print_help() {
    println!("Robotrace Sim v0.08");
    println!();
    println!("USAGE:");
    println!("  robotrace-sim                         # abre a interface única v0.08");
    println!("  robotrace-sim ui                      # abre a interface única v0.08");
    println!("  robotrace-sim run <projeto.rtsim> [--headless] [--duration 10s] [--csv out.csv] [--replay out.rtlog]");
    println!("  robotrace-sim benchmark <projeto.rtsim> [--duration 10s] [--physics-dt-us 500]");
    println!("  robotrace-sim export <resultado.rtlog> --format csv [--output resultado.csv]");
    println!("  robotrace-sim import-log <real.csv> [--output real_normalized.csv]");
    println!("  robotrace-sim compare <projeto.rtsim> --real real.csv [--output comparacao.csv] [--report comparacao.txt]");
    println!("  robotrace-sim tune <projeto.rtsim> --real real.csv [--output ajuste.json]");
    println!();
    println!("v0.08 adds real-log import, simulation-vs-real comparison, trajectory/sensor/speed error metrics and coarse parameter tuning.");
}

fn print_run_help() {
    println!("USAGE: robotrace-sim run <projeto.rtsim> --headless --duration 10s --csv resultado.csv --replay resultado.rtlog");
}

fn print_benchmark_help() {
    println!("USAGE: robotrace-sim benchmark <projeto.rtsim> --duration 10s --physics-dt-us 500");
}

fn print_export_help() {
    println!("USAGE: robotrace-sim export <resultado.rtlog> --format csv --output resultado.csv");
}

fn print_import_log_help() {
    println!("USAGE: robotrace-sim import-log <real.csv> --output real_normalized.csv");
    println!("Accepted time columns: t_us, time_us, t_s, time_s, t_ms, time_ms.");
}

fn print_compare_help() {
    println!("USAGE: robotrace-sim compare <projeto.rtsim> --real real.csv --output comparacao.csv --report comparacao.txt");
}

fn print_tune_help() {
    println!("USAGE: robotrace-sim tune <projeto.rtsim> --real real.csv --output ajuste.json");
    println!("The v0.08 tuner evaluates a deterministic coarse grid over tire friction and motor stall torque.");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_duration_suffixes() {
        assert_eq!(parse_duration_us("10s").unwrap(), 10_000_000);
        assert_eq!(parse_duration_us("2.5ms").unwrap(), 2_500);
        assert_eq!(parse_duration_us("42us").unwrap(), 42);
    }

    #[test]
    fn output_extension_selects_replay() {
        let mut csv = None;
        let mut replay = None;
        assign_output_path(PathBuf::from("run.rtlog"), &mut csv, &mut replay);
        assert!(csv.is_none());
        assert_eq!(replay.unwrap(), PathBuf::from("run.rtlog"));
    }
}
