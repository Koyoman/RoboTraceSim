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
        "batch" => {
            Err("batch is planned after v0.4; use repeated 'run' commands for now".to_string())
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
        return Err("v0.4 export supports only --format csv".to_string());
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

fn default_export_path(input: &Path) -> PathBuf {
    let mut out = input.to_path_buf();
    out.set_extension("csv");
    out
}

fn print_run_summary(summary: &RunSummary) {
    println!("Robotrace Sim v0.4 headless run complete");
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
    println!("Robotrace Sim v0.4 benchmark");
    println!("project:          {}", summary.project_name);
    println!("simulated time:   {:.6} s", summary.simulated_time_s);
    println!("steps:            {}", summary.steps);
    println!("wall time:        {:.6} s", summary.wall_time.as_secs_f64());
    println!("steps/s:          {:.0}", summary.steps_per_second);
    println!("real-time factor: {:.2}x", summary.realtime_factor);
}

fn print_help() {
    println!("Robotrace Sim v0.4");
    println!();
    println!("USAGE:");
    println!("  robotrace-sim                         # abre a interface única v0.4");
    println!("  robotrace-sim ui                      # abre a interface única v0.4");
    println!("  robotrace-sim run <projeto.rtsim> [--headless] [--duration 10s] [--csv out.csv] [--replay out.rtlog]");
    println!("  robotrace-sim benchmark <projeto.rtsim> [--duration 10s] [--physics-dt-us 500]");
    println!("  robotrace-sim export <resultado.rtlog> --format csv [--output resultado.csv]");
    println!();
    println!("v0.4 adds a unified egui/eframe interface with Home, track editor, robot editor, visual simulator and replay viewer while preserving the deterministic CLI core.");
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
