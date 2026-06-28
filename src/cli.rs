use crate::config::load_project;
use crate::sim::{run_simulation, RunOptions, RunSummary};
use std::path::PathBuf;

pub fn run_cli(args: Vec<String>) -> Result<(), String> {
    if args.len() < 2 {
        print_help();
        return Ok(());
    }

    match args[1].as_str() {
        "run" => run_command(&args[2..]),
        "benchmark" => benchmark_command(&args[2..]),
        "batch" => Err("batch is planned after v0.1; use repeated 'run' commands for now".to_string()),
        "export" => Err("export .rrlog is planned after binary replay lands; v0.1 writes CSV directly".to_string()),
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
    let mut duration_us: Option<u64> = None;

    let mut i = 0usize;
    while i < args.len() {
        match args[i].as_str() {
            "--headless" => headless = true,
            "--duration" => {
                i += 1;
                let value = args.get(i).ok_or_else(|| "--duration needs a value".to_string())?;
                duration_us = Some(parse_duration_us(value)?);
            }
            flag if flag.starts_with("--duration=") => {
                duration_us = Some(parse_duration_us(flag.trim_start_matches("--duration="))?);
            }
            "--output" | "-o" => {
                i += 1;
                let value = args.get(i).ok_or_else(|| "--output needs a file path".to_string())?;
                output_csv = Some(PathBuf::from(value));
            }
            flag if flag.starts_with("--output=") => {
                output_csv = Some(PathBuf::from(flag.trim_start_matches("--output=")));
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
                let value = args.get(i).ok_or_else(|| "--duration needs a value".to_string())?;
                duration_us = Some(parse_duration_us(value)?);
            }
            flag if flag.starts_with("--duration=") => {
                duration_us = Some(parse_duration_us(flag.trim_start_matches("--duration="))?);
            }
            "--physics-dt-us" => {
                i += 1;
                let value = args.get(i).ok_or_else(|| "--physics-dt-us needs a value".to_string())?;
                physics_dt_override_us = Some(value.parse::<u64>().map_err(|_| "invalid --physics-dt-us".to_string())?);
            }
            flag if flag.starts_with("--physics-dt-us=") => {
                physics_dt_override_us = Some(flag.trim_start_matches("--physics-dt-us=").parse::<u64>().map_err(|_| "invalid --physics-dt-us".to_string())?);
            }
            "--help" | "-h" => {
                print_benchmark_help();
                return Ok(());
            }
            value if value.starts_with('-') => return Err(format!("unknown benchmark option '{value}'")),
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
            headless: true,
            benchmark: true,
            physics_dt_override_us,
        },
    )?;
    print_benchmark_summary(&summary);
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
    let value = number.trim().parse::<f64>().map_err(|_| format!("invalid duration '{text}'"))?;
    if !value.is_finite() || value < 0.0 {
        return Err("duration must be a finite non-negative value".to_string());
    }
    Ok((value * multiplier).round() as u64)
}

fn print_run_summary(summary: &RunSummary) {
    println!("Robotrace Sim v0.1 headless run complete");
    println!("project: {}", summary.project_name);
    println!("robot:   {}", summary.robot_name);
    println!("track:   {}", summary.track_name);
    println!("sim:     {:.6} s, {} fixed steps", summary.simulated_time_s, summary.steps);
    println!("final:   x={:.4} m y={:.4} m yaw={:.4} rad", summary.final_pose.x, summary.final_pose.y, summary.final_pose.yaw);
    if let Some(path) = summary.csv_path.as_ref() {
        println!("csv:     {}", path.display());
    }
}

fn print_benchmark_summary(summary: &RunSummary) {
    println!("Robotrace Sim v0.1 benchmark");
    println!("project:          {}", summary.project_name);
    println!("simulated time:   {:.6} s", summary.simulated_time_s);
    println!("steps:            {}", summary.steps);
    println!("wall time:        {:.6} s", summary.wall_time.as_secs_f64());
    println!("steps/s:          {:.0}", summary.steps_per_second);
    println!("real-time factor: {:.2}x", summary.realtime_factor);
}

fn print_help() {
    println!("Robotrace Sim v0.1");
    println!();
    println!("USAGE:");
    println!("  robotrace-sim run <projeto.rtsim> [--headless] [--duration 10s] [--output out.csv]");
    println!("  robotrace-sim benchmark <projeto.rtsim> [--duration 10s] [--physics-dt-us 500]");
    println!();
    println!("v0.1 implements the headless simulation core. UI, .rrlog replay, batch and export are placeholders.");
}

fn print_run_help() {
    println!("USAGE: robotrace-sim run <projeto.rtsim> --headless --duration 10s --output resultado.csv");
}

fn print_benchmark_help() {
    println!("USAGE: robotrace-sim benchmark <projeto.rtsim> --duration 10s --physics-dt-us 500");
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
}
