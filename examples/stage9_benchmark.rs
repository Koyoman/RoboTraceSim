//! Reproducible Stage 9 matrix. Run with --release; durations are simulated microseconds.
use robotrace_sim::{
    config::load_project,
    experiments::jobs::SimulationWorker,
    math::Vec2,
    sim::{run_simulation, RunOptions, SimulationCore},
};
use std::{
    io::Write,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
#[cfg(windows)]
fn peak_bytes() -> usize {
    #[repr(C)]
    #[derive(Default)]
    struct Counters {
        cb: u32,
        faults: u32,
        peak: usize,
        working: usize,
        paged_peak: usize,
        paged: usize,
        nonpaged_peak: usize,
        nonpaged: usize,
        pagefile: usize,
        pagefile_peak: usize,
    }
    unsafe extern "system" {
        fn GetCurrentProcess() -> *mut std::ffi::c_void;
        fn K32GetProcessMemoryInfo(h: *mut std::ffi::c_void, c: *mut Counters, n: u32) -> i32;
    }
    let mut c = Counters::default();
    c.cb = std::mem::size_of::<Counters>() as u32;
    unsafe {
        if K32GetProcessMemoryInfo(GetCurrentProcess(), &mut c, c.cb) == 0 {
            return 0;
        }
    }
    c.peak
}
#[cfg(not(windows))]
fn peak_bytes() -> usize {
    0
}
fn bytes(dir: &Path) -> u64 {
    std::fs::read_dir(dir)
        .unwrap()
        .filter_map(Result::ok)
        .map(|e| e.metadata().map(|m| m.len()).unwrap_or(0))
        .sum()
}
fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().collect();
    let root = PathBuf::from(
        args.get(1)
            .ok_or("usage: stage9_benchmark NEW_OUTPUT_DIR [repeats=3] [duration_us=100000]")?,
    );
    let repeats: usize = args.get(2).map(|v| v.parse().unwrap()).unwrap_or(3);
    let duration: u64 = args.get(3).map(|v| v.parse().unwrap()).unwrap_or(100_000);
    if repeats == 0 || duration == 0 {
        return Err("positive repeats/duration required".into());
    }
    std::fs::create_dir(&root).map_err(|e| e.to_string())?;
    std::fs::write(root.join("machine.txt"),format!("OS={} ARCH={} CPUs={} CPU={} profile={} duration_us={} repeats={}\nPeak RSS is process lifetime high-water mark (not per-case allocations); zero means unavailable. Total includes initialization and synchronous scientific logs. loop includes writing/flushing. IO throughput is bytes/total, not isolated disk bandwidth. Worker poll measures API responsiveness, not rendered FPS.\n",std::env::consts::OS,std::env::consts::ARCH,std::thread::available_parallelism().map(|n|n.get()).unwrap_or(0),std::env::var("PROCESSOR_IDENTIFIER").unwrap_or_default(),if cfg!(debug_assertions){"debug"}else{"release"},duration,repeats)).map_err(|e|e.to_string())?;
    let mut out =
        std::fs::File::create(root.join("measurements.csv")).map_err(|e| e.to_string())?;
    writeln!(out,"case,mode,logs,repeat,total_s,loop_s,realtime_factor,output_bytes,bytes_per_total_s,process_peak_rss_bytes,first_preview_ms,max_poll_us").unwrap();
    let mut failures =
        std::fs::File::create(root.join("failures.jsonl")).map_err(|e| e.to_string())?;
    for (name, project, sensors, wheels, complex) in [
        ("reference4", "examples/basic/projeto.rtsim", 4, 4, false),
        ("sensors16", "examples/basic/projeto.rtsim", 16, 4, false),
        ("complex64", "examples/basic/projeto.rtsim", 64, 4, true),
        (
            "simplified4",
            "examples/physics/simplified.rtsim",
            4,
            4,
            false,
        ),
        (
            "simplified8",
            "examples/physics/simplified.rtsim",
            4,
            8,
            false,
        ),
        (
            "realistic4",
            "examples/physics/realistic.rtsim",
            4,
            4,
            false,
        ),
        ("power4", "examples/power/projeto.rtsim", 4, 4, false),
        (
            "simplified8-straight",
            "examples/physics/simplified.rtsim",
            4,
            8,
            false,
        ),
    ] {
        let mut cfg = load_project(Path::new(project)).map_err(|e| e.to_string())?;
        cfg.project.time.physics_dt_us = 50;
        cfg.track.environment.race_enabled = false;
        cfg.track.environment.stop_on_exit = false;
        if sensors != cfg.robot.sensors.len() {
            let template = cfg.robot.sensors[0].clone();
            cfg.robot.sensors = (0..sensors)
                .map(|i| {
                    let mut s = template.clone();
                    s.id = format!("sensor-{i}");
                    s.position_m.y = (i as f64 - (sensors - 1) as f64 / 2.) * 0.001;
                    s
                })
                .collect();
        }
        if name == "simplified8-straight" {
            cfg.robot.controller.kp = 0.;
            cfg.robot.controller.ki = 0.;
            cfg.robot.controller.kd = 0.;
            cfg.robot.controller.base_pwm = 0.1;
        }
        if wheels == 8 {
            let assembly = cfg.robot.assembly.as_mut().unwrap();
            let extra = assembly.wheels.clone();
            for mut w in extra {
                w.id = format!("extra-{}", w.id);
                w.position_m.x += 0.005;
                assembly.wheels.push(w);
            }
        }
        if complex {
            cfg.track.parametric = None;
            cfg.track.centerline = (0..4096)
                .map(|i| {
                    let t = i as f64 * std::f64::consts::TAU / 4095.;
                    Vec2::new(t.cos(), t.sin())
                })
                .collect();
        }
        for (mode, logs) in [
            ("session", false),
            ("runner", false),
            ("worker", false),
            ("runner", true),
            ("worker", true),
        ] {
            for repeat in 0..repeats {
                let dir = root.join(format!("{name}-{mode}-{logs}-{repeat}"));
                std::fs::create_dir(&dir).unwrap();
                let options = RunOptions {
                    duration_us: Some(duration),
                    output_csv: logs.then(|| dir.join("result.csv")),
                    output_replay: logs.then(|| dir.join("result.rtlog")),
                    headless: true,
                    benchmark: !logs,
                    physics_dt_override_us: None,
                };
                let start = Instant::now();
                let mut first = -1.;
                let mut max_poll = 0f64;
                let outcome = (|| -> Result<f64, String> {
                    let loop_s;
                    if mode == "session" {
                        let mut core = SimulationCore::new(cfg.clone(), Some(duration))?;
                        let now = Instant::now();
                        while core.try_step()? {}
                        loop_s = now.elapsed().as_secs_f64();
                    } else if mode == "worker" {
                        let w = SimulationWorker::spawn(cfg.clone(), options, 1, false);
                        let summary = loop {
                            let t = Instant::now();
                            let preview = w.control.latest();
                            max_poll = max_poll.max(t.elapsed().as_secs_f64() * 1e6);
                            if preview.is_some() && first < 0. {
                                first = start.elapsed().as_secs_f64() * 1000.;
                            }
                            if let Some(result) = w.take_result() {
                                break result?;
                            }
                            std::thread::sleep(Duration::from_millis(1));
                        };
                        loop_s = summary.wall_time.as_secs_f64();
                    } else {
                        loop_s = run_simulation(cfg.clone(), options)?
                            .wall_time
                            .as_secs_f64();
                    }
                    Ok(loop_s)
                })();
                let loop_s = match outcome {
                    Ok(v) => v,
                    Err(e) => {
                        let error = robotrace_sim::json::JsonValue::String(e).to_json().unwrap();
                        writeln!(failures,r#"{{"case":"{name}","mode":"{mode}","logs":{logs},"repeat":{repeat},"total_s":{},"error":{error}}}"#,start.elapsed().as_secs_f64()).unwrap();
                        failures.flush().unwrap();
                        continue;
                    }
                };
                let total = start.elapsed().as_secs_f64();
                let count = bytes(&dir);
                writeln!(out,"{name},{mode},{logs},{repeat},{total:.9},{loop_s:.9},{:.6},{count},{:.0},{},{first:.3},{max_poll:.3}",duration as f64*1e-6/loop_s,count as f64/total,peak_bytes()).unwrap();
                out.flush().unwrap();
            }
        }
        println!("completed {name}");
    }
    // True CLI includes process startup. Use original project snapshots; no mutated matrix configs.
    let exe = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join(if cfg!(windows) {
            "robotrace-sim.exe"
        } else {
            "robotrace-sim"
        });
    if !exe.exists() {
        return Err("build the release headless binary first to measure CLI".into());
    }
    for (name, project) in [
        ("cli-reference", "examples/basic/projeto.rtsim"),
        ("cli-simplified", "examples/physics/simplified.rtsim"),
    ] {
        for repeat in 0..repeats {
            let mut command = std::process::Command::new(&exe);
            command.args([
                "benchmark",
                project,
                "--duration",
                &format!("{}ms", duration as f64 / 1000.),
                "--physics-dt-us",
                "50",
            ]);
            #[cfg(windows)]
            {
                use std::os::windows::process::CommandExt;
                command.creation_flags(0x08000000);
            }
            let start = Instant::now();
            let result = command.output().map_err(|e| e.to_string())?;
            if !result.status.success() {
                return Err(String::from_utf8_lossy(&result.stderr).into());
            }
            let text = String::from_utf8_lossy(&result.stdout);
            let wall: f64 = text
                .lines()
                .find_map(|l| l.strip_prefix("wall time:"))
                .and_then(|v| v.split_whitespace().next())
                .ok_or("missing CLI timing")?
                .parse()
                .map_err(|_| "invalid CLI timing")?;
            writeln!(
                out,
                "{name},cli,false,{repeat},{:.9},{wall:.9},{:.6},0,0,0,-1,0",
                start.elapsed().as_secs_f64(),
                duration as f64 * 1e-6 / wall
            )
            .unwrap();
        }
    }
    Ok(())
}
