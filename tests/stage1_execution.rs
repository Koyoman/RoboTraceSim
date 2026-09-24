use robotrace_sim::calibration::run_simulation_samples;
use robotrace_sim::config::{load_project, LoadedConfig, TimeConfig};
use robotrace_sim::core::clock::duration_seconds_to_us;
use robotrace_sim::replay::load_replay_samples;
use robotrace_sim::sim::{run_simulation, RunOptions, SimulationCore, SimulationSession};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_DIR: AtomicU64 = AtomicU64::new(0);
struct TestDir(PathBuf);
impl TestDir {
    fn new() -> Self {
        let path = std::env::temp_dir().join(format!(
            "robotrace-stage1-{}-{}",
            std::process::id(),
            NEXT_DIR.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for TestDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn fixture(dir: &Path, physics_dt_us: u64) -> PathBuf {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let robot = root
        .join("examples/basic/robot_suction.json")
        .to_string_lossy()
        .replace('\\', "/");
    let track = root
        .join("examples/basic/track.json")
        .to_string_lossy()
        .replace('\\', "/");
    let path = dir.join("project.rtsim");
    std::fs::write(
        &path,
        format!(
            r#"{{
        "rtsim_schema":"rtsim-project-v1", "name":"stage1",
        "robot":"{robot}", "track":"{track}",
        "time":{{"physics_dt_us":{physics_dt_us},"controller_period_us":1000,
            "sensor_period_us":500,"encoder_period_us":500,"imu_period_us":500,
            "log_period_us":1000,"render_period_us":16667}},
        "simulation":{{"duration_s":0.00215,"start_pose_m":[0.0,0.035,0.0]}}
    }}"#
        ),
    )
    .unwrap();
    path
}

fn config() -> LoadedConfig {
    let dir = TestDir::new();
    load_project(&fixture(&dir.0, 50)).unwrap()
}

fn options(csv: PathBuf, replay: PathBuf) -> RunOptions {
    RunOptions {
        duration_us: Some(2150),
        output_csv: Some(csv),
        output_replay: Some(replay),
        headless: true,
        benchmark: false,
        physics_dt_override_us: Some(50),
    }
}

#[test]
fn twenty_physics_steps_per_control_interval_and_observations_are_pure() {
    let mut core = SimulationCore::new(config(), Some(2000)).unwrap();
    let initial = core.sample();
    assert_eq!(core.event_counts().controller, 1);
    assert_eq!(core.event_counts().sensor, 1);
    assert_eq!(core.event_counts().imu, 1);
    assert_eq!(core.event_counts().encoder, 1);
    for _ in 0..5 {
        assert_eq!(initial, core.sample());
    }
    assert_eq!(initial, core.advance_steps(0));
    assert_eq!(core.steps(), 0);
    core.advance_steps(19);
    assert_eq!(core.time_us(), 950);
    assert_eq!(core.event_counts().controller, 1);
    assert_eq!(core.sample().pwm_left, initial.pwm_left);
    assert_eq!(core.sample().pwm_right, initial.pwm_right);
    assert!(core.step());
    assert_eq!(core.time_us(), 1000);
    assert_eq!(core.steps(), 20);
    assert_eq!(core.event_counts().controller, 2);
    assert_eq!(core.event_counts().sensor, 3);
    assert_eq!(core.event_counts().imu, 3);
    assert_eq!(core.event_counts().encoder, 3);
    // At a shared deadline, controller sees the newly acquired sensor output.
    assert_eq!(core.sample().line_error_m, core.sample().line_position_m);
    core.advance_steps(20);
    assert_eq!(core.steps(), 40);
    assert_eq!(core.event_counts().controller, 3);
    let terminal = core.sample();
    let counts = core.event_counts();
    assert!(!core.step());
    assert_eq!(terminal, core.step_once());
    assert_eq!(terminal, core.advance_steps(100));
    assert_eq!(counts, core.event_counts());
}

#[test]
fn zero_duration_has_one_initial_terminal_sample_and_no_integration() {
    let cfg = config();
    let mut core = SimulationCore::new(cfg.clone(), Some(0)).unwrap();
    assert!(core.is_finished());
    assert_eq!(core.progress(), 1.0);
    assert!(core.should_log());
    assert!(!core.step());
    assert_eq!(core.steps(), 0);
    assert_eq!(core.event_counts().controller, 1);
    assert_eq!(
        run_simulation_samples(cfg, Some(0)).unwrap(),
        vec![core.sample()]
    );
    let dir = TestDir::new();
    let mut opts = options(dir.0.join("zero.csv"), dir.0.join("zero.rtlog"));
    opts.duration_us = Some(0);
    let summary = run_simulation(config(), opts).unwrap();
    assert_eq!(summary.steps, 0);
    assert_eq!(summary.samples, 1);
    assert_eq!(summary.simulated_time_s, 0.0);
    assert_eq!(summary.steps_per_second, 0.0);
    assert_eq!(
        load_replay_samples(&dir.0.join("zero.rtlog"), 2)
            .unwrap()
            .samples,
        vec![core.sample()]
    );
}

#[test]
fn advancement_grid_is_explicit_and_block_size_does_not_change_rng_or_state() {
    let cfg = config();
    let mut one = SimulationCore::new(cfg.clone(), Some(2150)).unwrap();
    let mut chunks = SimulationSession::new(cfg.clone(), Some(2150)).unwrap();
    let mut until = SimulationCore::new(cfg, Some(2150)).unwrap();
    while one.step() {
        let _ = one.sample();
        let _ = one.sample();
    }
    for n in [0, 3, 11, 2, 27] {
        chunks.advance_steps(n);
    }
    assert_eq!(until.advance_until(0).unwrap().t_us, 0);
    assert!(until.advance_until(1).is_err());
    assert!(until.advance_until(2200).is_err());
    assert_eq!(until.time_us(), 0);
    until.advance_until(1000).unwrap();
    assert!(until.advance_until(950).is_err());
    until.advance_until(2150).unwrap();
    assert_eq!(one.sample(), chunks.sample());
    assert_eq!(one.sample(), until.sample());
    assert_eq!(one.event_counts(), chunks.event_counts());
}

#[test]
fn reject_invalid_periods_and_duration_but_not_independent_render_rate() {
    let base = config();
    for time in [
        TimeConfig {
            physics_dt_us: 0,
            ..base.project.time
        },
        TimeConfig {
            sensor_period_us: 75,
            ..base.project.time
        },
        TimeConfig {
            controller_period_us: 0,
            ..base.project.time
        },
        TimeConfig {
            encoder_period_us: 25,
            ..base.project.time
        },
        TimeConfig {
            imu_period_us: 525,
            ..base.project.time
        },
        TimeConfig {
            log_period_us: 75,
            ..base.project.time
        },
        TimeConfig {
            render_period_us: 0,
            ..base.project.time
        },
    ] {
        let mut cfg = base.clone();
        cfg.project.time = time;
        assert!(SimulationCore::new(cfg, Some(2000)).is_err());
    }
    assert!(SimulationCore::new(base.clone(), Some(2151)).is_err());
    assert!(SimulationCore::with_time_override(base.clone(), Some(2000), Some(75)).is_err());
    assert!(SimulationCore::new(base.clone(), Some(2000)).is_ok());
    for seconds in [f64::NAN, f64::INFINITY, -1.0, 0.000_000_5, 1e20] {
        let mut cfg = base.clone();
        cfg.project.duration_s = seconds;
        assert!(SimulationCore::new(cfg, None).is_err());
    }
    assert_eq!(duration_seconds_to_us(0.1).unwrap(), 100_000);
    assert_eq!(duration_seconds_to_us(0.00215).unwrap(), 2150);
}

#[test]
fn headless_gui_session_calibration_and_cli_produce_identical_samples() {
    let dir = TestDir::new();
    let path = fixture(&dir.0, 500);
    let cfg = load_project(&path).unwrap();
    let summary = run_simulation(
        cfg.clone(),
        options(dir.0.join("api.csv"), dir.0.join("api.rtlog")),
    )
    .unwrap();
    assert_eq!(summary.steps, 43);
    assert_eq!(summary.samples, 4);
    assert_eq!(summary.duration_us, 2150);
    assert_eq!(summary.effective_config.time.physics_dt_us, 50);
    assert_eq!(summary.effective_config.physics_dt_override_us, Some(50));
    let replay = load_replay_samples(&dir.0.join("api.rtlog"), 100).unwrap();
    assert_eq!(
        replay.samples.iter().map(|s| s.t_us).collect::<Vec<_>>(),
        vec![0, 1000, 2000, 2150]
    );
    let mut effective_cfg = cfg.clone();
    effective_cfg.project.time.physics_dt_us = 50;
    assert_eq!(
        replay.samples,
        run_simulation_samples(effective_cfg.clone(), Some(2150)).unwrap()
    );
    let mut session = SimulationSession::new(effective_cfg, Some(2150)).unwrap();
    for expected in &replay.samples {
        assert_eq!(*expected, session.advance_until(expected.t_us).unwrap());
    }
    assert_eq!(summary.final_pose, session.state().pose);
    let output = Command::new(env!("CARGO_BIN_EXE_robotrace-sim"))
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .args([
            "run",
            path.to_str().unwrap(),
            "--headless",
            "--duration",
            "2150us",
            "--physics-dt-us",
            "50",
            "--csv",
        ])
        .arg(dir.0.join("cli.csv"))
        .arg("--replay")
        .arg(dir.0.join("cli.rtlog"))
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains("43 fixed steps"));
    // v4 embeds a unique run identity; physical rows and deterministic metadata still match.
    assert_eq!(
        std::fs::read(dir.0.join("api.csv")).unwrap(),
        std::fs::read(dir.0.join("cli.csv")).unwrap()
    );
    assert_eq!(
        load_replay_samples(&dir.0.join("api.rtlog"), 100)
            .unwrap()
            .samples,
        load_replay_samples(&dir.0.join("cli.rtlog"), 100)
            .unwrap()
            .samples
    );
    let normalize = |text: String| {
        let mut value = robotrace_sim::json::parse_json(&text).unwrap();
        if let robotrace_sim::json::JsonValue::Object(map) = &mut value {
            assert!(map.remove("run_id").is_some());
        }
        value.to_json().unwrap()
    };
    for extension in ["csv.metadata.json", "rtlog.metadata.json"] {
        assert_eq!(
            normalize(std::fs::read_to_string(dir.0.join(format!("api.{extension}"))).unwrap()),
            normalize(std::fs::read_to_string(dir.0.join(format!("cli.{extension}"))).unwrap())
        );
    }
    assert_eq!(summary.metadata_paths.len(), 2);
    let meta = std::fs::read_to_string(&summary.metadata_paths[0]).unwrap();
    assert_eq!(
        normalize(meta),
        normalize(session.effective_config().to_json().replace(
            "\"physics_dt_override_us\": null",
            "\"physics_dt_override_us\": 50"
        ))
    );
}

#[test]
fn log_schedule_includes_terminal_tick_without_duplicate_on_regular_boundary() {
    for (duration, expected) in [
        (0, vec![0]),
        (50, vec![0, 50]),
        (1000, vec![0, 1000]),
        (1050, vec![0, 1000, 1050]),
    ] {
        let rows = run_simulation_samples(config(), Some(duration)).unwrap();
        assert_eq!(rows.iter().map(|s| s.t_us).collect::<Vec<_>>(), expected);
    }
}

#[test]
fn config_rejects_non_integer_wrong_type_and_infinite_periods() {
    let dir = TestDir::new();
    let path = fixture(&dir.0, 50);
    let original = std::fs::read_to_string(&path).unwrap();
    for value in [
        "50.5",
        "-50",
        "0",
        "1e309",
        "9007199254740992",
        "\"50\"",
        "null",
        "true",
    ] {
        let invalid = original.replace(
            "\"physics_dt_us\":50",
            &format!("\"physics_dt_us\":{value}"),
        );
        std::fs::write(&path, invalid).unwrap();
        assert!(load_project(&path).is_err(), "accepted {value}");
    }
    std::fs::write(
        &path,
        original.replace("\"sensor_period_us\":500", "\"sensor_period_us\":75"),
    )
    .unwrap();
    assert!(load_project(&path).is_err());
}

#[test]
fn invalid_duration_cli_fails_before_creating_output_files() {
    let dir = TestDir::new();
    let path = fixture(&dir.0, 50);
    for duration in ["0.5us", "2151us", "NaNs", "18446744073709551616us"] {
        let output = Command::new(env!("CARGO_BIN_EXE_robotrace-sim"))
            .args([
                "run",
                path.to_str().unwrap(),
                "--headless",
                "--duration",
                duration,
                "--csv",
            ])
            .arg(dir.0.join("invalid.csv"))
            .output()
            .unwrap();
        assert!(!output.status.success(), "accepted {duration}");
        assert!(!dir.0.join("invalid.csv").exists());
    }
}
