use robotrace_sim::{
    config::{load_project, LoadedConfig},
    experiments::{
        batch::*,
        jobs::{check_cancelled, BackgroundJob, RunControl, SimulationWorker},
    },
    replay::*,
    sim::{run_simulation, RunOptions, SimulationCore},
};
use std::{
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, Instant},
};
fn cfg() -> LoadedConfig {
    load_project(Path::new("examples/sensing/projeto.rtsim")).unwrap()
}
fn directory() -> PathBuf {
    let p =
        PathBuf::from("target").join(format!("stage9-test-{}", robotrace_sim::sim::new_run_id()));
    std::fs::create_dir_all(&p).unwrap();
    p
}
fn opts(path: Option<PathBuf>) -> RunOptions {
    RunOptions {
        duration_us: Some(10_000),
        output_csv: None,
        output_replay: path,
        headless: true,
        benchmark: false,
        physics_dt_override_us: None,
    }
}
fn wait(mut predicate: impl FnMut() -> bool) {
    let start = Instant::now();
    while !predicate() {
        assert!(start.elapsed() < Duration::from_secs(20), "timeout");
        std::thread::sleep(Duration::from_millis(1));
    }
}
#[test]
fn worker_pause_step_resume_and_visual_capacity_preserve_science() {
    let dir = directory();
    let reference = dir.join("reference.rtlog");
    run_simulation(cfg(), opts(Some(reference.clone()))).unwrap();
    let expected = load_replay_samples(&reference, 10000).unwrap().samples;
    for capacity in [1, 8] {
        let path = dir.join(format!("worker-{capacity}.rtlog"));
        let mut original = cfg();
        let mut worker =
            SimulationWorker::spawn(original.clone(), opts(Some(path.clone())), capacity, true);
        original.robot.controller.base_pwm = 0.;
        wait(|| worker.control.latest().is_some());
        worker.control.step(3);
        let mut paused = None;
        wait(|| {
            if let Some(p) = worker.control.latest() {
                paused = Some(p);
            }
            paused.as_ref().is_some_and(|p| p.steps == 3)
        });
        assert_eq!(paused.unwrap().sample.t_us, 150);
        worker.control.resume();
        wait(|| worker.is_finished());
        let summary = worker.take_result().unwrap().unwrap();
        assert_eq!(summary.run_id, worker.run_id);
        worker.join().unwrap();
        assert_eq!(load_replay_samples(&path, 10000).unwrap().samples, expected);
    }
}
#[test]
fn cancel_paused_worker_finalizes_replay_and_drop_joins() {
    let dir = directory();
    let path = dir.join("cancel.rtlog");
    let mut w = SimulationWorker::spawn(cfg(), opts(Some(path.clone())), 1, true);
    wait(|| w.control.latest().is_some());
    w.control.step(7);
    let mut t = 0;
    wait(|| {
        if let Some(p) = w.control.latest() {
            t = p.steps;
        }
        t == 7
    });
    w.control.cancel();
    w.join().unwrap();
    let s = w.take_result().unwrap().unwrap();
    assert_eq!(s.steps, 7);
    assert_eq!(s.termination_reason, "cancelled");
    let mut r = IndexedReplay::open(&path, 128 * 1024).unwrap();
    assert_eq!(r.termination, "cancelled");
    assert_eq!(r.sample(r.samples - 1).unwrap().t_us, 350);
    let w = SimulationWorker::spawn(cfg(), opts(None), 1, true);
    drop(w);
}
#[test]
fn checkpoint_preserves_full_continuation() {
    for project in [
        "examples/sensing/projeto.rtsim",
        "examples/power/projeto.rtsim",
        "examples/physics/simplified.rtsim",
    ] {
        let mut c =
            SimulationCore::new(load_project(Path::new(project)).unwrap(), Some(5_000)).unwrap();
        for _ in 0..13 {
            c.try_step().unwrap();
        }
        let cp = c.checkpoint().unwrap();
        let mut restored = cp.restore();
        while !c.is_finished() {
            assert_eq!(c.sample(), restored.sample());
            assert_eq!(
                format!("{:?}", c.sensor_readings()),
                format!("{:?}", restored.sensor_readings())
            );
            assert_eq!(
                format!("{:?}", c.contact_state()),
                format!("{:?}", restored.contact_state())
            );
            assert_eq!(
                format!("{:?}", c.power_state()),
                format!("{:?}", restored.power_state())
            );
            c.try_step().unwrap();
            restored.try_step().unwrap();
        }
        assert_eq!(c.sample(), restored.sample());
    }
}
fn fixture(path: &Path, n: u64) {
    let mut sample = SimulationCore::new(cfg(), Some(0)).unwrap().sample();
    let mut w =
        BinaryReplayLogger::create_with_metadata(path, sample.sensor_adc.len(), "{\"test\":true}")
            .unwrap();
    for i in 0..n {
        sample.t_us = i * 100;
        sample.x_m = i as f64;
        sample.yaw_rad = if i % 2 == 0 { 3.1 } else { -3.1 };
        sample.encoder_left_ticks = i64::MAX - i as i64;
        sample.pwm_left = i as f64;
        w.write_sample(&sample).unwrap();
    }
    w.finish("duration").unwrap();
}
#[test]
fn indexed_replay_bounds_memory_seeks_and_interpolates_pose_only() {
    let dir = directory();
    let path = dir.join("large.rtlog");
    fixture(&path, 4097);
    let budget = 128 * 1024;
    assert!(std::fs::metadata(&path).unwrap().len() > budget as u64);
    let mut r = IndexedReplay::open(&path, budget).unwrap();
    for i in [0, 4096, 64, 4000, 1] {
        let s = r.sample(i).unwrap();
        assert_eq!(s.x_m, i as f64);
        assert_eq!(s.encoder_left_ticks, i64::MAX - i as i64);
        assert!(r.cached_bytes() <= budget);
    }
    let s = r.at_time(50, true).unwrap();
    assert_eq!(s.x_m, 0.5);
    assert!((s.yaw_rad - std::f64::consts::PI).abs() < 1e-12);
    assert_eq!(s.pwm_left, 0.);
    assert_eq!(s.t_us, 0);
    assert!(load_replay_samples(&path, 100).is_err());
    assert!(IndexedReplay::open(&path, 1).is_err());
}
#[test]
fn replay_rejects_truncation_version_metadata_and_block_corruption() {
    let dir = directory();
    let path = dir.join("good.rtlog");
    fixture(&path, 130);
    let bytes = std::fs::read(&path).unwrap();
    for cut in [0, 7, 16, 30, bytes.len() - 1, bytes.len() - 24] {
        let p = dir.join(format!("cut-{cut}"));
        std::fs::write(&p, &bytes[..cut]).unwrap();
        assert!(IndexedReplay::open(&p, 128 * 1024).is_err());
    }
    for offset in [8, 28, bytes.len() - 32] {
        let mut b = bytes.clone();
        b[offset] ^= 1;
        let p = dir.join(format!("corrupt-{offset}"));
        std::fs::write(&p, b).unwrap();
        assert!(IndexedReplay::open(&p, 128 * 1024).is_err());
    }
    let mut b = bytes;
    b[100] ^= 1;
    let p = dir.join("block");
    std::fs::write(&p, b).unwrap();
    let mut r = IndexedReplay::open(&p, 128 * 1024).unwrap();
    assert!(r.sample(0).is_err());
}
#[test]
fn legacy_v3_reads_but_cannot_certify_complete_end() {
    let dir = directory();
    let path = dir.join("v3.rtlog");
    let s = SimulationCore::new(cfg(), Some(0)).unwrap().sample();
    let mut w = LegacyReplayLogger::create(&path, s.sensor_adc.len()).unwrap();
    w.write_sample(&s).unwrap();
    w.flush().unwrap();
    drop(w);
    let mut r = IndexedReplay::open(&path, 128 * 1024).unwrap();
    assert!(r.legacy);
    assert_eq!(r.sample(0).unwrap(), s);
    assert!(r.termination.contains("unverifiable"));
    drop(r);
    let bytes = std::fs::read(&path).unwrap();
    std::fs::write(&path, &bytes[..bytes.len() - 1]).unwrap();
    assert!(IndexedReplay::open(&path, 128 * 1024).is_err());
}
#[test]
fn replay_metadata_reconstructs_frozen_configuration() {
    let dir = directory();
    let path = dir.join("snapshot.rtlog");
    run_simulation(cfg(), opts(Some(path.clone()))).unwrap();
    let r = IndexedReplay::open(&path, 128 * 1024).unwrap();
    let j = robotrace_sim::json::parse_json(&r.metadata).unwrap();
    let c = robotrace_sim::config::config_from_snapshot(j.get("experiment").unwrap()).unwrap();
    assert_eq!(c.robot.sensors.len(), cfg().robot.sensors.len());
    assert_eq!(c.project.time.physics_dt_us, 50);
}
#[test]
fn batch_sweeps_isolate_runs_and_cancel_without_losing_summary() {
    let dir = directory();
    let manifest = dir.join("batch.json");
    let project = Path::new("examples/sensing/projeto.rtsim")
        .canonicalize()
        .unwrap()
        .to_string_lossy()
        .replace('\\', "/");
    std::fs::write(
        &manifest,
        format!(
            r#"{{"experiments":[{{"project":"{project}","seeds":[1,2],"base_pwm":[0.2,0.3]}}]}}"#
        ),
    )
    .unwrap();
    let mut configs = expand_manifest(&manifest).unwrap();
    assert_eq!(configs.len(), 4);
    assert_ne!(
        configs[0].robot.sensors[0].acquisition.seed,
        configs[2].robot.sensors[0].acquisition.seed
    );
    for c in &mut configs {
        c.project.duration_s = 0.001;
    }
    let result = run_batch(
        configs.clone(),
        &dir.join("done"),
        2,
        Arc::new(RunControl::new(1, false)),
        None,
    )
    .unwrap();
    assert_eq!(result.len(), 4);
    assert!(result.iter().all(|r| r.error.is_none()));
    let token = Arc::new(RunControl::new(1, false));
    token.cancel();
    let rows = run_batch(configs, &dir.join("cancelled"), 2, token, None).unwrap();
    assert!(rows.iter().all(|r| r.status == "not_started_cancelled"));
}
#[test]
fn auxiliary_job_cancels_cooperatively() {
    let job = BackgroundJob::<()>::spawn(|| loop {
        check_cancelled()?;
        std::thread::yield_now();
    });
    job.cancel();
    let mut result = None;
    wait(|| {
        result = job.take_result();
        result.is_some()
    });
    assert!(result.unwrap().unwrap_err().contains("cancelled"));
}

#[test]
fn cancelled_export_preserves_existing_artifact() {
    let dir = directory();
    let output = dir.join("existing.csv");
    std::fs::write(&output, "completed").unwrap();
    let original = output.clone();
    let barrier = Arc::new(std::sync::Barrier::new(2));
    let b = barrier.clone();
    let job = BackgroundJob::spawn(move || {
        robotrace_sim::experiments::jobs::write_output_atomic(&output, |p| {
            std::fs::write(p, "incomplete").unwrap();
            b.wait();
            loop {
                check_cancelled()?;
                std::thread::yield_now();
            }
        })
    });
    barrier.wait();
    job.cancel();
    let mut result: Option<Result<(), String>> = None;
    wait(|| {
        result = job.take_result();
        result.is_some()
    });
    assert!(result.unwrap().is_err());
    drop(job);
    assert_eq!(std::fs::read_to_string(original).unwrap(), "completed");
    assert_eq!(std::fs::read_dir(dir).unwrap().count(), 1);
}
#[test]
fn active_batch_cancel_finalizes_current_run() {
    let dir = directory();
    let target = dir.join("batch");
    let token = Arc::new(RunControl::new(1, true));
    let c = token.clone();
    let handle = std::thread::spawn(move || run_batch(vec![cfg(), cfg()], &target, 1, c, None));
    wait(|| token.latest().is_some());
    token.cancel();
    let outcomes = handle.join().unwrap().unwrap();
    assert_eq!(outcomes[0].status, "cancelled");
    assert_eq!(outcomes[1].status, "not_started_cancelled");
    let r = IndexedReplay::open(&dir.join("batch/run-00000/result.rtlog"), 128 * 1024).unwrap();
    assert_eq!(r.termination, "cancelled");
}

#[test]
fn native_firmware_checkpoint_is_explicitly_rejected() {
    use robotrace_sim::control::{native::Firmware, ControllerInput, TimedCommand};
    struct F;
    impl Firmware for F {
        fn reset(&mut self) -> Result<(), String> {
            Ok(())
        }
        fn step(&mut self, i: &ControllerInput) -> Result<TimedCommand, String> {
            Ok(TimedCommand {
                t_us: i.frame.t_us,
                pwm: [0.; 2],
                downforce_pwm: 0.,
                modes: [robotrace_sim::models::power::ActuatorMode::Drive; 2],
            })
        }
    }
    let mut core = SimulationCore::new(cfg(), Some(1000)).unwrap();
    core.install_firmware(Box::new(F)).unwrap();
    assert!(core.checkpoint().is_err());
}
