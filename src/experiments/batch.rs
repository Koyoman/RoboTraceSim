use crate::{
    config::{load_project, LoadedConfig},
    experiments::jobs::RunControl,
    json::JsonValue as J,
    sim::{new_run_id, run_controlled, RunOptions},
};
use std::{
    collections::VecDeque,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
#[derive(Debug, Clone)]
pub struct BatchOutcome {
    pub index: usize,
    pub run_id: String,
    pub status: String,
    pub steps: Option<u64>,
    pub error: Option<String>,
}
impl BatchOutcome {
    pub fn value(&self) -> J {
        J::Object(
            [
                ("index".into(), J::Number(self.index as f64)),
                ("run_id".into(), J::String(self.run_id.clone())),
                ("status".into(), J::String(self.status.clone())),
                (
                    "steps".into(),
                    self.steps.map(|n| J::Number(n as f64)).unwrap_or(J::Null),
                ),
                (
                    "error".into(),
                    self.error.clone().map(J::String).unwrap_or(J::Null),
                ),
            ]
            .into_iter()
            .collect(),
        )
    }
}
pub fn expand_manifest(path: &Path) -> Result<Vec<LoadedConfig>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let root = crate::json::parse_json(&text).map_err(|e| e.to_string())?;
    let entries = root
        .get("experiments")
        .and_then(J::as_array)
        .ok_or("manifest requires experiments array")?;
    let mut runs = Vec::new();
    for entry in entries {
        let project = entry
            .get("project")
            .and_then(J::as_str)
            .ok_or("experiment requires project")?;
        let cfg = load_project(path.parent().unwrap_or(Path::new(".")).join(project))
            .map_err(|e| e.to_string())?;
        let numbers = |key: &str, default: f64| -> Result<Vec<f64>, String> {
            if let Some(v) = entry.get(key) {
                let a = v.as_array().ok_or(format!("{key} must be array"))?;
                if a.is_empty() {
                    return Err(format!("{key} cannot be empty"));
                }
                a.iter()
                    .map(|x| {
                        x.as_f64()
                            .filter(|n| n.is_finite())
                            .ok_or(format!("invalid {key}"))
                    })
                    .collect()
            } else {
                Ok(vec![default])
            }
        };
        let seeds = numbers("seeds", 1371.)?;
        let periods = numbers("physics_dt_us", cfg.project.time.physics_dt_us as f64)?;
        let pwms = numbers("base_pwm", cfg.robot.controller.base_pwm)?;
        let presets: Vec<Option<String>> = if let Some(v) = entry.get("presets") {
            v.as_array()
                .ok_or("presets must be array")?
                .iter()
                .map(|x| {
                    x.as_str()
                        .map(|s| Some(s.to_owned()))
                        .ok_or("invalid preset".to_string())
                })
                .collect::<Result<_, _>>()?
        } else {
            vec![None]
        };
        if presets.is_empty() {
            return Err("presets cannot be empty".into());
        }
        for seed in &seeds {
            if *seed < 0. || *seed > 9007199254740991. || seed.fract() != 0. {
                return Err("seed must be exact nonnegative integer".into());
            }
            for dt in &periods {
                if *dt <= 0. || *dt > 9007199254740991. || dt.fract() != 0. {
                    return Err("physics step must be positive integer".into());
                }
                for pwm in &pwms {
                    if pwm.abs() > 1. {
                        return Err("base_pwm outside [-1,1]".into());
                    }
                    for preset in &presets {
                        if runs.len() >= 10000 {
                            return Err("batch limited to 10000 experiments".into());
                        }
                        let mut c = cfg.clone();
                        c.project.time.physics_dt_us = *dt as u64;
                        c.robot.controller.base_pwm = *pwm;
                        for s in &mut c.robot.sensors {
                            let h = s
                                .id
                                .bytes()
                                .fold(2166136261u64, |h, b| (h ^ b as u64).wrapping_mul(16777619));
                            s.acquisition.seed = (*seed as u64 ^ h) & ((1u64 << 53) - 1);
                        }
                        c.robot.gyro.seed = *seed as u64 ^ 0x1234;
                        c.robot.sensing.encoder.seed = *seed as u64 ^ 0x2345;
                        c.robot.sensing.imu.seed = *seed as u64 ^ 0x3456;
                        if let Some(p) = preset {
                            c.robot.physics =
                                Some(crate::models::fidelity::FidelityConfig::preset(p)?);
                        }
                        runs.push(c);
                    }
                }
            }
        }
    }
    if runs.is_empty() {
        return Err("empty batch".into());
    }
    Ok(runs)
}
pub fn run_batch(
    configs: Vec<LoadedConfig>,
    output: &Path,
    parallelism: usize,
    control: Arc<RunControl>,
    cancel_file: Option<PathBuf>,
) -> Result<Vec<BatchOutcome>, String> {
    if !(1..=32).contains(&parallelism) {
        return Err("batch jobs must be 1..32".into());
    }
    std::fs::create_dir(output)
        .map_err(|e| format!("batch requires a new output directory: {e}"))?;
    let expected = configs.len();
    let queue = Mutex::new(configs.into_iter().enumerate().collect::<VecDeque<_>>());
    let outcomes = Mutex::new(Vec::new());
    let done = std::sync::atomic::AtomicBool::new(false);
    std::thread::scope(|scope| {
        if let Some(path) = cancel_file {
            let control = control.clone();
            let done = &done;
            scope.spawn(move || {
                while !done.load(std::sync::atomic::Ordering::Relaxed) {
                    if path.exists() {
                        control.cancel();
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(50));
                }
            });
        }
        let mut workers = Vec::new();
        for _ in 0..parallelism {
            let control = control.clone();
            let queue = &queue;
            let outcomes = &outcomes;
            workers.push(scope.spawn(move || loop {
                if control.cancelled() {
                    break;
                }
                let Some((index, cfg)) = queue.lock().unwrap().pop_front() else {
                    break;
                };
                let dir = output.join(format!("run-{index:05}"));
                let id = new_run_id();
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    std::fs::create_dir(&dir)
                        .map_err(|e| e.to_string())
                        .and_then(|_| {
                            run_controlled(
                                cfg,
                                RunOptions {
                                    duration_us: None,
                                    output_csv: Some(dir.join("result.csv")),
                                    output_replay: Some(dir.join("result.rtlog")),
                                    headless: true,
                                    benchmark: false,
                                    physics_dt_override_us: None,
                                },
                                Some(&control),
                                id.clone(),
                            )
                        })
                }))
                .unwrap_or_else(|_| Err("batch run panicked".into()));
                let row = match result {
                    Ok(s) => BatchOutcome {
                        index,
                        run_id: id,
                        status: s.termination_reason,
                        steps: Some(s.steps),
                        error: None,
                    },
                    Err(e) => BatchOutcome {
                        index,
                        run_id: id,
                        status: "failed".into(),
                        steps: None,
                        error: Some(e),
                    },
                };
                let written =
                    std::fs::write(dir.join("summary.json"), row.value().to_json().unwrap());
                let mut row = row;
                if let Err(e) = written {
                    row.status = "failed".into();
                    row.error = Some(e.to_string());
                }
                outcomes.lock().unwrap().push(row);
            }));
        }
        for worker in workers {
            let _ = worker.join();
        }
        done.store(true, std::sync::atomic::Ordering::Relaxed);
    });
    let mut result = outcomes.into_inner().unwrap();
    for (index, _) in queue.into_inner().unwrap() {
        result.push(BatchOutcome {
            index,
            run_id: String::new(),
            status: "not_started_cancelled".into(),
            steps: Some(0),
            error: None,
        });
    }
    for index in 0..expected {
        if !result.iter().any(|r| r.index == index) {
            result.push(BatchOutcome {
                index,
                run_id: String::new(),
                status: "failed".into(),
                steps: None,
                error: Some("worker exited before returning run outcome".into()),
            });
        }
    }
    result.sort_by_key(|r| r.index);
    std::fs::write(
        output.join("summary.json"),
        J::Array(result.iter().map(BatchOutcome::value).collect())
            .to_json()
            .unwrap(),
    )
    .map_err(|e| e.to_string())?;
    Ok(result)
}
