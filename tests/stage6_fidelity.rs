use robotrace_sim::{
    config::load_project,
    math::Vec2,
    models::{
        chassis::{load_center, support_loads},
        fidelity::FidelityConfig,
    },
    sim::SimulationCore,
};
use std::path::Path;
fn config(preset: &str) -> robotrace_sim::config::LoadedConfig {
    load_project(Path::new(&format!("examples/physics/{preset}.rtsim"))).unwrap()
}
#[test]
fn presets_run_and_have_distinct_motion_with_expanded_snapshot() {
    let mut positions = Vec::new();
    for preset in ["ideal", "simplified", "realistic"] {
        let cfg = config(preset);
        let mut core = SimulationCore::new(cfg, Some(100000)).unwrap();
        core.advance_until(100000).unwrap();
        positions.push(core.state().pose.x);
        assert_eq!(core.contact_state().wheels.len(), 4);
        assert!(core
            .resolved_experiment()
            .to_json()
            .contains("longitudinal_stiffness"));
        assert!(core.effective_config().to_json().contains("per-wheel-v1"));
    }
    assert!((positions[0] - positions[1]).abs() > 1e-5);
    assert!((positions[1] - positions[2]).abs() > 1e-10);
}
#[test]
fn support_force_and_moment_balance_transfer_and_tipping() {
    let p = [
        Vec2::new(-0.1, -0.1),
        Vec2::new(-0.1, 0.1),
        Vec2::new(0.1, -0.1),
        Vec2::new(0.1, 0.1),
    ];
    let center = load_center(Vec2::new(0.01, 0.02), 1., 0.05, 10., Vec2::new(2., 3.));
    let n = support_loads(&p, 10., center).unwrap();
    assert!((n.iter().sum::<f64>() - 10.).abs() < 1e-10);
    assert!((p.iter().zip(&n).map(|(p, n)| p.x * n).sum::<f64>() - 10. * center.x).abs() < 1e-10);
    assert!((p.iter().zip(&n).map(|(p, n)| p.y * n).sum::<f64>() - 10. * center.y).abs() < 1e-10);
    let rear = n[0] + n[1];
    let front = n[2] + n[3];
    assert!((rear - front).abs() < 1e-10); // positive acceleration cancels initial forward COM offset
    assert!(support_loads(&p, 10., Vec2::new(1., 0.)).is_err());
    let edge = support_loads(&p, 10., Vec2::new(0.1, 0.)).unwrap();
    assert!(edge[0] + edge[1] < 1e-8);
}
#[test]
fn model_registry_rejects_unknown_options_and_incompatible_refinements() {
    for text in [
        r#"{"preset":"magic"}"#,
        r#"{"preset":"simplified","contact":"unknown"}"#,
        r#"{"preset":"simplified","vertical":true}"#,
        r#"{"preset":"ideal","rolling":true}"#,
        r#"{"preset":"simplified","relaxation_s":0.1}"#,
    ] {
        let j = robotrace_sim::json::parse_json(text).unwrap();
        assert!(FidelityConfig::from_value(&j).is_err());
    }
    let mut f = FidelityConfig::preset("realistic").unwrap();
    f.longitudinal_stiffness = f64::NAN;
    assert!(f.validate().is_err());
    assert!(robotrace_sim::models::fidelity::MODELS
        .iter()
        .all(|m| !m.state.is_empty()
            && !m.parameters.is_empty()
            && !m.dependencies.is_empty()
            && !m.limitations.is_empty()));
}
#[test]
fn persistence_preserves_overrides_and_rejects_invalid_loaded_models() {
    let mut cfg = config("realistic");
    let f = cfg.robot.physics.as_mut().unwrap();
    f.relaxation_s = 0.001;
    f.load_exponent = 0.2;
    f.radial_stiffness_n_m = 100000.;
    let path = Path::new("target").join(format!("stage6-robot-{}.json", std::process::id()));
    robotrace_sim::io::persistence::save_robot_to_file(&cfg.robot, &path).unwrap();
    let r = robotrace_sim::config::load_robot_from_file(&path).unwrap();
    assert_eq!(r.physics, cfg.robot.physics);
    let mut core = SimulationCore::new(cfg, Some(5000)).unwrap();
    core.advance_until(5000).unwrap();
    assert!(core
        .contact_state()
        .wheels
        .iter()
        .any(|w| w.radial_compression_m > 0.));
}
#[test]
fn unsupported_ideal_geometry_fails_but_individual_passive_geometry_runs() {
    let mut cfg = config("simplified");
    let a = cfg.robot.assembly.as_mut().unwrap();
    a.wheels[1].kind = robotrace_sim::models::robot::WheelKind::Passive;
    a.wheels[1].motor = None;
    a.wheels[1].radius_m *= 1.1;
    let mut core = SimulationCore::new(cfg.clone(), Some(1000)).unwrap();
    core.advance_until(1000).unwrap();
    cfg.robot.physics = Some(FidelityConfig::preset("ideal").unwrap());
    cfg.robot.assembly.as_mut().unwrap().wheels[0].angle_deg = 15.;
    assert!(SimulationCore::new(cfg, Some(1000)).is_err());
}
#[test]
fn contact_stream_uses_sensor_log_ticks_and_is_written_with_replay() {
    let cfg = config("simplified");
    let path = Path::new("target").join(format!("stage6-log-{}.rtlog", std::process::id()));
    let summary = robotrace_sim::sim::run_simulation(
        cfg,
        robotrace_sim::sim::RunOptions {
            duration_us: Some(2000),
            output_csv: None,
            output_replay: Some(path.clone()),
            headless: true,
            benchmark: false,
            physics_dt_override_us: None,
        },
    )
    .unwrap();
    let csv = std::fs::read_to_string(format!("{}.contacts.csv", path.display())).unwrap();
    assert_eq!(csv.lines().count(), 1 + 4 * summary.samples as usize);
    assert!(csv.contains("force_long_n"));
    assert!(csv.lines().last().unwrap().starts_with("2000,"));
}
#[test]
fn reducing_time_step_converges_in_individual_contact_run() {
    let mut x = Vec::new();
    for dt in [100, 50, 25] {
        let mut cfg = config("simplified");
        cfg.project.time.physics_dt_us = dt;
        cfg.robot.controller.kp = 0.;
        cfg.robot.controller.ki = 0.;
        cfg.robot.controller.kd = 0.;
        cfg.robot.controller.base_pwm = 0.5;
        let mut c = SimulationCore::new(cfg, Some(20000)).unwrap();
        c.advance_until(20000).unwrap();
        x.push(c.state().pose.x);
    }
    assert!(
        (x[2] - x[1]).abs() <= 1.2 * (x[1] - x[0]).abs() + 1e-9,
        "positions={x:?}"
    );
}

#[test]
fn external_downforce_moment_is_not_silently_clipped_to_support_rectangle() {
    let mut cfg = config("simplified");
    cfg.robot.normal_force.model = robotrace_sim::io::models::NormalForceKind::Constant;
    cfg.robot.normal_force.max_force_n = 100.;
    cfg.robot.normal_force.position_m = Vec2::new(2., 0.);
    assert!(SimulationCore::new(cfg, Some(1000))
        .err()
        .unwrap()
        .contains("support equilibrium impossible"));
}

#[test]
fn disabling_realistic_contact_override_recovers_simplified_execution() {
    let mut base = config("simplified");
    base.robot.controller.kp = 0.;
    base.robot.controller.ki = 0.;
    base.robot.controller.kd = 0.;
    let mut reduced = base.clone();
    let mut f = FidelityConfig::preset("realistic").unwrap();
    f.contact = "coulomb".into();
    reduced.robot.physics = Some(f);
    let mut a = SimulationCore::new(base, Some(5000)).unwrap();
    let mut b = SimulationCore::new(reduced, Some(5000)).unwrap();
    a.advance_until(5000).unwrap();
    b.advance_until(5000).unwrap();
    assert_eq!(a.sample(), b.sample());
    for (a, b) in a
        .contact_state()
        .wheels
        .iter()
        .zip(&b.contact_state().wheels)
    {
        assert_eq!(a.force_long_n, b.force_long_n);
        assert_eq!(a.force_lat_n, b.force_lat_n);
    }
}
#[test]
fn com_height_changes_runtime_loads_under_acceleration() {
    let mut cfg = config("simplified");
    cfg.robot.controller.kp = 0.;
    cfg.robot.controller.ki = 0.;
    cfg.robot.controller.kd = 0.;
    cfg.robot.controller.base_pwm = 0.8;
    cfg.robot.assembly.as_mut().unwrap().measured_com_height_m = 0.005;
    let mut stationary = cfg.clone();
    stationary.robot.physics.as_mut().unwrap().normal = "static".into();
    let mut a = SimulationCore::new(cfg, Some(3000)).unwrap();
    let mut b = SimulationCore::new(stationary, Some(3000)).unwrap();
    a.advance_until(3000).unwrap();
    b.advance_until(3000).unwrap();
    let delta = a
        .contact_state()
        .wheels
        .iter()
        .zip(&b.contact_state().wheels)
        .map(|(a, b)| (a.normal_n - b.normal_n).abs())
        .sum::<f64>();
    assert!(delta > 1e-5);
    let total_a = a
        .contact_state()
        .wheels
        .iter()
        .map(|w| w.normal_n)
        .sum::<f64>();
    let total_b = b
        .contact_state()
        .wheels
        .iter()
        .map(|w| w.normal_n)
        .sum::<f64>();
    assert!((total_a - total_b).abs() < 1e-9);
}
