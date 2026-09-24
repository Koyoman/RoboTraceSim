use super::*;

fn config() -> LoadedConfig {
    let mut cfg = crate::config::load_project(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples/basic/projeto_suction.rtsim"),
    )
    .unwrap();
    cfg.robot.normal_force.model = crate::io::models::NormalForceKind::None;
    cfg.robot.tire.rolling_resistance = 0.0;
    for wheel in &mut cfg.robot.assembly.as_mut().unwrap().wheels {
        wheel.tire.rolling_resistance = 0.0;
    }
    cfg.project.time.physics_dt_us = 50;
    cfg.track.environment.start_source = crate::track::definition::StartSource::Project;
    cfg.project.start_pose = Pose2::new(0.8, 0.8, 0.0);
    cfg
}
fn core(cfg: LoadedConfig) -> SimulationCore {
    SimulationCore::new(cfg, Some(1_000_000)).unwrap()
}
fn tick(c: &mut SimulationCore, pwm: f64) {
    c.ctrl_output.pwm_left = pwm;
    c.ctrl_output.pwm_right = pwm;
    c.try_step().unwrap();
}

#[test]
fn voltage_sag_reduces_actual_torque_and_acceleration() {
    let cfg = config();
    let mut low = cfg.clone();
    low.robot.battery.full_voltage_v = 3.7;
    low.robot.battery.empty_voltage_v = 3.0;
    let mut high = core(cfg);
    let mut low = core(low);
    tick(&mut high, 0.02);
    tick(&mut low, 0.02);
    assert!(
        low.last_physics.motor_left.wheel_torque_nm
            < high.last_physics.motor_left.wheel_torque_nm * 0.6
    );
    assert!(low.state.vx_body_m_s < high.state.vx_body_m_s * 0.6);
}

#[test]
fn battery_limit_reduces_actuation_and_accounts_for_downforce() {
    let cfg = config();
    let mut limited = cfg.clone();
    limited.robot.battery.current_limit_a = 0.01;
    let mut free = core(cfg);
    let mut limited = core(limited);
    tick(&mut free, 1.0);
    tick(&mut limited, 1.0);
    assert!(limited.last_physics.battery.current_a <= 0.01 + 1e-10);
    assert!(limited.state.vx_body_m_s < free.state.vx_body_m_s * 0.2);
    let mut cfg = config();
    cfg.robot.normal_force.model = crate::io::models::NormalForceKind::Suction;
    cfg.robot.battery.current_limit_a = 0.01;
    let mut c = core(cfg);
    c.ctrl_output.pwm_downforce = 1.0;
    tick(&mut c, 1.0);
    assert!(c.last_physics.normal.current_a <= 0.01 + 1e-10);
    assert!(c.last_physics.battery.current_a <= 0.01 + 1e-10);
    assert!(c.last_physics.motor_left.wheel_torque_nm.abs() < 1e-10);
}

#[test]
fn coast_preserves_speed_brake_dissipates_without_reversing() {
    for mode in ["coast", "brake"] {
        let mut cfg = config();
        cfg.robot.driver.mode = mode.into();
        let mut c = core(cfg);
        c.state.vx_body_m_s = 0.1;
        c.state.wheel_omega_left_rad_s = 10.0;
        c.state.wheel_omega_right_rad_s = 10.0;
        let mut previous = energy(&c.state, mechanics(&c.cfg));
        for _ in 0..2000 {
            tick(&mut c, 0.0);
            let e = c.diagnostics().kinetic_energy_j;
            assert!(e <= previous + 1e-10);
            previous = e;
            assert!(c.state.vx_body_m_s >= -1e-8);
            assert_eq!(c.last_physics.battery.current_a, 0.0);
        }
        if mode == "coast" {
            assert!((c.state.vx_body_m_s - 0.1).abs() < 1e-8);
        } else {
            assert!(c.state.vx_body_m_s < 0.001);
        }
    }
}

#[test]
fn commanded_reverse_crosses_zero_with_bounded_energy() {
    let mut c = core(config());
    for _ in 0..200 {
        tick(&mut c, 0.3);
    }
    assert!(c.state.vx_body_m_s > 0.0);
    for _ in 0..1000 {
        tick(&mut c, -0.3);
        assert!(c.diagnostics().dissipation_j >= -1e-8);
    }
    assert!(c.state.vx_body_m_s < 0.0);
    assert!(c.last_physics.motor_left.current_a < 0.0);
}

#[test]
fn straight_drive_converges_at_100_50_and_25_microseconds() {
    fn run(dt: u64) -> RobotState {
        let mut cfg = config();
        cfg.project.time.physics_dt_us = dt;
        let mut c = core(cfg);
        for _ in 0..100_000 / dt {
            tick(&mut c, 0.2);
        }
        c.state
    }
    let a = run(100);
    let b = run(50);
    let c = run(25);
    let reference = run(5);
    let ea = (a.pose.x - reference.pose.x).abs();
    let eb = (b.pose.x - reference.pose.x).abs();
    let ec = (c.pose.x - reference.pose.x).abs();
    println!(
        "straight convergence position error 100/50/25 us: {ea:e}, {eb:e}, {ec:e}; x50={} x5={}",
        b.pose.x, reference.pose.x
    );
    assert!(ec < eb && eb < ea);
    assert!((b.pose.x - reference.pose.x).abs() < 1e-5);
    assert!((b.vx_body_m_s - reference.vx_body_m_s).abs() < 1e-4);
}

#[test]
fn electrical_input_bounds_kinetic_energy_and_zero_limit_cuts_drive() {
    let mut c = core(config());
    let mut electrical = 0.0;
    for i in 0..4000 {
        tick(&mut c, if i < 2000 { 0.3 } else { -0.3 });
        electrical += c.diagnostics().electrical_energy_j;
        assert!(c.diagnostics().kinetic_energy_j <= electrical + 1e-9);
    }
    let mut cfg = config();
    cfg.robot.battery.current_limit_a = 0.0;
    let mut c = core(cfg);
    for _ in 0..10 {
        tick(&mut c, 1.0);
        assert_eq!(c.last_physics.battery.current_a, 0.0);
    }
    assert_eq!(c.state.vx_body_m_s, 0.0);
}

#[test]
fn divergence_and_non_finite_state_are_reported() {
    let mut c = core(config());
    c.state.yaw_rate_rad_s = 1e6;
    assert!(c.try_step().unwrap_err().contains("rotation exceeds"));
    let mut c = core(config());
    c.state.vx_body_m_s = f64::INFINITY;
    assert!(c.try_step().unwrap_err().contains("non-finite state"));
}

#[test]
fn basic_project_turns_with_implicit_energy_accounting_at_500_us() {
    let cfg = crate::config::load_project(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/basic/projeto.rtsim"),
    )
    .unwrap();
    let mut c = SimulationCore::new(cfg, Some(100_000)).unwrap();
    while c.try_step().unwrap() {
        assert!(c.diagnostics().dissipation_j >= -1e-8);
    }
    assert_eq!(c.time_us(), 100_000);
}

#[test]
fn invalid_physical_parameters_are_rejected_before_running() {
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut cfg = config();
        cfg.robot.chassis.mass_kg = value;
        assert!(SimulationCore::new(cfg, Some(1000)).is_err());
        let mut cfg = config();
        cfg.robot.assembly.as_mut().unwrap().wheels[0].inertia_kg_m2 = value;
        assert!(SimulationCore::new(cfg, Some(1000)).is_err());
        let mut cfg = config();
        cfg.robot.motor_left.nominal_voltage_v = value;
        assert!(SimulationCore::new(cfg, Some(1000)).is_err());
    }
    let mut cfg = config();
    cfg.robot.motor_left.stall_torque_nm = 100.0;
    assert!(SimulationCore::new(cfg, Some(1000)).is_err());
}

#[test]
fn fault_is_sticky_reports_timestamp_and_does_not_commit_invalid_tick() {
    let mut c = core(config());
    tick(&mut c, 0.2);
    let saved = c.state;
    let time = c.time_us();
    c.ctrl_output.pwm_left = f64::NAN;
    let error = c.try_step().unwrap_err();
    assert!(error.contains("t=50 us") && error.contains("controller"));
    assert_eq!(c.state.pose, saved.pose);
    assert_eq!(c.time_us(), time);
    assert!(c.is_finished());
    assert!(!c.step());
    assert_eq!(c.try_step().unwrap_err(), error);
    assert!(c.advance_until(100).is_err());
}
