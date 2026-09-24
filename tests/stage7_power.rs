use robotrace_sim::{
    config::load_project,
    models::{electrical::*, power::*},
    sim::SimulationCore,
};
use std::path::Path;
fn config() -> robotrace_sim::config::LoadedConfig {
    let mut c = load_project(Path::new("examples/power/projeto.rtsim")).unwrap();
    c.robot.controller.kp = 0.;
    c.robot.controller.ki = 0.;
    c.robot.controller.kd = 0.;
    c.robot.controller.base_pwm = 0.4;
    c.robot.normal_force.model = robotrace_sim::io::models::NormalForceKind::None;
    c.track.environment.race_enabled = false;
    c.track.environment.stop_on_exit = false;
    c
}
fn command(c: &mut SimulationCore, pwm: f64, mode: ActuatorMode) {
    c.command_actuators(ActuatorCommand {
        t_us: c.time_us(),
        pwm: [pwm; 2],
        modes: [mode; 2],
    })
    .unwrap();
}
#[test]
fn rl_step_converges_to_analytic_locked_rotor_solution() {
    let m = ElectricalMotor::default();
    let t = 0.0005;
    let exact = 6. / m.resistance_ohm * (1. - (-m.resistance_ohm * t / m.inductance_h).exp());
    let mut errors = Vec::new();
    for count in [50, 100, 200] {
        let mut i = 0.;
        let dt = t / count as f64;
        for _ in 0..count {
            i = current_step(i, 6., 0., &m, m.resistance_ohm, dt);
        }
        errors.push((i - exact).abs());
    }
    assert!(errors[2] < errors[1] && errors[1] < errors[0]);
    assert!(errors[2] < 0.002);
}
#[test]
fn source_ocv_resistance_rc_and_thermal_match_reference_equations() {
    let c = config();
    let p = c.robot.powertrain.as_ref().unwrap();
    let mut s = PowerState::new(p, &c.robot.battery);
    s.soc = 0.5;
    let dt = 0.01;
    let (v, soc, rc) = battery_trial(p, &c.robot.battery, &s, 1., dt);
    assert!((soc - (0.5 - dt / (c.robot.battery.capacity_mah * 3.6))).abs() < 1e-12);
    assert!((rc - p.rc_resistance_ohm * dt / (p.rc_time_s + dt)).abs() < 1e-12);
    assert!(
        (v - (open_voltage(p, &c.robot.battery, soc) - resistance(p, &c.robot.battery, soc) - rc))
            .abs()
            < 1e-12
    );
    s.polarization_v = rc;
    let (_, _, recovered) = battery_trial(p, &c.robot.battery, &s, 0., dt);
    assert!(recovered < rc);
    let heat = 2.;
    let end = temperature_step(p.ambient_c, heat, p, 1.);
    let expected = p.ambient_c
        + heat
            * p.thermal_resistance_k_w
            * (1. - (-1. / (p.thermal_capacity_j_k * p.thermal_resistance_k_w)).exp());
    assert!((end - expected).abs() < 1e-12);
}
#[test]
fn coupled_voltage_and_power_balance_include_auxiliary_load() {
    let mut cfg = config();
    cfg.robot.powertrain.as_mut().unwrap().auxiliary_power_w = 0.2;
    let mut c = SimulationCore::new(cfg, Some(1000)).unwrap();
    c.advance_until(1000).unwrap();
    let p = c.power_state().unwrap();
    assert!(p.current_a > 0. && p.voltage_v < 8.4);
    assert!(p.residual_v.abs() < 1e-7);
    assert!(
        (p.voltage_v * p.current_a
            - p.auxiliary_power_w
            - p.downforce_power_w
            - p.motors.iter().map(|m| m.bus_power_w).sum::<f64>())
        .abs()
            < 1e-8
    );
    for m in &p.motors {
        assert!(m.balance_residual_w.abs() < 1e-6);
        assert!(m.magnetic_energy_j > 0.);
        assert!(m.temperature_c > 25.);
    }
}
#[test]
fn current_limit_latency_and_explicit_brake_coast_reverse() {
    let mut cfg = config();
    cfg.robot.driver.current_limit_a = 0.1;
    cfg.robot.powertrain.as_mut().unwrap().command_latency_us = 100;
    let mut c = SimulationCore::new(cfg, Some(5000)).unwrap();
    command(&mut c, 1., ActuatorMode::Drive);
    c.advance_until(100).unwrap();
    assert!(c
        .power_state()
        .unwrap()
        .motors
        .iter()
        .all(|m| m.current_a == 0.));
    c.advance_until(500).unwrap();
    assert!(c
        .power_state()
        .unwrap()
        .motors
        .iter()
        .all(|m| m.current_a.abs() <= 0.100001));
    assert!(!c.power_state().unwrap().events.is_empty());
    command(&mut c, 0., ActuatorMode::Coast);
    c.advance_until(1500).unwrap();
    assert!(c
        .power_state()
        .unwrap()
        .motors
        .iter()
        .all(|m| m.current_a.abs() < 1e-6));
    command(&mut c, 0., ActuatorMode::Brake);
    c.advance_until(2500).unwrap();
    assert!(c
        .power_state()
        .unwrap()
        .motors
        .iter()
        .all(|m| m.current_a <= 1e-6));
    command(&mut c, -0.8, ActuatorMode::Drive);
    c.advance_until(5000).unwrap();
    assert!(c
        .power_state()
        .unwrap()
        .motors
        .iter()
        .all(|m| m.current_a < 0.));
}
#[test]
fn protection_stops_without_advancing_invalid_tick_and_writes_event() {
    let mut cfg = config();
    cfg.robot.powertrain.as_mut().unwrap().undervoltage_v = 20.;
    let mut c = SimulationCore::new(cfg, Some(1000)).unwrap();
    assert!(!c.try_step().unwrap());
    assert!(c.is_finished());
    assert_eq!(c.time_us(), 0);
    assert!(c.termination_reason().contains("undervoltage"));
    assert_eq!(c.power_state().unwrap().events.len(), 1);
}
#[test]
fn shaft_units_models_and_incompatible_buses_are_rejected() {
    let mut p = PowertrainConfig::default();
    p.motors[0].parameter_shaft = "output".into();
    assert!(p.validate().is_err());
    p.motors[0].ratio = 1.;
    assert!(p.validate().is_ok());
    p.motors[0].model = "BLDC".into();
    assert!(p.validate().is_err());
    let bad = robotrace_sim::json::parse_json(r#"{"motors":[],"separate_bus":true}"#).unwrap();
    assert!(PowertrainConfig::from_value(&bad).is_err());
    let mut cfg = config();
    cfg.robot.powertrain.as_mut().unwrap().command_latency_us = 1;
    assert!(SimulationCore::new(cfg, Some(1000)).is_err());
}
#[test]
fn frozen_electrical_config_and_sidecars_roundtrip() {
    let cfg = config();
    let path = Path::new("target").join(format!("stage7-robot-{}.json", std::process::id()));
    robotrace_sim::io::persistence::save_robot_to_file(&cfg.robot, &path).unwrap();
    let r = robotrace_sim::config::load_robot_from_file(&path).unwrap();
    assert_eq!(r.powertrain, cfg.robot.powertrain);
    let out = Path::new("target").join(format!("stage7-log-{}.csv", std::process::id()));
    let summary = robotrace_sim::sim::run_simulation(
        cfg,
        robotrace_sim::sim::RunOptions {
            duration_us: Some(1000),
            output_csv: Some(out.clone()),
            output_replay: None,
            headless: true,
            benchmark: false,
            physics_dt_override_us: None,
        },
    )
    .unwrap();
    let log = std::fs::read_to_string(format!("{}.power.csv", out.display())).unwrap();
    assert_eq!(log.lines().count(), 1 + 2 * summary.samples as usize);
    assert!(log.contains("balance_residual_w"));
    assert!(
        std::fs::read_to_string(format!("{}.power.events.json", out.display()))
            .unwrap()
            .contains("rtsim-power-events-v1")
    );
}

#[test]
fn braking_energy_is_dumped_or_returned_only_when_enabled() {
    let mut energies = Vec::new();
    for regeneration in [false, true] {
        let mut cfg = config();
        cfg.robot.driver.voltage_drop_v = 0.;
        cfg.robot.battery.initial_soc = 0.5;
        let p = cfg.robot.powertrain.as_mut().unwrap();
        p.regeneration = regeneration;
        p.charge_limit_a = 1.;
        p.auxiliary_power_w = 0.;
        let mut c = SimulationCore::new(cfg, Some(40000)).unwrap();
        command(&mut c, 0.8, ActuatorMode::Drive);
        c.advance_until(30000).unwrap();
        command(&mut c, 0.015, ActuatorMode::Drive);
        c.advance_until(40000).unwrap();
        let p = c.power_state().unwrap();
        energies.push((p.regenerated_energy_j, p.dumped_energy_j));
    }
    assert_eq!(energies[0].0, 0.);
    assert!(energies[0].1 > 0., "{energies:?}");
    assert!(energies[1].0 > 0., "{energies:?}");
}
#[test]
fn coupled_time_step_converges_and_ideal_supply_does_not_sag() {
    let mut values = Vec::new();
    for dt in [100, 50, 25] {
        let mut cfg = config();
        cfg.project.time.physics_dt_us = dt;
        let mut c = SimulationCore::new(cfg, Some(2000)).unwrap();
        c.advance_until(2000).unwrap();
        values.push(c.state().pose.x);
        assert!(c
            .power_state()
            .unwrap()
            .motors
            .iter()
            .all(|m| m.balance_residual_w.abs() < 1e-6));
    }
    assert!(
        (values[2] - values[1]).abs() <= 1.2 * (values[1] - values[0]).abs() + 1e-9,
        "{values:?}"
    );
    let mut cfg = config();
    cfg.robot.powertrain.as_mut().unwrap().source = "ideal".into();
    let v = cfg.robot.battery.nominal_voltage_v;
    let mut c = SimulationCore::new(cfg, Some(1000)).unwrap();
    c.advance_until(1000).unwrap();
    assert_eq!(c.power_state().unwrap().voltage_v, v);
}

#[test]
fn auxiliary_overcurrent_and_thermal_protection_preserve_last_valid_state() {
    for thermal in [false, true] {
        let mut cfg = config();
        let p = cfg.robot.powertrain.as_mut().unwrap();
        p.source = "ideal".into();
        if thermal {
            p.max_temperature_c = p.ambient_c + 1e-9;
        } else {
            p.auxiliary_power_w = 10000.;
        }
        let mut c = SimulationCore::new(cfg, Some(1000)).unwrap();
        assert!(!c.try_step().unwrap());
        assert_eq!(c.time_us(), 0);
        assert!(c.termination_reason().contains(if thermal {
            "overtemperature"
        } else {
            "overcurrent"
        }));
    }
}
#[test]
fn simple_dc_runs_and_battery_ledger_closes_with_rc_storage() {
    let mut cfg = config();
    for m in &mut cfg.robot.powertrain.as_mut().unwrap().motors {
        m.model = "dc_simple".into();
        m.inductance_h = 0.;
    }
    let mut c = SimulationCore::new(cfg, Some(2000)).unwrap();
    while c.try_step().unwrap() {
        let p = c.power_state().unwrap();
        assert!(p.battery_balance_residual_w.abs() < 1e-6);
        assert!(p.motors.iter().all(|m| m.magnetic_energy_j == 0.));
        assert!(p.regenerated_energy_j <= p.bus_recovered_energy_j + 1e-12);
    }
    assert_eq!(c.time_us(), 2000);
}

#[test]
fn complete_example_handles_direction_changes_with_fans_and_logs_finite_power() {
    let cfg = load_project(Path::new("examples/power/projeto.rtsim")).unwrap();
    let mut c = SimulationCore::new(cfg, Some(20000)).unwrap();
    while c.try_step().unwrap() {
        let p = c.power_state().unwrap();
        assert!(p.voltage_v.is_finite());
        assert!(p.battery_balance_residual_w.abs() < 1e-5);
    }
    assert_eq!(c.time_us(), 20000);
}
