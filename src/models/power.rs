//! State and observable contracts of the averaged electrical powertrain.
use super::electrical::*;
use crate::{config::BatteryConfig, json::JsonValue as J};
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorMode {
    Drive,
    Brake,
    Coast,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorCommand {
    pub t_us: u64,
    pub pwm: [f64; 2],
    pub modes: [ActuatorMode; 2],
}
#[derive(Debug, Clone, Default)]
pub struct MotorElectricalState {
    pub current_a: f64,
    pub rotor_speed_rad_s: f64,
    pub temperature_c: f64,
    pub applied_voltage_v: f64,
    pub copper_loss_w: f64,
    pub bridge_loss_w: f64,
    pub gear_loss_w: f64,
    pub bearing_loss_w: f64,
    pub magnetic_energy_j: f64,
    pub numerical_loss_w: f64,
    pub shaft_power_w: f64,
    pub bus_power_w: f64,
    pub dump_power_w: f64,
    pub balance_residual_w: f64,
    pub current_limited: bool,
}
#[derive(Debug, Clone)]
pub struct PowerState {
    pub manual_command: Option<ActuatorCommand>,
    pub motors: [MotorElectricalState; 2],
    pub voltage_v: f64,
    pub current_a: f64,
    pub soc: f64,
    pub polarization_v: f64,
    pub source_power_w: f64,
    pub battery_loss_w: f64,
    pub auxiliary_power_w: f64,
    pub downforce_power_w: f64,
    pub regenerated_energy_j: f64,
    pub bus_recovered_energy_j: f64,
    pub polarization_energy_j: f64,
    pub battery_numerical_loss_w: f64,
    pub battery_balance_residual_w: f64,
    pub dumped_energy_j: f64,
    pub iterations: u32,
    pub residual_v: f64,
    pub events: Vec<(u64, String)>,
    pub fault: Option<String>,
    pub applied: ActuatorCommand,
    pub pending: std::collections::VecDeque<ActuatorCommand>,
    pub last_requested: Option<ActuatorCommand>,
}
impl PowerState {
    pub fn new(p: &PowertrainConfig, b: &BatteryConfig) -> Self {
        Self {
            manual_command: None,
            motors: std::array::from_fn(|_| MotorElectricalState {
                temperature_c: p.ambient_c,
                ..Default::default()
            }),
            voltage_v: open_voltage(p, b, b.initial_soc),
            current_a: 0.,
            soc: b.initial_soc,
            polarization_v: 0.,
            source_power_w: 0.,
            battery_loss_w: 0.,
            auxiliary_power_w: 0.,
            downforce_power_w: 0.,
            regenerated_energy_j: 0.,
            bus_recovered_energy_j: 0.,
            polarization_energy_j: 0.,
            battery_numerical_loss_w: 0.,
            battery_balance_residual_w: 0.,
            dumped_energy_j: 0.,
            iterations: 0,
            residual_v: 0.,
            events: vec![],
            fault: None,
            applied: ActuatorCommand {
                t_us: 0,
                pwm: [0.; 2],
                modes: [ActuatorMode::Coast; 2],
            },
            pending: Default::default(),
            last_requested: None,
        }
    }
    pub fn request(&mut self, command: ActuatorCommand, latency: u64) -> Result<(), String> {
        if command.pwm.iter().any(|x| !x.is_finite() || x.abs() > 1.) {
            return Err("invalid actuator command".into());
        }
        if self
            .last_requested
            .is_some_and(|c| c.pwm == command.pwm && c.modes == command.modes)
        {
            return Ok(());
        }
        if self.pending.len() >= 10000 {
            return Err("actuator queue capacity exceeded".into());
        }
        let mut c = command;
        c.t_us = c
            .t_us
            .checked_add(latency)
            .ok_or("command timestamp overflow")?;
        self.pending.push_back(c);
        self.last_requested = Some(command);
        Ok(())
    }
    pub fn deliver(&mut self, t_us: u64) {
        while self.pending.front().is_some_and(|c| c.t_us <= t_us) {
            self.applied = self.pending.pop_front().unwrap();
        }
    }
    pub fn trip(&mut self, t_us: u64, reason: &str) {
        if self.fault.is_none() {
            self.events.push((t_us, reason.into()));
            self.fault = Some(reason.into());
        }
    }
    pub fn events_json(&self) -> String {
        J::Object(
            [
                ("schema".into(), J::String("rtsim-power-events-v1".into())),
                (
                    "fault".into(),
                    self.fault.clone().map(J::String).unwrap_or(J::Null),
                ),
                (
                    "events".into(),
                    J::Array(
                        self.events
                            .iter()
                            .map(|(t, s)| {
                                J::Object(
                                    [
                                        ("t_us".into(), J::Number(*t as f64)),
                                        ("kind".into(), J::String(s.clone())),
                                    ]
                                    .into_iter()
                                    .collect(),
                                )
                            })
                            .collect(),
                    ),
                ),
            ]
            .into_iter()
            .collect(),
        )
        .to_json()
        .unwrap()
    }
}
pub fn open_voltage(p: &PowertrainConfig, b: &BatteryConfig, soc: f64) -> f64 {
    if p.source == "ideal" {
        b.nominal_voltage_v
    } else {
        interpolate(
            &p.ocv_curve,
            soc,
            b.empty_voltage_v + (b.full_voltage_v - b.empty_voltage_v) * soc,
        )
    }
}
pub fn resistance(p: &PowertrainConfig, b: &BatteryConfig, soc: f64) -> f64 {
    if p.source == "ideal" {
        0.
    } else {
        interpolate(&p.resistance_curve, soc, b.internal_resistance_ohm) + p.wiring_resistance_ohm
    }
}
pub fn battery_trial(
    p: &PowertrainConfig,
    b: &BatteryConfig,
    old: &PowerState,
    current: f64,
    dt: f64,
) -> (f64, f64, f64) {
    if p.source == "ideal" {
        return (b.nominal_voltage_v, old.soc, 0.);
    }
    let soc = old.soc - current * dt / (b.capacity_mah * 3.6);
    let rc = (old.polarization_v + dt / p.rc_time_s * p.rc_resistance_ohm * current)
        / (1. + dt / p.rc_time_s);
    (
        open_voltage(p, b, soc) - resistance(p, b, soc) * current - rc,
        soc,
        rc,
    )
}
/// PWM is a held average; it does not claim to resolve individual switching edges.
pub fn duty(pwm: f64, bits: u32, deadband: f64) -> f64 {
    if pwm.abs() <= deadband {
        return 0.;
    }
    let levels = ((1u64 << bits.min(30)) - 1).max(1) as f64;
    (pwm.clamp(-1., 1.) * levels).round() / levels
}
/// Exact thermal decay under held heat input; electrical integration remains at physics ticks.
pub fn temperature_step(temp: f64, heat_w: f64, p: &PowertrainConfig, dt: f64) -> f64 {
    let target = p.ambient_c + heat_w * p.thermal_resistance_k_w;
    target + (temp - target) * (-dt / (p.thermal_capacity_j_k * p.thermal_resistance_k_w)).exp()
}
