use crate::config::{DriverConfig, MotorConfig};
use crate::math::{clamp, clamp_unit};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Default)]
pub struct MotorOutput {
    pub wheel_torque_nm: f64,
    pub motor_torque_nm: f64,
    pub current_a: f64,
    pub voltage_v: f64,
}

pub trait MotorModel {
    fn step(&self, pwm: f64, wheel_omega_rad_s: f64, battery_voltage_v: f64, driver: &DriverConfig) -> MotorOutput;
}

#[derive(Debug, Clone)]
pub struct DcMotorSimple {
    cfg: MotorConfig,
}

impl DcMotorSimple {
    pub fn new(cfg: MotorConfig) -> Self {
        Self { cfg }
    }
}

impl MotorModel for DcMotorSimple {
    fn step(&self, pwm: f64, wheel_omega_rad_s: f64, battery_voltage_v: f64, driver: &DriverConfig) -> MotorOutput {
        let pwm = clamp_unit(pwm);
        let effective_voltage = clamp(battery_voltage_v - driver.voltage_drop_v, 0.0, battery_voltage_v.max(0.0));
        let command_voltage = pwm * effective_voltage;
        let no_load_rad_s = self.cfg.no_load_rpm * 2.0 * PI / 60.0;
        let motor_omega = wheel_omega_rad_s * self.cfg.gear_ratio;

        // Simple linear DC motor curve:
        // T_motor = T_stall * (V/V_nominal - omega/omega_no_load).
        // This captures back-EMF and braking under negative PWM without modeling inductance.
        let voltage_fraction = if battery_voltage_v.abs() > 1e-9 { command_voltage / battery_voltage_v } else { 0.0 };
        let speed_fraction = if no_load_rad_s.abs() > 1e-9 { motor_omega / no_load_rad_s } else { 0.0 };
        let motor_torque = clamp(
            self.cfg.stall_torque_nm * (voltage_fraction - speed_fraction),
            -self.cfg.stall_torque_nm,
            self.cfg.stall_torque_nm,
        );
        let wheel_torque = motor_torque * self.cfg.gear_ratio * self.cfg.efficiency;
        let current_a = self.cfg.stall_current_a * (motor_torque.abs() / self.cfg.stall_torque_nm.max(1e-12));

        MotorOutput { wheel_torque_nm: wheel_torque, motor_torque_nm: motor_torque, current_a, voltage_v: command_voltage }
    }
}
