use crate::config::{DriverConfig, MotorConfig};
use crate::math::{clamp, clamp_unit};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Default)]
pub struct MotorOutput {
    pub wheel_torque_nm: f64,
    pub motor_torque_nm: f64,
    pub current_a: f64,
    pub supply_current_a: f64,
    pub voltage_v: f64,
    pub applied_pwm: f64,
    pub braking: bool,
    pub coasting: bool,
}

pub trait MotorModel {
    fn step(
        &self,
        pwm: f64,
        wheel_omega_rad_s: f64,
        battery_voltage_v: f64,
        driver: &DriverConfig,
    ) -> MotorOutput;
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
    fn step(
        &self,
        pwm: f64,
        wheel_omega_rad_s: f64,
        battery_voltage_v: f64,
        driver: &DriverConfig,
    ) -> MotorOutput {
        let pwm = quantize_pwm(clamp_unit(pwm), driver.pwm_resolution_bits);
        let no_load_rad_s = self.cfg.no_load_rpm * 2.0 * PI / 60.0;
        let motor_omega = wheel_omega_rad_s * self.cfg.gear_ratio;
        let speed_fraction = if no_load_rad_s.abs() > 1e-9 {
            motor_omega / no_load_rad_s
        } else {
            0.0
        };

        if pwm.abs() <= driver.command_deadband.max(0.0) {
            return zero_pwm_response(&self.cfg, speed_fraction, driver);
        }

        let available_voltage = clamp(
            battery_voltage_v - driver.voltage_drop_v.max(0.0),
            0.0,
            battery_voltage_v.max(0.0),
        );
        let command_voltage = pwm * available_voltage;
        let voltage_fraction = if battery_voltage_v.abs() > 1e-9 {
            command_voltage / battery_voltage_v
        } else {
            0.0
        };

        // Linear DC motor model with back-EMF. Negative PWM naturally generates reverse torque.
        let raw_motor_torque = clamp(
            self.cfg.stall_torque_nm * (voltage_fraction - speed_fraction),
            -self.cfg.stall_torque_nm,
            self.cfg.stall_torque_nm,
        );
        finish_output(
            &self.cfg,
            raw_motor_torque,
            command_voltage,
            pwm,
            driver,
            false,
            false,
        )
    }
}

fn zero_pwm_response(cfg: &MotorConfig, speed_fraction: f64, driver: &DriverConfig) -> MotorOutput {
    match driver.mode.to_ascii_lowercase().as_str() {
        "coast" | "free" | "hi-z" | "hiz" => MotorOutput {
            coasting: true,
            ..MotorOutput::default()
        },
        _ => {
            // Brake mode approximates low-side/short-brake behavior: the motor terminals are
            // shorted, so back-EMF creates a torque opposing rotation without drawing battery power.
            let brake_torque = clamp(
                -cfg.stall_torque_nm * speed_fraction,
                -cfg.stall_torque_nm,
                cfg.stall_torque_nm,
            );
            finish_output(cfg, brake_torque, 0.0, 0.0, driver, true, false)
        }
    }
}

fn finish_output(
    cfg: &MotorConfig,
    raw_motor_torque: f64,
    command_voltage: f64,
    pwm: f64,
    driver: &DriverConfig,
    braking: bool,
    coasting: bool,
) -> MotorOutput {
    let raw_current_a =
        cfg.stall_current_a * (raw_motor_torque.abs() / cfg.stall_torque_nm.max(1e-12));
    let current_limit = driver.current_limit_a.max(0.0);
    let scale = if current_limit > 0.0 && raw_current_a > current_limit {
        current_limit / raw_current_a
    } else {
        1.0
    };
    let motor_torque = raw_motor_torque * scale;
    let current_a = raw_current_a * scale;
    let wheel_torque = motor_torque * cfg.gear_ratio * cfg.efficiency;
    let supply_current_a = if braking || coasting {
        0.0
    } else {
        current_a * pwm.abs()
    };

    MotorOutput {
        wheel_torque_nm: wheel_torque,
        motor_torque_nm: motor_torque,
        current_a,
        supply_current_a,
        voltage_v: command_voltage,
        applied_pwm: pwm,
        braking,
        coasting,
    }
}

fn quantize_pwm(pwm: f64, bits: u32) -> f64 {
    if bits == 0 || bits >= 31 {
        return pwm;
    }
    let levels = ((1u32 << bits) - 1).max(1) as f64;
    (pwm * levels).round() / levels
}

#[cfg(test)]
mod tests {
    use super::*;

    fn motor_cfg() -> MotorConfig {
        MotorConfig {
            model: "DcMotorSimple".to_string(),
            gear_ratio: 10.0,
            efficiency: 1.0,
            no_load_rpm: 1000.0,
            stall_torque_nm: 0.01,
            stall_current_a: 2.0,
        }
    }

    fn driver(mode: &str) -> DriverConfig {
        DriverConfig {
            model: "PwmHBridge".to_string(),
            pwm_frequency_hz: 20_000.0,
            mode: mode.to_string(),
            voltage_drop_v: 0.0,
            pwm_resolution_bits: 10,
            command_deadband: 0.001,
            current_limit_a: 10.0,
        }
    }

    #[test]
    fn coast_zero_pwm_is_free_running() {
        let m = DcMotorSimple::new(motor_cfg());
        let out = m.step(0.0, 10.0, 7.4, &driver("coast"));
        assert_eq!(out.wheel_torque_nm, 0.0);
        assert!(out.coasting);
    }

    #[test]
    fn brake_zero_pwm_opposes_rotation() {
        let m = DcMotorSimple::new(motor_cfg());
        let out = m.step(0.0, 10.0, 7.4, &driver("brake"));
        assert!(out.wheel_torque_nm < 0.0);
        assert!(out.braking);
        assert_eq!(out.supply_current_a, 0.0);
    }
}
