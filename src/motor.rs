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

/// Affine quasi-static motor, integrated implicitly together with contact.
#[derive(Clone, Copy, Default)]
pub(crate) struct MotorDrive {
    pub lower_torque: Option<f64>,
    pub upper_torque: Option<f64>,
    pub viscous_damping: f64,
    pub target_omega: f64,
    pub damping: f64,
    pub torque_limit: f64,
    pwm: f64,
    voltage: f64,
    torque_per_amp: f64,
    gear_efficiency: f64,
    braking: bool,
    coasting: bool,
}

impl MotorDrive {
    pub(crate) fn affine(target: f64, damping: f64, lower: f64, upper: f64, viscous: f64) -> Self {
        Self {
            target_omega: target,
            damping,
            torque_limit: lower.abs().max(upper.abs()),
            lower_torque: Some(lower),
            upper_torque: Some(upper),
            viscous_damping: viscous,
            torque_per_amp: 1.,
            gear_efficiency: 1.,
            ..Self::default()
        }
    }
    pub(crate) fn lower(self) -> f64 {
        self.lower_torque.unwrap_or(-self.torque_limit)
    }
    pub(crate) fn upper(self) -> f64 {
        self.upper_torque.unwrap_or(self.torque_limit)
    }

    #[cfg(test)]
    pub(crate) fn constant_torque(torque: f64) -> Self {
        Self {
            target_omega: 1e10 * torque.signum(),
            damping: 1.0,
            torque_limit: torque.abs(),
            ..Self::default()
        }
    }
    pub fn output(self, wheel_torque: f64) -> MotorOutput {
        if self.coasting {
            return MotorOutput {
                coasting: true,
                ..MotorOutput::default()
            };
        }
        let current = wheel_torque / self.torque_per_amp;
        MotorOutput {
            wheel_torque_nm: wheel_torque,
            motor_torque_nm: wheel_torque / self.gear_efficiency,
            current_a: current,
            // Non-regenerative bridge: returned energy is dissipated, never credited to SOC.
            supply_current_a: if self.braking {
                0.0
            } else {
                (current * self.pwm).max(0.0)
            },
            voltage_v: self.voltage,
            applied_pwm: self.pwm,
            braking: self.braking,
            coasting: self.coasting,
        }
    }
}

impl DcMotorSimple {
    pub(crate) fn drive(
        &self,
        pwm: f64,
        voltage: f64,
        driver: &DriverConfig,
        supply_budget: f64,
    ) -> MotorDrive {
        let pwm = quantize_pwm(clamp_unit(pwm), driver.pwm_resolution_bits);
        let zero = pwm.abs() <= driver.command_deadband;
        let coast = zero
            && matches!(
                driver.mode.to_ascii_lowercase().as_str(),
                "coast" | "free" | "hi-z" | "hiz"
            );
        if coast {
            return MotorDrive {
                coasting: true,
                ..MotorDrive::default()
            };
        }
        let pwm = if zero { 0.0 } else { pwm };
        let voltage = pwm * (voltage - driver.voltage_drop_v).max(0.0);
        let resistance = self.cfg.nominal_voltage_v / self.cfg.stall_current_a;
        let ke = self.cfg.nominal_voltage_v / (self.cfg.no_load_rpm * 2.0 * PI / 60.0);
        let kt = self.cfg.stall_torque_nm / self.cfg.stall_current_a;
        let gear_efficiency = self.cfg.gear_ratio * self.cfg.efficiency;
        let torque_per_amp = kt * gear_efficiency;
        // Equal reserved DC bus shares. Conservative during braking/reversal.
        let limit = if zero {
            driver.current_limit_a
        } else {
            driver.current_limit_a.min(supply_budget / pwm.abs())
        };
        MotorDrive {
            lower_torque: None,
            upper_torque: None,
            viscous_damping: 0.,
            target_omega: voltage / (ke * self.cfg.gear_ratio),
            damping: torque_per_amp * ke * self.cfg.gear_ratio / resistance,
            torque_limit: limit * torque_per_amp,
            pwm,
            voltage,
            torque_per_amp,
            gear_efficiency,
            braking: zero,
            coasting: false,
        }
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
        let drive = self.drive(pwm, battery_voltage_v, driver, f64::INFINITY);
        let torque = clamp(
            drive.damping * (drive.target_omega - wheel_omega_rad_s),
            -drive.torque_limit,
            drive.torque_limit,
        );
        drive.output(torque)
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
            nominal_voltage_v: 7.4,
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
