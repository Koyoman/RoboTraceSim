use crate::config::PidConfig;
use crate::math::clamp;
use crate::sensor::SensorOutput;

#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerOutput {
    pub pwm_left: f64,
    pub pwm_right: f64,
    pub error_m: f64,
    pub correction: f64,
}

pub trait Controller {
    fn step(&mut self, sensor: &SensorOutput, dt_s: f64) -> ControllerOutput;
}

#[derive(Debug, Clone)]
pub struct BuiltInPid {
    cfg: PidConfig,
    integral: f64,
    prev_error: f64,
    initialized: bool,
}

impl BuiltInPid {
    pub fn new(cfg: PidConfig) -> Self {
        Self { cfg, integral: 0.0, prev_error: 0.0, initialized: false }
    }
}

impl Controller for BuiltInPid {
    fn step(&mut self, sensor: &SensorOutput, dt_s: f64) -> ControllerOutput {
        let error = sensor.line_position_m - self.cfg.target_position_m;
        self.integral += error * dt_s;
        let derivative = if self.initialized && dt_s > 0.0 {
            (error - self.prev_error) / dt_s
        } else {
            0.0
        };
        self.prev_error = error;
        self.initialized = true;

        // Positive error means the line is to the left of the sensor center.
        // The robot turns left by reducing the left PWM and increasing the right PWM.
        let correction = self.cfg.kp * error + self.cfg.ki * self.integral + self.cfg.kd * derivative;
        let pwm_left = clamp(self.cfg.base_pwm - correction, -self.cfg.max_pwm, self.cfg.max_pwm);
        let pwm_right = clamp(self.cfg.base_pwm + correction, -self.cfg.max_pwm, self.cfg.max_pwm);

        ControllerOutput { pwm_left, pwm_right, error_m: error, correction }
    }
}
