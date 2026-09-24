use crate::config::PidConfig;
use crate::control::{ControllerInput, TimedCommand};
use crate::models::{power::ActuatorMode, sensing::ControlSettings};
use crate::sensor::SensorOutput;
#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerOutput {
    pub pwm_left: f64,
    pub pwm_right: f64,
    pub pwm_downforce: f64,
    pub error_m: f64,
    pub correction: f64,
}
pub trait Controller {
    fn step(&mut self, sensor: &SensorOutput, dt_s: f64) -> ControllerOutput;
}
#[derive(Debug, Clone)]
pub struct BuiltInPid {
    cfg: PidConfig,
    settings: ControlSettings,
    integral: f64,
    prev_error: f64,
    derivative: f64,
    initialized: bool,
    speed_integral: [f64; 2],
    lost_since: Option<u64>,
    radius: [f64; 2],
    invert: [f64; 2],
    pub estimate: crate::control::estimator::Estimator,
}
impl BuiltInPid {
    pub fn from_robot(robot: &crate::config::RobotConfig, base: f64, line: f64) -> Self {
        Self {
            cfg: robot.controller,
            settings: robot.sensing.control.clone(),
            integral: 0.,
            prev_error: 0.,
            derivative: 0.,
            initialized: false,
            speed_integral: [0.; 2],
            lost_since: None,
            radius: std::array::from_fn(|i| {
                robot
                    .assembly
                    .as_ref()
                    .and_then(|a| {
                        a.wheels.iter().find(|w| {
                            w.motor.as_deref()
                                == Some(if i == 0 { "motor:left" } else { "motor:right" })
                        })
                    })
                    .map(|w| w.radius_m)
                    .unwrap_or(robot.drivetrain.wheel_radius_m)
            }),
            invert: [
                if robot.encoder.invert_left { -1. } else { 1. },
                if robot.encoder.invert_right { -1. } else { 1. },
            ],
            estimate: crate::control::estimator::Estimator::new(robot, base, line),
        }
    }
    pub fn step_input(&mut self, input: &ControllerInput, dt: f64) -> TimedCommand {
        let f = &input.frame;
        self.estimate.update(f);
        if !f.line_visible {
            let since = *self.lost_since.get_or_insert(f.t_us);
            self.integral = 0.;
            self.speed_integral = [0.; 2];
            self.initialized = false;
            self.derivative = 0.;
            let search = if f.t_us - since >= self.settings.loss_timeout_us {
                self.settings.recovery_pwm.min(self.cfg.max_pwm)
            } else {
                0.
            };
            let sign = if self.prev_error < 0. { -1. } else { 1. };
            return TimedCommand {
                t_us: f.t_us,
                pwm: [-sign * search, sign * search],
                downforce_pwm: self.cfg.downforce_pwm,
                modes: if search == 0. {
                    [ActuatorMode::Brake; 2]
                } else {
                    [ActuatorMode::Drive; 2]
                },
            };
        }
        self.lost_since = None;
        let error = f.line_position_m - self.cfg.target_position_m;
        let raw = if self.initialized && dt > 0. {
            (error - self.prev_error) / dt
        } else {
            0.
        };
        let a = if self.settings.derivative_tau_s > 0. {
            1. - (-dt / self.settings.derivative_tau_s).exp()
        } else {
            1.
        };
        self.derivative += a * (raw - self.derivative);
        self.prev_error = error;
        self.initialized = true;
        let candidate = (self.integral + error * dt)
            .clamp(-self.settings.integral_limit, self.settings.integral_limit);
        let proposed =
            self.cfg.kp * error + self.cfg.ki * candidate + self.cfg.kd * self.derivative;
        let limit = self.cfg.max_pwm.max(0.);
        if !input.actuators.current_limited.iter().any(|v| *v)
            && ((self.cfg.base_pwm.abs() + proposed.abs() <= limit) || proposed * error < 0.)
        {
            self.integral = candidate;
        }
        let correction =
            self.cfg.kp * error + self.cfg.ki * self.integral + self.cfg.kd * self.derivative;
        let mut pwm = [
            self.cfg.base_pwm - correction,
            self.cfg.base_pwm + correction,
        ];
        if self.settings.speed_mode == 1 {
            if !f.encoder.valid {
                pwm = [0.; 2];
            } else {
                let target = self.settings.target_speed_m_s * self.estimate.state.speed_factor;
                let targets = [target - correction, target + correction];
                let measured = [
                    f.encoder.left.velocity_rad_s * self.radius[0] * self.invert[0],
                    f.encoder.right.velocity_rad_s * self.radius[1] * self.invert[1],
                ];
                for side in 0..2 {
                    let e = targets[side] - measured[side];
                    let next = (self.speed_integral[side] + e * dt)
                        .clamp(-self.settings.integral_limit, self.settings.integral_limit);
                    let u = self.settings.speed_kp * e + self.settings.speed_ki * next;
                    if !input.actuators.current_limited[side] && (u.abs() <= limit || u * e < 0.) {
                        self.speed_integral[side] = next;
                    }
                    pwm[side] = self.settings.speed_kp * e
                        + self.settings.speed_ki * self.speed_integral[side];
                }
            }
        }
        TimedCommand {
            t_us: f.t_us,
            pwm: pwm.map(|v| v.clamp(-limit, limit)),
            downforce_pwm: self.cfg.downforce_pwm.clamp(0., 1.),
            modes: [ActuatorMode::Drive; 2],
        }
    }
    pub fn output(&self, c: TimedCommand) -> ControllerOutput {
        ControllerOutput {
            pwm_left: c.pwm[0],
            pwm_right: c.pwm[1],
            pwm_downforce: c.downforce_pwm,
            error_m: self.prev_error,
            correction: (c.pwm[1] - c.pwm[0]) / 2.,
        }
    }
}
impl Controller for BuiltInPid {
    fn step(&mut self, s: &SensorOutput, dt: f64) -> ControllerOutput {
        let input = ControllerInput {
            frame: crate::control::SensorFrame::from_readings(
                s.t_us,
                s,
                Default::default(),
                Default::default(),
            ),
            ..Default::default()
        };
        let c = self.step_input(&input, dt);
        self.output(c)
    }
}
