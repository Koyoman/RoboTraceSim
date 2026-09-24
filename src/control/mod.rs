pub mod estimator;
pub mod native;
pub mod replay_controller;
use crate::{encoder::EncoderOutput, gyro::GyroOutput, sensor::SensorOutput};
#[derive(Debug, Clone)]
pub struct OpticalSample {
    pub id: String,
    pub acquired_us: u64,
    pub available_us: u64,
    pub age_us: u64,
    pub valid: bool,
    pub adc: u32,
    pub digital: Option<bool>,
}
#[derive(Debug, Clone, Default)]
pub struct ImuSample {
    pub acquired_us: u64,
    pub available_us: u64,
    pub age_us: u64,
    pub valid: bool,
    pub yaw_rate_rad_s: f64,
    pub acceleration_m_s2: crate::math::Vec2,
}
#[derive(Debug, Clone, Default)]
pub struct SensorFrame {
    pub t_us: u64,
    pub optical: Vec<OpticalSample>,
    pub line_position_m: f64,
    pub line_visible: bool,
    pub confidence: f64,
    pub encoder: EncoderOutput,
    pub imu: ImuSample,
}
impl SensorFrame {
    pub fn from_readings(t_us: u64, s: &SensorOutput, e: EncoderOutput, g: GyroOutput) -> Self {
        Self {
            t_us,
            optical: s
                .channels
                .iter()
                .map(|r| OpticalSample {
                    id: r.id.clone(),
                    acquired_us: r.acquired_us,
                    available_us: r.available_us,
                    age_us: r.age_us,
                    valid: r.valid,
                    adc: r.adc,
                    digital: r.digital,
                })
                .collect(),
            line_position_m: s.line_position_m,
            line_visible: s.line_visible,
            confidence: s.confidence,
            encoder: e,
            imu: ImuSample {
                acquired_us: g.t_us,
                available_us: g.available_us,
                age_us: g.age_us,
                valid: g.valid,
                yaw_rate_rad_s: g.yaw_rate_rad_s,
                acceleration_m_s2: g.acceleration_m_s2,
            },
        }
    }
}
#[derive(Debug, Clone, Default)]
pub struct ActuatorFeedback {
    pub applied_pwm: [f64; 2],
    pub current_limited: [bool; 2],
}
#[derive(Debug, Clone, Default)]
pub struct ControllerInput {
    pub frame: SensorFrame,
    pub actuators: ActuatorFeedback,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimedCommand {
    pub t_us: u64,
    pub pwm: [f64; 2],
    pub downforce_pwm: f64,
    pub modes: [crate::models::power::ActuatorMode; 2],
}
