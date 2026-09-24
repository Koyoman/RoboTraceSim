//! Odometry and learned profile consume measured ticks/gyro/optical channels only.
use super::SensorFrame;
use crate::{math::Pose2, models::sensing::ControlSettings};
#[derive(Debug, Clone, Default)]
pub struct Estimate {
    pub pose: Pose2,
    pub distance_m: f64,
    pub lap: u32,
    pub marks: u64,
    pub profile: Vec<(f64, f64)>,
    pub speed_factor: f64,
}
#[derive(Debug, Clone)]
pub struct Estimator {
    pub state: Estimate,
    settings: ControlSettings,
    meters_per_tick: [f64; 2],
    width: f64,
    last_ticks: Option<[i64; 2]>,
    last_time: Option<u64>,
    mark_high: bool,
    last_mark: Option<u64>,
    lap_origin: Option<f64>,
    channels: Vec<(
        crate::config::RobotSensorInstance,
        crate::models::sensing::Acquisition,
    )>,
    base: f64,
    line: f64,
    last_profile_distance: f64,
}
impl Estimator {
    pub fn new(robot: &crate::config::RobotConfig, base: f64, line: f64) -> Self {
        let e = &robot.sensing.encoder;
        let k = std::f64::consts::TAU
            / (robot.encoder.ticks_per_rev as f64 * e.quadrature as f64 * e.shaft_ratio);
        let radii: [f64; 2] = std::array::from_fn(|side| {
            robot
                .assembly
                .as_ref()
                .and_then(|a| {
                    a.wheels.iter().find(|w| {
                        w.motor.as_deref()
                            == Some(if side == 0 {
                                "motor:left"
                            } else {
                                "motor:right"
                            })
                    })
                })
                .map(|w| w.radius_m)
                .unwrap_or(robot.drivetrain.wheel_radius_m)
        });
        Self {
            state: Estimate {
                speed_factor: 1.,
                ..Default::default()
            },
            settings: robot.sensing.control.clone(),
            meters_per_tick: [
                radii[0] * k * if robot.encoder.invert_left { -1. } else { 1. },
                radii[1] * k * if robot.encoder.invert_right { -1. } else { 1. },
            ],
            width: robot.drivetrain.track_width_m,
            last_ticks: None,
            last_time: None,
            mark_high: false,
            last_mark: None,
            lap_origin: None,
            channels: robot
                .sensors
                .iter()
                .filter(|s| s.enabled)
                .map(|s| {
                    (
                        s.clone(),
                        robot
                            .sensing
                            .optical
                            .get(&s.id)
                            .cloned()
                            .unwrap_or_default(),
                    )
                })
                .collect(),
            base,
            line,
            last_profile_distance: 0.,
        }
    }
    pub fn update(&mut self, f: &SensorFrame) {
        let dt = self
            .last_time
            .map(|t| (f.t_us - t) as f64 * 1e-6)
            .unwrap_or(0.);
        self.last_time = Some(f.t_us);
        let mut ds = 0.;
        let mut de = 0.;
        if f.encoder.valid {
            let ticks = [f.encoder.left.ticks, f.encoder.right.ticks];
            if let Some(last) = self.last_ticks {
                let dl = (ticks[0] - last[0]) as f64 * self.meters_per_tick[0];
                let dr = (ticks[1] - last[1]) as f64 * self.meters_per_tick[1];
                ds = (dl + dr) / 2.;
                de = (dr - dl) / self.width;
            }
            self.last_ticks = Some(ticks);
        }
        let w = if f.imu.valid && f.encoder.valid {
            self.settings.gyro_weight
        } else if f.imu.valid {
            1.
        } else {
            0.
        };
        let dy = (1. - w) * de + w * f.imu.yaw_rate_rad_s * dt;
        let heading = self.state.pose.yaw + dy / 2.;
        self.state.pose.x += ds * heading.cos();
        self.state.pose.y += ds * heading.sin();
        self.state.pose.yaw += dy;
        self.state.distance_m += ds.abs();
        let count = f
            .optical
            .iter()
            .filter(|r| r.valid)
            .filter(|r| {
                let strength = self
                    .channels
                    .iter()
                    .find(|(s, _)| s.id == r.id)
                    .map(|(s, a)| {
                        crate::sensor::calibrated_strength(s, a, r.adc, self.base, self.line)
                    })
                    .unwrap_or(0.);
                strength >= self.settings.mark_threshold
            })
            .count();
        let high = count >= self.settings.mark_min_channels as usize;
        if high
            && !self.mark_high
            && self
                .last_mark
                .is_none_or(|t| f.t_us - t >= self.settings.mark_refractory_us)
        {
            self.last_mark = Some(f.t_us);
            self.state.marks += 1;
            if count == self.channels.len() && !self.channels.is_empty() {
                if let Some(origin) = self.lap_origin {
                    if self.state.distance_m - origin >= self.settings.lap_min_distance_m {
                        self.state.lap += 1;
                        self.lap_origin = Some(self.state.distance_m);
                    }
                } else {
                    self.lap_origin = Some(self.state.distance_m);
                }
            }
        }
        self.mark_high = high;
        if self.state.lap == 0
            && self.lap_origin.is_some()
            && self.state.distance_m - self.last_profile_distance >= 0.01
            && self.state.profile.len() < 4096
        {
            self.last_profile_distance = self.state.distance_m;
            self.state.profile.push((
                self.state.distance_m - self.lap_origin.unwrap(),
                if dy.abs() > ds.abs() * 2. {
                    self.settings.curve_speed_factor
                } else {
                    1.
                },
            ));
        }
        self.state.speed_factor = if self.settings.profile_enabled == 1 && self.state.lap > 0 {
            let at = self.state.distance_m - self.lap_origin.unwrap_or(0.);
            self.state
                .profile
                .iter()
                .find(|(distance, _)| *distance >= at)
                .map(|(_, factor)| *factor)
                .unwrap_or(1.)
        } else {
            1.
        };
    }
}
