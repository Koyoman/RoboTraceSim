use crate::config::{
    RobotSensorInstance, SensorDetectionArea as Area, SensorResponseModel as Response, SensorType,
};
use crate::math::{clamp01, Pose2, Vec2};
use crate::models::sensing::Acquisition;
use crate::rng::DeterministicRng;
use crate::track::TrackModel;
use std::collections::VecDeque;

pub trait SensorModel {
    fn sample(&mut self, track: &dyn TrackModel, pose: Pose2, t_us: u64) -> SensorOutput;
}
#[derive(Debug, Clone, Default)]
pub struct ChannelReading {
    pub id: String,
    pub acquired_us: u64,
    pub available_us: u64,
    pub age_us: u64,
    pub valid: bool,
    pub raw_reflectance: f64,
    pub filtered: f64,
    pub adc: u32,
    pub digital: Option<bool>,
}
#[derive(Debug, Clone, Default)]
pub struct SensorOutput {
    pub t_us: u64,
    pub adc: Vec<u32>,
    pub channels: Vec<ChannelReading>,
    pub line_position_m: f64,
    pub line_visible: bool,
    pub confidence: f64,
}
#[derive(Debug, Clone)]
struct Channel {
    instance: RobotSensorInstance,
    settings: Acquisition,
    points: Vec<Vec2>,
    rng: DeterministicRng,
    adc_rng: DeterministicRng,
    filtered: Option<f64>,
    digital: bool,
    last_acquired: Option<u64>,
    pending: VecDeque<ChannelReading>,
    delivered: ChannelReading,
}
#[derive(Debug, Clone)]
pub struct SimpleLineSensor {
    channels: Vec<Channel>,
    last_position_m: f64,
}
/// Deterministic midpoint area quadrature in the sensor's local plane.
/// Point radius is only visual metadata; Circle/Rectangle/Polygon are area footprints.
pub fn footprint(area: &Area, n: u32) -> Vec<Vec2> {
    let n = n.clamp(1, 64);
    let mut points = Vec::new();
    if let Area::Circle { radius_m } = area {
        if n == 1 {
            return vec![Vec2::default()];
        }
        for ring in 0..n {
            for angle in 0..n {
                let r = radius_m * ((ring as f64 + 0.5) / n as f64).sqrt();
                let phi = std::f64::consts::TAU * (angle as f64 + 0.5) / n as f64;
                points.push(Vec2::new(r * phi.cos(), r * phi.sin()));
            }
        }
        return points;
    }
    if let Area::Cone { range_m, angle_deg } = area {
        for ring in 0..n {
            for angle in 0..n {
                let r = range_m * ((ring as f64 + 0.5) / n as f64).sqrt();
                let phi = angle_deg.to_radians() * ((angle as f64 + 0.5) / n as f64 - 0.5);
                points.push(Vec2::new(r * phi.cos(), r * phi.sin()));
            }
        }
        return points;
    }

    let (xmin, xmax, ymin, ymax) = match area {
        Area::Point { .. } => return vec![Vec2::default()],
        Area::Rectangle { width_m, height_m } => {
            (-width_m / 2., width_m / 2., -height_m / 2., height_m / 2.)
        }
        Area::Circle { radius_m } => (-radius_m, *radius_m, -radius_m, *radius_m),
        Area::Cone { range_m, .. } => (0., *range_m, -range_m, *range_m),
        Area::CustomPolygon { points_m } => (
            points_m.iter().map(|p| p.x).fold(f64::INFINITY, f64::min),
            points_m
                .iter()
                .map(|p| p.x)
                .fold(f64::NEG_INFINITY, f64::max),
            points_m.iter().map(|p| p.y).fold(f64::INFINITY, f64::min),
            points_m
                .iter()
                .map(|p| p.y)
                .fold(f64::NEG_INFINITY, f64::max),
        ),
    };
    for ix in 0..n {
        for iy in 0..n {
            let p = Vec2::new(
                xmin + (xmax - xmin) * (ix as f64 + 0.5) / n as f64,
                ymin + (ymax - ymin) * (iy as f64 + 0.5) / n as f64,
            );
            let inside = match area {
                Area::Circle { radius_m } => p.norm2() <= radius_m * radius_m,
                Area::Cone { range_m, angle_deg } => {
                    p.norm() <= *range_m && p.y.atan2(p.x).abs() <= angle_deg.to_radians() / 2.
                }
                Area::CustomPolygon { points_m } => {
                    let mut hit = false;
                    for (a, b) in points_m
                        .iter()
                        .zip(points_m.iter().cycle().skip(1))
                        .take(points_m.len())
                    {
                        if (a.y > p.y) != (b.y > p.y)
                            && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x
                        {
                            hit = !hit;
                        }
                    }
                    hit
                }
                _ => true,
            };
            if inside {
                points.push(p);
            }
        }
    }
    points
}
impl SimpleLineSensor {
    pub fn from_robot(robot: &crate::config::RobotConfig) -> Self {
        Self {
            channels: robot
                .sensors
                .iter()
                .filter(|s| s.enabled)
                .map(|s| {
                    let settings = robot
                        .sensing
                        .optical
                        .get(&s.id)
                        .cloned()
                        .unwrap_or_default();
                    Channel {
                        instance: s.clone(),
                        points: footprint(&s.asset.detection_area, settings.area_samples),
                        settings,
                        rng: DeterministicRng::new(s.acquisition.seed),
                        adc_rng: DeterministicRng::new(s.acquisition.seed ^ 0x9e3779b97f4a7c15),
                        filtered: None,
                        digital: false,
                        last_acquired: None,
                        pending: VecDeque::new(),
                        delivered: ChannelReading {
                            id: s.id.clone(),
                            ..Default::default()
                        },
                    }
                })
                .collect(),
            last_position_m: 0.,
        }
    }
    pub fn count(&self) -> usize {
        self.channels.len()
    }
    pub fn ids(&self) -> Vec<&str> {
        self.channels
            .iter()
            .map(|c| c.instance.id.as_str())
            .collect()
    }
    /// Reuse held readings on ticks without acquisition or delivery. No RNG draws are skipped.
    pub fn advance_into(
        &mut self,
        track: &dyn TrackModel,
        pose: Pose2,
        t_us: u64,
        default_period: u64,
        output: &mut SensorOutput,
    ) {
        let due = self.channels.iter().any(|c| {
            let period = if c.settings.period_us == 0 {
                default_period
            } else {
                c.settings.period_us
            };
            (t_us >= c.settings.phase_us
                && (t_us - c.settings.phase_us) % period == 0
                && c.last_acquired != Some(t_us))
                || c.pending.front().is_some_and(|r| r.available_us <= t_us)
        });
        if due {
            *output = self.advance(track, pose, t_us, default_period);
        } else {
            output.t_us = t_us;
            for r in &mut output.channels {
                r.age_us = t_us.saturating_sub(r.acquired_us);
            }
        }
    }
    pub fn advance(
        &mut self,
        track: &dyn TrackModel,
        pose: Pose2,
        t_us: u64,
        default_period: u64,
    ) -> SensorOutput {
        for c in &mut self.channels {
            let period = if c.settings.period_us == 0 {
                default_period
            } else {
                c.settings.period_us
            };
            if t_us >= c.settings.phase_us
                && (t_us - c.settings.phase_us) % period == 0
                && c.last_acquired != Some(t_us)
            {
                Self::acquire(c, track, pose, t_us);
            }
        }
        self.deliver(track, t_us)
    }
    fn acquire(c: &mut Channel, track: &dyn TrackModel, pose: Pose2, t_us: u64) {
        let instance = &c.instance;
        let cfg = &instance.acquisition;
        let frame = Pose2::new(
            instance.position_m.x,
            instance.position_m.y,
            instance.angle_deg.to_radians(),
        );
        let raw = c
            .points
            .iter()
            .map(|p| track.reflectance_at(pose.transform_point(frame.transform_point(*p))))
            .sum::<f64>()
            / c.points.len() as f64;
        let response = response_value(instance, raw);
        let noisy = clamp01(response + c.rng.gaussian(cfg.reflectance_noise_std));
        let dt = c
            .last_acquired
            .map(|t| (t_us - t) as f64 * 1e-6)
            .unwrap_or(0.);
        let filtered = match c.filtered {
            Some(old) if c.settings.filter_tau_s > 0. => {
                old + (noisy - old) * (1. - (-dt / c.settings.filter_tau_s).exp())
            }
            _ => noisy,
        };
        c.filtered = Some(filtered);
        c.last_acquired = Some(t_us);
        let max = ((1u32 << cfg.adc_bits) - 1) as f64;
        let adc = (filtered * max + c.adc_rng.gaussian(cfg.adc_noise_lsb))
            .round()
            .clamp(0., max) as u32;
        let digital = if instance.asset.sensor_type == SensorType::LineDigital {
            let x = adc as f64 / max;
            let mid = match instance.asset.response_model {
                Response::Threshold { threshold } => threshold,
                _ => c.settings.threshold,
            };
            if c.digital {
                if x < mid - c.settings.hysteresis / 2. {
                    c.digital = false;
                }
            } else if x >= mid + c.settings.hysteresis / 2. {
                c.digital = true;
            }
            Some(c.digital)
        } else {
            None
        };
        c.pending.push_back(ChannelReading {
            id: instance.id.clone(),
            acquired_us: t_us,
            available_us: t_us
                .saturating_add(c.settings.conversion_us)
                .saturating_add(c.settings.latency_us),
            age_us: 0,
            valid: true,
            raw_reflectance: raw,
            filtered,
            adc: if let Some(d) = digital {
                if d {
                    max as u32
                } else {
                    0
                }
            } else {
                adc
            },
            digital,
        });
    }
    fn deliver(&mut self, track: &dyn TrackModel, t_us: u64) -> SensorOutput {
        let base = track.base_reflectance();
        let line = track.line_reflectance();
        let mut sum = 0.;
        let mut weighted = 0.;
        let mut readings = Vec::new();
        for c in &mut self.channels {
            while c.pending.front().is_some_and(|r| r.available_us <= t_us) {
                c.delivered = c.pending.pop_front().unwrap();
            }
            let mut r = c.delivered.clone();
            r.age_us = t_us.saturating_sub(r.acquired_us);
            if r.valid {
                let signal = calibrated_strength(&c.instance, &c.settings, r.adc, base, line);
                sum += signal;
                weighted += c.instance.position_m.y * signal;
            }
            readings.push(r);
        }
        let visible = sum > 0.05;
        if visible {
            self.last_position_m = weighted / sum;
        }
        SensorOutput {
            t_us,
            adc: readings.iter().map(|r| r.adc).collect(),
            channels: readings,
            line_position_m: self.last_position_m,
            line_visible: visible,
            confidence: if self.channels.is_empty() {
                0.
            } else {
                clamp01(sum / self.channels.len() as f64)
            },
        }
    }
}
impl SensorModel for SimpleLineSensor {
    fn sample(&mut self, track: &dyn TrackModel, pose: Pose2, t_us: u64) -> SensorOutput {
        for c in &mut self.channels {
            if c.last_acquired != Some(t_us) {
                Self::acquire(c, track, pose, t_us);
            }
        }
        self.deliver(track, t_us)
    }
}

pub fn response_value(instance: &RobotSensorInstance, value: f64) -> f64 {
    match &instance.asset.response_model {
        Response::Ideal => value,
        Response::Threshold { threshold } => {
            if instance.asset.sensor_type == SensorType::LineDigital {
                value
            } else if value >= *threshold {
                1.
            } else {
                0.
            }
        }
        Response::Linear { gain, offset } => value * gain + offset,
        Response::Polynomial { coefficients } => {
            coefficients.iter().rev().fold(0., |v, a| v * value + a)
        }
        Response::LookupTable { points } => {
            if value <= points[0].input {
                points[0].output
            } else {
                points
                    .windows(2)
                    .find(|w| value <= w[1].input)
                    .map(|w| {
                        w[0].output
                            + (w[1].output - w[0].output) * (value - w[0].input)
                                / (w[1].input - w[0].input)
                    })
                    .unwrap_or(points.last().unwrap().output)
            }
        }
        Response::Custom { .. } => unreachable!("unsupported response rejected"),
    }
}

/// Converts delivered ADC through fixed calibration endpoints; never queries position.
pub fn calibrated_strength(
    instance: &RobotSensorInstance,
    settings: &Acquisition,
    adc: u32,
    base: f64,
    line: f64,
) -> f64 {
    let map = |x: f64| {
        let value = clamp01(response_value(instance, x));
        if instance.asset.sensor_type == SensorType::LineDigital {
            let threshold = if let Response::Threshold { threshold } = instance.asset.response_model
            {
                threshold
            } else {
                settings.threshold
            };
            if value >= threshold {
                1.
            } else {
                0.
            }
        } else {
            value
        }
    };
    let b = map(base);
    let l = map(line);
    let contrast = l - b;
    if contrast.abs() < 1e-9 {
        return 0.;
    }
    let measured = adc as f64 / ((1u32 << instance.acquisition.adc_bits) - 1) as f64;
    clamp01((measured - b) / contrast)
}
