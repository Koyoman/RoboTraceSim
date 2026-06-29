use crate::config::LineSensorConfig;
use crate::math::{clamp, clamp01, Pose2, Vec2};
use crate::rng::DeterministicRng;
use crate::track::TrackModel;

pub trait SensorModel {
    fn sample(&mut self, track: &dyn TrackModel, pose: Pose2, t_us: u64) -> SensorOutput;
}

#[derive(Debug, Clone)]
pub struct SensorOutput {
    pub t_us: u64,
    pub adc: Vec<u32>,
    pub line_position_m: f64,
    pub line_visible: bool,
    pub confidence: f64,
}

#[derive(Debug, Clone)]
pub struct SimpleLineSensor {
    cfg: LineSensorConfig,
    rng: DeterministicRng,
    last_position_m: f64,
}

impl SimpleLineSensor {
    pub fn new(cfg: LineSensorConfig) -> Self {
        let rng = DeterministicRng::new(cfg.seed);
        Self {
            cfg,
            rng,
            last_position_m: 0.0,
        }
    }

    pub fn count(&self) -> usize {
        self.cfg.count
    }
}

impl SensorModel for SimpleLineSensor {
    fn sample(&mut self, track: &dyn TrackModel, pose: Pose2, t_us: u64) -> SensorOutput {
        let count = self.cfg.count;
        let max_adc = (1u32
            .checked_shl(self.cfg.adc_bits)
            .unwrap_or(0)
            .saturating_sub(1)) as f64;
        let spacing = if count > 1 {
            self.cfg.width_m / (count as f64 - 1.0)
        } else {
            0.0
        };
        let base = track.base_reflectance();
        let line = track.line_reflectance();
        let contrast = (line - base).abs().max(1e-9);

        let mut adc = Vec::with_capacity(count);
        let mut weighted_sum = 0.0;
        let mut signal_sum = 0.0;

        for i in 0..count {
            // Index 0 is the leftmost sensor in robot coordinates. Positive local y is left.
            let y_local = self.cfg.width_m * 0.5 - i as f64 * spacing;
            let world = pose.transform_point(Vec2::new(self.cfg.forward_offset_m, y_local));
            let ideal_reflectance = track.reflectance_at(world);
            let noisy_reflectance = clamp01(
                ideal_reflectance * self.cfg.gain
                    + self.cfg.offset
                    + self.rng.gaussian(self.cfg.reflectance_noise_std),
            );
            let reflectance_level = clamp01(noisy_reflectance);
            let line_signal = if line >= base {
                clamp01((noisy_reflectance - base) / contrast)
            } else {
                clamp01((base - noisy_reflectance) / contrast)
            };
            let adc_float = reflectance_level * max_adc + self.rng.gaussian(self.cfg.adc_noise_lsb);
            let value = clamp(adc_float.round(), 0.0, max_adc) as u32;
            adc.push(value);
            weighted_sum += y_local * line_signal;
            signal_sum += line_signal;
        }

        let visible_threshold = 0.05;
        let line_visible = signal_sum > visible_threshold;
        let line_position_m = if line_visible {
            weighted_sum / signal_sum
        } else {
            self.last_position_m
        };
        self.last_position_m = line_position_m;

        SensorOutput {
            t_us,
            adc,
            line_position_m,
            line_visible,
            confidence: clamp01(signal_sum / count as f64),
        }
    }
}
