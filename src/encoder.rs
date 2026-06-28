use crate::config::EncoderConfig;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Default)]
pub struct EncoderSideOutput {
    pub ticks: i64,
    pub delta_ticks: i64,
    pub velocity_rad_s: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct EncoderOutput {
    pub t_us: u64,
    pub left: EncoderSideOutput,
    pub right: EncoderSideOutput,
}

#[derive(Debug, Clone)]
pub struct QuantizedEncoder {
    cfg: EncoderConfig,
    last_left_ticks: i64,
    last_right_ticks: i64,
    last_t_us: Option<u64>,
}

impl QuantizedEncoder {
    pub fn new(cfg: EncoderConfig) -> Self {
        Self {
            cfg,
            last_left_ticks: 0,
            last_right_ticks: 0,
            last_t_us: None,
        }
    }

    pub fn sample(
        &mut self,
        left_angle_rad: f64,
        right_angle_rad: f64,
        t_us: u64,
    ) -> EncoderOutput {
        let left_ticks =
            angle_to_ticks(left_angle_rad, self.cfg.ticks_per_rev, self.cfg.invert_left);
        let right_ticks = angle_to_ticks(
            right_angle_rad,
            self.cfg.ticks_per_rev,
            self.cfg.invert_right,
        );
        let dt_s = self
            .last_t_us
            .map(|last| (t_us.saturating_sub(last) as f64 / 1_000_000.0).max(1e-12));
        let left_delta = left_ticks - self.last_left_ticks;
        let right_delta = right_ticks - self.last_right_ticks;
        let left_velocity = dt_s
            .map(|dt| ticks_to_angle(left_delta, self.cfg.ticks_per_rev) / dt)
            .unwrap_or(0.0);
        let right_velocity = dt_s
            .map(|dt| ticks_to_angle(right_delta, self.cfg.ticks_per_rev) / dt)
            .unwrap_or(0.0);

        self.last_left_ticks = left_ticks;
        self.last_right_ticks = right_ticks;
        self.last_t_us = Some(t_us);

        EncoderOutput {
            t_us,
            left: EncoderSideOutput {
                ticks: left_ticks,
                delta_ticks: left_delta,
                velocity_rad_s: left_velocity,
            },
            right: EncoderSideOutput {
                ticks: right_ticks,
                delta_ticks: right_delta,
                velocity_rad_s: right_velocity,
            },
        }
    }
}

fn angle_to_ticks(angle_rad: f64, ticks_per_rev: u32, inverted: bool) -> i64 {
    let sign = if inverted { -1.0 } else { 1.0 };
    (sign * angle_rad / (2.0 * PI) * ticks_per_rev.max(1) as f64).round() as i64
}

fn ticks_to_angle(ticks: i64, ticks_per_rev: u32) -> f64 {
    ticks as f64 * 2.0 * PI / ticks_per_rev.max(1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoder_quantizes_to_integer_ticks() {
        let cfg = EncoderConfig {
            model: "QuantizedEncoder".to_string(),
            ticks_per_rev: 100,
            invert_left: false,
            invert_right: false,
        };
        let mut enc = QuantizedEncoder::new(cfg);
        let out = enc.sample(std::f64::consts::PI, -std::f64::consts::PI, 1_000);
        assert_eq!(out.left.ticks, 50);
        assert_eq!(out.right.ticks, -50);
    }
}
