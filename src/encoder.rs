use crate::config::EncoderConfig;
use crate::models::sensing::EncoderSettings;
use crate::rng::DeterministicRng;
use std::collections::VecDeque;
#[derive(Debug, Clone, Copy, Default)]
pub struct EncoderSideOutput {
    pub ticks: i64,
    pub delta_ticks: i64,
    pub velocity_rad_s: f64,
}
#[derive(Debug, Clone, Copy, Default)]
pub struct EncoderOutput {
    pub t_us: u64,
    pub available_us: u64,
    pub age_us: u64,
    pub valid: bool,
    pub left: EncoderSideOutput,
    pub right: EncoderSideOutput,
}
#[derive(Debug, Clone)]
pub struct QuantizedEncoder {
    cfg: EncoderConfig,
    settings: EncoderSettings,
    previous: [i64; 2],
    observed: [i64; 2],
    filtered: [f64; 2],
    last: Option<u64>,
    rng: [DeterministicRng; 2],
    pending: VecDeque<EncoderOutput>,
    delivered: EncoderOutput,
}
impl QuantizedEncoder {
    pub fn new(cfg: EncoderConfig) -> Self {
        Self::with_settings(cfg, EncoderSettings::default())
    }
    pub fn with_settings(cfg: EncoderConfig, settings: EncoderSettings) -> Self {
        Self {
            cfg,
            rng: [
                DeterministicRng::new(settings.seed),
                DeterministicRng::new(settings.seed ^ 0xa0761d6478bd642f),
            ],
            settings,
            previous: [0; 2],
            observed: [0; 2],
            filtered: [0.; 2],
            last: None,
            pending: VecDeque::new(),
            delivered: Default::default(),
        }
    }
    pub fn effective_ticks_per_wheel_rev(&self) -> f64 {
        self.cfg.ticks_per_rev as f64 * self.settings.quadrature as f64 * self.settings.shaft_ratio
    }
    pub fn sample(&mut self, left: f64, right: f64, t_us: u64) -> EncoderOutput {
        let scale = self.effective_ticks_per_wheel_rev();
        let dt = self.last.map(|t| (t_us - t) as f64 * 1e-6);
        let angles = [left, right];
        let invert = [self.cfg.invert_left, self.cfg.invert_right];
        let mut sides = [EncoderSideOutput::default(); 2];
        for side in 0..2 {
            let ticks = (angles[side] / std::f64::consts::TAU
                * scale
                * if invert[side] { -1. } else { 1. })
            .round() as i64;
            let delta = ticks - self.previous[side];
            self.previous[side] = ticks;
            // Loss is sampled independently per side and acquisition. Large batches use a bounded binomial approximation.
            let count = delta.unsigned_abs();
            let lost = if self.settings.loss_probability == 0. {
                0
            } else if self.settings.loss_probability == 1. {
                count
            } else if count <= 4096 {
                (0..count)
                    .filter(|_| self.rng[side].next_f64() < self.settings.loss_probability)
                    .count() as u64
            } else {
                let p = self.settings.loss_probability;
                (count as f64 * p + self.rng[side].gaussian((count as f64 * p * (1. - p)).sqrt()))
                    .round()
                    .clamp(0., count as f64) as u64
            };
            let observed = delta.signum() * (count - lost) as i64;
            self.observed[side] += observed;
            let v = dt
                .filter(|dt| *dt > 0.)
                .map(|dt| observed as f64 * std::f64::consts::TAU / scale / dt)
                .unwrap_or(0.);
            let alpha = if self.settings.filter_tau_s > 0. {
                dt.map(|dt| 1. - (-dt / self.settings.filter_tau_s).exp())
                    .unwrap_or(1.)
            } else {
                1.
            };
            self.filtered[side] += alpha * (v - self.filtered[side]);
            sides[side] = EncoderSideOutput {
                ticks: self.observed[side],
                delta_ticks: observed,
                velocity_rad_s: self.filtered[side],
            };
        }
        self.last = Some(t_us);
        self.pending.push_back(EncoderOutput {
            t_us,
            available_us: t_us.saturating_add(self.settings.latency_us),
            age_us: 0,
            valid: true,
            left: sides[0],
            right: sides[1],
        });
        self.deliver(t_us)
    }
    pub fn deliver(&mut self, t_us: u64) -> EncoderOutput {
        while self.pending.front().is_some_and(|v| v.available_us <= t_us) {
            self.delivered = self.pending.pop_front().unwrap();
        }
        self.delivered.age_us = t_us.saturating_sub(self.delivered.t_us);
        self.delivered
    }
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
