use crate::config::GyroConfig;
use crate::math::Vec2;
use crate::models::sensing::ImuSettings;
use crate::rng::DeterministicRng;
use std::collections::VecDeque;
#[derive(Debug, Clone, Copy, Default)]
pub struct GyroOutput {
    pub t_us: u64,
    pub available_us: u64,
    pub age_us: u64,
    pub valid: bool,
    pub yaw_rate_rad_s: f64,
    pub bias_rad_s: f64,
    pub acceleration_m_s2: Vec2,
}
#[derive(Debug, Clone)]
pub struct NoisyGyro {
    cfg: GyroConfig,
    settings: ImuSettings,
    rng: DeterministicRng,
    drift_rng: DeterministicRng,
    accel_rng: [DeterministicRng; 2],
    bias_rad_s: f64,
    last: Option<u64>,
    filtered: Option<(f64, Vec2)>,
    pending: VecDeque<GyroOutput>,
    delivered: GyroOutput,
}
impl NoisyGyro {
    pub fn new(cfg: GyroConfig) -> Self {
        Self::with_settings(cfg, ImuSettings::default())
    }
    pub fn with_settings(cfg: GyroConfig, settings: ImuSettings) -> Self {
        Self {
            rng: DeterministicRng::new(cfg.seed),
            drift_rng: DeterministicRng::new(settings.seed),
            accel_rng: [
                DeterministicRng::new(settings.seed ^ 0xe7037ed1a0b428db),
                DeterministicRng::new(settings.seed ^ 0x8ebc6af09c88c6e3),
            ],
            bias_rad_s: cfg.bias_rad_s,
            cfg,
            settings,
            last: None,
            filtered: None,
            pending: VecDeque::new(),
            delivered: Default::default(),
        }
    }
    pub fn sample(&mut self, rate: f64, t_us: u64) -> GyroOutput {
        self.sample_imu(rate, Vec2::default(), t_us)
    }
    pub fn sample_imu(&mut self, rate: f64, accel: Vec2, t_us: u64) -> GyroOutput {
        let dt = self.last.map(|t| (t_us - t) as f64 * 1e-6).unwrap_or(0.);
        self.bias_rad_s += self
            .drift_rng
            .gaussian(self.settings.drift_std_rad_s_sqrt_s * dt.sqrt());
        let angle = self.settings.yaw_misalignment_deg.to_radians();
        let mut a = Vec2::new(
            angle.cos() * accel.x
                + angle.sin() * accel.y
                + self.settings.accel_bias_x_m_s2
                + self.accel_rng[0].gaussian(self.settings.accel_noise_std_m_s2),
            -angle.sin() * accel.x
                + angle.cos() * accel.y
                + self.settings.accel_bias_y_m_s2
                + self.accel_rng[1].gaussian(self.settings.accel_noise_std_m_s2),
        );
        let mut w = rate + self.bias_rad_s + self.rng.gaussian(self.cfg.noise_std_rad_s);
        if let Some((old, prev)) = self.filtered {
            if self.settings.filter_tau_s > 0. {
                let alpha = 1. - (-dt / self.settings.filter_tau_s).exp();
                w = old + alpha * (w - old);
                a = prev + (a - prev) * alpha;
            }
        }
        self.filtered = Some((w, a));
        self.last = Some(t_us);
        let limit = self.settings.accel_limit_m_s2;
        self.pending.push_back(GyroOutput {
            t_us,
            available_us: t_us.saturating_add(self.settings.latency_us),
            age_us: 0,
            valid: true,
            yaw_rate_rad_s: w.clamp(-self.cfg.saturation_rad_s, self.cfg.saturation_rad_s),
            bias_rad_s: self.bias_rad_s,
            acceleration_m_s2: Vec2::new(a.x.clamp(-limit, limit), a.y.clamp(-limit, limit)),
        });
        self.deliver(t_us)
    }
    pub fn deliver(&mut self, t_us: u64) -> GyroOutput {
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
    fn zero_noise_gyro_is_exact_plus_bias() {
        let cfg = GyroConfig {
            model: "NoisyGyro".to_string(),
            noise_std_rad_s: 0.0,
            bias_rad_s: 0.02,
            saturation_rad_s: 10.0,
            seed: 1,
        };
        let mut gyro = NoisyGyro::new(cfg);
        assert!((gyro.sample(1.0, 0).yaw_rate_rad_s - 1.02).abs() < 1e-12);
    }
}
