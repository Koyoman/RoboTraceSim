use crate::config::GyroConfig;
use crate::math::clamp;
use crate::rng::DeterministicRng;

#[derive(Debug, Clone, Copy, Default)]
pub struct GyroOutput {
    pub t_us: u64,
    pub yaw_rate_rad_s: f64,
    pub bias_rad_s: f64,
}

#[derive(Debug, Clone)]
pub struct NoisyGyro {
    cfg: GyroConfig,
    rng: DeterministicRng,
    bias_rad_s: f64,
}

impl NoisyGyro {
    pub fn new(cfg: GyroConfig) -> Self {
        let rng = DeterministicRng::new(cfg.seed);
        let bias_rad_s = cfg.bias_rad_s;
        Self {
            cfg,
            rng,
            bias_rad_s,
        }
    }

    pub fn sample(&mut self, true_yaw_rate_rad_s: f64, t_us: u64) -> GyroOutput {
        let noise = self.rng.gaussian(self.cfg.noise_std_rad_s);
        let measured = true_yaw_rate_rad_s + self.bias_rad_s + noise;
        GyroOutput {
            t_us,
            yaw_rate_rad_s: clamp(
                measured,
                -self.cfg.saturation_rad_s,
                self.cfg.saturation_rad_s,
            ),
            bias_rad_s: self.bias_rad_s,
        }
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
