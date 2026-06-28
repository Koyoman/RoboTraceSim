#[derive(Debug, Clone, Copy)]
pub struct DeterministicRng {
    state: u64,
    cached_gaussian: Option<f64>,
}

impl DeterministicRng {
    pub fn new(seed: u64) -> Self {
        let state = if seed == 0 {
            0x9E37_79B9_7F4A_7C15
        } else {
            seed
        };
        Self {
            state,
            cached_gaussian: None,
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        // xorshift64*, deterministic and dependency-free. It is not cryptographic;
        // it is used only to make simulated noise repeatable across runs.
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    pub fn next_f64(&mut self) -> f64 {
        const SCALE: f64 = 1.0 / ((1u64 << 53) as f64);
        ((self.next_u64() >> 11) as f64) * SCALE
    }

    pub fn gaussian(&mut self, std_dev: f64) -> f64 {
        if std_dev <= 0.0 || !std_dev.is_finite() {
            return 0.0;
        }
        if let Some(z) = self.cached_gaussian.take() {
            return z * std_dev;
        }
        let u1 = self.next_f64().clamp(1e-12, 1.0);
        let u2 = self.next_f64();
        let radius = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        let z0 = radius * theta.cos();
        let z1 = radius * theta.sin();
        self.cached_gaussian = Some(z1);
        z0 * std_dev
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seeded_rng_is_repeatable() {
        let mut a = DeterministicRng::new(42);
        let mut b = DeterministicRng::new(42);
        for _ in 0..8 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }
}
