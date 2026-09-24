/// Conservative interoperable integer limit for the JSON parser's f64.
pub const MAX_EXACT_US: u64 = (1u64 << 53) - 1;

/// Convert a decimal duration to integer microseconds without silent truncation.
/// Only floating-point representation error (a few ULPs) is tolerated.
pub(crate) fn duration_to_us(value: f64, multiplier: f64) -> Result<u64, String> {
    let us = value * multiplier;
    if !us.is_finite() || us < 0.0 || us > MAX_EXACT_US as f64 {
        return Err(format!(
            "duration must be finite, non-negative and <= {MAX_EXACT_US} us"
        ));
    }
    let rounded = us.round();
    let tolerance = (4.0 * f64::EPSILON * us.abs().max(1.0)).min(0.000_001);
    if (us - rounded).abs() > tolerance || (us > 0.0 && rounded == 0.0) {
        return Err("duration must represent a whole number of microseconds".into());
    }
    Ok(rounded as u64)
}

pub fn duration_seconds_to_us(seconds: f64) -> Result<u64, String> {
    duration_to_us(seconds, 1_000_000.0)
}

#[derive(Debug, Clone)]
pub(crate) struct Clock {
    now_us: u64,
    duration_us: u64,
    dt_us: u64,
}

impl Clock {
    pub fn new(duration_us: u64, dt_us: u64) -> Result<Self, String> {
        if duration_us > MAX_EXACT_US {
            return Err(format!("duration_us must be <= {MAX_EXACT_US}"));
        }
        if dt_us == 0 || duration_us % dt_us != 0 {
            return Err(format!("duration_us ({duration_us}) must be a multiple of physics_dt_us ({dt_us}); choose a duration on the physical tick grid"));
        }
        Ok(Self {
            now_us: 0,
            duration_us,
            dt_us,
        })
    }

    pub fn now_us(&self) -> u64 {
        self.now_us
    }
    pub fn duration_us(&self) -> u64 {
        self.duration_us
    }
    pub fn steps(&self) -> u64 {
        self.now_us / self.dt_us
    }
    pub fn is_finished(&self) -> bool {
        self.now_us == self.duration_us
    }

    pub fn advance(&mut self) -> bool {
        if self.is_finished() {
            return false;
        }
        // Duration and current time are on the same grid, so this cannot overflow.
        self.now_us += self.dt_us;
        true
    }

    pub fn validate_target(&self, target_us: u64) -> Result<(), String> {
        if target_us < self.now_us || target_us > self.duration_us || target_us % self.dt_us != 0 {
            return Err(format!(
                "target_us must be on the {} us grid between {} and {}",
                self.dt_us, self.now_us, self.duration_us
            ));
        }
        Ok(())
    }
}
