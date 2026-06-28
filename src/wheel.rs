use crate::math::clamp;

#[derive(Debug, Clone, Copy, Default)]
pub struct WheelForces {
    pub force_n: f64,
    pub desired_force_n: f64,
    pub max_force_n: f64,
    pub slip_ratio: f64,
}

pub trait TireModel {
    fn longitudinal_force(&self, desired_force_n: f64, normal_force_n: f64, mu: f64) -> WheelForces;
}

#[derive(Debug, Clone, Copy)]
pub struct CoulombFrictionWheel;

impl TireModel for CoulombFrictionWheel {
    fn longitudinal_force(&self, desired_force_n: f64, normal_force_n: f64, mu: f64) -> WheelForces {
        let max_force_n = (mu * normal_force_n).max(0.0);
        let force_n = clamp(desired_force_n, -max_force_n, max_force_n);
        let excess = (desired_force_n.abs() - max_force_n).max(0.0);
        let slip_ratio = if max_force_n > 1e-9 { excess / max_force_n } else { 0.0 };
        WheelForces { force_n, desired_force_n, max_force_n, slip_ratio }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn force_is_limited_by_mu_n() {
        let tire = CoulombFrictionWheel;
        let out = tire.longitudinal_force(20.0, 10.0, 1.2);
        assert!((out.force_n - 12.0).abs() < 1e-12);
        assert!(out.slip_ratio > 0.0);
    }
}
