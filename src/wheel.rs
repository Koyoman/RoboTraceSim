use crate::config::TireConfig;
use crate::math::clamp;

#[derive(Debug, Clone, Copy, Default)]
pub struct TireInput {
    pub desired_force_n: f64,
    pub normal_force_n: f64,
    pub mu: f64,
    pub ground_speed_m_s: f64,
    pub wheel_omega_rad_s: f64,
    pub wheel_radius_m: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct WheelForces {
    pub force_n: f64,
    pub desired_force_n: f64,
    pub max_force_n: f64,
    pub slip_ratio: f64,
    pub wheel_surface_speed_m_s: f64,
    pub saturated: bool,
}

pub trait TireModel {
    fn longitudinal_force(&self, input: TireInput) -> WheelForces;
}

#[derive(Debug, Clone)]
pub struct SlipRatioWheel {
    cfg: TireConfig,
}

impl SlipRatioWheel {
    pub fn new(cfg: TireConfig) -> Self {
        Self { cfg }
    }
}

impl TireModel for SlipRatioWheel {
    fn longitudinal_force(&self, input: TireInput) -> WheelForces {
        let max_force_n = (input.mu * input.normal_force_n).max(0.0);
        let force_n = clamp(input.desired_force_n, -max_force_n, max_force_n);
        let surface_speed = input.wheel_omega_rad_s * input.wheel_radius_m;
        let denom = input
            .ground_speed_m_s
            .abs()
            .max(surface_speed.abs())
            .max(self.cfg.slip_velocity_epsilon_m_s.max(1e-6));
        let kinematic_slip = (surface_speed - input.ground_speed_m_s) / denom;
        let excess = (input.desired_force_n.abs() - max_force_n).max(0.0);
        let overload_slip = if max_force_n > 1e-9 {
            excess / max_force_n
        } else {
            0.0
        };
        let slip_ratio = if self.cfg.model.eq_ignore_ascii_case("CoulombFrictionWheel") {
            overload_slip.copysign(input.desired_force_n)
        } else if kinematic_slip.abs() > overload_slip.abs() {
            kinematic_slip
        } else {
            overload_slip.copysign(input.desired_force_n)
        };
        let saturated = input.desired_force_n.abs() > max_force_n + 1e-9;

        WheelForces {
            force_n,
            desired_force_n: input.desired_force_n,
            max_force_n,
            slip_ratio,
            wheel_surface_speed_m_s: surface_speed,
            saturated,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(model: &str) -> TireConfig {
        TireConfig {
            model: model.to_string(),
            mu_longitudinal: 1.2,
            mu_lateral: 1.0,
            rolling_resistance: 0.0,
            slip_velocity_epsilon_m_s: 0.05,
        }
    }

    #[test]
    fn force_is_limited_by_mu_n() {
        let tire = SlipRatioWheel::new(cfg("SlipRatioWheel"));
        let out = tire.longitudinal_force(TireInput {
            desired_force_n: 20.0,
            normal_force_n: 10.0,
            mu: 1.2,
            ground_speed_m_s: 0.0,
            wheel_omega_rad_s: 0.0,
            wheel_radius_m: 0.01,
        });
        assert!((out.force_n - 12.0).abs() < 1e-12);
        assert!(out.slip_ratio > 0.0);
        assert!(out.saturated);
    }

    #[test]
    fn slip_ratio_uses_surface_minus_ground_speed() {
        let tire = SlipRatioWheel::new(cfg("SlipRatioWheel"));
        let out = tire.longitudinal_force(TireInput {
            desired_force_n: 1.0,
            normal_force_n: 10.0,
            mu: 1.2,
            ground_speed_m_s: 1.0,
            wheel_omega_rad_s: 150.0,
            wheel_radius_m: 0.01,
        });
        assert!(out.slip_ratio > 0.0);
    }
}
