//! Aggregate wheel contact telemetry. Dynamics live in core::integrator.
#[derive(Debug, Clone, Copy, Default)]
pub struct WheelForces {
    pub force_n: f64,
    pub desired_force_n: f64,
    pub max_force_n: f64,
    pub slip_ratio: f64,
    pub wheel_surface_speed_m_s: f64,
    pub saturated: bool,
}
