use crate::config::NormalForceConfig;
use crate::math::{clamp, clamp01, Vec2};

#[derive(Debug, Clone, Copy, Default)]
pub struct NormalForceInput {
    pub mass_kg: f64,
    pub center_of_mass_m: Vec2,
    pub wheelbase_m: f64,
    pub track_width_m: f64,
    pub battery_voltage_v: f64,
    pub command_pwm: f64,
    pub speed_m_s: f64,
    pub dt_us: u64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct NormalForceOutput {
    pub front_left_n: f64,
    pub front_right_n: f64,
    pub rear_left_n: f64,
    pub rear_right_n: f64,
    pub extra_downforce_n: f64,
    pub fan_force_n: f64,
    pub suction_force_n: f64,
    pub command_pwm: f64,
    pub current_a: f64,
}

impl NormalForceOutput {
    pub fn total_normal_n(&self) -> f64 {
        self.front_left_n + self.front_right_n + self.rear_left_n + self.rear_right_n
    }

    pub fn left_n(&self) -> f64 {
        self.front_left_n + self.rear_left_n
    }

    pub fn right_n(&self) -> f64 {
        self.front_right_n + self.rear_right_n
    }
}

pub trait NormalForceModel {
    fn step(&mut self, input: NormalForceInput) -> NormalForceOutput;
}

#[derive(Debug, Clone)]
pub struct ConfiguredNormalForce {
    cfg: NormalForceConfig,
    fan_force_state_n: Vec<f64>,
    suction_force_state_n: f64,
}

impl ConfiguredNormalForce {
    pub fn new(cfg: NormalForceConfig) -> Self {
        let fan_force_state_n = vec![0.0; cfg.fans.len()];
        Self {
            cfg,
            fan_force_state_n,
            suction_force_state_n: 0.0,
        }
    }
}

impl NormalForceModel for ConfiguredNormalForce {
    fn step(&mut self, input: NormalForceInput) -> NormalForceOutput {
        let mut out = NormalForceOutput {
            command_pwm: clamp01(input.command_pwm),
            ..NormalForceOutput::default()
        };

        add_point_load(
            &mut out,
            (input.mass_kg.max(0.0) * 9.80665).max(0.0),
            input.center_of_mass_m,
            input.wheelbase_m,
            input.track_width_m,
        );

        match self.cfg.model.to_ascii_lowercase().as_str() {
            "constantdownforce" => {
                let position = self.cfg.position_m;
                let force = self.cfg.max_force_n.max(0.0);
                out.extra_downforce_n += force;
                add_point_load(
                    &mut out,
                    force,
                    position,
                    input.wheelbase_m,
                    input.track_width_m,
                );
            }
            "fandownforce" | "measureddownforcecurve" | "closedloopdownforce" => {
                let command_pwm = out.command_pwm;
                if self.cfg.fans.is_empty() {
                    let mut target_force = force_from_curve_or_square(
                        command_pwm,
                        self.cfg.max_force_n,
                        &self.cfg.force_curve,
                    );
                    if self.cfg.speed_sensitivity > 0.0 {
                        target_force *=
                            (1.0 + self.cfg.speed_sensitivity * input.speed_m_s.abs()).max(0.0);
                    }
                    self.suction_force_state_n = first_order(
                        self.suction_force_state_n,
                        target_force.max(0.0),
                        self.cfg.response_time_s,
                        input.dt_us,
                    );
                    let force = self.suction_force_state_n.max(0.0);
                    out.extra_downforce_n += force;
                    out.fan_force_n += force;
                    out.current_a += self.cfg.max_current_a.max(0.0) * command_pwm;
                    add_point_load(
                        &mut out,
                        force,
                        self.cfg.position_m,
                        input.wheelbase_m,
                        input.track_width_m,
                    );
                }
                for (idx, fan) in self.cfg.fans.iter().enumerate() {
                    let fan_pwm = clamp01(command_pwm * fan.pwm_scale * fan.enabled_pwm);
                    let mut target_force =
                        force_from_curve_or_square(fan_pwm, fan.max_force_n, &fan.force_curve);
                    if self.cfg.speed_sensitivity > 0.0 {
                        target_force *=
                            (1.0 + self.cfg.speed_sensitivity * input.speed_m_s.abs()).max(0.0);
                    }
                    let state = self.fan_force_state_n.get_mut(idx).expect("fan state len");
                    *state = first_order(*state, target_force, fan.response_time_s, input.dt_us);
                    let force = (*state).max(0.0);
                    out.extra_downforce_n += force;
                    out.fan_force_n += force;
                    out.current_a += fan_current_a(
                        fan_pwm,
                        fan.max_current_a,
                        input.battery_voltage_v,
                        fan.nominal_voltage_v,
                    );
                    add_point_load(
                        &mut out,
                        force,
                        fan.position_m,
                        input.wheelbase_m,
                        input.track_width_m,
                    );
                }
            }
            "suctiondownforce" => {
                let pwm = out.command_pwm;
                let mut target_force =
                    force_from_curve_or_square(pwm, self.cfg.max_force_n, &self.cfg.force_curve);
                if self.cfg.force_curve.is_empty() {
                    let delta_p = self.cfg.max_delta_pressure_pa.max(0.0) * pwm * pwm;
                    target_force = delta_p * self.cfg.chamber_area_m2.max(0.0);
                }
                target_force *= clamp01(1.0 - self.cfg.leakage_factor);
                if self.cfg.max_force_n > 0.0 {
                    target_force = target_force.min(self.cfg.max_force_n);
                }
                self.suction_force_state_n = first_order(
                    self.suction_force_state_n,
                    target_force.max(0.0),
                    self.cfg.response_time_s,
                    input.dt_us,
                );
                let force = self.suction_force_state_n.max(0.0);
                out.extra_downforce_n += force;
                out.suction_force_n += force;
                out.current_a += self.cfg.max_current_a.max(0.0) * pwm;
                add_point_load(
                    &mut out,
                    force,
                    self.cfg.position_m,
                    input.wheelbase_m,
                    input.track_width_m,
                );
            }
            _ => {}
        }

        out
    }
}

fn first_order(current: f64, target: f64, response_time_s: f64, dt_us: u64) -> f64 {
    let dt_s = dt_us as f64 / 1_000_000.0;
    if response_time_s <= 1e-9 {
        target
    } else {
        current + (target - current) * clamp01(dt_s / response_time_s)
    }
}

fn force_from_curve_or_square(pwm: f64, max_force_n: f64, curve: &[(f64, f64)]) -> f64 {
    if curve.is_empty() {
        return max_force_n.max(0.0) * pwm * pwm;
    }
    interp_curve(pwm, curve).max(0.0)
}

fn interp_curve(x: f64, curve: &[(f64, f64)]) -> f64 {
    if curve.is_empty() {
        return 0.0;
    }
    let x = clamp01(x);
    if x <= curve[0].0 {
        return curve[0].1;
    }
    for pair in curve.windows(2) {
        let (x0, y0) = pair[0];
        let (x1, y1) = pair[1];
        if x <= x1 {
            let denom = (x1 - x0).abs().max(1e-12);
            let a = clamp((x - x0) / denom, 0.0, 1.0);
            return y0 + (y1 - y0) * a;
        }
    }
    curve.last().map(|p| p.1).unwrap_or(0.0)
}

fn fan_current_a(
    pwm: f64,
    max_current_a: f64,
    battery_voltage_v: f64,
    nominal_voltage_v: f64,
) -> f64 {
    let voltage_scale = if nominal_voltage_v > 1e-9 {
        (battery_voltage_v / nominal_voltage_v).max(0.0)
    } else {
        1.0
    };
    max_current_a.max(0.0) * pwm * voltage_scale
}

fn add_point_load(
    out: &mut NormalForceOutput,
    force_n: f64,
    position_m: Vec2,
    wheelbase_m: f64,
    track_width_m: f64,
) {
    if force_n <= 0.0 {
        return;
    }
    let half_wheelbase = wheelbase_m.max(1e-6) * 0.5;
    let half_track = track_width_m.max(1e-6) * 0.5;
    let front_share = clamp(
        (position_m.x + half_wheelbase) / (2.0 * half_wheelbase),
        0.0,
        1.0,
    );
    let left_share = clamp((position_m.y + half_track) / (2.0 * half_track), 0.0, 1.0);

    out.front_left_n += force_n * front_share * left_share;
    out.front_right_n += force_n * front_share * (1.0 - left_share);
    out.rear_left_n += force_n * (1.0 - front_share) * left_share;
    out.rear_right_n += force_n * (1.0 - front_share) * (1.0 - left_share);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::NormalForceConfig;

    fn base_cfg(model: &str) -> NormalForceConfig {
        NormalForceConfig {
            model: model.to_string(),
            command_pwm_default: 1.0,
            position_m: Vec2::new(0.0, 0.0),
            max_force_n: 2.0,
            max_current_a: 1.0,
            response_time_s: 0.0,
            chamber_area_m2: 0.01,
            max_delta_pressure_pa: 100.0,
            leakage_factor: 0.0,
            speed_sensitivity: 0.0,
            force_curve: Vec::new(),
            fans: Vec::new(),
        }
    }

    #[test]
    fn centered_weight_splits_evenly() {
        let mut model = ConfiguredNormalForce::new(base_cfg("NoDownforce"));
        let out = model.step(NormalForceInput {
            mass_kg: 0.2,
            center_of_mass_m: Vec2::new(0.0, 0.0),
            wheelbase_m: 0.1,
            track_width_m: 0.08,
            dt_us: 500,
            ..NormalForceInput::default()
        });
        let expected = 0.2 * 9.80665 / 4.0;
        assert!((out.front_left_n - expected).abs() < 1e-12);
        assert!((out.rear_right_n - expected).abs() < 1e-12);
    }

    #[test]
    fn forward_left_load_increases_front_left() {
        let mut cfg = base_cfg("ConstantDownforce");
        cfg.position_m = Vec2::new(0.05, 0.04);
        cfg.max_force_n = 1.0;
        let mut model = ConfiguredNormalForce::new(cfg);
        let out = model.step(NormalForceInput {
            mass_kg: 0.0,
            center_of_mass_m: Vec2::new(0.0, 0.0),
            wheelbase_m: 0.1,
            track_width_m: 0.08,
            command_pwm: 1.0,
            dt_us: 500,
            ..NormalForceInput::default()
        });
        assert!(out.front_left_n > 0.99);
        assert!(out.front_right_n < 1e-9);
    }
}
