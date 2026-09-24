//! Per-wheel implicit impulses with a coupled friction ellipse.
use super::{fidelity::FidelityConfig, robot::WheelKind};
use crate::{
    config::RobotConfig, math::Vec2, motor::MotorDrive, sim::RobotState, track::TrackModel,
};
#[derive(Debug, Clone, Default)]
pub struct WheelState {
    pub omega: f64,
    pub angle: f64,
    pub caster_angle: f64,
    pub force_long_n: f64,
    pub force_lat_n: f64,
    pub normal_n: f64,
    pub slip: f64,
    pub slip_angle_rad: f64,
    pub rolling_torque_nm: f64,
    pub radial_compression_m: f64,
    pub regime: &'static str,
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ContactKind {
    Ideal,
    #[default]
    Coulomb,
    Brush,
}
#[derive(Debug, Clone, Default)]
pub struct ContactState {
    pub kind: ContactKind,
    pub quasi_static: bool,
    motor_torque: [f64; 2],
    pub wheels: Vec<WheelState>,
    pub acceleration: Vec2,
    pub iterations: usize,
}
pub(crate) struct StepResult {
    pub torques: [f64; 2],
    pub forces: [f64; 2],
    pub lateral: f64,
    pub energy: f64,
    pub work: f64,
    pub dissipation: f64,
}
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}
fn apply(v: &mut [f64], row: &[f64], inv: &[f64], delta: f64) {
    for k in 0..v.len() {
        v[k] += row[k] * inv[k] * delta;
    }
}
fn metric(a: &[f64], b: &[f64], inv: &[f64]) -> f64 {
    a.iter().zip(b).zip(inv).map(|((a, b), m)| a * b * m).sum()
}
fn kinetic(v: &[f64], inv: &[f64]) -> f64 {
    v.iter().zip(inv).map(|(v, m)| 0.5 * v * v / m).sum()
}
/// Minimize 1/2 x'Hx + g'x in an ellipse. Zero axes are fixed zero.
fn block_min(h: [f64; 3], g: [f64; 2], bounds: [f64; 2]) -> [f64; 2] {
    if bounds[0] == 0. {
        return [0., (-g[1] / h[2]).clamp(-bounds[1], bounds[1])];
    }
    if bounds[1] == 0. {
        return [(-g[0] / h[0]).clamp(-bounds[0], bounds[0]), 0.];
    }
    // Scale to unit disk once; safeguarded Newton solves the same convex KKT equation.
    let a0 = h[0] * bounds[0] * bounds[0];
    let d0 = h[2] * bounds[1] * bounds[1];
    let b = h[1] * bounds[0] * bounds[1];
    let c = [g[0] * bounds[0], g[1] * bounds[1]];
    let solve = |lambda: f64| {
        let a = a0 + lambda;
        let d = d0 + lambda;
        let determinant = a * d - b * b;
        [
            (-d * c[0] + b * c[1]) / determinant,
            (b * c[0] - a * c[1]) / determinant,
        ]
    };
    let norm = |x: [f64; 2]| x[0] * x[0] + x[1] * x[1];
    let free = solve(0.);
    if norm(free) <= 1. {
        return [free[0] * bounds[0], free[1] * bounds[1]];
    }
    let mut lo = 0.;
    let mut hi = c[0].hypot(c[1]);
    let mut lambda = hi * 0.5;
    for _ in 0..64 {
        let y = solve(lambda);
        let f = norm(y) - 1.;
        if f.abs() < 1e-13 {
            let scale = norm(y).sqrt().max(1.);
            return [y[0] / scale * bounds[0], y[1] / scale * bounds[1]];
        }
        if f > 0. {
            lo = lambda;
        } else {
            hi = lambda;
        }
        let a = a0 + lambda;
        let d = d0 + lambda;
        let det = a * d - b * b;
        let z = [(d * y[0] - b * y[1]) / det, (a * y[1] - b * y[0]) / det];
        let derivative = -2. * (y[0] * z[0] + y[1] * z[1]);
        let next = lambda - f / derivative;
        lambda = if next.is_finite() && next > lo && next < hi {
            next
        } else {
            (lo + hi) * 0.5
        };
    }
    let y = solve(hi);
    [y[0] * bounds[0], y[1] * bounds[1]]
}
impl ContactState {
    pub fn new(robot: &RobotConfig) -> Self {
        Self {
            kind: match robot.physics.as_ref().map(|f| f.contact.as_str()) {
                Some("ideal") => ContactKind::Ideal,
                Some("brush") => ContactKind::Brush,
                _ => ContactKind::Coulomb,
            },
            quasi_static: robot
                .physics
                .as_ref()
                .is_some_and(|f| f.normal == "quasi_static"),
            wheels: robot
                .assembly
                .as_ref()
                .unwrap()
                .wheels
                .iter()
                .map(|w| WheelState {
                    caster_angle: w.angle_deg.to_radians(),
                    regime: "rest",
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
    }
    pub(crate) fn step(
        &mut self,
        body: &mut RobotState,
        robot: &RobotConfig,
        f: &FidelityConfig,
        track: &dyn TrackModel,
        normals: &[f64],
        drives: [MotorDrive; 2],
        dt: f64,
    ) -> Result<StepResult, String> {
        let assembly = robot.assembly.as_ref().ok_or("assembly missing")?;
        let wheels = &assembly.wheels;
        let n = wheels.len();
        let mut inv = vec![
            1. / robot.chassis.mass_kg,
            1. / robot.chassis.mass_kg,
            1. / robot.chassis.inertia_kg_m2,
        ];
        inv.extend(wheels.iter().map(|w| 1. / w.inertia_kg_m2));
        let mut v = vec![body.vx_body_m_s, body.vy_body_m_s, body.yaw_rate_rad_s];
        v.extend(self.wheels.iter().map(|w| w.omega));
        let before = v.clone();
        let mut rows = Vec::with_capacity(n);
        let mut latrows = Vec::with_capacity(n);
        let mut bounds = Vec::with_capacity(n);
        let mut compliance = Vec::with_capacity(n);
        let mut memory = Vec::with_capacity(n);
        let mut storage_before = 0.;
        for (i, w) in wheels.iter().enumerate() {
            let r = w.position_m - robot.chassis.center_of_mass_m;
            if w.kind == WheelKind::Caster {
                // Overdamped swivel with positive trail; no invented fixed lateral constraint.
                let speed = Vec2::new(v[0] - v[2] * r.y, v[1] + v[2] * r.x);
                let a = self.wheels[i].caster_angle;
                let lateral = -a.sin() * speed.x + a.cos() * speed.y;
                self.wheels[i].caster_angle =
                    crate::math::wrap_angle(a + dt * lateral / f.caster_trail_m);
            }
            let angle = if w.kind == WheelKind::Caster {
                self.wheels[i].caster_angle
            } else {
                w.angle_deg.to_radians()
            };
            let (s, c) = angle.sin_cos();
            let compression = if f.radial_stiffness_n_m > 0. {
                normals[i] / f.radial_stiffness_n_m
            } else {
                0.
            };
            if compression > w.radius_m * 0.1 {
                return Err(format!(
                    "{}: quasi-static radial compression exceeds 10% of radius",
                    w.id
                ));
            }
            self.wheels[i].radial_compression_m = compression;
            let radius = w.radius_m - compression;
            let mut row = [0.; 11];
            row[0] = -c;
            row[1] = -s;
            row[2] = c * r.y - s * r.x;
            row[3 + i] = radius;
            let mut lat = [0.; 11];
            lat[0] = s;
            lat[1] = -c;
            lat[2] = -s * r.y - c * r.x;
            let mu = track.surface_mu_at(body.pose.transform_point(w.position_m));
            let factor = if normals[i] > 0. {
                (normals[i] / f.reference_load_n).powf(-f.load_exponent)
            } else {
                0.
            };
            bounds.push([
                w.tire.mu_longitudinal.min(mu) * normals[i] * factor * dt,
                w.tire.mu_lateral.min(mu) * normals[i] * factor * dt,
            ]);
            let cs = if self.kind == ContactKind::Brush {
                [f.longitudinal_stiffness, f.lateral_stiffness]
            } else {
                [f64::INFINITY; 2]
            };
            compliance.push(cs.map(|c| (1. + f.relaxation_s / dt) / (dt * c)));
            memory.push(
                [self.wheels[i].force_long_n, self.wheels[i].force_lat_n]
                    .into_iter()
                    .zip(cs)
                    .map(|(force, c)| -f.relaxation_s * force / (dt * c))
                    .collect::<Vec<_>>(),
            );
            storage_before += f.relaxation_s
                * 0.5
                * (self.wheels[i].force_long_n.powi(2) / cs[0]
                    + self.wheels[i].force_lat_n.powi(2) / cs[1]);
            rows.push(row);
            latrows.push(lat);
        }
        if bounds
            .iter()
            .flatten()
            .chain(compliance.iter().flatten())
            .any(|v| !v.is_finite() || *v < 0.)
        {
            return Err(
                "non-finite contact limits/compliance; check physical parameter scale".into(),
            );
        }
        let groups: [Vec<usize>; 2] = ["motor:left", "motor:right"].map(|id| {
            wheels
                .iter()
                .enumerate()
                .filter(|(_, w)| w.motor.as_deref() == Some(id))
                .map(|(i, _)| i)
                .collect()
        });
        let motor_rows: [[f64; 11]; 2] = std::array::from_fn(|side| {
            let mut r = [0.; 11];
            for i in &groups[side] {
                r[3 + i] = 1. / groups[side].len() as f64;
            }
            r
        });
        let mut jm = [0.; 2];
        let mut jb = [0.; 2];
        let mut j = vec![[0.; 2]; n];
        let mut jr = vec![0.; n];
        // Reuse the preceding solution only as an initial iterate; still solve the new system.
        for side in 0..2 {
            if !groups[side].is_empty() && drives[side].damping > 0. {
                jm[side] = (self.motor_torque[side] * dt).clamp(
                    -drives[side].torque_limit * dt,
                    drives[side].torque_limit * dt,
                );
                apply(&mut v, &motor_rows[side], &inv, jm[side]);
            }
        }
        for i in 0..n {
            j[i] = [
                -self.wheels[i].force_long_n * dt,
                -self.wheels[i].force_lat_n * dt,
            ];
            for k in 0..2 {
                if bounds[i][k] == 0. {
                    j[i][k] = 0.;
                }
            }
            let norm = (0..2)
                .map(|k| {
                    if bounds[i][k] > 0. {
                        (j[i][k] / bounds[i][k]).powi(2)
                    } else {
                        0.
                    }
                })
                .sum::<f64>()
                .sqrt()
                .max(1.);
            j[i] = j[i].map(|j| j / norm);
            apply(&mut v, &rows[i], &inv, j[i][0]);
            apply(&mut v, &latrows[i], &inv, j[i][1]);
            let bound = if f.rolling {
                wheels[i].tire.rolling_resistance * normals[i] * wheels[i].radius_m * dt
            } else {
                0.
            };
            jr[i] = (self.wheels[i].rolling_torque_nm * dt).clamp(-bound, bound);
            v[3 + i] += inv[3 + i] * jr[i];
        }
        let matrices: Vec<_> = (0..n)
            .map(|i| {
                [
                    metric(&rows[i], &rows[i], &inv) + compliance[i][0],
                    metric(&rows[i], &latrows[i], &inv),
                    metric(&latrows[i], &latrows[i], &inv) + compliance[i][1],
                ]
            })
            .collect();
        let motor_diagonal: [f64; 2] = std::array::from_fn(|i| {
            metric(&motor_rows[i], &motor_rows[i], &inv)
                + if drives[i].damping > 0. {
                    1. / (dt * drives[i].damping)
                } else {
                    0.
                }
        });
        let mut iterations = 0;
        for iteration in 0..512 {
            let mut change = 0f64;
            for side in 0..2 {
                let d = drives[side];
                if groups[side].is_empty() || d.damping == 0. {
                    continue;
                }
                let cfm = 1. / (dt * d.damping);
                let row = &motor_rows[side];
                let diagonal = motor_diagonal[side];
                let next = (jm[side] - (dot(row, &v) - d.target_omega + cfm * jm[side]) / diagonal)
                    .clamp(d.lower() * dt, d.upper() * dt);
                let delta = next - jm[side];
                change = change.max(delta.abs());
                apply(&mut v, row, &inv, delta);
                jm[side] = next;
                if d.viscous_damping > 0. {
                    let cfm = 1. / (dt * d.viscous_damping);
                    let diagonal = metric(row, row, &inv) + cfm;
                    let delta = -(dot(row, &v) + cfm * jb[side]) / diagonal;
                    jb[side] += delta;
                    change = change.max(delta.abs());
                    apply(&mut v, row, &inv, delta);
                }
            }
            for i in 0..n {
                let a = &rows[i];
                let b = &latrows[i];
                let cfm = compliance[i];
                let h = matrices[i];
                let g = [
                    dot(a, &v) - h[0] * j[i][0] - h[1] * j[i][1] + cfm[0] * j[i][0] + memory[i][0],
                    dot(b, &v) - h[1] * j[i][0] - h[2] * j[i][1] + cfm[1] * j[i][1] + memory[i][1],
                ];
                let next = block_min(h, g, bounds[i]);
                for k in 0..2 {
                    let delta = next[k] - j[i][k];
                    change = change.max(delta.abs());
                    apply(&mut v, if k == 0 { a } else { b }, &inv, delta);
                }
                j[i] = next;
                let bound = if f.rolling {
                    wheels[i].tire.rolling_resistance * normals[i] * wheels[i].radius_m * dt
                } else {
                    0.
                };
                let next = (jr[i] - v[3 + i] / inv[3 + i]).clamp(-bound, bound);
                let delta = next - jr[i];
                v[3 + i] += inv[3 + i] * delta;
                jr[i] = next;
                change = change.max(delta.abs());
            }
            iterations = iteration + 1;
            if change < 1e-12 {
                break;
            }
        }
        if iterations == 512 {
            return Err(
                "per-wheel solver did not converge; reduce physics step or contact stiffness"
                    .into(),
            );
        }
        if v.iter().any(|v| !v.is_finite()) {
            return Err("non-finite per-wheel velocity".into());
        }
        self.iterations = iterations;
        self.motor_torque = jm.map(|j| j / dt);
        let mut forces = [0.; 2];
        let mut lateral = 0.;
        let mut storage_after = 0.;
        for i in 0..n {
            let w = &wheels[i];
            let cs = &mut self.wheels[i];
            let angle = if w.kind == WheelKind::Caster {
                cs.caster_angle
            } else {
                w.angle_deg.to_radians()
            };
            let (s, c) = angle.sin_cos();
            cs.omega = v[3 + i];
            cs.angle += dt * (before[3 + i] + v[3 + i]) * 0.5;
            cs.force_long_n = -j[i][0] / dt;
            cs.force_lat_n = -j[i][1] / dt;
            cs.normal_n = normals[i];
            cs.rolling_torque_nm = jr[i] / dt;
            let slip = dot(&rows[i], &v);
            let side_slip = dot(&latrows[i], &v);
            let surface = cs.omega * (w.radius_m - cs.radial_compression_m);
            let ground = surface - slip;
            cs.slip = slip
                / surface
                    .abs()
                    .max(ground.abs())
                    .max(w.tire.slip_velocity_epsilon_m_s.max(1e-6));
            cs.slip_angle_rad =
                (-side_slip).atan2(ground.abs().max(w.tire.slip_velocity_epsilon_m_s.max(1e-6)));
            let utilization = (0..2)
                .map(|k| {
                    if bounds[i][k] > 0. {
                        (j[i][k] / bounds[i][k]).powi(2)
                    } else {
                        0.
                    }
                })
                .sum::<f64>();
            cs.regime = if normals[i] == 0. {
                "airborne"
            } else if utilization >= 1. - 1e-6 {
                "sliding"
            } else {
                "adhering"
            };
            let fx = c * cs.force_long_n - s * cs.force_lat_n;
            let fy = s * cs.force_long_n + c * cs.force_lat_n;
            forces[if w.position_m.y >= 0. { 0 } else { 1 }] += fx;
            lateral += fy;
            if self.kind == ContactKind::Brush {
                storage_after += f.relaxation_s
                    * 0.5
                    * (cs.force_long_n.powi(2) / f.longitudinal_stiffness
                        + cs.force_lat_n.powi(2) / f.lateral_stiffness);
            }
        }
        let work = (0..2).map(|i| jm[i] * dot(&motor_rows[i], &v)).sum::<f64>();
        let energy = kinetic(&v, &inv);
        let residual = energy + storage_after - kinetic(&before, &inv) - storage_before - work;
        if residual > 1e-7 * (1. + energy + work.abs()) {
            return Err(format!("per-wheel energy creation: {residual} J"));
        }
        self.acceleration = Vec2::new((v[0] - before[0]) / dt, (v[1] - before[1]) / dt);
        body.vx_body_m_s = v[0];
        body.vy_body_m_s = v[1];
        body.yaw_rate_rad_s = v[2];
        for (side, g) in groups.iter().enumerate() {
            let omega = if g.is_empty() {
                0.
            } else {
                g.iter().map(|i| v[3 + i]).sum::<f64>() / g.len() as f64
            };
            if side == 0 {
                body.wheel_omega_left_rad_s = omega;
            } else {
                body.wheel_omega_right_rad_s = omega;
            }
        }
        Ok(StepResult {
            torques: jm.map(|v| v / dt),
            forces,
            lateral,
            energy,
            work,
            dissipation: (-residual).max(0.),
        })
    }
}

impl ContactState {
    /// Ideal actuator is a prescribed shaft speed, not an electrical/energy model.
    pub(crate) fn ideal_step(
        &mut self,
        body: &mut RobotState,
        robot: &RobotConfig,
        normals: &[f64],
        drives: [MotorDrive; 2],
        dt: f64,
    ) -> Result<StepResult, String> {
        let wheels = &robot.assembly.as_ref().unwrap().wheels;
        let left = wheels
            .iter()
            .find(|w| w.motor.as_deref() == Some("motor:left"))
            .ok_or("ideal left motor missing")?;
        let right = wheels
            .iter()
            .find(|w| w.motor.as_deref() == Some("motor:right"))
            .ok_or("ideal right motor missing")?;
        let speeds = [
            drives[0].target_omega * left.radius_m,
            drives[1].target_omega * right.radius_m,
        ];
        let yaw = (speeds[1] - speeds[0]) / (left.position_m.y - right.position_m.y);
        let vx = speeds[0] + yaw * left.position_m.y;
        body.vx_body_m_s = vx - yaw * robot.chassis.center_of_mass_m.y;
        body.vy_body_m_s = yaw * robot.chassis.center_of_mass_m.x;
        body.yaw_rate_rad_s = yaw;
        body.wheel_omega_left_rad_s = drives[0].target_omega;
        body.wheel_omega_right_rad_s = drives[1].target_omega;
        for (i, w) in wheels.iter().enumerate() {
            let velocity = Vec2::new(vx - yaw * w.position_m.y, yaw * w.position_m.x);
            let c = &mut self.wheels[i];
            if w.kind == WheelKind::Caster && velocity.norm() > 1e-9 {
                c.caster_angle = velocity.y.atan2(velocity.x);
            }
            let a = if w.kind == WheelKind::Caster {
                c.caster_angle
            } else {
                w.angle_deg.to_radians()
            };
            let omega = (a.cos() * velocity.x + a.sin() * velocity.y) / w.radius_m;
            c.angle += dt * (c.omega + omega) / 2.;
            c.omega = omega;
            c.normal_n = normals[i];
            c.force_long_n = 0.;
            c.force_lat_n = 0.;
            c.slip = 0.;
            c.slip_angle_rad = 0.;
            c.regime = "ideal";
        }
        self.acceleration = Vec2::default();
        self.iterations = 0;
        Ok(StepResult {
            torques: [0.; 2],
            forces: [0.; 2],
            lateral: 0.,
            energy: 0.,
            work: 0.,
            dissipation: 0.,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn robot() -> RobotConfig {
        let mut r =
            crate::config::load_project(std::path::Path::new("examples/physics/simplified.rtsim"))
                .unwrap()
                .robot;
        super::super::robot::resolve_robot(&mut r).unwrap();
        r
    }
    struct Surface(f64);
    impl TrackModel for Surface {
        fn reflectance_at(&self, _: Vec2) -> f64 {
            0.
        }
        fn surface_mu_at(&self, _: Vec2) -> f64 {
            self.0
        }
        fn distance_to_line_m(&self, _: Vec2) -> f64 {
            0.
        }
        fn base_reflectance(&self) -> f64 {
            0.
        }
        fn line_reflectance(&self) -> f64 {
            1.
        }
    }
    #[test]
    fn force_moment_balance_and_combined_budget_in_turn() {
        let r = robot();
        let mut c = ContactState::new(&r);
        let f = r.physics.clone().unwrap();
        let mut b = RobotState {
            vx_body_m_s: 0.8,
            vy_body_m_s: 0.2,
            yaw_rate_rad_s: 1.,
            ..Default::default()
        };
        for w in &mut c.wheels {
            w.omega = 100.;
        }
        let before = b;
        let dt = 0.00005;
        let n = vec![0.5; 4];
        let result = c
            .step(
                &mut b,
                &r,
                &f,
                &Surface(0.6),
                &n,
                [MotorDrive::default(); 2],
                dt,
            )
            .unwrap();
        let mut force = Vec2::default();
        let mut moment = 0.;
        for (w, s) in r.assembly.as_ref().unwrap().wheels.iter().zip(&c.wheels) {
            let limit = 0.3;
            assert!(s.force_long_n.hypot(s.force_lat_n) <= limit + 1e-8);
            let (sin, cos) = w.angle_deg.to_radians().sin_cos();
            let f = Vec2::new(
                cos * s.force_long_n - sin * s.force_lat_n,
                sin * s.force_long_n + cos * s.force_lat_n,
            );
            force = force + f;
            let p = w.position_m - r.chassis.center_of_mass_m;
            moment += p.x * f.y - p.y * f.x;
        }
        assert!(
            (r.chassis.mass_kg * (b.vx_body_m_s - before.vx_body_m_s) / dt - force.x).abs() < 1e-8
        );
        assert!(
            (r.chassis.mass_kg * (b.vy_body_m_s - before.vy_body_m_s) / dt - force.y).abs() < 1e-8
        );
        assert!(
            (r.chassis.inertia_kg_m2 * (b.yaw_rate_rad_s - before.yaw_rate_rad_s) / dt - moment)
                .abs()
                < 1e-8
        );
        assert!(result.dissipation >= 0.);
    }
    #[test]
    fn load_sensitivity_changes_saturated_force_by_declared_power_law() {
        let r = robot();
        let mut f = FidelityConfig::preset("realistic").unwrap();
        f.rolling = false;
        let mut a = ContactState::new(&r);
        let mut b = a.clone();
        let mut body = RobotState {
            vx_body_m_s: 10.,
            ..Default::default()
        };
        let mut body2 = body;
        a.step(
            &mut body,
            &r,
            &f,
            &Surface(0.2),
            &[2.; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        f.load_exponent = 0.5;
        b.step(
            &mut body2,
            &r,
            &f,
            &Surface(0.2),
            &[2.; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        for (a, b) in a.wheels.iter().zip(&b.wheels) {
            assert!((a.force_long_n.abs() - 0.4).abs() < 1e-8);
            assert!((b.force_long_n.abs() - 0.4 / 2f64.sqrt()).abs() < 1e-8);
        }
    }
    #[test]
    fn zero_normal_has_no_contact_force_and_free_spin() {
        let r = robot();
        let mut c = ContactState::new(&r);
        for w in &mut c.wheels {
            w.omega = 10.;
        }
        let mut b = RobotState::default();
        c.step(
            &mut b,
            &r,
            r.physics.as_ref().unwrap(),
            &Surface(1.),
            &[0.; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        for w in c.wheels {
            assert_eq!(w.omega, 10.);
            assert_eq!(w.force_long_n, 0.);
            assert_eq!(w.force_lat_n, 0.);
            assert_eq!(w.regime, "airborne");
        }
    }
    #[test]
    fn rolling_stops_without_reversal_and_brush_relaxation_is_passive() {
        let r = robot();
        let mut c = ContactState::new(&r);
        let mut b = RobotState {
            vx_body_m_s: 0.02,
            ..Default::default()
        };
        for (w, s) in r
            .assembly
            .as_ref()
            .unwrap()
            .wheels
            .iter()
            .zip(&mut c.wheels)
        {
            s.omega = b.vx_body_m_s / w.radius_m;
        }
        let mut f = FidelityConfig::preset("realistic").unwrap();
        f.relaxation_s = 0.001;
        for _ in 0..2000 {
            let result = c
                .step(
                    &mut b,
                    &r,
                    &f,
                    &Surface(1.),
                    &[0.5; 4],
                    [MotorDrive::default(); 2],
                    0.00005,
                )
                .unwrap();
            assert!(result.dissipation >= 0.);
            assert!(b.vx_body_m_s > -1e-7);
        }
        assert!(b.vx_body_m_s.abs() < 0.02);
    }
    #[test]
    fn passive_and_caster_supports_roll_without_motor_and_asymmetric_geometry_runs() {
        let mut r = robot();
        let wheels = &mut r.assembly.as_mut().unwrap().wheels;
        wheels[1].kind = WheelKind::Passive;
        wheels[1].motor = None;
        wheels[3].kind = WheelKind::Caster;
        wheels[3].motor = None;
        wheels[3].radius_m *= 1.2;
        wheels[3].position_m.x *= 0.8;
        let mut c = ContactState::new(&r);
        let mut b = RobotState {
            vx_body_m_s: 0.3,
            vy_body_m_s: 0.1,
            ..Default::default()
        };
        c.step(
            &mut b,
            &r,
            r.physics.as_ref().unwrap(),
            &Surface(1.),
            &[0.5; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        assert!(c.wheels[1].omega > 0.);
        assert!(c.wheels[3].omega > 0.);
        assert!(c.wheels[3].caster_angle > 0.);
    }
    #[test]
    fn reverse_motor_changes_sign_and_contact_is_not_requested_torque_only() {
        let r = robot();
        let mut c = ContactState::new(&r);
        let mut b = RobotState::default();
        let f = r.physics.as_ref().unwrap();
        for _ in 0..100 {
            c.step(
                &mut b,
                &r,
                f,
                &Surface(1.),
                &[0.5; 4],
                [MotorDrive::constant_torque(0.002); 2],
                0.00005,
            )
            .unwrap();
        }
        assert!(b.vx_body_m_s > 0.);
        for _ in 0..480 {
            c.step(
                &mut b,
                &r,
                f,
                &Surface(1.),
                &[0.5; 4],
                [MotorDrive::constant_torque(-0.002); 2],
                0.00005,
            )
            .unwrap();
        }
        assert!(b.vx_body_m_s < 0.);
    }
    #[test]
    fn removing_refinements_recovers_identical_brush_and_radial_compression_is_bounded() {
        let r = robot();
        let mut f = FidelityConfig::preset("realistic").unwrap();
        let mut c = ContactState::new(&r);
        let mut b = RobotState {
            vx_body_m_s: 0.1,
            ..Default::default()
        };
        let mut c2 = c.clone();
        let mut b2 = b;
        let f2 = f.clone();
        f.load_exponent = 0.;
        f.relaxation_s = 0.;
        f.radial_stiffness_n_m = 0.;
        c.step(
            &mut b,
            &r,
            &f,
            &Surface(1.),
            &[0.5; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        c2.step(
            &mut b2,
            &r,
            &f2,
            &Surface(1.),
            &[0.5; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        assert_eq!(b.vx_body_m_s, b2.vx_body_m_s);
        assert_eq!(c.wheels[0].force_long_n, c2.wheels[0].force_long_n);
        f.radial_stiffness_n_m = 100000.;
        c.step(
            &mut b,
            &r,
            &f,
            &Surface(1.),
            &[0.5; 4],
            [MotorDrive::default(); 2],
            0.00005,
        )
        .unwrap();
        assert!((c.wheels[0].radial_compression_m - 0.000005).abs() < 1e-12);
        f.radial_stiffness_n_m = 1.;
        assert!(c
            .step(
                &mut b,
                &r,
                &f,
                &Surface(1.),
                &[0.5; 4],
                [MotorDrive::default(); 2],
                0.00005
            )
            .is_err());
    }
}

#[cfg(test)]
mod stage9_kernel_tests {
    use super::*;
    fn reference(h: [f64; 3], g: [f64; 2], bounds: [f64; 2]) -> [f64; 2] {
        let solve = |lambda: f64| {
            let a = h[0] + lambda / bounds[0].powi(2);
            let d = h[2] + lambda / bounds[1].powi(2);
            let det = a * d - h[1] * h[1];
            [
                (-d * g[0] + h[1] * g[1]) / det,
                (h[1] * g[0] - a * g[1]) / det,
            ]
        };
        let norm = |x: [f64; 2]| (x[0] / bounds[0]).powi(2) + (x[1] / bounds[1]).powi(2);
        let free = solve(0.);
        if norm(free) <= 1. {
            return free;
        }
        // In unit-disk coordinates, ||(H + lambda I)^-1 g|| <= ||g||/lambda.
        let mut hi = (g[0] * bounds[0]).hypot(g[1] * bounds[1]);
        let mut lo = 0.;
        for _ in 0..48 {
            let mid = (lo + hi) / 2.;
            if norm(solve(mid)) > 1. {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        solve(hi)
    }
    #[test]
    fn optimized_kernel_matches_bisection_reference() {
        for i in 1..300 {
            let f = i as f64;
            let h = [0.01 + f, 0.1, 0.2 + f * 0.3];
            let g = [(f * 0.8).sin() * 100., (f * 0.3).cos() * 50.];
            let bounds = [0.01 + (i % 11) as f64 * 0.1, 0.02 + (i % 7) as f64 * 0.2];
            let a = block_min(h, g, bounds);
            let b = reference(h, g, bounds);
            assert!((a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9);
        }
    }
}
