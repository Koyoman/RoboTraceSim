//! Planar impulse integration. Velocities are at COM; geometry uses body origin.
use crate::math::{wrap_angle, Vec2};
use crate::sim::RobotState;

#[derive(Clone, Copy)]
pub(crate) struct Mechanics {
    pub mass: f64,
    pub inertia: f64,
    pub wheel_inertia: f64,
    pub radius: f64,
    pub half_track: f64,
    pub com: Vec2,
}

pub(crate) struct ContactResult {
    pub forces: [f64; 2],
    pub motor_torques: [f64; 2],
    pub lateral_force: f64,
    pub rolling_torques: [f64; 2],
    pub iterations: usize,
}

fn rotate(v: Vec2, angle: f64) -> Vec2 {
    let (s, c) = angle.sin_cos();
    Vec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

pub(crate) fn energy(s: &RobotState, m: Mechanics) -> f64 {
    0.5 * m.mass * (s.vx_body_m_s.powi(2) + s.vy_body_m_s.powi(2))
        + 0.5 * m.inertia * s.yaw_rate_rad_s.powi(2)
        + 0.5
            * m.wheel_inertia
            * (s.wheel_omega_left_rad_s.powi(2) + s.wheel_omega_right_rad_s.powi(2))
}

/// Integrate world momentum, including exact transport of stored body velocities.
/// Uses trapezoidal pose quadrature. Force/impulse directions are frozen for a tick.
pub(crate) fn integrate_pose(s: &mut RobotState, before: RobotState, com: Vec2, dt: f64) {
    let old_world = rotate(
        Vec2::new(before.vx_body_m_s, before.vy_body_m_s),
        before.pose.yaw,
    );
    let new_world = rotate(Vec2::new(s.vx_body_m_s, s.vy_body_m_s), before.pose.yaw);
    let old_com = before.pose.transform_point(com);
    let new_com = old_com + (old_world + new_world) * (0.5 * dt);
    s.pose.yaw =
        wrap_angle(before.pose.yaw + 0.5 * (before.yaw_rate_rad_s + s.yaw_rate_rad_s) * dt);
    let offset = rotate(com, s.pose.yaw);
    s.pose.x = new_com.x - offset.x;
    s.pose.y = new_com.y - offset.y;
    let body = rotate(new_world, -s.pose.yaw);
    s.vx_body_m_s = body.x;
    s.vy_body_m_s = body.y;
    s.wheel_angle_left_rad += 0.5 * (before.wheel_omega_left_rad_s + s.wheel_omega_left_rad_s) * dt;
    s.wheel_angle_right_rad +=
        0.5 * (before.wheel_omega_right_rad_s + s.wheel_omega_right_rad_s) * dt;
}

/// Maximum-dissipation box-friction constraints for two aggregate drive sides,
/// one lateral contact at the axle center and rolling loss on each wheel shaft.
/// Wheel and chassis reaction impulses are equal/opposite; no velocity snapping.
pub(crate) fn solve_contacts(
    s: &mut RobotState,
    m: Mechanics,
    longitudinal_limits: [f64; 2],
    lateral_limit: f64,
    rolling_limits: [f64; 2],
    drives: [crate::motor::MotorDrive; 2],
    dt: f64,
) -> Result<ContactResult, String> {
    let mut v = [
        s.vx_body_m_s,
        s.vy_body_m_s,
        s.yaw_rate_rad_s,
        s.wheel_omega_left_rad_s,
        s.wheel_omega_right_rad_s,
    ];
    let inverse_mass = [
        1.0 / m.mass,
        1.0 / m.mass,
        1.0 / m.inertia,
        1.0 / m.wheel_inertia,
        1.0 / m.wheel_inertia,
    ];
    let rows = [
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
        [-1.0, 0.0, m.half_track - m.com.y, m.radius, 0.0],
        [-1.0, 0.0, -m.half_track - m.com.y, 0.0, m.radius],
        [0.0, 1.0, -m.com.x, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
    ];
    let bounds = [
        drives[0].torque_limit * dt,
        drives[1].torque_limit * dt,
        longitudinal_limits[0] * dt,
        longitudinal_limits[1] * dt,
        lateral_limit * dt,
        rolling_limits[0] * dt,
        rolling_limits[1] * dt,
    ];
    // Small bounded quadratic solve. Active-set elimination avoids the slow
    // convergence of sequential impulses when wheel inertia is tiny.
    let mut matrix = [[0.0; 7]; 7];
    let mut linear = [0.0; 7];
    for i in 0..7 {
        linear[i] = (0..5).map(|k| rows[i][k] * v[k]).sum();
        for j in 0..7 {
            matrix[i][j] = (0..5)
                .map(|k| rows[i][k] * inverse_mass[k] * rows[j][k])
                .sum();
        }
        if i < 2 && drives[i].damping > 0.0 {
            matrix[i][i] += 1.0 / (dt * drives[i].damping);
            linear[i] -= drives[i].target_omega;
        }
    }
    let (impulse, iterations) = bounded_solve(matrix, linear, bounds)?;
    for i in 0..7 {
        for k in 0..5 {
            v[k] += inverse_mass[k] * rows[i][k] * impulse[i];
        }
    }
    if !v.iter().all(|x| x.is_finite()) {
        return Err("contact: non-finite velocity".into());
    }
    s.vx_body_m_s = v[0];
    s.vy_body_m_s = v[1];
    s.yaw_rate_rad_s = v[2];
    s.wheel_omega_left_rad_s = v[3];
    s.wheel_omega_right_rad_s = v[4];
    Ok(ContactResult {
        forces: [-impulse[2] / dt, -impulse[3] / dt],
        motor_torques: [impulse[0] / dt, impulse[1] / dt],
        lateral_force: impulse[4] / dt,
        rolling_torques: [impulse[5] / dt, impulse[6] / dt],
        iterations,
    })
}

fn bounded_solve(
    a: [[f64; 7]; 7],
    b: [f64; 7],
    bounds: [f64; 7],
) -> Result<([f64; 7], usize), String> {
    if !a
        .iter()
        .flatten()
        .chain(b.iter())
        .chain(bounds.iter())
        .all(|x| x.is_finite())
    {
        return Err("contact: non-finite system".into());
    }
    let mut x = [0.0; 7];
    // 0=free, -1=lower bound, 1=upper bound, 2=fixed zero.
    let mut active = std::array::from_fn::<_, 7, _>(|i| if bounds[i] == 0.0 { 2_i8 } else { 0 });
    for iteration in 1..=64 {
        let mut system = [[0.0; 8]; 7];
        for i in 0..7 {
            if active[i] != 0 {
                system[i][i] = 1.0;
                system[i][7] = x[i];
            } else {
                system[i][..7].copy_from_slice(&a[i]);
                system[i][7] = -b[i];
                for j in 0..7 {
                    if active[j] != 0 {
                        system[i][7] -= a[i][j] * x[j];
                        system[i][j] = 0.0;
                    }
                }
            }
        }
        let candidate = eliminate(system)?;
        let mut alpha = 1.0;
        let mut hit = None;
        for i in 0..7 {
            if active[i] == 0 {
                let d = candidate[i] - x[i];
                let (fraction, side) = if candidate[i] > bounds[i] {
                    ((bounds[i] - x[i]) / d, 1)
                } else if candidate[i] < -bounds[i] {
                    ((-bounds[i] - x[i]) / d, -1)
                } else {
                    continue;
                };
                if fraction < alpha {
                    alpha = fraction.max(0.0);
                    hit = Some((i, side));
                }
            }
        }
        for i in 0..7 {
            x[i] += alpha * (candidate[i] - x[i]);
        }
        if let Some((i, side)) = hit {
            x[i] = bounds[i] * side as f64;
            active[i] = side;
            continue;
        }
        let mut release = None;
        let mut violation = 1e-9;
        for i in 0..7 {
            if active[i] == 1 || active[i] == -1 {
                let gradient = b[i] + (0..7).map(|j| a[i][j] * x[j]).sum::<f64>();
                let wrong = gradient * active[i] as f64;
                if wrong > violation {
                    violation = wrong;
                    release = Some(i);
                }
            }
        }
        if let Some(i) = release {
            active[i] = 0;
        } else {
            return Ok((x, iteration));
        }
    }
    Err("contact: active-set solver did not converge in 64 iterations".into())
}

fn eliminate(mut a: [[f64; 8]; 7]) -> Result<[f64; 7], String> {
    for k in 0..7 {
        let pivot = (k..7)
            .max_by(|&i, &j| a[i][k].abs().total_cmp(&a[j][k].abs()))
            .unwrap();
        a.swap(k, pivot);
        if a[k][k] == 0.0 || !a[k][k].is_finite() {
            return Err("contact: singular effective mass".into());
        }
        for i in k + 1..7 {
            let factor = a[i][k] / a[k][k];
            for j in k..8 {
                a[i][j] -= factor * a[k][j];
            }
        }
    }
    let mut x = [0.0; 7];
    for i in (0..7).rev() {
        x[i] = (a[i][7] - (i + 1..7).map(|j| a[i][j] * x[j]).sum::<f64>()) / a[i][i];
    }
    if !x.iter().all(|x| x.is_finite()) {
        return Err("contact: non-finite solution".into());
    }
    Ok(x)
}
