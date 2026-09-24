use super::integrator::*;
use crate::math::{Pose2, Vec2};
use crate::motor::MotorDrive;
use crate::sim::RobotState;

fn mechanics() -> Mechanics {
    Mechanics {
        mass: 1.0,
        inertia: 0.1,
        wheel_inertia: 0.01,
        radius: 0.1,
        half_track: 0.2,
        com: Vec2::default(),
    }
}
fn near(a: f64, b: f64, tolerance: f64) {
    assert!((a - b).abs() < tolerance, "{a} != {b} (tol {tolerance})");
}

#[test]
fn free_world_motion_with_rotating_body_and_offset_com() {
    let mut s = RobotState {
        pose: Pose2::new(0.0, 0.0, 0.0),
        vx_body_m_s: 2.0,
        vy_body_m_s: 1.0,
        yaw_rate_rad_s: 3.0,
        ..RobotState::default()
    };
    let com = Vec2::new(0.04, -0.03);
    for _ in 0..1000 {
        let old = s;
        integrate_pose(&mut s, old, com, 0.001);
    }
    let world = s.pose.transform_point(com);
    near(world.x, 2.04, 1e-10);
    near(world.y, 0.97, 1e-10);
    near(s.vx_body_m_s, 2.0 * 3.0_f64.cos() + 3.0_f64.sin(), 1e-10);
    near(s.vy_body_m_s, -2.0 * 3.0_f64.sin() + 3.0_f64.cos(), 1e-10);
}

#[test]
fn constant_force_and_known_yaw_torque_have_analytic_motion() {
    let mut s = RobotState::default();
    for _ in 0..1000 {
        let old = s;
        // World force 2 N, mass 1 kg, yaw torque .3 Nm, inertia .1.
        let yaw = s.pose.yaw;
        s.vx_body_m_s += 2.0 * yaw.cos() * 0.001;
        s.vy_body_m_s -= 2.0 * yaw.sin() * 0.001;
        s.yaw_rate_rad_s += 3.0 * 0.001;
        integrate_pose(&mut s, old, Vec2::default(), 0.001);
    }
    near(s.pose.x, 1.0, 1e-10);
    near(s.pose.y, 0.0, 1e-10);
    near(s.pose.yaw, 1.5, 1e-10);
    near(s.yaw_rate_rad_s, 3.0, 1e-10);
}

#[test]
fn sticking_acceleration_includes_both_wheel_inertias() {
    let m = mechanics();
    let mut s = RobotState::default();
    let dt = 0.001;
    let tau = 0.01;
    let out = solve_contacts(
        &mut s,
        m,
        [100.0; 2],
        100.0,
        [0.0; 2],
        [MotorDrive::constant_torque(tau); 2],
        dt,
    )
    .unwrap();
    let a = (2.0 * tau / m.radius) / (m.mass + 2.0 * m.wheel_inertia / m.radius.powi(2));
    near(s.vx_body_m_s, a * dt, 1e-10);
    near(s.wheel_omega_left_rad_s, s.vx_body_m_s / m.radius, 1e-9);
    near(out.forces[0] + out.forces[1], m.mass * a, 1e-7);
    near(s.yaw_rate_rad_s, 0.0, 1e-9);
}

#[test]
fn differential_torque_includes_reflected_yaw_inertia() {
    let m = mechanics();
    let mut s = RobotState::default();
    let dt = 0.001;
    let tau = 0.01;
    solve_contacts(
        &mut s,
        m,
        [100.0; 2],
        100.0,
        [0.0; 2],
        [
            MotorDrive::constant_torque(-tau),
            MotorDrive::constant_torque(tau),
        ],
        dt,
    )
    .unwrap();
    let alpha = (2.0 * tau * m.half_track / m.radius)
        / (m.inertia + 2.0 * m.wheel_inertia * (m.half_track / m.radius).powi(2));
    near(s.yaw_rate_rad_s, alpha * dt, 1e-10);
    near(s.vx_body_m_s, 0.0, 1e-9);
}

#[test]
fn slip_relaxes_without_snapping_or_creating_energy() {
    let m = mechanics();
    let mut s = RobotState {
        wheel_omega_left_rad_s: 100.0,
        wheel_omega_right_rad_s: 100.0,
        ..RobotState::default()
    };
    for _ in 0..1000 {
        let before = energy(&s, m);
        let old = s;
        let out = solve_contacts(
            &mut s,
            m,
            [1.0; 2],
            1.0,
            [0.0; 2],
            [MotorDrive::default(); 2],
            0.001,
        )
        .unwrap();
        assert!(energy(&s, m) <= before + 1e-10);
        assert!(out.forces.iter().all(|f| f.abs() <= 1.0 + 1e-10));
        assert!((s.wheel_omega_left_rad_s - old.wheel_omega_left_rad_s).abs() <= 0.011);
    }
}

#[test]
fn rolling_loss_stops_without_reverse_or_energy_increase() {
    let m = mechanics();
    let mut s = RobotState {
        vx_body_m_s: 0.001,
        wheel_omega_left_rad_s: 0.01,
        wheel_omega_right_rad_s: 0.01,
        ..RobotState::default()
    };
    for _ in 0..100 {
        let before = energy(&s, m);
        solve_contacts(
            &mut s,
            m,
            [100.0; 2],
            100.0,
            [0.01; 2],
            [MotorDrive::default(); 2],
            0.001,
        )
        .unwrap();
        assert!(energy(&s, m) <= before + 1e-12);
        assert!(s.vx_body_m_s >= -1e-9);
    }
    assert!(s.vx_body_m_s.abs() < 1e-9);
}

#[test]
fn turn_requires_centripetal_force_and_converges_with_smaller_ticks() {
    fn run(dt: f64) -> (RobotState, f64) {
        let m = mechanics();
        let mut s = RobotState {
            vx_body_m_s: 1.0,
            yaw_rate_rad_s: 2.0,
            ..RobotState::default()
        };
        let mut lateral = 0.0;
        for _ in 0..(0.1 / dt).round() as usize {
            let old = s;
            let out = solve_contacts(
                &mut s,
                m,
                [0.0; 2],
                100.0,
                [0.0; 2],
                [MotorDrive::default(); 2],
                dt,
            )
            .unwrap();
            lateral = out.lateral_force;
            integrate_pose(&mut s, old, m.com, dt);
        }
        (s, lateral)
    }
    let (coarse, _) = run(0.001);
    let (fine, force) = run(0.00005);
    let expected_x = 0.2_f64.sin() / 2.0;
    let expected_y = (1.0 - 0.2_f64.cos()) / 2.0;
    assert!((fine.pose.y - expected_y).abs() < (coarse.pose.y - expected_y).abs());
    near(fine.pose.x, expected_x, 2e-6);
    near(fine.pose.y, expected_y, 1.1e-5);
    near(force, 2.0, 1e-4);
}
