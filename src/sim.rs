use crate::config::{LoadedConfig, TimeConfig};
use crate::controller::{BuiltInPid, Controller, ControllerOutput};
use crate::math::{clamp, wrap_angle, Pose2};
use crate::motor::{DcMotorSimple, MotorModel, MotorOutput};
use crate::sensor::{SensorModel, SensorOutput, SimpleLineSensor};
use crate::telemetry::{CsvLogger, TelemetrySample};
use crate::track::{TrackModel, VectorTrack};
use crate::wheel::{CoulombFrictionWheel, TireModel, WheelForces};
use std::path::PathBuf;
use std::time::{Duration, Instant};

const G: f64 = 9.80665;

#[derive(Debug, Clone)]
pub struct RunOptions {
    pub duration_us: Option<u64>,
    pub output_csv: Option<PathBuf>,
    pub headless: bool,
    pub benchmark: bool,
    pub physics_dt_override_us: Option<u64>,
}

#[derive(Debug, Clone)]
pub struct RunSummary {
    pub project_name: String,
    pub robot_name: String,
    pub track_name: String,
    pub duration_us: u64,
    pub steps: u64,
    pub final_pose: Pose2,
    pub simulated_time_s: f64,
    pub wall_time: Duration,
    pub steps_per_second: f64,
    pub realtime_factor: f64,
    pub csv_path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RobotState {
    pub pose: Pose2,
    pub vx_body_m_s: f64,
    pub vy_body_m_s: f64,
    pub yaw_rate_rad_s: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct LastPhysics {
    motor_left: MotorOutput,
    motor_right: MotorOutput,
    wheel_left: WheelForces,
    wheel_right: WheelForces,
    normal_left_n: f64,
    normal_right_n: f64,
    battery_voltage_v: f64,
}

pub fn run_simulation(cfg: LoadedConfig, options: RunOptions) -> Result<RunSummary, String> {
    let mut time = cfg.project.time;
    if let Some(dt) = options.physics_dt_override_us {
        if dt == 0 {
            return Err("--physics-dt-us must be > 0".to_string());
        }
        time.physics_dt_us = dt;
    }

    if !options.headless && !options.benchmark {
        eprintln!("warning: UI is not implemented in v0.1; running headless core");
    }

    validate_time(&time)?;

    let duration_us = options
        .duration_us
        .unwrap_or_else(|| (cfg.project.duration_s * 1_000_000.0).round() as u64);
    let steps = duration_us / time.physics_dt_us;

    let mut state = RobotState { pose: cfg.project.start_pose, ..RobotState::default() };
    let track = VectorTrack::new(cfg.track.clone());
    let mut sensor = SimpleLineSensor::new(cfg.robot.line_sensor.clone());
    let mut controller = BuiltInPid::new(cfg.robot.controller);
    let motor_left = DcMotorSimple::new(cfg.robot.motor_left.clone());
    let motor_right = DcMotorSimple::new(cfg.robot.motor_right.clone());
    let tire = CoulombFrictionWheel;

    let mut sensor_output = sensor.sample(&track, state.pose, 0);
    let mut ctrl_output = ControllerOutput::default();
    let mut last_physics = LastPhysics { battery_voltage_v: cfg.robot.battery.nominal_voltage_v, ..LastPhysics::default() };

    let csv_path = if options.benchmark {
        None
    } else {
        choose_csv_path(&cfg, options.output_csv.clone())
    };
    let mut logger = match csv_path.as_ref() {
        Some(path) => Some(CsvLogger::create(path, sensor.count()).map_err(|e| format!("failed to create CSV log {}: {e}", path.display()))?),
        None => None,
    };

    let mut next_sensor_us = 0u64;
    let mut next_controller_us = 0u64;
    let mut next_log_us = 0u64;
    let start_wall = Instant::now();

    for step_idx in 0..=steps {
        let t_us = step_idx * time.physics_dt_us;

        while t_us >= next_sensor_us {
            sensor_output = sensor.sample(&track, state.pose, t_us);
            next_sensor_us = next_sensor_us.saturating_add(time.sensor_period_us);
        }

        while t_us >= next_controller_us {
            ctrl_output = controller.step(&sensor_output, time.controller_period_us as f64 / 1_000_000.0);
            next_controller_us = next_controller_us.saturating_add(time.controller_period_us);
        }

        while t_us >= next_log_us {
            if let Some(logger) = logger.as_mut() {
                let sample = make_telemetry_sample(t_us, &state, &sensor_output, &ctrl_output, &last_physics);
                logger.write_sample(&sample).map_err(|e| format!("failed to write CSV log: {e}"))?;
            }
            next_log_us = next_log_us.saturating_add(time.log_period_us);
        }

        if step_idx == steps {
            break;
        }

        last_physics = physics_step(
            &mut state,
            &cfg,
            &track,
            &tire,
            &motor_left,
            &motor_right,
            ctrl_output,
            time.physics_dt_us,
        );
    }

    if let Some(logger) = logger.as_mut() {
        logger.flush().map_err(|e| format!("failed to flush CSV log: {e}"))?;
    }

    let wall_time = start_wall.elapsed();
    let simulated_time_s = duration_us as f64 / 1_000_000.0;
    let wall_s = wall_time.as_secs_f64().max(1e-12);
    let executed_steps = steps + 1;

    Ok(RunSummary {
        project_name: cfg.project.name,
        robot_name: cfg.robot.name,
        track_name: cfg.track.name,
        duration_us,
        steps: executed_steps,
        final_pose: state.pose,
        simulated_time_s,
        wall_time,
        steps_per_second: executed_steps as f64 / wall_s,
        realtime_factor: simulated_time_s / wall_s,
        csv_path,
    })
}

fn validate_time(time: &TimeConfig) -> Result<(), String> {
    if time.physics_dt_us == 0 || time.controller_period_us == 0 || time.sensor_period_us == 0 || time.log_period_us == 0 {
        return Err("all scheduler periods must be positive".to_string());
    }
    if time.controller_period_us < time.physics_dt_us || time.sensor_period_us < time.physics_dt_us || time.log_period_us < time.physics_dt_us {
        return Err("controller/sensor/log periods must be >= physics_dt_us in v0.1".to_string());
    }
    Ok(())
}

fn choose_csv_path(cfg: &LoadedConfig, output_override: Option<PathBuf>) -> Option<PathBuf> {
    if let Some(path) = output_override {
        return Some(path);
    }
    let base_dir = cfg.project_path.parent().unwrap_or_else(|| std::path::Path::new("."));
    cfg.project.csv_output.as_ref().map(|p| if p.is_absolute() { p.clone() } else { base_dir.join(p) })
}

#[allow(clippy::too_many_arguments)]
fn physics_step(
    state: &mut RobotState,
    cfg: &LoadedConfig,
    track: &dyn TrackModel,
    tire: &dyn TireModel,
    motor_left: &dyn MotorModel,
    motor_right: &dyn MotorModel,
    cmd: ControllerOutput,
    dt_us: u64,
) -> LastPhysics {
    let dt = dt_us as f64 / 1_000_000.0;
    let mass = cfg.robot.chassis.mass_kg.max(1e-9);
    let inertia = cfg.robot.chassis.inertia_kg_m2.max(1e-12);
    let wheel_radius = cfg.robot.drivetrain.wheel_radius_m.max(1e-9);
    let half_track = cfg.robot.drivetrain.track_width_m * 0.5;
    let total_normal = mass * G; // NoDownforce v0.1.
    let normal_left = total_normal * 0.5;
    let normal_right = total_normal * 0.5;

    let battery_v = estimate_battery_voltage(cfg, cmd);
    let left_ground_speed = state.vx_body_m_s - state.yaw_rate_rad_s * half_track;
    let right_ground_speed = state.vx_body_m_s + state.yaw_rate_rad_s * half_track;
    let omega_left = left_ground_speed / wheel_radius;
    let omega_right = right_ground_speed / wheel_radius;

    let m_left = motor_left.step(cmd.pwm_left, omega_left, battery_v, &cfg.robot.driver);
    let m_right = motor_right.step(cmd.pwm_right, omega_right, battery_v, &cfg.robot.driver);

    let desired_left = m_left.wheel_torque_nm / wheel_radius;
    let desired_right = m_right.wheel_torque_nm / wheel_radius;
    let surface_mu = track.surface_mu_at(crate::math::Vec2::new(state.pose.x, state.pose.y));
    let mu_long = cfg.robot.tire.mu_longitudinal.min(surface_mu);

    let mut w_left = tire.longitudinal_force(desired_left, normal_left, mu_long);
    let mut w_right = tire.longitudinal_force(desired_right, normal_right, mu_long);

    apply_rolling_resistance(&mut w_left, left_ground_speed, normal_left, cfg.robot.tire.rolling_resistance);
    apply_rolling_resistance(&mut w_right, right_ground_speed, normal_right, cfg.robot.tire.rolling_resistance);

    let fx_body = w_left.force_n + w_right.force_n;
    let max_lateral = cfg.robot.tire.mu_lateral * total_normal;
    let desired_lateral = -state.vy_body_m_s * mass / dt.max(1e-9);
    let fy_body = clamp(desired_lateral, -max_lateral, max_lateral);

    let yaw_damping = 0.00008;
    let torque_z = (w_right.force_n - w_left.force_n) * half_track - yaw_damping * state.yaw_rate_rad_s;

    state.vx_body_m_s += (fx_body / mass) * dt;
    state.vy_body_m_s += (fy_body / mass) * dt;
    state.yaw_rate_rad_s += (torque_z / inertia) * dt;

    let c = state.pose.yaw.cos();
    let s = state.pose.yaw.sin();
    let vx_world = state.vx_body_m_s * c - state.vy_body_m_s * s;
    let vy_world = state.vx_body_m_s * s + state.vy_body_m_s * c;
    state.pose.x += vx_world * dt;
    state.pose.y += vy_world * dt;
    state.pose.yaw = wrap_angle(state.pose.yaw + state.yaw_rate_rad_s * dt);

    LastPhysics { motor_left: m_left, motor_right: m_right, wheel_left: w_left, wheel_right: w_right, normal_left_n: normal_left, normal_right_n: normal_right, battery_voltage_v: battery_v }
}

fn estimate_battery_voltage(cfg: &LoadedConfig, cmd: ControllerOutput) -> f64 {
    // v0.1 battery model: nominal voltage minus a small deterministic sag estimate from PWM demand.
    // The more detailed LipoBattery model can replace this without changing the physics loop API.
    let demand = cmd.pwm_left.abs() + cmd.pwm_right.abs();
    let sag = cfg.robot.battery.internal_resistance_ohm * demand * 0.35;
    (cfg.robot.battery.nominal_voltage_v - sag).max(0.0)
}

fn apply_rolling_resistance(w: &mut WheelForces, ground_speed: f64, normal: f64, coeff: f64) {
    if ground_speed.abs() < 1e-6 {
        return;
    }
    let rr = coeff.max(0.0) * normal;
    w.force_n -= rr * ground_speed.signum();
}

fn make_telemetry_sample(
    t_us: u64,
    state: &RobotState,
    sensor: &SensorOutput,
    ctrl: &ControllerOutput,
    phys: &LastPhysics,
) -> TelemetrySample {
    TelemetrySample {
        t_us,
        x_m: state.pose.x,
        y_m: state.pose.y,
        yaw_rad: state.pose.yaw,
        vx_body_m_s: state.vx_body_m_s,
        vy_body_m_s: state.vy_body_m_s,
        yaw_rate_rad_s: state.yaw_rate_rad_s,
        line_position_m: sensor.line_position_m,
        line_error_m: ctrl.error_m,
        line_visible: sensor.line_visible,
        line_confidence: sensor.confidence,
        pwm_left: ctrl.pwm_left,
        pwm_right: ctrl.pwm_right,
        motor_current_left_a: phys.motor_left.current_a,
        motor_current_right_a: phys.motor_right.current_a,
        motor_torque_left_nm: phys.motor_left.wheel_torque_nm,
        motor_torque_right_nm: phys.motor_right.wheel_torque_nm,
        wheel_force_left_n: phys.wheel_left.force_n,
        wheel_force_right_n: phys.wheel_right.force_n,
        desired_wheel_force_left_n: phys.wheel_left.desired_force_n,
        desired_wheel_force_right_n: phys.wheel_right.desired_force_n,
        slip_left: phys.wheel_left.slip_ratio,
        slip_right: phys.wheel_right.slip_ratio,
        normal_left_n: phys.normal_left_n,
        normal_right_n: phys.normal_right_n,
        battery_voltage_v: phys.battery_voltage_v,
        sensor_adc: sensor.adc.clone(),
    }
}
