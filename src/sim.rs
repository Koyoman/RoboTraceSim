use crate::battery::{BatteryOutput, VoltageSagBattery};
use crate::config::{LoadedConfig, TimeConfig};
use crate::controller::{BuiltInPid, Controller, ControllerOutput};
use crate::encoder::{EncoderOutput, QuantizedEncoder};
use crate::gyro::{GyroOutput, NoisyGyro};
use crate::math::{clamp, wrap_angle, Pose2};
use crate::motor::{DcMotorSimple, MotorModel, MotorOutput};
use crate::normal_force::{
    ConfiguredNormalForce, NormalForceInput, NormalForceModel, NormalForceOutput,
};
use crate::replay::BinaryReplayLogger;
use crate::rtsim_track::{validate_track, Severity, TrackRulesMode};
use crate::sensor::{SensorModel, SensorOutput, SimpleLineSensor};
use crate::telemetry::{CsvLogger, TelemetrySample};
use crate::track::{TrackModel, VectorTrack};
use crate::wheel::{SlipRatioWheel, TireInput, TireModel, WheelForces};
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct RunOptions {
    pub duration_us: Option<u64>,
    pub output_csv: Option<PathBuf>,
    pub output_replay: Option<PathBuf>,
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
    pub replay_path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RobotState {
    pub pose: Pose2,
    pub vx_body_m_s: f64,
    pub vy_body_m_s: f64,
    pub yaw_rate_rad_s: f64,
    pub wheel_omega_left_rad_s: f64,
    pub wheel_omega_right_rad_s: f64,
    pub wheel_angle_left_rad: f64,
    pub wheel_angle_right_rad: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct LastPhysics {
    motor_left: MotorOutput,
    motor_right: MotorOutput,
    wheel_left: WheelForces,
    wheel_right: WheelForces,
    normal: NormalForceOutput,
    battery: BatteryOutput,
}

/// Incremental simulation session used by the v0.08/v0.08 visual simulator.
///
/// It reuses the same deterministic fixed-step core as `run_simulation`, but exposes
/// small stepping methods so the GUI can advance and draw the robot without making
/// the interface own or duplicate the physics.
pub struct SimulationSession {
    cfg: LoadedConfig,
    time: TimeConfig,
    duration_us: u64,
    max_steps: u64,
    step_idx: u64,
    pub state: RobotState,
    track: VectorTrack,
    sensor: SimpleLineSensor,
    encoder: QuantizedEncoder,
    gyro: NoisyGyro,
    controller: BuiltInPid,
    motor_left: DcMotorSimple,
    motor_right: DcMotorSimple,
    tire: SlipRatioWheel,
    normal_force: ConfiguredNormalForce,
    battery: VoltageSagBattery,
    sensor_output: SensorOutput,
    encoder_output: EncoderOutput,
    gyro_output: GyroOutput,
    ctrl_output: ControllerOutput,
    last_physics: LastPhysics,
    next_sensor_us: u64,
    next_encoder_us: u64,
    next_imu_us: u64,
    next_controller_us: u64,
}

impl SimulationSession {
    pub fn new(cfg: LoadedConfig, duration_us: Option<u64>) -> Result<Self, String> {
        validate_track_for_simulation(&cfg)?;
        validate_time(&cfg.project.time)?;
        let time = cfg.project.time;
        let duration_us =
            duration_us.unwrap_or_else(|| (cfg.project.duration_s * 1_000_000.0).round() as u64);
        let max_steps = duration_us / time.physics_dt_us;
        let state = RobotState {
            pose: cfg.project.start_pose,
            ..RobotState::default()
        };
        let track = VectorTrack::new(cfg.track.clone());
        let mut sensor = SimpleLineSensor::new(cfg.robot.line_sensor.clone());
        let mut encoder = QuantizedEncoder::new(cfg.robot.encoder.clone());
        let mut gyro = NoisyGyro::new(cfg.robot.gyro.clone());
        let mut controller = BuiltInPid::new(cfg.robot.controller);
        let motor_left = DcMotorSimple::new(cfg.robot.motor_left.clone());
        let motor_right = DcMotorSimple::new(cfg.robot.motor_right.clone());
        let tire = SlipRatioWheel::new(cfg.robot.tire.clone());
        let mut normal_force = ConfiguredNormalForce::new(cfg.robot.normal_force.clone());
        let battery = VoltageSagBattery::new(cfg.robot.battery.clone());

        let sensor_output = sensor.sample(&track, state.pose, 0);
        let encoder_output =
            encoder.sample(state.wheel_angle_left_rad, state.wheel_angle_right_rad, 0);
        let gyro_output = gyro.sample(state.yaw_rate_rad_s, 0);
        let ctrl_output = controller.step(
            &sensor_output,
            time.controller_period_us as f64 / 1_000_000.0,
        );
        let mut last_physics = LastPhysics {
            battery: battery.output(),
            ..LastPhysics::default()
        };
        last_physics.normal = normal_force.step(NormalForceInput {
            mass_kg: cfg.robot.chassis.mass_kg.max(1e-9),
            center_of_mass_m: cfg.robot.chassis.center_of_mass_m,
            wheelbase_m: cfg.robot.drivetrain.wheelbase_m.max(1e-6),
            track_width_m: cfg.robot.drivetrain.track_width_m.max(1e-6),
            battery_voltage_v: battery.terminal_voltage_v(),
            command_pwm: ctrl_output.pwm_downforce,
            speed_m_s: state.vx_body_m_s,
            dt_us: 0,
        });

        Ok(Self {
            cfg,
            time,
            duration_us,
            max_steps,
            step_idx: 0,
            state,
            track,
            sensor,
            encoder,
            gyro,
            controller,
            motor_left,
            motor_right,
            tire,
            normal_force,
            battery,
            sensor_output,
            encoder_output,
            gyro_output,
            ctrl_output,
            last_physics,
            next_sensor_us: time.sensor_period_us,
            next_encoder_us: time.encoder_period_us,
            next_imu_us: time.imu_period_us,
            next_controller_us: time.controller_period_us,
        })
    }

    pub fn time_us(&self) -> u64 {
        self.step_idx * self.time.physics_dt_us
    }

    pub fn duration_us(&self) -> u64 {
        self.duration_us
    }

    pub fn progress(&self) -> f64 {
        if self.duration_us == 0 {
            1.0
        } else {
            (self.time_us() as f64 / self.duration_us as f64).min(1.0)
        }
    }

    pub fn is_finished(&self) -> bool {
        self.step_idx >= self.max_steps
    }

    pub fn sample(&self) -> TelemetrySample {
        make_telemetry_sample(
            self.time_us(),
            &self.state,
            &self.sensor_output,
            &self.encoder_output,
            &self.gyro_output,
            &self.ctrl_output,
            &self.last_physics,
        )
    }

    pub fn advance_steps(&mut self, steps: u64) -> TelemetrySample {
        let count = steps.max(1);
        for _ in 0..count {
            if self.is_finished() {
                break;
            }
            self.step_once();
        }
        self.sample()
    }

    pub fn step_once(&mut self) -> TelemetrySample {
        let t_us = self.time_us();
        while t_us >= self.next_sensor_us {
            self.sensor_output = self.sensor.sample(&self.track, self.state.pose, t_us);
            self.next_sensor_us = self
                .next_sensor_us
                .saturating_add(self.time.sensor_period_us);
        }
        while t_us >= self.next_encoder_us {
            self.encoder_output = self.encoder.sample(
                self.state.wheel_angle_left_rad,
                self.state.wheel_angle_right_rad,
                t_us,
            );
            self.next_encoder_us = self
                .next_encoder_us
                .saturating_add(self.time.encoder_period_us);
        }
        while t_us >= self.next_imu_us {
            self.gyro_output = self.gyro.sample(self.state.yaw_rate_rad_s, t_us);
            self.next_imu_us = self.next_imu_us.saturating_add(self.time.imu_period_us);
        }
        while t_us >= self.next_controller_us {
            self.ctrl_output = self.controller.step(
                &self.sensor_output,
                self.time.controller_period_us as f64 / 1_000_000.0,
            );
            self.next_controller_us = self
                .next_controller_us
                .saturating_add(self.time.controller_period_us);
        }

        let sample = self.sample();
        self.last_physics = physics_step(
            &mut self.state,
            &self.cfg,
            &self.track,
            &self.tire,
            &self.motor_left,
            &self.motor_right,
            &mut self.battery,
            &mut self.normal_force,
            self.ctrl_output,
            self.time.physics_dt_us,
        );
        self.step_idx = self.step_idx.saturating_add(1);
        sample
    }
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
        eprintln!("warning: visual UI is available from the app shell; run command continues to use the deterministic headless core");
    }

    validate_track_for_simulation(&cfg)?;
    validate_time(&time)?;

    let duration_us = options
        .duration_us
        .unwrap_or_else(|| (cfg.project.duration_s * 1_000_000.0).round() as u64);
    let steps = duration_us / time.physics_dt_us;

    let mut state = RobotState {
        pose: cfg.project.start_pose,
        ..RobotState::default()
    };
    let track = VectorTrack::new(cfg.track.clone());
    let mut sensor = SimpleLineSensor::new(cfg.robot.line_sensor.clone());
    let mut encoder = QuantizedEncoder::new(cfg.robot.encoder.clone());
    let mut gyro = NoisyGyro::new(cfg.robot.gyro.clone());
    let mut controller = BuiltInPid::new(cfg.robot.controller);
    let motor_left = DcMotorSimple::new(cfg.robot.motor_left.clone());
    let motor_right = DcMotorSimple::new(cfg.robot.motor_right.clone());
    let tire = SlipRatioWheel::new(cfg.robot.tire.clone());
    let mut normal_force = ConfiguredNormalForce::new(cfg.robot.normal_force.clone());
    let mut battery = VoltageSagBattery::new(cfg.robot.battery.clone());

    let mut sensor_output = sensor.sample(&track, state.pose, 0);
    let mut encoder_output =
        encoder.sample(state.wheel_angle_left_rad, state.wheel_angle_right_rad, 0);
    let mut gyro_output = gyro.sample(state.yaw_rate_rad_s, 0);
    let mut ctrl_output = ControllerOutput::default();
    let mut last_physics = LastPhysics {
        battery: battery.output(),
        ..LastPhysics::default()
    };

    let csv_path = if options.benchmark {
        None
    } else {
        choose_csv_path(&cfg, options.output_csv.clone())
    };
    let replay_path = if options.benchmark {
        None
    } else {
        choose_replay_path(&cfg, options.output_replay.clone())
    };

    let mut logger = match csv_path.as_ref() {
        Some(path) => Some(
            CsvLogger::create(path, sensor.count())
                .map_err(|e| format!("failed to create CSV log {}: {e}", path.display()))?,
        ),
        None => None,
    };
    let mut replay = match replay_path.as_ref() {
        Some(path) => Some(
            BinaryReplayLogger::create(path, sensor.count())
                .map_err(|e| format!("failed to create replay log {}: {e}", path.display()))?,
        ),
        None => None,
    };

    let mut next_sensor_us = 0u64;
    let mut next_encoder_us = 0u64;
    let mut next_imu_us = 0u64;
    let mut next_controller_us = 0u64;
    let mut next_log_us = 0u64;
    let start_wall = Instant::now();

    for step_idx in 0..=steps {
        let t_us = step_idx * time.physics_dt_us;

        while t_us >= next_sensor_us {
            sensor_output = sensor.sample(&track, state.pose, t_us);
            next_sensor_us = next_sensor_us.saturating_add(time.sensor_period_us);
        }

        while t_us >= next_encoder_us {
            encoder_output = encoder.sample(
                state.wheel_angle_left_rad,
                state.wheel_angle_right_rad,
                t_us,
            );
            next_encoder_us = next_encoder_us.saturating_add(time.encoder_period_us);
        }

        while t_us >= next_imu_us {
            gyro_output = gyro.sample(state.yaw_rate_rad_s, t_us);
            next_imu_us = next_imu_us.saturating_add(time.imu_period_us);
        }

        while t_us >= next_controller_us {
            ctrl_output = controller.step(
                &sensor_output,
                time.controller_period_us as f64 / 1_000_000.0,
            );
            next_controller_us = next_controller_us.saturating_add(time.controller_period_us);
        }

        while t_us >= next_log_us {
            if step_idx == 0 && last_physics.normal.total_normal_n() <= 0.0 {
                last_physics.normal = normal_force.step(NormalForceInput {
                    mass_kg: cfg.robot.chassis.mass_kg.max(1e-9),
                    center_of_mass_m: cfg.robot.chassis.center_of_mass_m,
                    wheelbase_m: cfg.robot.drivetrain.wheelbase_m.max(1e-6),
                    track_width_m: cfg.robot.drivetrain.track_width_m.max(1e-6),
                    battery_voltage_v: battery.terminal_voltage_v(),
                    command_pwm: ctrl_output.pwm_downforce,
                    speed_m_s: state.vx_body_m_s,
                    dt_us: 0,
                });
            }
            let sample = make_telemetry_sample(
                t_us,
                &state,
                &sensor_output,
                &encoder_output,
                &gyro_output,
                &ctrl_output,
                &last_physics,
            );
            if let Some(logger) = logger.as_mut() {
                logger
                    .write_sample(&sample)
                    .map_err(|e| format!("failed to write CSV log: {e}"))?;
            }
            if let Some(replay) = replay.as_mut() {
                replay
                    .write_sample(&sample)
                    .map_err(|e| format!("failed to write replay log: {e}"))?;
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
            &mut battery,
            &mut normal_force,
            ctrl_output,
            time.physics_dt_us,
        );
    }

    if let Some(logger) = logger.as_mut() {
        logger
            .flush()
            .map_err(|e| format!("failed to flush CSV log: {e}"))?;
    }
    if let Some(replay) = replay.as_mut() {
        replay
            .flush()
            .map_err(|e| format!("failed to flush replay log: {e}"))?;
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
        replay_path,
    })
}

fn validate_track_for_simulation(cfg: &LoadedConfig) -> Result<(), String> {
    let Some(track) = &cfg.track.parametric else {
        return Ok(());
    };
    if track.rules.mode != TrackRulesMode::Strict {
        return Ok(());
    }
    let issues: Vec<_> = validate_track(track)
        .into_iter()
        .filter(|issue| issue.severity == Severity::Error)
        .collect();
    if issues.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "track validation blocked simulation in strict mode: {}",
            issues
                .iter()
                .take(3)
                .map(|issue| issue.message.as_str())
                .collect::<Vec<_>>()
                .join("; ")
        ))
    }
}

fn validate_time(time: &TimeConfig) -> Result<(), String> {
    if time.physics_dt_us == 0
        || time.controller_period_us == 0
        || time.sensor_period_us == 0
        || time.encoder_period_us == 0
        || time.imu_period_us == 0
        || time.log_period_us == 0
    {
        return Err("all scheduler periods must be positive".to_string());
    }
    if time.controller_period_us < time.physics_dt_us
        || time.sensor_period_us < time.physics_dt_us
        || time.encoder_period_us < time.physics_dt_us
        || time.imu_period_us < time.physics_dt_us
        || time.log_period_us < time.physics_dt_us
    {
        return Err(
            "controller/sensor/encoder/imu/log periods must be >= physics_dt_us".to_string(),
        );
    }
    Ok(())
}

fn choose_csv_path(cfg: &LoadedConfig, output_override: Option<PathBuf>) -> Option<PathBuf> {
    if let Some(path) = output_override {
        return Some(path);
    }
    let base_dir = cfg
        .project_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."));
    cfg.project.csv_output.as_ref().map(|p| {
        if p.is_absolute() {
            p.clone()
        } else {
            base_dir.join(p)
        }
    })
}

fn choose_replay_path(cfg: &LoadedConfig, output_override: Option<PathBuf>) -> Option<PathBuf> {
    if let Some(path) = output_override {
        return Some(path);
    }
    let base_dir = cfg
        .project_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."));
    cfg.project.replay_output.as_ref().map(|p| {
        if p.is_absolute() {
            p.clone()
        } else {
            base_dir.join(p)
        }
    })
}

#[allow(clippy::too_many_arguments)]
fn physics_step(
    state: &mut RobotState,
    cfg: &LoadedConfig,
    track: &dyn TrackModel,
    tire: &dyn TireModel,
    motor_left: &dyn MotorModel,
    motor_right: &dyn MotorModel,
    battery: &mut VoltageSagBattery,
    normal_force: &mut dyn NormalForceModel,
    cmd: ControllerOutput,
    dt_us: u64,
) -> LastPhysics {
    let dt = dt_us as f64 / 1_000_000.0;
    let mass = cfg.robot.chassis.mass_kg.max(1e-9);
    let inertia = cfg.robot.chassis.inertia_kg_m2.max(1e-12);
    let wheel_radius = cfg.robot.drivetrain.wheel_radius_m.max(1e-9);
    let wheel_inertia = cfg.robot.drivetrain.wheel_inertia_kg_m2.max(1e-12);
    let half_track = cfg.robot.drivetrain.track_width_m * 0.5;

    let battery_v = battery.terminal_voltage_v();
    let normal = normal_force.step(NormalForceInput {
        mass_kg: mass,
        center_of_mass_m: cfg.robot.chassis.center_of_mass_m,
        wheelbase_m: cfg.robot.drivetrain.wheelbase_m.max(1e-6),
        track_width_m: cfg.robot.drivetrain.track_width_m.max(1e-6),
        battery_voltage_v: battery_v,
        command_pwm: cmd.pwm_downforce,
        speed_m_s: state.vx_body_m_s,
        dt_us,
    });
    let total_normal = normal.total_normal_n();
    let normal_left = normal.left_n();
    let normal_right = normal.right_n();

    let left_ground_speed = state.vx_body_m_s - state.yaw_rate_rad_s * half_track;
    let right_ground_speed = state.vx_body_m_s + state.yaw_rate_rad_s * half_track;

    let m_left = motor_left.step(
        cmd.pwm_left,
        state.wheel_omega_left_rad_s,
        battery_v,
        &cfg.robot.driver,
    );
    let m_right = motor_right.step(
        cmd.pwm_right,
        state.wheel_omega_right_rad_s,
        battery_v,
        &cfg.robot.driver,
    );

    let desired_left = m_left.wheel_torque_nm / wheel_radius;
    let desired_right = m_right.wheel_torque_nm / wheel_radius;
    let surface_mu = track.surface_mu_at(crate::math::Vec2::new(state.pose.x, state.pose.y));
    let mu_long = cfg.robot.tire.mu_longitudinal.min(surface_mu);

    let mut w_left = tire.longitudinal_force(TireInput {
        desired_force_n: desired_left,
        normal_force_n: normal_left,
        mu: mu_long,
        ground_speed_m_s: left_ground_speed,
        wheel_omega_rad_s: state.wheel_omega_left_rad_s,
        wheel_radius_m: wheel_radius,
    });
    let mut w_right = tire.longitudinal_force(TireInput {
        desired_force_n: desired_right,
        normal_force_n: normal_right,
        mu: mu_long,
        ground_speed_m_s: right_ground_speed,
        wheel_omega_rad_s: state.wheel_omega_right_rad_s,
        wheel_radius_m: wheel_radius,
    });

    apply_rolling_resistance(
        &mut w_left,
        left_ground_speed,
        normal_left,
        cfg.robot.tire.rolling_resistance,
    );
    apply_rolling_resistance(
        &mut w_right,
        right_ground_speed,
        normal_right,
        cfg.robot.tire.rolling_resistance,
    );

    let fx_body = w_left.force_n + w_right.force_n;
    let max_lateral = cfg.robot.tire.mu_lateral * total_normal;
    let desired_lateral = -state.vy_body_m_s * mass / dt.max(1e-9);
    let fy_body = clamp(desired_lateral, -max_lateral, max_lateral);

    let yaw_damping = 0.00008;
    let torque_z =
        (w_right.force_n - w_left.force_n) * half_track - yaw_damping * state.yaw_rate_rad_s;

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

    let next_left_ground_speed = state.vx_body_m_s - state.yaw_rate_rad_s * half_track;
    let next_right_ground_speed = state.vx_body_m_s + state.yaw_rate_rad_s * half_track;
    update_wheel_kinematics(
        &mut state.wheel_omega_left_rad_s,
        &mut state.wheel_angle_left_rad,
        m_left.wheel_torque_nm,
        &w_left,
        next_left_ground_speed,
        wheel_radius,
        wheel_inertia,
        dt,
    );
    update_wheel_kinematics(
        &mut state.wheel_omega_right_rad_s,
        &mut state.wheel_angle_right_rad,
        m_right.wheel_torque_nm,
        &w_right,
        next_right_ground_speed,
        wheel_radius,
        wheel_inertia,
        dt,
    );

    let battery_out = battery.step(
        m_left.supply_current_a + m_right.supply_current_a + normal.current_a,
        dt_us,
    );

    LastPhysics {
        motor_left: m_left,
        motor_right: m_right,
        wheel_left: w_left,
        wheel_right: w_right,
        normal,
        battery: battery_out,
    }
}

fn update_wheel_kinematics(
    omega: &mut f64,
    angle: &mut f64,
    wheel_torque_nm: f64,
    wheel_force: &WheelForces,
    ground_speed_m_s: f64,
    radius_m: f64,
    inertia_kg_m2: f64,
    dt_s: f64,
) {
    if wheel_force.saturated {
        let tire_reaction_torque = wheel_force.force_n * radius_m;
        *omega += ((wheel_torque_nm - tire_reaction_torque) / inertia_kg_m2) * dt_s;
    } else {
        // Static contact region: enforce the rolling constraint instead of accumulating
        // numerical wheel slip when the tire can transmit the demanded force.
        *omega = ground_speed_m_s / radius_m;
    }
    *angle += *omega * dt_s;
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
    encoder: &EncoderOutput,
    gyro: &GyroOutput,
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
        pwm_downforce: ctrl.pwm_downforce,
        motor_current_left_a: phys.motor_left.current_a,
        motor_current_right_a: phys.motor_right.current_a,
        motor_torque_left_nm: phys.motor_left.wheel_torque_nm,
        motor_torque_right_nm: phys.motor_right.wheel_torque_nm,
        motor_voltage_left_v: phys.motor_left.voltage_v,
        motor_voltage_right_v: phys.motor_right.voltage_v,
        wheel_force_left_n: phys.wheel_left.force_n,
        wheel_force_right_n: phys.wheel_right.force_n,
        desired_wheel_force_left_n: phys.wheel_left.desired_force_n,
        desired_wheel_force_right_n: phys.wheel_right.desired_force_n,
        slip_left: phys.wheel_left.slip_ratio,
        slip_right: phys.wheel_right.slip_ratio,
        wheel_surface_speed_left_m_s: phys.wheel_left.wheel_surface_speed_m_s,
        wheel_surface_speed_right_m_s: phys.wheel_right.wheel_surface_speed_m_s,
        normal_left_n: phys.normal.left_n(),
        normal_right_n: phys.normal.right_n(),
        normal_front_left_n: phys.normal.front_left_n,
        normal_front_right_n: phys.normal.front_right_n,
        normal_rear_left_n: phys.normal.rear_left_n,
        normal_rear_right_n: phys.normal.rear_right_n,
        downforce_extra_n: phys.normal.extra_downforce_n,
        downforce_fan_n: phys.normal.fan_force_n,
        downforce_suction_n: phys.normal.suction_force_n,
        downforce_current_a: phys.normal.current_a,
        battery_voltage_v: phys.battery.terminal_voltage_v,
        battery_current_a: phys.battery.current_a,
        encoder_left_ticks: encoder.left.ticks,
        encoder_right_ticks: encoder.right.ticks,
        encoder_left_velocity_rad_s: encoder.left.velocity_rad_s,
        encoder_right_velocity_rad_s: encoder.right.velocity_rad_s,
        gyro_yaw_rate_rad_s: gyro.yaw_rate_rad_s,
        gyro_bias_rad_s: gyro.bias_rad_s,
        sensor_adc: sensor.adc.clone(),
    }
}
