use crate::battery::{BatteryOutput, VoltageSagBattery};
use crate::config::{LoadedConfig, TimeConfig};
use crate::controller::{BuiltInPid, ControllerOutput};
use crate::core::clock::{duration_seconds_to_us, Clock};
use crate::core::integrator::{energy, integrate_pose, solve_contacts, Mechanics};
use crate::core::scheduler::{events_at, validate_time};
use crate::encoder::{EncoderOutput, QuantizedEncoder};
use crate::gyro::{GyroOutput, NoisyGyro};
use crate::math::Pose2;
use crate::motor::{DcMotorSimple, MotorOutput};
use crate::normal_force::{
    ConfiguredNormalForce, NormalForceInput, NormalForceModel, NormalForceOutput,
};
use crate::replay::BinaryReplayLogger;
use crate::rtsim_track::{validate_track, Severity, TrackRulesMode};
#[cfg(test)]
use crate::sensor::SensorModel;
use crate::sensor::{SensorOutput, SimpleLineSensor};
use crate::telemetry::PhysicsDiagnostics;
use crate::telemetry::{CsvLogger, TelemetrySample};
use crate::track::{TrackModel, VectorTrack};
use crate::wheel::WheelForces;
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
    pub run_id: String,
    pub termination_reason: String,
    pub race_events: Vec<crate::track::events::RaceEvent>,
    pub project_name: String,
    pub robot_name: String,
    pub track_name: String,
    pub duration_us: u64,
    /// Number of physical integrations, excluding the initial observation.
    pub steps: u64,
    /// Scheduled observations including endpoints; counted but not written in benchmarks.
    pub samples: u64,
    pub effective_config: EffectiveRunConfig,
    pub metadata_paths: Vec<PathBuf>,
    pub warnings: Vec<String>,
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
    diagnostics: PhysicsDiagnostics,
}

/// Configuration actually used by this run, including CLI overrides and runtime seeds.
#[derive(Debug, Clone, PartialEq)]
pub struct EffectiveRunConfig {
    pub run_id: String,
    pub sensing_model: String,
    pub controller_model: String,
    pub power_model: String,
    pub physics_model: String,
    pub time: TimeConfig,
    pub duration_us: u64,
    pub line_sensor_seed: u64,
    pub gyro_seed: u64,
    pub physics_dt_override_us: Option<u64>,
}

/// Number of acquisitions/controller calls, including one initialization at t=0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EventCounts {
    pub sensor: u64,
    pub encoder: u64,
    pub imu: u64,
    pub controller: u64,
}

/// Sole owner of simulated time, physical state, device state and RNG streams.
/// Public observations are pure. Each tick is fully processed before exposure.
pub struct SimulationCore {
    power: Option<crate::models::power::PowerState>,
    cfg: LoadedConfig,
    mechanical_cfg: Option<Box<LoadedConfig>>,
    contacts: crate::models::contact::ContactState,
    resolved: crate::io::experiment::ResolvedExperiment,
    time: TimeConfig,
    clock: Clock,
    effective_config: EffectiveRunConfig,
    event_counts: EventCounts,
    state: RobotState,
    track: VectorTrack,
    race: crate::track::events::RaceState,
    sensor: SimpleLineSensor,
    encoder: QuantizedEncoder,
    gyro: NoisyGyro,
    controller: BuiltInPid,
    replay_controller: Option<crate::control::replay_controller::ReplayController>,
    external_actuators: bool,
    native_controller: Option<crate::control::native::NativeAdapter>,
    last_imu_velocity: (u64, crate::math::Vec2),
    motor_left: DcMotorSimple,
    motor_right: DcMotorSimple,
    normal_force: ConfiguredNormalForce,
    battery: VoltageSagBattery,
    sensor_output: SensorOutput,
    encoder_output: EncoderOutput,
    gyro_output: GyroOutput,
    ctrl_output: ControllerOutput,
    control_command: crate::control::TimedCommand,
    last_physics: LastPhysics,
    failure: Option<String>,
}

/// Compatibility name for the incremental GUI API; no second executor.
pub type SimulationSession = SimulationCore;

impl SimulationCore {
    pub fn new(cfg: LoadedConfig, duration_us: Option<u64>) -> Result<Self, String> {
        Self::with_time_override(cfg, duration_us, None)
    }

    pub fn with_time_override(
        mut cfg: LoadedConfig,
        duration_us: Option<u64>,
        physics_dt_override_us: Option<u64>,
    ) -> Result<Self, String> {
        if let Some(dt) = physics_dt_override_us {
            cfg.project.time.physics_dt_us = dt;
        }
        validate_time(&cfg.project.time)?;
        let time = cfg.project.time;
        let duration_us = match duration_us {
            Some(us) => us,
            None => duration_seconds_to_us(cfg.project.duration_s)?,
        };
        let clock = Clock::new(duration_us, time.physics_dt_us)?;
        cfg.project.duration_s = duration_us as f64 / 1_000_000.0;
        let resolved = crate::io::experiment::ResolvedExperiment::new(&cfg)?;
        cfg = resolved.config().clone();
        validate_physics(&cfg)?;
        validate_track_for_simulation(&cfg)?;

        let effective_config = EffectiveRunConfig {
            run_id: new_run_id(),
            sensing_model: "individual-area-timed-v1".into(),
            controller_model: if cfg.robot.sensing.replay_csv.is_some() {
                "replay-v1"
            } else {
                "observable-pid-v1"
            }
            .into(),
            power_model: if cfg.robot.powertrain.is_some() {
                "coupled-dc-v1".into()
            } else {
                "staggered-simple-v1".into()
            },
            physics_model: cfg
                .robot
                .physics
                .as_ref()
                .map(|f| format!("per-wheel-v1/{}", f.contact))
                .unwrap_or_else(|| "planar-impulse-v2".into()),
            time,
            duration_us,
            line_sensor_seed: cfg
                .robot
                .sensors
                .first()
                .map(|s| s.acquisition.seed)
                .unwrap_or(0),
            gyro_seed: if cfg.robot.gyro.seed == 0 {
                0x9E37_79B9_7F4A_7C15
            } else {
                cfg.robot.gyro.seed
            },
            physics_dt_override_us,
        };
        let state = RobotState {
            pose: cfg.project.start_pose,
            ..RobotState::default()
        };
        let track = VectorTrack::try_new(cfg.track.clone())?;
        let race = crate::track::events::RaceState::new(
            &track,
            track.robot_inside(&cfg.robot, state.pose),
        );
        let mut sensor = SimpleLineSensor::from_robot(&cfg.robot);
        let mut encoder = QuantizedEncoder::with_settings(
            cfg.robot.encoder.clone(),
            cfg.robot.sensing.encoder.clone(),
        );
        let mut gyro =
            NoisyGyro::with_settings(cfg.robot.gyro.clone(), cfg.robot.sensing.imu.clone());
        let mut controller = BuiltInPid::from_robot(
            &cfg.robot,
            track.base_reflectance(),
            track.line_reflectance(),
        );
        let motor_left = DcMotorSimple::new(cfg.robot.motor_left.clone());
        let motor_right = DcMotorSimple::new(cfg.robot.motor_right.clone());
        let mut normal_force = if let Some(p) = &cfg.robot.powertrain {
            ConfiguredNormalForce::new_coupled(
                cfg.robot.normal_force.clone(),
                cfg.robot.battery.nominal_voltage_v,
                p.suction_gap_m,
                p.suction_leak_per_m,
            )?
        } else {
            ConfiguredNormalForce::new(cfg.robot.normal_force.clone())
        };
        let mut power = cfg
            .robot
            .powertrain
            .as_ref()
            .map(|p| crate::models::power::PowerState::new(p, &cfg.robot.battery));
        let battery = VoltageSagBattery::new(cfg.robot.battery.clone());
        let sensor_output = sensor.advance(&track, state.pose, 0, time.sensor_period_us);
        let encoder_output =
            encoder.sample(state.wheel_angle_left_rad, state.wheel_angle_right_rad, 0);
        let gyro_output = gyro.sample(state.yaw_rate_rad_s, 0);
        let mut replay_controller = cfg
            .robot
            .sensing
            .replay_csv
            .as_ref()
            .map(|text| {
                crate::control::replay_controller::ReplayController::from_csv(
                    text,
                    time.physics_dt_us,
                )
            })
            .transpose()?;
        let initial_input = crate::control::ControllerInput {
            frame: crate::control::SensorFrame::from_readings(
                0,
                &sensor_output,
                encoder_output,
                gyro_output,
            ),
            ..Default::default()
        };
        let mut command = if let Some(replay) = &mut replay_controller {
            replay.at(0)
        } else {
            controller.step_input(&initial_input, time.controller_period_us as f64 * 1e-6)
        };
        if cfg.robot.powertrain.is_none() {
            command.modes = [crate::models::power::ActuatorMode::Drive; 2];
        }
        let ctrl_output = controller.output(command);
        if let Some(p) = &mut power {
            p.request(
                crate::models::power::ActuatorCommand {
                    t_us: 0,
                    pwm: command.pwm,
                    modes: command.modes,
                },
                cfg.robot.powertrain.as_ref().unwrap().command_latency_us,
            )?;
            p.manual_command = Some(p.last_requested.unwrap());
        }

        validate_outputs(&sensor_output, &encoder_output, &gyro_output, &ctrl_output)
            .map_err(|e| format!("initialization failure at t=0 us: {e}"))?;
        let mut last_physics = LastPhysics {
            battery: battery.output(),
            ..LastPhysics::default()
        };
        last_physics.normal = normal_force.step(NormalForceInput {
            mass_kg: cfg.robot.chassis.mass_kg,
            center_of_mass_m: cfg.robot.chassis.center_of_mass_m,
            wheelbase_m: cfg.robot.drivetrain.wheelbase_m,
            track_width_m: cfg.robot.drivetrain.track_width_m,
            battery_voltage_v: power
                .as_ref()
                .map_or(battery.terminal_voltage_v(), |p| p.voltage_v),
            command_pwm: ctrl_output.pwm_downforce,
            speed_m_s: state.vx_body_m_s,
            dt_us: 0,
        });

        let mut contacts = crate::models::contact::ContactState::new(&cfg.robot);
        if cfg.robot.physics.is_some() {
            let a = cfg.robot.assembly.as_ref().unwrap();
            let positions: Vec<_> = a.wheels.iter().map(|w| w.position_m).collect();
            let n = last_physics.normal;
            let total = n.total_normal_n();
            let center = if total > 0. {
                n.load_first_moment_nm * (1. / total)
            } else {
                cfg.robot.chassis.center_of_mass_m
            };
            let loads = crate::models::chassis::support_loads(&positions, total, center)?;
            for (w, n) in contacts.wheels.iter_mut().zip(loads) {
                w.normal_n = n;
            }
        }
        if let Some(p) = &power {
            last_physics.battery.terminal_voltage_v = p.voltage_v;
            last_physics.battery.open_circuit_voltage_v = p.voltage_v;
        }
        let mechanical_cfg = cfg.robot.powertrain.as_ref().map(|p| {
            let mut mechanical = cfg.clone();
            for (side, id) in ["motor:left", "motor:right"].iter().enumerate() {
                let wheel = mechanical
                    .robot
                    .assembly
                    .as_mut()
                    .unwrap()
                    .wheels
                    .iter_mut()
                    .find(|w| w.motor.as_deref() == Some(id))
                    .unwrap();
                wheel.inertia_kg_m2 +=
                    p.motors[side].rotor_inertia_kg_m2 * p.motors[side].ratio.powi(2);
            }
            Box::new(mechanical)
        });
        Ok(Self {
            mechanical_cfg,
            replay_controller,
            external_actuators: false,
            native_controller: None,
            last_imu_velocity: (0, crate::math::Vec2::default()),
            power,
            contacts,
            resolved,
            cfg,
            time,
            clock,
            effective_config,
            event_counts: EventCounts {
                sensor: 1,
                encoder: 1,
                imu: 1,
                controller: 1,
            },
            state,
            track,
            race,
            sensor,
            encoder,
            gyro,
            controller,
            motor_left,
            motor_right,
            normal_force,
            battery,
            sensor_output,
            encoder_output,
            gyro_output,
            ctrl_output,
            control_command: command,
            last_physics,
            failure: None,
        })
    }

    pub fn time_us(&self) -> u64 {
        self.clock.now_us()
    }

    pub fn duration_us(&self) -> u64 {
        self.clock.duration_us()
    }

    pub fn progress(&self) -> f64 {
        if self.duration_us() == 0 {
            1.0
        } else {
            (self.time_us() as f64 / self.duration_us() as f64).min(1.0)
        }
    }

    pub fn is_finished(&self) -> bool {
        self.clock.is_finished()
            || self.failure.is_some()
            || self.race.termination().is_some()
            || self.power.as_ref().is_some_and(|p| p.fault.is_some())
    }

    pub fn robot_over_line(&self) -> bool {
        self.track.robot_over_line(&self.cfg.robot, self.state.pose)
    }
    pub fn race_state(&self) -> &crate::track::events::RaceState {
        &self.race
    }
    pub fn track_runtime(&self) -> &VectorTrack {
        &self.track
    }
    pub fn termination_reason(&self) -> &str {
        if self.failure.is_some() {
            "failure"
        } else if let Some(reason) = self.power.as_ref().and_then(|p| p.fault.as_deref()) {
            reason
        } else if let Some(reason) = self.race.termination() {
            reason
        } else if self.clock.is_finished() {
            "duration"
        } else {
            "running"
        }
    }
    pub fn fan_states(&self) -> Vec<(String, f64, f64)> {
        self.normal_force.fan_readings()
    }
    pub fn suction_pressure_pa(&self) -> f64 {
        self.last_physics.normal.suction_force_n
            / self.cfg.robot.normal_force.chamber_area_m2.max(1e-12)
    }
    pub fn power_state(&self) -> Option<&crate::models::power::PowerState> {
        self.power.as_ref()
    }
    pub fn command_actuators(
        &mut self,
        command: crate::models::power::ActuatorCommand,
    ) -> Result<(), String> {
        if command.t_us != self.time_us() {
            return Err(
                "actuator command must be issued at the current simulation timestamp".into(),
            );
        }
        let p = self
            .power
            .as_mut()
            .ok_or("electrical powertrain is not enabled")?;
        p.request(
            command,
            self.cfg
                .robot
                .powertrain
                .as_ref()
                .unwrap()
                .command_latency_us,
        )?;
        p.manual_command = Some(command);
        self.external_actuators = true;
        self.control_command = crate::control::TimedCommand {
            t_us: command.t_us,
            pwm: command.pwm,
            modes: command.modes,
            downforce_pwm: self.ctrl_output.pwm_downforce,
        };
        Ok(())
    }
    pub fn use_controller_actuators(&mut self) {
        self.external_actuators = false;
        if let Some(p) = &mut self.power {
            p.manual_command = None;
        }
    }
    pub fn contact_state(&self) -> &crate::models::contact::ContactState {
        &self.contacts
    }
    pub fn physics_models(&self) -> Option<&crate::models::fidelity::FidelityConfig> {
        self.cfg.robot.physics.as_ref()
    }
    pub fn contact_surfaces(&self) -> Vec<(String, crate::track::runtime::SurfaceSample)> {
        self.cfg
            .robot
            .assembly
            .as_ref()
            .unwrap()
            .wheels
            .iter()
            .map(|w| {
                (
                    w.id.clone(),
                    self.track
                        .surface_at(self.state.pose.transform_point(w.position_m)),
                )
            })
            .collect()
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

    pub fn resolved_experiment(&self) -> &crate::io::experiment::ResolvedExperiment {
        &self.resolved
    }
    pub fn sensor_ids(&self) -> Vec<&str> {
        self.sensor.ids()
    }

    pub fn state(&self) -> &RobotState {
        &self.state
    }
    pub fn effective_config(&self) -> &EffectiveRunConfig {
        &self.effective_config
    }
    pub fn event_counts(&self) -> EventCounts {
        self.event_counts
    }
    pub fn steps(&self) -> u64 {
        self.clock.steps()
    }

    /// Includes t=0 and the terminal tick even when off the regular log grid.
    pub fn should_log(&self) -> bool {
        events_at(self.time, self.time_us()).log || self.is_finished()
    }

    pub fn advance_steps(&mut self, steps: u64) -> TelemetrySample {
        for _ in 0..steps {
            if !self.step() {
                break;
            }
        }
        self.sample()
    }

    /// Rejects backwards, off-grid and beyond-end targets without changing state.
    pub fn advance_until(&mut self, target_us: u64) -> Result<TelemetrySample, String> {
        self.clock.validate_target(target_us)?;
        while self.time_us() < target_us {
            if !self.try_step()? {
                break;
            }
        }
        Ok(self.sample())
    }

    /// Integrate one interval with held commands, then acquire and control at
    /// the new timestamp. Returns false at the end, without changing any state.
    pub fn step(&mut self) -> bool {
        self.try_step().unwrap_or(false)
    }

    pub fn failure(&self) -> Option<&str> {
        self.failure.as_deref()
    }
    pub fn diagnostics(&self) -> &PhysicsDiagnostics {
        &self.last_physics.diagnostics
    }

    /// Failed ticks stop the run, retain its last valid state and report the interval start.
    pub fn try_step(&mut self) -> Result<bool, String> {
        crate::experiments::jobs::check_cancelled()?;
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        if self.is_finished() {
            return Ok(false);
        }
        let previous_pose = self.state.pose;
        let mut next = self.state;
        let mut next_contacts = self.contacts.clone();
        let mut next_power = self.power.clone();
        let mut next_normal = self.normal_force.clone();
        let result = if let Some(power) = &mut next_power {
            coupled_power_step(
                &mut next,
                self.mechanical_cfg.as_deref().unwrap(),
                &self.track,
                &mut next_contacts,
                &mut next_normal,
                power,
                self.ctrl_output,
                self.time_us(),
                self.time.physics_dt_us,
            )
        } else {
            physics_step(
                &mut next,
                &self.cfg,
                &self.track,
                &self.motor_left,
                &self.motor_right,
                &mut self.battery,
                &mut self.normal_force,
                self.ctrl_output,
                self.time.physics_dt_us,
                &mut next_contacts,
            )
        };
        if next_power.as_ref().is_some_and(|p| p.fault.is_some()) {
            self.power = next_power;
            return Ok(false);
        }
        match result {
            Ok(physics) => {
                self.last_physics = physics;
                self.state = next;
                self.contacts = next_contacts;
                if next_power.is_some() {
                    self.normal_force = next_normal;
                    self.power = next_power;
                }
            }
            Err(error) => {
                let error = format!("physics failure at t={} us: {error}", self.time_us());
                self.failure = Some(error.clone());
                return Err(error);
            }
        }
        self.clock.advance();
        self.race.update(
            crate::math::Vec2::new(previous_pose.x, previous_pose.y),
            crate::math::Vec2::new(self.state.pose.x, self.state.pose.y),
            self.track.robot_inside(&self.cfg.robot, self.state.pose),
            self.time_us(),
        );
        self.process_events();
        if let Some(e) = &self.failure {
            return Err(e.clone());
        }
        if let Err(error) = validate_outputs(
            &self.sensor_output,
            &self.encoder_output,
            &self.gyro_output,
            &self.ctrl_output,
        ) {
            let error = format!("event failure at t={} us: {error}", self.time_us());
            self.failure = Some(error.clone());
            return Err(error);
        }
        Ok(true)
    }

    /// Compatibility helper: advances once, returns the fully processed new tick.
    pub fn step_once(&mut self) -> TelemetrySample {
        self.step();
        self.sample()
    }

    pub fn controller_input(&self) -> crate::control::ControllerInput {
        crate::control::ControllerInput {
            frame: crate::control::SensorFrame::from_readings(
                self.time_us(),
                &self.sensor_output,
                self.encoder_output,
                self.gyro_output,
            ),
            actuators: crate::control::ActuatorFeedback {
                applied_pwm: [
                    self.last_physics.motor_left.applied_pwm,
                    self.last_physics.motor_right.applied_pwm,
                ],
                current_limited: self
                    .power
                    .as_ref()
                    .map(|p| p.motors.each_ref().map(|m| m.current_limited))
                    .unwrap_or([false; 2]),
            },
        }
    }
    pub fn control_command(&self) -> crate::control::TimedCommand {
        self.control_command
    }
    pub fn sensor_readings(&self) -> &SensorOutput {
        &self.sensor_output
    }
    pub fn estimated_state(&self) -> &crate::control::estimator::Estimate {
        &self.controller.estimate.state
    }
    pub fn install_firmware(
        &mut self,
        firmware: Box<dyn crate::control::native::Firmware>,
    ) -> Result<(), String> {
        if self.time_us() != 0 {
            return Err("install firmware before advancing simulation".into());
        }
        let mut native = crate::control::native::NativeAdapter::new(firmware)?;
        let command = native.step(&self.controller_input())?;
        if self.power.is_none()
            && command
                .modes
                .iter()
                .any(|m| *m != crate::models::power::ActuatorMode::Drive)
        {
            return Err("explicit brake/coast firmware requires coupled powertrain".into());
        }
        self.apply_control_command(command)?;
        self.native_controller = Some(native);
        self.effective_config.controller_model = "native-api-v1".into();
        self.replay_controller = None;
        Ok(())
    }
    fn apply_control_command(
        &mut self,
        command: crate::control::TimedCommand,
    ) -> Result<(), String> {
        if self.external_actuators {
            return Ok(());
        }
        self.ctrl_output = self.controller.output(command);
        self.control_command = command;
        if self.power.is_some() {
            self.command_actuators(crate::models::power::ActuatorCommand {
                t_us: command.t_us,
                pwm: command.pwm,
                modes: command.modes,
            })?;
            self.external_actuators = false;
        } else if command
            .modes
            .iter()
            .any(|m| *m != crate::models::power::ActuatorMode::Drive)
        {
            return Err("explicit non-drive commands require coupled powertrain".into());
        }
        Ok(())
    }
    fn process_events(&mut self) {
        let t_us = self.time_us();
        let due = events_at(self.time, t_us);
        self.sensor.advance_into(
            &self.track,
            self.state.pose,
            t_us,
            self.time.sensor_period_us,
            &mut self.sensor_output,
        );
        if due.sensor {
            self.event_counts.sensor += 1;
        }
        if due.encoder {
            self.encoder_output = self.encoder.sample(
                self.state.wheel_angle_left_rad,
                self.state.wheel_angle_right_rad,
                t_us,
            );
            self.event_counts.encoder += 1;
        } else {
            self.encoder_output = self.encoder.deliver(t_us);
        }
        if due.imu {
            let v = crate::math::Vec2::new(self.state.vx_body_m_s, self.state.vy_body_m_s);
            let dt = (t_us - self.last_imu_velocity.0) as f64 * 1e-6;
            let derivative = (v - self.last_imu_velocity.1) * (1. / dt.max(1e-12));
            let acceleration = derivative
                + crate::math::Vec2::new(
                    -self.state.yaw_rate_rad_s * v.y,
                    self.state.yaw_rate_rad_s * v.x,
                );
            self.gyro_output = self
                .gyro
                .sample_imu(self.state.yaw_rate_rad_s, acceleration, t_us);
            self.last_imu_velocity = (t_us, v);
            self.event_counts.imu += 1;
        } else {
            self.gyro_output = self.gyro.deliver(t_us);
        }
        let input = due.controller.then(|| self.controller_input());
        let command = if let Some(replay) = &mut self.replay_controller {
            if due.controller {
                self.controller
                    .estimate
                    .update(&input.as_ref().unwrap().frame);
            }
            Some(Ok(replay.at(t_us)))
        } else if due.controller {
            if let Some(native) = &mut self.native_controller {
                self.controller
                    .estimate
                    .update(&input.as_ref().unwrap().frame);
                Some(native.step(input.as_ref().unwrap()))
            } else {
                {
                    let mut c = self.controller.step_input(
                        input.as_ref().unwrap(),
                        self.time.controller_period_us as f64 * 1e-6,
                    );
                    if self.power.is_none() {
                        c.modes = [crate::models::power::ActuatorMode::Drive; 2];
                    }
                    Some(Ok(c))
                }
            }
        } else {
            None
        };
        if due.controller {
            self.event_counts.controller += 1;
        }
        if let Some(command) = command {
            if let Err(e) = command.and_then(|c| self.apply_control_command(c)) {
                self.failure = Some(format!("controller failure at {t_us} us: {e}"));
            }
        }
    }
}

pub fn new_run_id() -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static ID: AtomicU64 = AtomicU64::new(0);
    format!(
        "{:x}-{}-{}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos(),
        std::process::id(),
        ID.fetch_add(1, Ordering::Relaxed)
    )
}
pub fn run_simulation(cfg: LoadedConfig, options: RunOptions) -> Result<RunSummary, String> {
    run_controlled(cfg, options, None, new_run_id())
}
pub fn run_controlled(
    cfg: LoadedConfig,
    options: RunOptions,
    control: Option<&crate::experiments::jobs::RunControl>,
    run_id: String,
) -> Result<RunSummary, String> {
    let csv_path = if options.benchmark {
        None
    } else {
        choose_csv_path(&cfg, options.output_csv)
    };
    let replay_path = if options.benchmark {
        None
    } else {
        choose_replay_path(&cfg, options.output_replay)
    };
    if csv_path.is_some() && csv_path == replay_path {
        return Err("CSV and replay outputs must use different paths".into());
    }
    // Validate and initialize before creating any output file.
    let mut core = SimulationCore::with_time_override(
        cfg,
        options.duration_us,
        options.physics_dt_override_us,
    )?;
    core.effective_config.run_id = run_id.clone();
    let mut logger = csv_path
        .as_ref()
        .map(|path| {
            CsvLogger::create(path, core.sensor.count())
                .map_err(|e| format!("failed to create CSV log {}: {e}", path.display()))
        })
        .transpose()?;
    let mut replay = replay_path
        .as_ref()
        .map(|path| {
            BinaryReplayLogger::create_with_metadata(path, core.sensor.count(), &format!(r#"{{"effective":{},"experiment":{},"channels":{},"sidecar_suffixes":[".contacts.csv",".power.csv",".sensors.jsonl",".commands.csv"],"sidecar_policy":"appended to replay filename; optional model channels; completion certified only by replay footer"}}"#,core.effective_config.to_json(),core.resolved.to_json(),crate::replay::CHANNEL_SCHEMA_JSON))
                .map_err(|e| format!("failed to create replay {}: {e}", path.display()))
        })
        .transpose()?;
    let mut metadata_paths = Vec::new();
    for path in csv_path.iter().chain(replay_path.iter()) {
        let mut metadata_path = path.as_os_str().to_os_string();
        metadata_path.push(".metadata.json");
        let metadata_path = PathBuf::from(metadata_path);
        std::fs::write(&metadata_path, core.effective_config.to_json()).map_err(|e| {
            format!(
                "failed to write run metadata {}: {e}",
                metadata_path.display()
            )
        })?;
        metadata_paths.push(metadata_path);
        let mut experiment_path = path.as_os_str().to_os_string();
        experiment_path.push(".experiment.json");
        core.resolved.save(PathBuf::from(experiment_path))?;
    }
    let contact_ids: Vec<String> = core
        .cfg
        .robot
        .assembly
        .as_ref()
        .unwrap()
        .wheels
        .iter()
        .map(|w| w.id.clone())
        .collect();
    let mut contact_logs = Vec::new();
    if core.cfg.robot.physics.is_some() {
        for path in csv_path.iter().chain(replay_path.iter()) {
            let mut p = path.as_os_str().to_os_string();
            p.push(".contacts.csv");
            let p = PathBuf::from(p);
            contact_logs
                .push(crate::telemetry::ContactLogger::create(&p).map_err(|e| e.to_string())?);
            metadata_paths.push(p);
        }
    }
    let mut power_logs = Vec::new();
    if core.power.is_some() {
        for path in csv_path.iter().chain(replay_path.iter()) {
            let mut p = path.as_os_str().to_os_string();
            p.push(".power.csv");
            let p = PathBuf::from(p);
            power_logs.push(crate::telemetry::PowerLogger::create(&p).map_err(|e| e.to_string())?);
            metadata_paths.push(p);
        }
    }
    let mut sensor_logs = Vec::new();
    for path in csv_path.iter().chain(replay_path.iter()) {
        sensor_logs.push(crate::telemetry::SensorLogger::create(path).map_err(|e| e.to_string())?);
    }
    let start_wall = Instant::now();
    let mut samples = 0;
    let mut cancelled = false;
    loop {
        if let Some(control) = control {
            cancelled = !control.boundary(&core);
        }
        for log in &mut sensor_logs {
            log.command(core.control_command())
                .map_err(|e| e.to_string())?;
        }
        let logged = core.should_log() || cancelled;
        if logged {
            for log in &mut sensor_logs {
                log.sample(&core).map_err(|e| e.to_string())?;
            }
            if let Some(p) = &core.power {
                for logger in &mut power_logs {
                    logger
                        .write_sample(core.time_us(), p)
                        .map_err(|e| e.to_string())?;
                }
            }
            samples += 1;
            for logger in &mut contact_logs {
                logger
                    .write_sample(core.time_us(), &contact_ids, &core.contacts)
                    .map_err(|e| e.to_string())?;
            }
            if logger.is_some() || replay.is_some() {
                let sample = core.sample();
                if let Some(logger) = logger.as_mut() {
                    logger
                        .write_sample(&sample)
                        .map_err(|e| format!("failed to write CSV: {e}"))?;
                }
                if let Some(replay) = replay.as_mut() {
                    replay
                        .write_sample(&sample)
                        .map_err(|e| format!("failed to write replay: {e}"))?;
                }
            }
        }
        if cancelled {
            break;
        }
        if !core.try_step()? {
            if !logged && core.is_finished() {
                continue;
            }
            break;
        }
    }
    if let Some(logger) = logger.as_mut() {
        logger.flush().map_err(|e| e.to_string())?;
    }
    for logger in &mut contact_logs {
        logger.flush().map_err(|e| e.to_string())?;
    }
    for logger in &mut power_logs {
        logger.flush().map_err(|e| e.to_string())?;
    }
    for log in &mut sensor_logs {
        log.flush().map_err(|e| e.to_string())?;
    }
    if let Some(p) = &core.power {
        for path in csv_path.iter().chain(replay_path.iter()) {
            let mut out = path.as_os_str().to_os_string();
            out.push(".power.events.json");
            std::fs::write(PathBuf::from(out), p.events_json()).map_err(|e| e.to_string())?;
        }
    }
    for path in csv_path.iter().chain(replay_path.iter()) {
        let mut events_path = path.as_os_str().to_os_string();
        events_path.push(".events.json");
        std::fs::write(
            PathBuf::from(events_path),
            core.race.to_json(if cancelled {
                "cancelled"
            } else {
                core.termination_reason()
            }),
        )
        .map_err(|e| e.to_string())?;
    }
    if let Some(replay) = replay.as_mut() {
        replay
            .finish(if cancelled {
                "cancelled"
            } else {
                core.termination_reason()
            })
            .map_err(|e| e.to_string())?;
    }
    let wall_time = start_wall.elapsed();
    let simulated_time_s = core.time_us() as f64 / 1_000_000.0;
    let wall_s = wall_time.as_secs_f64().max(1e-12);
    Ok(RunSummary {
        run_id,
        termination_reason: if cancelled {
            "cancelled"
        } else {
            core.termination_reason()
        }
        .into(),
        race_events: core.race.events().to_vec(),
        project_name: core.cfg.project.name.clone(),
        robot_name: core.cfg.robot.name.clone(),
        track_name: core.cfg.track.name.clone(),
        duration_us: core.time_us(),
        steps: core.steps(),
        samples,
        effective_config: core.effective_config.clone(),
        metadata_paths,
        warnings: core.resolved.warnings().to_vec(),
        final_pose: core.state.pose,
        simulated_time_s,
        wall_time,
        steps_per_second: core.steps() as f64 / wall_s,
        realtime_factor: simulated_time_s / wall_s,
        csv_path,
        replay_path,
    })
}

impl EffectiveRunConfig {
    /// Effective settings are embedded in v4 replay and mirrored in JSON sidecars.
    pub fn to_json(&self) -> String {
        format!(concat!(
            "{{\n  \"schema\": \"rtsim-run-time-v1\",\n",
            "  \"engine_version\": \"{}\",\n",
            "  \"run_id\": \"{}\",\n",
            "  \"physics_model\": \"{}\",\n",
            "  \"power_model\": \"{}\",\n",
            "  \"sensing_model\": \"{}\",\n",
            "  \"controller_model\": \"{}\",\n",
            "  \"duration_us\": {},\n",
            "  \"physics_dt_override_us\": {},\n",
            "  \"time\": {{\n",
            "    \"physics_dt_us\": {},\n    \"controller_period_us\": {},\n",
            "    \"sensor_period_us\": {},\n    \"encoder_period_us\": {},\n",
            "    \"imu_period_us\": {},\n    \"log_period_us\": {},\n",
            "    \"render_period_us\": {}\n  }},\n",
            "  \"line_sensor_seed\": {},\n  \"gyro_seed\": {},\n",
            "  \"sample_phase\": \"state_at_t_after_zero_latency_acquisition_and_control\",\n",
            "  \"physics_outputs_phase\": \"previous_interval; at t=0 initialized normal and battery, other outputs zero\",\n",
            "  \"duration_policy\": \"reject_off_grid\",\n",
            "  \"terminal_sample\": true\n}}\n"
        ), env!("CARGO_PKG_VERSION"), self.run_id, self.physics_model, self.power_model, self.sensing_model,self.controller_model,self.duration_us,
        self.physics_dt_override_us.map(|v| v.to_string()).unwrap_or_else(|| "null".into()),
        self.time.physics_dt_us, self.time.controller_period_us, self.time.sensor_period_us,
        self.time.encoder_period_us, self.time.imu_period_us, self.time.log_period_us,
        self.time.render_period_us, self.line_sensor_seed, self.gyro_seed)
    }
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

fn mechanics(cfg: &LoadedConfig) -> Mechanics {
    Mechanics {
        mass: cfg.robot.chassis.mass_kg,
        inertia: cfg.robot.chassis.inertia_kg_m2,
        wheel_inertia: cfg.robot.drivetrain.wheel_inertia_kg_m2,
        radius: cfg.robot.drivetrain.wheel_radius_m,
        half_track: cfg.robot.drivetrain.track_width_m * 0.5,
        com: cfg.robot.chassis.center_of_mass_m,
    }
}

#[allow(clippy::too_many_arguments)]
fn physics_step(
    state: &mut RobotState,
    cfg: &LoadedConfig,
    track: &dyn TrackModel,
    motor_left: &DcMotorSimple,
    motor_right: &DcMotorSimple,
    battery: &mut VoltageSagBattery,
    normal_force: &mut ConfiguredNormalForce,
    cmd: ControllerOutput,
    dt_us: u64,
    contacts: &mut crate::models::contact::ContactState,
) -> Result<LastPhysics, String> {
    let dt = dt_us as f64 / 1_000_000.0;
    let m = mechanics(cfg);
    let before = *state;
    check_state(state)?;
    if ![cmd.pwm_left, cmd.pwm_right, cmd.pwm_downforce]
        .iter()
        .all(|x| x.is_finite())
    {
        return Err("controller: non-finite command".into());
    }
    let voltage = battery.terminal_voltage_v();
    // Staggered coupling: use the previous terminal voltage over this interval.
    // Downforce gets first claim; motors reserve equal shares of the remaining bus current.
    let budget = if voltage > 0.0 && battery.output().soc > 0.0 {
        cfg.robot
            .battery
            .current_limit_a
            .min(battery.output().soc * cfg.robot.battery.capacity_mah * 3.6 / dt)
    } else {
        0.0
    };
    let input = NormalForceInput {
        mass_kg: m.mass,
        center_of_mass_m: m.com,
        wheelbase_m: cfg.robot.drivetrain.wheelbase_m,
        track_width_m: m.half_track * 2.0,
        battery_voltage_v: voltage,
        command_pwm: cmd.pwm_downforce,
        speed_m_s: state.vx_body_m_s,
        dt_us,
    };
    let mut candidate = normal_force.clone();
    let mut normal = candidate.step(input);
    if !normal.current_a.is_finite() || !normal.total_normal_n().is_finite() {
        return Err("normal_force: non-finite output".into());
    }
    if normal.current_a > budget {
        let mut lo = 0.0;
        let mut hi = cmd.pwm_downforce.clamp(0.0, 1.0);
        for _ in 0..40 {
            let mid = (lo + hi) * 0.5;
            let mut trial = normal_force.clone();
            if trial
                .step(NormalForceInput {
                    command_pwm: mid,
                    ..input
                })
                .current_a
                <= budget
            {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        candidate = normal_force.clone();
        normal = candidate.step(NormalForceInput {
            command_pwm: lo,
            ..input
        });
    }
    *normal_force = candidate;
    let motor_budget = (budget - normal.current_a).max(0.0) * 0.5;
    let drives = [
        motor_left.drive(cmd.pwm_left, voltage, &cfg.robot.driver, motor_budget),
        motor_right.drive(cmd.pwm_right, voltage, &cfg.robot.driver, motor_budget),
    ];
    if let Some(f) = &cfg.robot.physics {
        return individual_physics(
            state, before, cfg, track, contacts, f, normal, drives, battery, voltage, budget, dt_us,
        );
    }
    let mut limits = [0.; 2];
    let mut lateral_limit = 0.;
    for w in &cfg
        .robot
        .assembly
        .as_ref()
        .ok_or("unresolved assembly")?
        .wheels
    {
        let left = w.position_m.y > 0.;
        let front = w.position_m.x > 0.;
        let n = match (left, front) {
            (true, true) => normal.front_left_n,
            (true, false) => normal.rear_left_n,
            (false, true) => normal.front_right_n,
            (false, false) => normal.rear_right_n,
        };
        let mu = track.surface_mu_at(state.pose.transform_point(w.position_m));
        if !mu.is_finite() || mu < 0. {
            return Err("track: invalid contact friction".into());
        }
        limits[if left { 0 } else { 1 }] += w.tire.mu_longitudinal.min(mu) * n;
        lateral_limit += w.tire.mu_lateral.min(mu) * n;
    }
    let normals = [normal.left_n(), normal.right_n()];
    let rolling = normals.map(|n| cfg.robot.tire.rolling_resistance * n * m.radius);
    let result = solve_contacts(state, m, limits, lateral_limit, rolling, drives, dt)?;
    let motors = [
        drives[0].output(result.motor_torques[0]),
        drives[1].output(result.motor_torques[1]),
    ];
    let load = motors[0].supply_current_a + motors[1].supply_current_a + normal.current_a;
    if !load.is_finite() || load > budget + 1e-8 {
        return Err("battery: current allocation exceeded".into());
    }
    let mut wheels = [WheelForces::default(); 2];
    for i in 0..2 {
        let omega = if i == 0 {
            state.wheel_omega_left_rad_s
        } else {
            state.wheel_omega_right_rad_s
        };
        let y = if i == 0 {
            m.half_track - m.com.y
        } else {
            -m.half_track - m.com.y
        };
        let ground = state.vx_body_m_s - state.yaw_rate_rad_s * y;
        let surface = omega * m.radius;
        wheels[i] = WheelForces {
            force_n: result.forces[i],
            desired_force_n: result.motor_torques[i] / m.radius,
            max_force_n: limits[i],
            wheel_surface_speed_m_s: surface,
            slip_ratio: (surface - ground)
                / surface
                    .abs()
                    .max(ground.abs())
                    .max(cfg.robot.tire.slip_velocity_epsilon_m_s),
            saturated: (surface - ground).abs() > 1e-8,
        };
    }
    let before_energy = energy(&before, m);
    let after_energy = energy(state, m);
    // Backward-Euler work convention, consistent with the implicit motor solve.
    // The residual includes contact loss AND numerical dissipation (0.5 dv^T M dv).
    let motor_work = dt
        * (result.motor_torques[0] * state.wheel_omega_left_rad_s
            + result.motor_torques[1] * state.wheel_omega_right_rad_s);
    let residual = after_energy - before_energy - motor_work;
    if ![after_energy, motor_work, residual]
        .iter()
        .all(|x| x.is_finite())
    {
        return Err("integrator: non-finite energy".into());
    }
    if residual > 1e-8 * (1.0 + before_energy + motor_work.abs()) {
        return Err(format!("contact: energy creation {residual} J"));
    }
    if state.yaw_rate_rad_s.abs() * dt > 0.25 {
        return Err("integrator: rotation exceeds 0.25 rad/tick; reduce physics_dt_us".into());
    }
    integrate_pose(state, before, m.com, dt);
    check_state(state)?;
    let battery_out = battery.step(load, dt_us);
    let diagnostics = PhysicsDiagnostics {
        kinetic_energy_j: after_energy,
        motor_work_j: motor_work,
        dissipation_j: -residual,
        electrical_energy_j: voltage * load * dt,
        lateral_force_n: result.lateral_force,
        rolling_torque_left_nm: result.rolling_torques[0],
        rolling_torque_right_nm: result.rolling_torques[1],
        solver_iterations: result.iterations,
    };
    Ok(LastPhysics {
        motor_left: motors[0],
        motor_right: motors[1],
        wheel_left: wheels[0],
        wheel_right: wheels[1],
        normal,
        battery: battery_out,
        diagnostics,
    })
}

fn validate_outputs(
    sensor: &SensorOutput,
    encoder: &EncoderOutput,
    gyro: &GyroOutput,
    controller: &ControllerOutput,
) -> Result<(), String> {
    if sensor
        .channels
        .iter()
        .any(|r| !r.raw_reflectance.is_finite() || !r.filtered.is_finite())
    {
        return Err("optical pipeline: non-finite value".into());
    }
    for (subsystem, values) in [
        ("sensor", &[sensor.line_position_m, sensor.confidence][..]),
        (
            "encoder",
            &[encoder.left.velocity_rad_s, encoder.right.velocity_rad_s][..],
        ),
        (
            "gyro",
            &[
                gyro.yaw_rate_rad_s,
                gyro.bias_rad_s,
                gyro.acceleration_m_s2.x,
                gyro.acceleration_m_s2.y,
            ][..],
        ),
        (
            "controller",
            &[
                controller.pwm_left,
                controller.pwm_right,
                controller.pwm_downforce,
                controller.error_m,
            ][..],
        ),
    ] {
        if values.iter().any(|v| !v.is_finite()) {
            return Err(format!("{subsystem}: non-finite output"));
        }
    }
    Ok(())
}

fn check_state(s: &RobotState) -> Result<(), String> {
    if ![
        s.pose.x,
        s.pose.y,
        s.pose.yaw,
        s.vx_body_m_s,
        s.vy_body_m_s,
        s.yaw_rate_rad_s,
        s.wheel_omega_left_rad_s,
        s.wheel_omega_right_rad_s,
        s.wheel_angle_left_rad,
        s.wheel_angle_right_rad,
    ]
    .iter()
    .all(|x| x.is_finite())
    {
        return Err("integrator: non-finite state".into());
    }
    Ok(())
}

pub(crate) fn validate_physics(cfg: &LoadedConfig) -> Result<(), String> {
    crate::io::validation::validate_robot_physics(&cfg.robot)?;
    check_state(&RobotState {
        pose: cfg.project.start_pose,
        ..RobotState::default()
    })
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

#[cfg(test)]
mod timing_tests {
    use super::*;

    #[test]
    fn initial_readings_are_the_first_rng_draws_and_metadata_is_valid_json() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("examples/basic/projeto_suction.rtsim");
        let mut cfg = crate::config::load_project(&path).unwrap();
        cfg.project.time.physics_dt_us = 50;
        let track = VectorTrack::new(cfg.track.clone());
        let mut sensor = SimpleLineSensor::from_robot(&cfg.robot);
        let expected_sensor = sensor.sample(&track, cfg.project.start_pose, 0);
        let mut gyro =
            NoisyGyro::with_settings(cfg.robot.gyro.clone(), cfg.robot.sensing.imu.clone());
        let expected_gyro = gyro.sample(0.0, 0);
        let core = SimulationCore::new(cfg, Some(0)).unwrap();
        assert_eq!(core.sample().sensor_adc, expected_sensor.adc);
        assert_eq!(
            core.sample().gyro_yaw_rate_rad_s,
            expected_gyro.yaw_rate_rad_s
        );
        let metadata = crate::json::parse_json(&core.effective_config().to_json()).unwrap();
        assert_eq!(metadata.get("duration_us").unwrap().as_u64(), Some(0));
        assert_eq!(
            metadata
                .get("time")
                .unwrap()
                .get("physics_dt_us")
                .unwrap()
                .as_u64(),
            Some(50)
        );
        assert_eq!(
            metadata.get("terminal_sample").unwrap().as_bool(),
            Some(true)
        );
    }
}

#[cfg(test)]
#[path = "physics_tests.rs"]
mod physics_tests;

#[allow(clippy::too_many_arguments)]
fn individual_physics(
    state: &mut RobotState,
    before: RobotState,
    cfg: &LoadedConfig,
    track: &dyn TrackModel,
    contacts: &mut crate::models::contact::ContactState,
    f: &crate::models::fidelity::FidelityConfig,
    mut normal: NormalForceOutput,
    drives: [crate::motor::MotorDrive; 2],
    battery: &mut VoltageSagBattery,
    voltage: f64,
    budget: f64,
    dt_us: u64,
) -> Result<LastPhysics, String> {
    let dt = dt_us as f64 * 1e-6;
    let robot = &cfg.robot;
    let a = robot.assembly.as_ref().unwrap();
    let mass = a.mass_properties(robot)?;
    let total = normal.total_normal_n();
    let center = if total > 0. {
        normal.load_first_moment_nm * (1. / total)
    } else {
        mass.center_m
    };
    let center = if contacts.quasi_static {
        crate::models::chassis::load_center(
            center,
            mass.mass_kg,
            mass.height_m,
            total,
            contacts.acceleration,
        )
    } else {
        center
    };
    let positions: Vec<_> = a.wheels.iter().map(|w| w.position_m).collect();
    let loads = crate::models::chassis::support_loads(&positions, total, center)?;
    let result = if contacts.kind == crate::models::contact::ContactKind::Ideal {
        contacts.ideal_step(state, robot, &loads, drives, dt)?
    } else {
        contacts.step(state, robot, f, track, &loads, drives, dt)?
    };
    let motors = if contacts.kind == crate::models::contact::ContactKind::Ideal {
        [MotorOutput::default(); 2]
    } else {
        [
            drives[0].output(result.torques[0]),
            drives[1].output(result.torques[1]),
        ]
    };
    let load = motors[0].supply_current_a + motors[1].supply_current_a + normal.current_a;
    if !load.is_finite() || load > budget + 1e-8 {
        return Err("per-wheel power allocation exceeded".into());
    }
    if state.yaw_rate_rad_s.abs() * dt > 0.25 {
        return Err("rotation exceeds 0.25 rad/tick; reduce physics step".into());
    }
    integrate_pose(state, before, mass.center_m, dt);
    let angle = before.pose.yaw - state.pose.yaw;
    let (sin, cos) = angle.sin_cos();
    let acceleration = contacts.acceleration;
    contacts.acceleration = crate::math::Vec2::new(
        cos * acceleration.x - sin * acceleration.y,
        sin * acceleration.x + cos * acceleration.y,
    );
    check_state(state)?;
    normal.front_left_n = 0.;
    normal.front_right_n = 0.;
    normal.rear_left_n = 0.;
    normal.rear_right_n = 0.;
    let mut wheels = [WheelForces::default(); 2];
    let mut rolling = [0.; 2];
    for (w, c) in a.wheels.iter().zip(&contacts.wheels) {
        let side = if w.position_m.y >= 0. { 0 } else { 1 };
        let slot = match (w.position_m.x >= 0., side == 0) {
            (true, true) => &mut normal.front_left_n,
            (true, false) => &mut normal.front_right_n,
            (false, true) => &mut normal.rear_left_n,
            (false, false) => &mut normal.rear_right_n,
        };
        *slot += c.normal_n;
        wheels[side].force_n += c.force_long_n;
        wheels[side].slip_ratio += c.slip;
        rolling[side] += c.rolling_torque_nm;
    }
    for side in 0..2 {
        let group: Vec<_> = a
            .wheels
            .iter()
            .enumerate()
            .filter(|(_, w)| (w.position_m.y >= 0.) == (side == 0))
            .collect();
        if !group.is_empty() {
            wheels[side].slip_ratio /= group.len() as f64;
            wheels[side].wheel_surface_speed_m_s = group
                .iter()
                .map(|(i, w)| contacts.wheels[*i].omega * w.radius_m)
                .sum::<f64>()
                / group.len() as f64;
        }
    }
    for side in 0..2 {
        wheels[side].force_n = result.forces[side];
    }
    let diagnostics = PhysicsDiagnostics {
        kinetic_energy_j: result.energy,
        motor_work_j: result.work,
        dissipation_j: result.dissipation,
        electrical_energy_j: voltage * load * dt,
        lateral_force_n: result.lateral,
        rolling_torque_left_nm: rolling[0],
        rolling_torque_right_nm: rolling[1],
        solver_iterations: contacts.iterations,
    };
    Ok(LastPhysics {
        motor_left: motors[0],
        motor_right: motors[1],
        wheel_left: wheels[0],
        wheel_right: wheels[1],
        normal,
        battery: battery.step(load, dt_us),
        diagnostics,
    })
}

include!("core/power_step.rs");

/// Opaque checkpoint is process-local and cannot be reconstructed from replay samples.
pub struct SimulationCheckpoint {
    core: SimulationCore,
}
impl SimulationCheckpoint {
    pub fn restore(&self) -> SimulationCore {
        self.core
            .checkpoint()
            .expect("checkpoint contains no external firmware")
            .core
    }
}
impl SimulationCore {
    pub fn checkpoint(&self) -> Result<SimulationCheckpoint, String> {
        if self.native_controller.is_some() {
            return Err("native firmware has no checkpoint contract".into());
        }
        Ok(SimulationCheckpoint {
            core: Self {
                power: self.power.clone(),
                cfg: self.cfg.clone(),
                mechanical_cfg: self.mechanical_cfg.clone(),
                contacts: self.contacts.clone(),
                resolved: self.resolved.clone(),
                time: self.time.clone(),
                clock: self.clock.clone(),
                effective_config: self.effective_config.clone(),
                event_counts: self.event_counts.clone(),
                state: self.state.clone(),
                track: self.track.clone(),
                race: self.race.clone(),
                sensor: self.sensor.clone(),
                encoder: self.encoder.clone(),
                gyro: self.gyro.clone(),
                controller: self.controller.clone(),
                replay_controller: self.replay_controller.clone(),
                external_actuators: self.external_actuators.clone(),
                native_controller: None,
                last_imu_velocity: self.last_imu_velocity.clone(),
                motor_left: self.motor_left.clone(),
                motor_right: self.motor_right.clone(),
                normal_force: self.normal_force.clone(),
                battery: self.battery.clone(),
                sensor_output: self.sensor_output.clone(),
                encoder_output: self.encoder_output.clone(),
                gyro_output: self.gyro_output.clone(),
                ctrl_output: self.ctrl_output.clone(),
                control_command: self.control_command.clone(),
                last_physics: self.last_physics.clone(),
                failure: self.failure.clone(),
            },
        })
    }
}
