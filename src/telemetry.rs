use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

#[derive(Debug, Clone, PartialEq)]
pub struct TelemetrySample {
    pub t_us: u64,
    pub x_m: f64,
    pub y_m: f64,
    pub yaw_rad: f64,
    pub vx_body_m_s: f64,
    pub vy_body_m_s: f64,
    pub yaw_rate_rad_s: f64,
    pub line_position_m: f64,
    pub line_error_m: f64,
    pub line_visible: bool,
    pub line_confidence: f64,
    pub pwm_left: f64,
    pub pwm_right: f64,
    pub pwm_downforce: f64,
    pub motor_current_left_a: f64,
    pub motor_current_right_a: f64,
    pub motor_torque_left_nm: f64,
    pub motor_torque_right_nm: f64,
    pub motor_voltage_left_v: f64,
    pub motor_voltage_right_v: f64,
    pub wheel_force_left_n: f64,
    pub wheel_force_right_n: f64,
    pub desired_wheel_force_left_n: f64,
    pub desired_wheel_force_right_n: f64,
    pub slip_left: f64,
    pub slip_right: f64,
    pub wheel_surface_speed_left_m_s: f64,
    pub wheel_surface_speed_right_m_s: f64,
    pub normal_left_n: f64,
    pub normal_right_n: f64,
    pub normal_front_left_n: f64,
    pub normal_front_right_n: f64,
    pub normal_rear_left_n: f64,
    pub normal_rear_right_n: f64,
    pub downforce_extra_n: f64,
    pub downforce_fan_n: f64,
    pub downforce_suction_n: f64,
    pub downforce_current_a: f64,
    pub battery_voltage_v: f64,
    pub battery_current_a: f64,
    pub encoder_left_ticks: i64,
    pub encoder_right_ticks: i64,
    pub encoder_left_velocity_rad_s: f64,
    pub encoder_right_velocity_rad_s: f64,
    pub gyro_yaw_rate_rad_s: f64,
    pub gyro_bias_rad_s: f64,
    pub sensor_adc: Vec<u32>,
}

pub struct CsvLogger {
    writer: BufWriter<File>,
    sensor_count: usize,
}

impl CsvLogger {
    pub fn create(path: &Path, sensor_count: usize) -> std::io::Result<Self> {
        let file = File::create(path)?;
        let mut logger = Self {
            writer: BufWriter::new(file),
            sensor_count,
        };
        logger.write_header()?;
        Ok(logger)
    }

    fn write_header(&mut self) -> std::io::Result<()> {
        write!(
            self.writer,
            "t_us,t_s,x_m,y_m,yaw_rad,vx_body_m_s,vy_body_m_s,yaw_rate_rad_s,line_position_m,line_error_m,line_visible,line_confidence,pwm_left,pwm_right,pwm_downforce,motor_current_left_a,motor_current_right_a,motor_torque_left_nm,motor_torque_right_nm,motor_voltage_left_v,motor_voltage_right_v,wheel_force_left_n,wheel_force_right_n,desired_wheel_force_left_n,desired_wheel_force_right_n,slip_left,slip_right,wheel_surface_speed_left_m_s,wheel_surface_speed_right_m_s,normal_left_n,normal_right_n,normal_front_left_n,normal_front_right_n,normal_rear_left_n,normal_rear_right_n,downforce_extra_n,downforce_fan_n,downforce_suction_n,downforce_current_a,battery_voltage_v,battery_current_a,encoder_left_ticks,encoder_right_ticks,encoder_left_velocity_rad_s,encoder_right_velocity_rad_s,gyro_yaw_rate_rad_s,gyro_bias_rad_s"
        )?;
        for i in 0..self.sensor_count {
            write!(self.writer, ",sensor_{:02}_adc", i)?;
        }
        writeln!(self.writer)?;
        Ok(())
    }

    pub fn write_sample(&mut self, s: &TelemetrySample) -> std::io::Result<()> {
        write!(
            self.writer,
            "{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{:.9},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{:.9},{:.9},{:.9},{:.9}",
            s.t_us,
            s.t_us as f64 / 1_000_000.0,
            s.x_m,
            s.y_m,
            s.yaw_rad,
            s.vx_body_m_s,
            s.vy_body_m_s,
            s.yaw_rate_rad_s,
            s.line_position_m,
            s.line_error_m,
            s.line_visible as u8,
            s.line_confidence,
            s.pwm_left,
            s.pwm_right,
            s.pwm_downforce,
            s.motor_current_left_a,
            s.motor_current_right_a,
            s.motor_torque_left_nm,
            s.motor_torque_right_nm,
            s.motor_voltage_left_v,
            s.motor_voltage_right_v,
            s.wheel_force_left_n,
            s.wheel_force_right_n,
            s.desired_wheel_force_left_n,
            s.desired_wheel_force_right_n,
            s.slip_left,
            s.slip_right,
            s.wheel_surface_speed_left_m_s,
            s.wheel_surface_speed_right_m_s,
            s.normal_left_n,
            s.normal_right_n,
            s.normal_front_left_n,
            s.normal_front_right_n,
            s.normal_rear_left_n,
            s.normal_rear_right_n,
            s.downforce_extra_n,
            s.downforce_fan_n,
            s.downforce_suction_n,
            s.downforce_current_a,
            s.battery_voltage_v,
            s.battery_current_a,
            s.encoder_left_ticks,
            s.encoder_right_ticks,
            s.encoder_left_velocity_rad_s,
            s.encoder_right_velocity_rad_s,
            s.gyro_yaw_rate_rad_s,
            s.gyro_bias_rad_s,
        )?;
        for i in 0..self.sensor_count {
            let value = s.sensor_adc.get(i).copied().unwrap_or(0);
            write!(self.writer, ",{}", value)?;
        }
        writeln!(self.writer)?;
        Ok(())
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

/// Per-interval energy accounting, available through SimulationCore::diagnostics.
/// Kept separate from the v3 replay layout to preserve existing recordings.
#[derive(Debug, Clone, Copy, Default)]
pub struct PhysicsDiagnostics {
    pub kinetic_energy_j: f64,
    pub motor_work_j: f64,
    pub dissipation_j: f64,
    pub electrical_energy_j: f64,
    pub lateral_force_n: f64,
    pub rolling_torque_left_nm: f64,
    pub rolling_torque_right_nm: f64,
    pub solver_iterations: usize,
}

/// Long-form per-contact stream; also accompanies binary replay at the same log ticks.
pub struct ContactLogger {
    writer: BufWriter<File>,
}
impl ContactLogger {
    pub fn create(path: &Path) -> std::io::Result<Self> {
        let mut writer = BufWriter::new(File::create(path)?);
        writeln!(writer,"t_us,wheel_id,omega_rad_s,angle_rad,caster_angle_rad,slip_ratio,slip_angle_rad,force_long_n,force_lat_n,normal_n,rolling_torque_nm,radial_compression_m,regime,solver_iterations")?;
        Ok(Self { writer })
    }
    pub fn write_sample(
        &mut self,
        t_us: u64,
        ids: &[String],
        state: &crate::models::contact::ContactState,
    ) -> std::io::Result<()> {
        for (id, w) in ids.iter().zip(&state.wheels) {
            let id = id.replace('"', "\"\"");
            writeln!(
                self.writer,
                "{},\"{}\",{},{},{},{},{},{},{},{},{},{},{},{}",
                t_us,
                id,
                w.omega,
                w.angle,
                w.caster_angle,
                w.slip,
                w.slip_angle_rad,
                w.force_long_n,
                w.force_lat_n,
                w.normal_n,
                w.rolling_torque_nm,
                w.radial_compression_m,
                w.regime,
                state.iterations
            )?;
        }
        Ok(())
    }
    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

pub struct PowerLogger {
    writer: BufWriter<File>,
}
impl PowerLogger {
    pub fn create(path: &Path) -> std::io::Result<Self> {
        let mut writer = BufWriter::new(File::create(path)?);
        writeln!(writer,"t_us,motor,bus_voltage_v,bus_current_a,soc,polarization_v,motor_current_a,rotor_speed_rad_s,motor_voltage_v,temperature_c,copper_w,bridge_w,gear_w,bearing_w,shaft_w,magnetic_j,numerical_w,dump_w,balance_residual_w,current_limited,circuit_iterations,voltage_residual_v,source_w,battery_loss_w,auxiliary_w,downforce_w,bus_recovered_j,source_returned_j,dumped_j,polarization_j,battery_numerical_w,battery_balance_residual_w")?;
        Ok(Self { writer })
    }
    pub fn write_sample(
        &mut self,
        t: u64,
        p: &crate::models::power::PowerState,
    ) -> std::io::Result<()> {
        for (i, m) in p.motors.iter().enumerate() {
            writeln!(
                self.writer,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                t,
                i,
                p.voltage_v,
                p.current_a,
                p.soc,
                p.polarization_v,
                m.current_a,
                m.rotor_speed_rad_s,
                m.applied_voltage_v,
                m.temperature_c,
                m.copper_loss_w,
                m.bridge_loss_w,
                m.gear_loss_w,
                m.bearing_loss_w,
                m.shaft_power_w,
                m.magnetic_energy_j,
                m.numerical_loss_w,
                m.dump_power_w,
                m.balance_residual_w,
                m.current_limited,
                p.iterations,
                p.residual_v, p.source_power_w, p.battery_loss_w, p.auxiliary_power_w, p.downforce_power_w, p.bus_recovered_energy_j, p.regenerated_energy_j, p.dumped_energy_j, p.polarization_energy_j, p.battery_numerical_loss_w, p.battery_balance_residual_w
            )?;
        }
        Ok(())
    }
    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

/// Full sensing diagnostics are separate from the robot-observable firmware frame.
pub struct SensorLogger {
    writer: BufWriter<File>,
    commands: BufWriter<File>,
    last: Option<crate::control::TimedCommand>,
}
impl SensorLogger {
    pub fn create(base: &Path) -> std::io::Result<Self> {
        let mut p = base.as_os_str().to_os_string();
        p.push(".sensors.jsonl");
        let mut c = base.as_os_str().to_os_string();
        c.push(".commands.csv");
        let mut commands = BufWriter::new(File::create(std::path::PathBuf::from(c))?);
        writeln!(
            commands,
            "t_us,pwm_left,pwm_right,downforce_pwm,mode_left,mode_right"
        )?;
        Ok(Self {
            writer: BufWriter::new(File::create(std::path::PathBuf::from(p))?),
            commands,
            last: None,
        })
    }
    pub fn command(&mut self, c: crate::control::TimedCommand) -> std::io::Result<()> {
        if self.last.is_some_and(|old| {
            old.pwm == c.pwm && old.modes == c.modes && old.downforce_pwm == c.downforce_pwm
        }) {
            return Ok(());
        }
        let mode = |m| match m {
            crate::models::power::ActuatorMode::Drive => "drive",
            crate::models::power::ActuatorMode::Brake => "brake",
            crate::models::power::ActuatorMode::Coast => "coast",
        };
        writeln!(
            self.commands,
            "{},{},{},{},{},{}",
            c.t_us,
            c.pwm[0],
            c.pwm[1],
            c.downforce_pwm,
            mode(c.modes[0]),
            mode(c.modes[1])
        )?;
        self.last = Some(c);
        Ok(())
    }
    pub fn sample(&mut self, core: &crate::sim::SimulationCore) -> std::io::Result<()> {
        use crate::json::JsonValue as J;
        let n = |x: f64| J::Number(x);
        let f = core.controller_input();
        let e = core.estimated_state();
        let channels = core
            .sensor_readings()
            .channels
            .iter()
            .map(|r| {
                J::Object(
                    [
                        ("id".into(), J::String(r.id.clone())),
                        ("acquired_us".into(), n(r.acquired_us as f64)),
                        ("available_us".into(), n(r.available_us as f64)),
                        ("age_us".into(), n(r.age_us as f64)),
                        ("valid".into(), J::Bool(r.valid)),
                        ("raw_reflectance_debug".into(), n(r.raw_reflectance)),
                        ("filtered_debug".into(), n(r.filtered)),
                        ("adc".into(), n(r.adc as f64)),
                        ("digital".into(), r.digital.map(J::Bool).unwrap_or(J::Null)),
                    ]
                    .into_iter()
                    .collect(),
                )
            })
            .collect();
        let value = J::Object(
            [
                ("schema".into(), J::String("rtsim-sensing-v1".into())),
                ("t_us".into(), n(core.time_us() as f64)),
                ("channels".into(), J::Array(channels)),
                ("encoder_acquired_us".into(), n(f.frame.encoder.t_us as f64)),
                (
                    "encoder_available_us".into(),
                    n(f.frame.encoder.available_us as f64),
                ),
                ("encoder_valid".into(), J::Bool(f.frame.encoder.valid)),
                (
                    "encoder_ticks".into(),
                    J::Array(vec![
                        n(f.frame.encoder.left.ticks as f64),
                        n(f.frame.encoder.right.ticks as f64),
                    ]),
                ),
                ("imu_acquired_us".into(), n(f.frame.imu.acquired_us as f64)),
                (
                    "imu_available_us".into(),
                    n(f.frame.imu.available_us as f64),
                ),
                ("imu_valid".into(), J::Bool(f.frame.imu.valid)),
                ("yaw_rate_rad_s".into(), n(f.frame.imu.yaw_rate_rad_s)),
                (
                    "acceleration_m_s2".into(),
                    J::Array(vec![
                        n(f.frame.imu.acceleration_m_s2.x),
                        n(f.frame.imu.acceleration_m_s2.y),
                    ]),
                ),
                (
                    "estimated_pose".into(),
                    J::Array(vec![n(e.pose.x), n(e.pose.y), n(e.pose.yaw)]),
                ),
                ("estimated_distance_m".into(), n(e.distance_m)),
                ("observed_marks".into(), n(e.marks as f64)),
                ("estimated_lap".into(), n(e.lap as f64)),
                ("profile_speed_factor".into(), n(e.speed_factor)),
                (
                    "learned_profile".into(),
                    J::Array(
                        e.profile
                            .iter()
                            .map(|(d, f)| J::Array(vec![n(*d), n(*f)]))
                            .collect(),
                    ),
                ),
            ]
            .into_iter()
            .collect(),
        );
        writeln!(
            self.writer,
            "{}",
            value.to_json().map_err(std::io::Error::other)?
        )
    }
    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()?;
        self.commands.flush()
    }
}
