use crate::json::{parse_json, JsonValue};
use crate::math::{Pose2, Vec2};
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug)]
pub enum ConfigError {
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    Json {
        path: PathBuf,
        message: String,
    },
    Missing {
        path: PathBuf,
        field: String,
    },
    Invalid {
        path: PathBuf,
        field: String,
        message: String,
    },
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io { path, source } => {
                write!(f, "failed to read {}: {}", path.display(), source)
            }
            ConfigError::Json { path, message } => {
                write!(f, "invalid JSON in {}: {}", path.display(), message)
            }
            ConfigError::Missing { path, field } => {
                write!(f, "missing field '{}' in {}", field, path.display())
            }
            ConfigError::Invalid {
                path,
                field,
                message,
            } => write!(
                f,
                "invalid field '{}' in {}: {}",
                field,
                path.display(),
                message
            ),
        }
    }
}

impl std::error::Error for ConfigError {}

type CfgResult<T> = Result<T, ConfigError>;

#[derive(Debug, Clone)]
pub struct LoadedConfig {
    pub project_path: PathBuf,
    pub project: ProjectConfig,
    pub robot: RobotConfig,
    pub track: TrackConfig,
}

#[derive(Debug, Clone)]
pub struct ProjectConfig {
    pub schema: String,
    pub name: String,
    pub robot_path: PathBuf,
    pub track_path: PathBuf,
    pub time: TimeConfig,
    pub duration_s: f64,
    pub start_pose: Pose2,
    pub csv_output: Option<PathBuf>,
    pub replay_output: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy)]
pub struct TimeConfig {
    pub physics_dt_us: u64,
    pub controller_period_us: u64,
    pub sensor_period_us: u64,
    pub imu_period_us: u64,
    pub encoder_period_us: u64,
    pub log_period_us: u64,
    pub render_period_us: u64,
}

impl Default for TimeConfig {
    fn default() -> Self {
        Self {
            physics_dt_us: 500,
            controller_period_us: 1_000,
            sensor_period_us: 500,
            imu_period_us: 500,
            encoder_period_us: 500,
            log_period_us: 1_000,
            render_period_us: 16_667,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RobotConfig {
    pub schema: String,
    pub name: String,
    pub chassis: ChassisConfig,
    pub drivetrain: DrivetrainConfig,
    pub normal_force: NormalForceConfig,
    pub tire: TireConfig,
    pub motor_left: MotorConfig,
    pub motor_right: MotorConfig,
    pub driver: DriverConfig,
    pub battery: BatteryConfig,
    pub line_sensor: LineSensorConfig,
    pub encoder: EncoderConfig,
    pub gyro: GyroConfig,
    pub controller: PidConfig,
}

#[derive(Debug, Clone, Copy)]
pub struct ChassisConfig {
    pub mass_kg: f64,
    pub inertia_kg_m2: f64,
    pub center_of_mass_m: Vec2,
    pub length_m: f64,
    pub width_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct DrivetrainConfig {
    pub wheel_radius_m: f64,
    pub wheel_width_m: f64,
    pub track_width_m: f64,
    pub wheelbase_m: f64,
    pub wheel_inertia_kg_m2: f64,
}

#[derive(Debug, Clone)]
pub struct FanConfig {
    pub position_m: Vec2,
    pub max_force_n: f64,
    pub max_current_a: f64,
    pub nominal_voltage_v: f64,
    pub response_time_s: f64,
    pub pwm_scale: f64,
    pub enabled_pwm: f64,
    pub force_curve: Vec<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct NormalForceConfig {
    pub model: String,
    pub command_pwm_default: f64,
    pub position_m: Vec2,
    pub max_force_n: f64,
    pub max_current_a: f64,
    pub response_time_s: f64,
    pub chamber_area_m2: f64,
    pub max_delta_pressure_pa: f64,
    pub leakage_factor: f64,
    pub speed_sensitivity: f64,
    pub force_curve: Vec<(f64, f64)>,
    pub fans: Vec<FanConfig>,
}

#[derive(Debug, Clone)]
pub struct TireConfig {
    pub model: String,
    pub mu_longitudinal: f64,
    pub mu_lateral: f64,
    pub rolling_resistance: f64,
    pub slip_velocity_epsilon_m_s: f64,
}

#[derive(Debug, Clone)]
pub struct MotorConfig {
    pub model: String,
    pub gear_ratio: f64,
    pub efficiency: f64,
    pub no_load_rpm: f64,
    pub stall_torque_nm: f64,
    pub stall_current_a: f64,
}

#[derive(Debug, Clone)]
pub struct DriverConfig {
    pub model: String,
    pub pwm_frequency_hz: f64,
    pub mode: String,
    pub voltage_drop_v: f64,
    pub pwm_resolution_bits: u32,
    pub command_deadband: f64,
    pub current_limit_a: f64,
}

#[derive(Debug, Clone)]
pub struct BatteryConfig {
    pub model: String,
    pub cells: u32,
    pub nominal_voltage_v: f64,
    pub full_voltage_v: f64,
    pub empty_voltage_v: f64,
    pub capacity_mah: f64,
    pub internal_resistance_ohm: f64,
    pub initial_soc: f64,
    pub current_limit_a: f64,
}

#[derive(Debug, Clone)]
pub struct LineSensorConfig {
    pub model: String,
    pub count: usize,
    pub width_m: f64,
    pub forward_offset_m: f64,
    pub adc_bits: u32,
    pub gain: f64,
    pub offset: f64,
    pub reflectance_noise_std: f64,
    pub adc_noise_lsb: f64,
    pub seed: u64,
}

#[derive(Debug, Clone)]
pub struct EncoderConfig {
    pub model: String,
    pub ticks_per_rev: u32,
    pub invert_left: bool,
    pub invert_right: bool,
}

#[derive(Debug, Clone)]
pub struct GyroConfig {
    pub model: String,
    pub noise_std_rad_s: f64,
    pub bias_rad_s: f64,
    pub saturation_rad_s: f64,
    pub seed: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub base_pwm: f64,
    pub max_pwm: f64,
    pub target_position_m: f64,
    pub downforce_pwm: f64,
}

#[derive(Debug, Clone)]
pub struct TrackConfig {
    pub schema: String,
    pub name: String,
    pub model: String,
    pub line_width_m: f64,
    pub base_reflectance: f64,
    pub line_reflectance: f64,
    pub surface_mu: f64,
    pub centerline: Vec<Vec2>,
}

pub fn load_project(project_path: impl AsRef<Path>) -> CfgResult<LoadedConfig> {
    let project_path = project_path.as_ref().to_path_buf();
    let project_json = read_json(&project_path)?;
    let project = parse_project_config(&project_path, &project_json)?;
    let base_dir = project_path.parent().unwrap_or_else(|| Path::new("."));

    let robot_path = normalize_child_path(base_dir, &project.robot_path);
    let track_path = normalize_child_path(base_dir, &project.track_path);

    let robot_json = read_json(&robot_path)?;
    let track_json = read_json(&track_path)?;
    let robot = parse_robot_config(&robot_path, &robot_json)?;
    let track = parse_track_config(&track_path, &track_json)?;

    Ok(LoadedConfig {
        project_path,
        project,
        robot,
        track,
    })
}

fn normalize_child_path(base_dir: &Path, child: &Path) -> PathBuf {
    if child.is_absolute() {
        child.to_path_buf()
    } else {
        base_dir.join(child)
    }
}

fn read_json(path: &Path) -> CfgResult<JsonValue> {
    let text = fs::read_to_string(path).map_err(|source| ConfigError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    parse_json(&text).map_err(|err| ConfigError::Json {
        path: path.to_path_buf(),
        message: err.to_string(),
    })
}

fn parse_project_config(path: &Path, root: &JsonValue) -> CfgResult<ProjectConfig> {
    let schema = str_field(path, root, "rtsim_schema", "rtsim-project-v1")?.to_string();
    let name = str_field(path, root, "name", "unnamed-project")?.to_string();
    let robot_path = PathBuf::from(required_str(path, root, "robot")?);
    let track_path = PathBuf::from(required_str(path, root, "track")?);
    let time = parse_time(path, root.get("time"))?;

    let sim = root.get("simulation");
    let duration_s = nested_num(path, sim, "duration_s", 10.0)?;
    let start_pose = nested_pose(path, sim, "start_pose_m", Pose2::new(0.0, 0.0, 0.0))?;

    let csv_output = root
        .get("log")
        .and_then(|v| v.get("csv"))
        .and_then(JsonValue::as_str)
        .map(PathBuf::from);
    let replay_output = root
        .get("log")
        .and_then(|v| v.get("replay"))
        .and_then(JsonValue::as_str)
        .map(PathBuf::from);

    Ok(ProjectConfig {
        schema,
        name,
        robot_path,
        track_path,
        time,
        duration_s,
        start_pose,
        csv_output,
        replay_output,
    })
}

fn parse_time(path: &Path, value: Option<&JsonValue>) -> CfgResult<TimeConfig> {
    let defaults = TimeConfig::default();
    let Some(time) = value else {
        return Ok(defaults);
    };
    let parsed = TimeConfig {
        physics_dt_us: num_field(time, "physics_dt_us", defaults.physics_dt_us as f64) as u64,
        controller_period_us: num_field(
            time,
            "controller_period_us",
            defaults.controller_period_us as f64,
        ) as u64,
        sensor_period_us: num_field(time, "sensor_period_us", defaults.sensor_period_us as f64)
            as u64,
        imu_period_us: num_field(time, "imu_period_us", defaults.imu_period_us as f64) as u64,
        encoder_period_us: num_field(time, "encoder_period_us", defaults.encoder_period_us as f64)
            as u64,
        log_period_us: num_field(time, "log_period_us", defaults.log_period_us as f64) as u64,
        render_period_us: num_field(time, "render_period_us", defaults.render_period_us as f64)
            as u64,
    };
    if parsed.physics_dt_us == 0
        || parsed.controller_period_us == 0
        || parsed.sensor_period_us == 0
        || parsed.imu_period_us == 0
        || parsed.encoder_period_us == 0
        || parsed.log_period_us == 0
    {
        Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: "time".to_string(),
            message: "periods must be positive integer microseconds".to_string(),
        })
    } else {
        Ok(parsed)
    }
}

fn parse_robot_config(path: &Path, root: &JsonValue) -> CfgResult<RobotConfig> {
    let schema = str_field(path, root, "robot_schema", "rtsim-robot-v2")?.to_string();
    let name = str_field(path, root, "name", "unnamed-robot")?.to_string();

    let chassis_json = required_obj(path, root, "chassis")?;
    let drivetrain_json = required_obj(path, root, "drivetrain")?;
    let normal_force_json = root.get("normal_force");
    let tire_json = required_obj(path, root, "tire")?;
    let motors_json = required_obj(path, root, "motors")?;
    let driver_json = root.get("driver");
    let battery_json = root.get("battery");
    let sensor_json = required_obj(path, root, "line_sensor")?;
    let encoder_json = root.get("encoder");
    let gyro_json = root.get("gyro");
    let controller_json = required_obj(path, root, "controller")?;

    let chassis = ChassisConfig {
        mass_kg: num_field(chassis_json, "mass_g", 180.0) / 1000.0,
        inertia_kg_m2: num_field(chassis_json, "inertia_kg_m2", 0.00045),
        center_of_mass_m: vec2_mm_field(chassis_json, "center_of_mass_mm", Vec2::new(0.0, 0.0)),
        length_m: num_field(chassis_json, "length_mm", 120.0) / 1000.0,
        width_m: num_field(chassis_json, "width_mm", 90.0) / 1000.0,
    };

    let drivetrain = DrivetrainConfig {
        wheel_radius_m: num_field(drivetrain_json, "wheel_radius_mm", 10.0) / 1000.0,
        wheel_width_m: num_field(drivetrain_json, "wheel_width_mm", 10.0) / 1000.0,
        track_width_m: num_field(drivetrain_json, "track_width_mm", 82.0) / 1000.0,
        wheelbase_m: num_field(
            drivetrain_json,
            "wheelbase_mm",
            num_field(chassis_json, "length_mm", 120.0) * 0.70,
        ) / 1000.0,
        wheel_inertia_kg_m2: num_field(drivetrain_json, "wheel_inertia_g_cm2", 1.0) * 1e-7,
    };

    let normal_force = parse_normal_force(path, normal_force_json)?;

    let tire = TireConfig {
        model: str_field(path, tire_json, "model", "SlipRatioWheel")?.to_string(),
        mu_longitudinal: num_field(tire_json, "mu_longitudinal", 1.2),
        mu_lateral: num_field(tire_json, "mu_lateral", 1.0),
        rolling_resistance: num_field(tire_json, "rolling_resistance", 0.015),
        slip_velocity_epsilon_m_s: num_field(tire_json, "slip_velocity_epsilon_m_s", 0.05),
    };

    let left_json = required_obj(path, motors_json, "left")?;
    let right_json = required_obj(path, motors_json, "right")?;
    let motor_left = parse_motor(left_json);
    let motor_right = parse_motor(right_json);

    let driver = DriverConfig {
        model: nested_str(driver_json, "model", "PwmHBridge").to_string(),
        pwm_frequency_hz: nested_num(path, driver_json, "pwm_frequency_hz", 20_000.0)?,
        mode: nested_str(driver_json, "mode", "brake").to_string(),
        voltage_drop_v: nested_num(path, driver_json, "voltage_drop_v", 0.2)?,
        pwm_resolution_bits: nested_num(path, driver_json, "pwm_resolution_bits", 10.0)? as u32,
        command_deadband: nested_num(path, driver_json, "command_deadband", 0.001)?,
        current_limit_a: nested_num(path, driver_json, "current_limit_a", 1000.0)?,
    };

    let cells = nested_num(path, battery_json, "cells", 2.0)? as u32;
    let nominal_voltage_v = nested_num(path, battery_json, "nominal_voltage_v", 7.4)?;
    let battery = BatteryConfig {
        model: nested_str(battery_json, "model", "VoltageSagBattery").to_string(),
        cells,
        nominal_voltage_v,
        full_voltage_v: nested_num(path, battery_json, "full_voltage_v", nominal_voltage_v)?,
        empty_voltage_v: nested_num(
            path,
            battery_json,
            "empty_voltage_v",
            3.2 * cells.max(1) as f64,
        )?,
        capacity_mah: nested_num(path, battery_json, "capacity_mah", 300.0)?,
        internal_resistance_ohm: nested_num(path, battery_json, "internal_resistance_ohm", 0.08)?,
        initial_soc: nested_num(path, battery_json, "initial_soc", 1.0)?,
        current_limit_a: nested_num(path, battery_json, "current_limit_a", 200.0)?,
    };

    let line_sensor = LineSensorConfig {
        model: str_field(path, sensor_json, "model", "NoisyAdcSensor")?.to_string(),
        count: num_field(sensor_json, "count", 16.0) as usize,
        width_m: num_field(sensor_json, "width_mm", 72.0) / 1000.0,
        forward_offset_m: num_field(sensor_json, "forward_offset_mm", 55.0) / 1000.0,
        adc_bits: num_field(sensor_json, "adc_bits", 12.0) as u32,
        gain: num_field(sensor_json, "gain", 1.0),
        offset: num_field(sensor_json, "offset", 0.0),
        reflectance_noise_std: num_field(sensor_json, "reflectance_noise_std", 0.01),
        adc_noise_lsb: num_field(sensor_json, "adc_noise_lsb", 1.0),
        seed: num_field(sensor_json, "seed", 0x51A5_0001 as f64) as u64,
    };

    let encoder = EncoderConfig {
        model: nested_str(encoder_json, "model", "QuantizedEncoder").to_string(),
        ticks_per_rev: nested_num(path, encoder_json, "ticks_per_rev", 360.0)? as u32,
        invert_left: nested_bool(encoder_json, "invert_left", false),
        invert_right: nested_bool(encoder_json, "invert_right", false),
    };

    let gyro = GyroConfig {
        model: nested_str(gyro_json, "model", "NoisyGyro").to_string(),
        noise_std_rad_s: nested_num(path, gyro_json, "noise_std_rad_s", 0.01)?,
        bias_rad_s: nested_num(path, gyro_json, "bias_rad_s", 0.0)?,
        saturation_rad_s: nested_num(path, gyro_json, "saturation_rad_s", 34.906585)?,
        seed: nested_num(path, gyro_json, "seed", 0x9A17_0002u64 as f64)? as u64,
    };

    let controller = PidConfig {
        kp: num_field(controller_json, "kp", 12.0),
        ki: num_field(controller_json, "ki", 0.0),
        kd: num_field(controller_json, "kd", 0.08),
        base_pwm: num_field(controller_json, "base_pwm", 0.35),
        max_pwm: num_field(controller_json, "max_pwm", 0.95),
        target_position_m: num_field(controller_json, "target_position_mm", 0.0) / 1000.0,
        downforce_pwm: num_field(
            controller_json,
            "downforce_pwm",
            normal_force.command_pwm_default,
        ),
    };

    if line_sensor.count < 2 {
        return Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: "line_sensor.count".to_string(),
            message: "must be at least 2".to_string(),
        });
    }
    if line_sensor.adc_bits == 0 || line_sensor.adc_bits > 24 {
        return Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: "line_sensor.adc_bits".to_string(),
            message: "must be between 1 and 24".to_string(),
        });
    }
    if encoder.ticks_per_rev == 0 {
        return Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: "encoder.ticks_per_rev".to_string(),
            message: "must be > 0".to_string(),
        });
    }

    Ok(RobotConfig {
        schema,
        name,
        chassis,
        drivetrain,
        normal_force,
        tire,
        motor_left,
        motor_right,
        driver,
        battery,
        line_sensor,
        encoder,
        gyro,
        controller,
    })
}

fn parse_normal_force(path: &Path, root: Option<&JsonValue>) -> CfgResult<NormalForceConfig> {
    let model = nested_str(root, "model", "NoDownforce").to_string();
    let default_pwm = if model.eq_ignore_ascii_case("NoDownforce") {
        0.0
    } else {
        1.0
    };
    let command_pwm_default = nested_num(
        path,
        root,
        "default_pwm",
        nested_num(path, root, "pwm", default_pwm)?,
    )?;
    let max_force_n = nested_num(path, root, "max_force_n", 0.0)?;
    let max_current_a = nested_num(path, root, "max_current_a", 0.0)?;
    let response_time_s = nested_num(path, root, "response_time_s", 0.0)?;
    let chamber_area_m2 = nested_num(path, root, "chamber_area_m2", 0.0)?;
    let max_delta_pressure_pa = nested_num(path, root, "max_delta_pressure_pa", 0.0)?;
    let leakage_factor = nested_num(path, root, "leakage_factor", 0.0)?;
    let speed_sensitivity = nested_num(path, root, "speed_sensitivity", 0.0)?;
    let position_m = root
        .map(|v| vec2_mm_field(v, "position_mm", Vec2::new(0.0, 0.0)))
        .unwrap_or(Vec2::new(0.0, 0.0));
    let force_curve = root
        .and_then(|v| v.get("force_curve").or_else(|| v.get("measured_curve")))
        .map(|v| parse_curve(path, v, "normal_force.force_curve"))
        .transpose()?
        .unwrap_or_default();
    let fans = root
        .and_then(|v| v.get("fans"))
        .map(|v| parse_fans(path, v))
        .transpose()?
        .unwrap_or_default();

    Ok(NormalForceConfig {
        model,
        command_pwm_default,
        position_m,
        max_force_n,
        max_current_a,
        response_time_s,
        chamber_area_m2,
        max_delta_pressure_pa,
        leakage_factor,
        speed_sensitivity,
        force_curve,
        fans,
    })
}

fn parse_fans(path: &Path, value: &JsonValue) -> CfgResult<Vec<FanConfig>> {
    let arr = value.as_array().ok_or_else(|| ConfigError::Invalid {
        path: path.to_path_buf(),
        field: "normal_force.fans".to_string(),
        message: "expected array".to_string(),
    })?;
    let mut fans = Vec::with_capacity(arr.len());
    for (i, fan) in arr.iter().enumerate() {
        let position_m = vec2_mm_field(fan, "position_mm", Vec2::new(0.0, 0.0));
        let force_curve = fan
            .get("force_curve")
            .or_else(|| fan.get("thrust_curve"))
            .or_else(|| fan.get("measured_curve"))
            .map(|v| parse_curve(path, v, &format!("normal_force.fans[{i}].force_curve")))
            .transpose()?
            .unwrap_or_default();
        fans.push(FanConfig {
            position_m,
            max_force_n: num_field(fan, "max_force_n", 0.0),
            max_current_a: num_field(fan, "max_current_a", 0.0),
            nominal_voltage_v: num_field(fan, "nominal_voltage_v", 7.4),
            response_time_s: num_field(fan, "response_time_s", 0.0),
            pwm_scale: num_field(fan, "pwm_scale", 1.0),
            enabled_pwm: num_field(fan, "pwm", 1.0),
            force_curve,
        });
    }
    Ok(fans)
}

fn parse_curve(path: &Path, value: &JsonValue, field: &str) -> CfgResult<Vec<(f64, f64)>> {
    let arr = value.as_array().ok_or_else(|| ConfigError::Invalid {
        path: path.to_path_buf(),
        field: field.to_string(),
        message: "expected array of [pwm, force_n] points".to_string(),
    })?;
    let mut curve = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        let pair = item.as_array().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[{i}]"),
            message: "expected [pwm, force_n]".to_string(),
        })?;
        if pair.len() != 2 {
            return Err(ConfigError::Invalid {
                path: path.to_path_buf(),
                field: format!("{field}[{i}]"),
                message: "expected [pwm, force_n]".to_string(),
            });
        }
        let x = pair[0].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[{i}][0]"),
            message: "expected number".to_string(),
        })?;
        let y = pair[1].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[{i}][1]"),
            message: "expected number".to_string(),
        })?;
        curve.push((x, y));
    }
    curve.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    Ok(curve)
}

fn parse_motor(value: &JsonValue) -> MotorConfig {
    MotorConfig {
        model: value
            .get("model")
            .and_then(JsonValue::as_str)
            .unwrap_or("DcMotorSimple")
            .to_string(),
        gear_ratio: num_field(value, "gear_ratio", 30.0),
        efficiency: num_field(value, "efficiency", 0.75),
        no_load_rpm: num_field(value, "no_load_rpm", 1_800.0),
        stall_torque_nm: num_field(value, "stall_torque_mnm", 5.0) / 1000.0,
        stall_current_a: num_field(value, "stall_current_a", 1.6),
    }
}

fn parse_track_config(path: &Path, root: &JsonValue) -> CfgResult<TrackConfig> {
    let schema = str_field(path, root, "track_schema", "rtsim-track-v1")?.to_string();
    let name = str_field(path, root, "name", "unnamed-track")?.to_string();
    let model = str_field(path, root, "model", "VectorTrack")?.to_string();
    let line_width_m = num_field(root, "line_width_mm", 19.0) / 1000.0;
    let base_reflectance = num_field(root, "base_reflectance", 0.86);
    let line_reflectance = num_field(root, "line_reflectance", 0.08);
    let surface_mu = num_field(root, "surface_mu", 1.2);
    let centerline = parse_centerline(path, root)?;

    Ok(TrackConfig {
        schema,
        name,
        model,
        line_width_m,
        base_reflectance,
        line_reflectance,
        surface_mu,
        centerline,
    })
}

fn parse_centerline(path: &Path, root: &JsonValue) -> CfgResult<Vec<Vec2>> {
    let arr = root
        .get("centerline_m")
        .and_then(JsonValue::as_array)
        .ok_or_else(|| ConfigError::Missing {
            path: path.to_path_buf(),
            field: "centerline_m".to_string(),
        })?;
    let mut points = Vec::with_capacity(arr.len());
    for (i, value) in arr.iter().enumerate() {
        let pair = value.as_array().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("centerline_m[{i}]"),
            message: "expected [x, y]".to_string(),
        })?;
        if pair.len() != 2 {
            return Err(ConfigError::Invalid {
                path: path.to_path_buf(),
                field: format!("centerline_m[{i}]"),
                message: "expected exactly two numbers".to_string(),
            });
        }
        let x = pair[0].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("centerline_m[{i}][0]"),
            message: "expected number".to_string(),
        })?;
        let y = pair[1].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("centerline_m[{i}][1]"),
            message: "expected number".to_string(),
        })?;
        points.push(Vec2::new(x, y));
    }
    if points.len() < 2 {
        return Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: "centerline_m".to_string(),
            message: "track needs at least two centerline points".to_string(),
        });
    }
    Ok(points)
}

fn required_obj<'a>(path: &Path, root: &'a JsonValue, field: &str) -> CfgResult<&'a JsonValue> {
    root.get(field).ok_or_else(|| ConfigError::Missing {
        path: path.to_path_buf(),
        field: field.to_string(),
    })
}

fn required_str<'a>(path: &Path, root: &'a JsonValue, field: &str) -> CfgResult<&'a str> {
    root.get(field)
        .and_then(JsonValue::as_str)
        .ok_or_else(|| ConfigError::Missing {
            path: path.to_path_buf(),
            field: field.to_string(),
        })
}

fn str_field<'a>(
    path: &Path,
    root: &'a JsonValue,
    field: &str,
    default: &'a str,
) -> CfgResult<&'a str> {
    match root.get(field) {
        Some(v) => v.as_str().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: "expected string".to_string(),
        }),
        None => Ok(default),
    }
}

fn num_field(root: &JsonValue, field: &str, default: f64) -> f64 {
    root.get(field)
        .and_then(JsonValue::as_f64)
        .unwrap_or(default)
}

fn nested_str<'a>(root: Option<&'a JsonValue>, field: &str, default: &'a str) -> &'a str {
    root.and_then(|v| v.get(field))
        .and_then(JsonValue::as_str)
        .unwrap_or(default)
}

fn nested_num(path: &Path, root: Option<&JsonValue>, field: &str, default: f64) -> CfgResult<f64> {
    match root.and_then(|v| v.get(field)) {
        Some(v) => v.as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: "expected number".to_string(),
        }),
        None => Ok(default),
    }
}

fn nested_bool(root: Option<&JsonValue>, field: &str, default: bool) -> bool {
    root.and_then(|v| v.get(field))
        .and_then(JsonValue::as_bool)
        .unwrap_or(default)
}

fn vec2_mm_field(root: &JsonValue, field: &str, default: Vec2) -> Vec2 {
    root.get(field)
        .and_then(JsonValue::as_array)
        .and_then(|a| {
            if a.len() == 2 {
                Some(Vec2::new(a[0].as_f64()? / 1000.0, a[1].as_f64()? / 1000.0))
            } else {
                None
            }
        })
        .unwrap_or(default)
}

fn nested_pose(
    path: &Path,
    root: Option<&JsonValue>,
    field: &str,
    default: Pose2,
) -> CfgResult<Pose2> {
    let Some(arr) = root
        .and_then(|v| v.get(field))
        .and_then(JsonValue::as_array)
    else {
        return Ok(default);
    };
    if arr.len() != 3 {
        return Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: "expected [x_m, y_m, yaw_rad]".to_string(),
        });
    }
    Ok(Pose2::new(
        arr[0].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[0]"),
            message: "expected number".to_string(),
        })?,
        arr[1].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[1]"),
            message: "expected number".to_string(),
        })?,
        arr[2].as_f64().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[2]"),
            message: "expected number".to_string(),
        })?,
    ))
}
