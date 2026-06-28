use crate::json::{parse_json, JsonValue};
use crate::math::{Pose2, Vec2};
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug)]
pub enum ConfigError {
    Io { path: PathBuf, source: std::io::Error },
    Json { path: PathBuf, message: String },
    Missing { path: PathBuf, field: String },
    Invalid { path: PathBuf, field: String, message: String },
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io { path, source } => write!(f, "failed to read {}: {}", path.display(), source),
            ConfigError::Json { path, message } => write!(f, "invalid JSON in {}: {}", path.display(), message),
            ConfigError::Missing { path, field } => write!(f, "missing field '{}' in {}", field, path.display()),
            ConfigError::Invalid { path, field, message } => write!(f, "invalid field '{}' in {}: {}", field, path.display(), message),
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
}

#[derive(Debug, Clone, Copy)]
pub struct TimeConfig {
    pub physics_dt_us: u64,
    pub controller_period_us: u64,
    pub sensor_period_us: u64,
    pub log_period_us: u64,
    pub render_period_us: u64,
}

impl Default for TimeConfig {
    fn default() -> Self {
        Self {
            physics_dt_us: 500,
            controller_period_us: 1_000,
            sensor_period_us: 500,
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
    pub tire: TireConfig,
    pub motor_left: MotorConfig,
    pub motor_right: MotorConfig,
    pub driver: DriverConfig,
    pub battery: BatteryConfig,
    pub line_sensor: LineSensorConfig,
    pub controller: PidConfig,
}

#[derive(Debug, Clone, Copy)]
pub struct ChassisConfig {
    pub mass_kg: f64,
    pub inertia_kg_m2: f64,
    pub center_of_mass_m: Vec2,
}

#[derive(Debug, Clone, Copy)]
pub struct DrivetrainConfig {
    pub wheel_radius_m: f64,
    pub wheel_width_m: f64,
    pub track_width_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct TireConfig {
    pub mu_longitudinal: f64,
    pub mu_lateral: f64,
    pub rolling_resistance: f64,
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
}

#[derive(Debug, Clone, Copy)]
pub struct BatteryConfig {
    pub nominal_voltage_v: f64,
    pub internal_resistance_ohm: f64,
}

#[derive(Debug, Clone)]
pub struct LineSensorConfig {
    pub model: String,
    pub count: usize,
    pub width_m: f64,
    pub forward_offset_m: f64,
    pub adc_bits: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub base_pwm: f64,
    pub max_pwm: f64,
    pub target_position_m: f64,
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

    Ok(LoadedConfig { project_path, project, robot, track })
}

fn normalize_child_path(base_dir: &Path, child: &Path) -> PathBuf {
    if child.is_absolute() {
        child.to_path_buf()
    } else {
        base_dir.join(child)
    }
}

fn read_json(path: &Path) -> CfgResult<JsonValue> {
    let text = fs::read_to_string(path).map_err(|source| ConfigError::Io { path: path.to_path_buf(), source })?;
    parse_json(&text).map_err(|err| ConfigError::Json { path: path.to_path_buf(), message: err.to_string() })
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

    Ok(ProjectConfig { schema, name, robot_path, track_path, time, duration_s, start_pose, csv_output })
}

fn parse_time(path: &Path, value: Option<&JsonValue>) -> CfgResult<TimeConfig> {
    let defaults = TimeConfig::default();
    let Some(time) = value else { return Ok(defaults); };
    Ok(TimeConfig {
        physics_dt_us: num_field(time, "physics_dt_us", defaults.physics_dt_us as f64) as u64,
        controller_period_us: num_field(time, "controller_period_us", defaults.controller_period_us as f64) as u64,
        sensor_period_us: num_field(time, "sensor_period_us", defaults.sensor_period_us as f64) as u64,
        log_period_us: num_field(time, "log_period_us", defaults.log_period_us as f64) as u64,
        render_period_us: num_field(time, "render_period_us", defaults.render_period_us as f64) as u64,
    })
    .and_then(|t| {
        if t.physics_dt_us == 0 || t.controller_period_us == 0 || t.sensor_period_us == 0 || t.log_period_us == 0 {
            Err(ConfigError::Invalid {
                path: path.to_path_buf(),
                field: "time".to_string(),
                message: "periods must be positive integer microseconds".to_string(),
            })
        } else {
            Ok(t)
        }
    })
}

fn parse_robot_config(path: &Path, root: &JsonValue) -> CfgResult<RobotConfig> {
    let schema = str_field(path, root, "robot_schema", "rtsim-robot-v1")?.to_string();
    let name = str_field(path, root, "name", "unnamed-robot")?.to_string();

    let chassis_json = required_obj(path, root, "chassis")?;
    let drivetrain_json = required_obj(path, root, "drivetrain")?;
    let tire_json = required_obj(path, root, "tire")?;
    let motors_json = required_obj(path, root, "motors")?;
    let driver_json = root.get("driver");
    let battery_json = root.get("battery");
    let sensor_json = required_obj(path, root, "line_sensor")?;
    let controller_json = required_obj(path, root, "controller")?;

    let chassis = ChassisConfig {
        mass_kg: num_field(chassis_json, "mass_g", 180.0) / 1000.0,
        inertia_kg_m2: num_field(chassis_json, "inertia_kg_m2", 0.00045),
        center_of_mass_m: vec2_mm_field(chassis_json, "center_of_mass_mm", Vec2::new(0.0, 0.0)),
    };

    let drivetrain = DrivetrainConfig {
        wheel_radius_m: num_field(drivetrain_json, "wheel_radius_mm", 10.0) / 1000.0,
        wheel_width_m: num_field(drivetrain_json, "wheel_width_mm", 10.0) / 1000.0,
        track_width_m: num_field(drivetrain_json, "track_width_mm", 82.0) / 1000.0,
    };

    let tire = TireConfig {
        mu_longitudinal: num_field(tire_json, "mu_longitudinal", 1.2),
        mu_lateral: num_field(tire_json, "mu_lateral", 1.0),
        rolling_resistance: num_field(tire_json, "rolling_resistance", 0.015),
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
    };

    let battery = BatteryConfig {
        nominal_voltage_v: nested_num(path, battery_json, "nominal_voltage_v", 7.4)?,
        internal_resistance_ohm: nested_num(path, battery_json, "internal_resistance_ohm", 0.08)?,
    };

    let line_sensor = LineSensorConfig {
        model: str_field(path, sensor_json, "model", "SensorArray16")?.to_string(),
        count: num_field(sensor_json, "count", 16.0) as usize,
        width_m: num_field(sensor_json, "width_mm", 72.0) / 1000.0,
        forward_offset_m: num_field(sensor_json, "forward_offset_mm", 55.0) / 1000.0,
        adc_bits: num_field(sensor_json, "adc_bits", 12.0) as u32,
    };

    let controller = PidConfig {
        kp: num_field(controller_json, "kp", 12.0),
        ki: num_field(controller_json, "ki", 0.0),
        kd: num_field(controller_json, "kd", 0.08),
        base_pwm: num_field(controller_json, "base_pwm", 0.35),
        max_pwm: num_field(controller_json, "max_pwm", 0.95),
        target_position_m: num_field(controller_json, "target_position_mm", 0.0) / 1000.0,
    };

    if line_sensor.count < 2 {
        return Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: "line_sensor.count".to_string(),
            message: "must be at least 2".to_string(),
        });
    }

    Ok(RobotConfig { schema, name, chassis, drivetrain, tire, motor_left, motor_right, driver, battery, line_sensor, controller })
}

fn parse_motor(value: &JsonValue) -> MotorConfig {
    MotorConfig {
        model: value.get("model").and_then(JsonValue::as_str).unwrap_or("DcMotorSimple").to_string(),
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

    Ok(TrackConfig { schema, name, model, line_width_m, base_reflectance, line_reflectance, surface_mu, centerline })
}

fn parse_centerline(path: &Path, root: &JsonValue) -> CfgResult<Vec<Vec2>> {
    let arr = root
        .get("centerline_m")
        .and_then(JsonValue::as_array)
        .ok_or_else(|| ConfigError::Missing { path: path.to_path_buf(), field: "centerline_m".to_string() })?;
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
    root.get(field).ok_or_else(|| ConfigError::Missing { path: path.to_path_buf(), field: field.to_string() })
}

fn required_str<'a>(path: &Path, root: &'a JsonValue, field: &str) -> CfgResult<&'a str> {
    root.get(field)
        .and_then(JsonValue::as_str)
        .ok_or_else(|| ConfigError::Missing { path: path.to_path_buf(), field: field.to_string() })
}

fn str_field<'a>(path: &Path, root: &'a JsonValue, field: &str, default: &'a str) -> CfgResult<&'a str> {
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
    root.get(field).and_then(JsonValue::as_f64).unwrap_or(default)
}

fn nested_str<'a>(root: Option<&'a JsonValue>, field: &str, default: &'a str) -> &'a str {
    root.and_then(|v| v.get(field)).and_then(JsonValue::as_str).unwrap_or(default)
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

fn nested_pose(path: &Path, root: Option<&JsonValue>, field: &str, default: Pose2) -> CfgResult<Pose2> {
    let Some(arr) = root.and_then(|v| v.get(field)).and_then(JsonValue::as_array) else {
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
        arr[0].as_f64().ok_or_else(|| ConfigError::Invalid { path: path.to_path_buf(), field: format!("{field}[0]"), message: "expected number".to_string() })?,
        arr[1].as_f64().ok_or_else(|| ConfigError::Invalid { path: path.to_path_buf(), field: format!("{field}[1]"), message: "expected number".to_string() })?,
        arr[2].as_f64().ok_or_else(|| ConfigError::Invalid { path: path.to_path_buf(), field: format!("{field}[2]"), message: "expected number".to_string() })?,
    ))
}
