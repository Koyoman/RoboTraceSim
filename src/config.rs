use crate::json::{parse_json, JsonValue};
use crate::math::{Pose2, Vec2};
use crate::rtsim_track::{
    build_geometry, resolve_robot_start_pose, resolve_rules, ArcSegment, RobotStartConfig,
    StartExitDirection, StartFinishMarking, StraightSegment, TrackArea, TrackClosureConfig,
    TrackCornerMarkersConfig, TrackMarkings, TrackPose, TrackRuleOverrides, TrackRulesConfig,
    TrackRulesMode, TrackSegment, TrackSurfaceConfig, TrackV2,
};
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
    pub sensors: Vec<RobotSensorInstance>,
    pub line_validity_areas: Vec<RobotLineValidityArea>,
    pub encoder: EncoderConfig,
    pub gyro: GyroConfig,
    pub controller: PidConfig,
}

/// Rectangle attached to the robot that may overlap the course line.
/// A robot remains valid while at least one enabled rectangle overlaps the line.
#[derive(Debug, Clone)]
pub struct RobotLineValidityArea {
    pub name: String,
    pub position_m: Vec2,
    pub length_m: f64,
    pub width_m: f64,
    pub angle_deg: f64,
    pub enabled: bool,
}

impl RobotLineValidityArea {
    pub fn overlaps_line_segment(
        &self,
        robot_pose: Pose2,
        line_start_m: Vec2,
        line_end_m: Vec2,
        line_width_m: f64,
    ) -> bool {
        if !self.enabled {
            return false;
        }
        let to_area_local = |point: Vec2| {
            let dx = point.x - robot_pose.x;
            let dy = point.y - robot_pose.y;
            let robot_c = robot_pose.yaw.cos();
            let robot_s = robot_pose.yaw.sin();
            let robot_x = dx * robot_c + dy * robot_s;
            let robot_y = -dx * robot_s + dy * robot_c;
            let area_x = robot_x - self.position_m.x;
            let area_y = robot_y - self.position_m.y;
            let angle = self.angle_deg.to_radians();
            let (s, c) = angle.sin_cos();
            Vec2::new(area_x * c + area_y * s, -area_x * s + area_y * c)
        };
        let start = to_area_local(line_start_m);
        let end = to_area_local(line_end_m);
        let line_radius = line_width_m.max(0.0) * 0.5;
        let half_x = self.length_m.max(0.0) * 0.5 + line_radius;
        let half_y = self.width_m.max(0.0) * 0.5 + line_radius;
        segment_intersects_axis_aligned_rect(start, end, half_x, half_y)
    }
}

fn segment_intersects_axis_aligned_rect(start: Vec2, end: Vec2, half_x: f64, half_y: f64) -> bool {
    let delta = end - start;
    let mut t_min: f64 = 0.0;
    let mut t_max: f64 = 1.0;
    for (origin, direction, half_extent) in [(start.x, delta.x, half_x), (start.y, delta.y, half_y)]
    {
        if direction.abs() < 1e-12 {
            if origin < -half_extent || origin > half_extent {
                return false;
            }
            continue;
        }
        let t1 = (-half_extent - origin) / direction;
        let t2 = (half_extent - origin) / direction;
        t_min = t_min.max(t1.min(t2));
        t_max = t_max.min(t1.max(t2));
        if t_min > t_max {
            return false;
        }
    }
    true
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
    pub visual_radius_m: f64,
    pub action_radius_m: f64,
    pub max_force_n: f64,
    pub max_current_a: f64,
    pub nominal_voltage_v: f64,
    pub nominal_current_a: f64,
    pub power_w: f64,
    pub min_pwm: f64,
    pub max_pwm: f64,
    pub response_time_s: f64,
    pub pwm_scale: f64,
    pub enabled_pwm: f64,
    pub curve_model: FanCurveModel,
    pub force_curve: Vec<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct NormalForceConfig {
    pub model: String,
    pub model_kind: DownforceModel,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorType {
    LineAnalog,
    LineDigital,
    DistanceInfrared,
    DistanceToF,
    Ultrasonic,
    Color,
    Encoder,
    Gyro,
    Accelerometer,
    Custom,
}

impl SensorType {
    pub fn as_str(self) -> &'static str {
        match self {
            SensorType::LineAnalog => "LineAnalog",
            SensorType::LineDigital => "LineDigital",
            SensorType::DistanceInfrared => "DistanceInfrared",
            SensorType::DistanceToF => "DistanceToF",
            SensorType::Ultrasonic => "Ultrasonic",
            SensorType::Color => "Color",
            SensorType::Encoder => "Encoder",
            SensorType::Gyro => "Gyro",
            SensorType::Accelerometer => "Accelerometer",
            SensorType::Custom => "Custom",
        }
    }

    pub fn from_str(value: &str) -> Self {
        match value.to_ascii_lowercase().as_str() {
            "lineanalog" | "line_analog" | "analogline" | "analog_line" => SensorType::LineAnalog,
            "linedigital" | "line_digital" | "digitalline" | "digital_line" => {
                SensorType::LineDigital
            }
            "distanceinfrared" | "distance_ir" | "infrared" | "ir" => SensorType::DistanceInfrared,
            "distancetof" | "tof" | "timeofflight" | "time_of_flight" => SensorType::DistanceToF,
            "ultrasonic" => SensorType::Ultrasonic,
            "color" => SensorType::Color,
            "encoder" => SensorType::Encoder,
            "gyro" | "gyroscope" => SensorType::Gyro,
            "accelerometer" | "accel" => SensorType::Accelerometer,
            _ => SensorType::Custom,
        }
    }
}

#[derive(Debug, Clone)]
pub enum SensorDetectionArea {
    Point { radius_m: f64 },
    Rectangle { width_m: f64, height_m: f64 },
    Circle { radius_m: f64 },
    Cone { range_m: f64, angle_deg: f64 },
    CustomPolygon { points_m: Vec<Vec2> },
}

#[derive(Debug, Clone)]
pub enum SensorResponseModel {
    Ideal,
    Threshold { threshold: f64 },
    Linear { gain: f64, offset: f64 },
    Polynomial { coefficients: Vec<f64> },
    LookupTable { points: Vec<SensorResponsePoint> },
    Custom { description: String },
}

#[derive(Debug, Clone, Copy)]
pub struct SensorResponsePoint {
    pub input: f64,
    pub output: f64,
}

#[derive(Debug, Clone)]
pub struct SensorAsset {
    pub name: String,
    pub model: String,
    pub sensor_type: SensorType,
    pub visual_width_m: f64,
    pub visual_height_m: f64,
    pub visual_radius_m: f64,
    pub detection_area: SensorDetectionArea,
    pub response_model: SensorResponseModel,
    pub notes: String,
}

#[derive(Debug, Clone)]
pub struct RobotSensorInstance {
    pub name: String,
    pub asset_path: PathBuf,
    pub asset: SensorAsset,
    pub position_m: Vec2,
    pub angle_deg: f64,
    pub enabled: bool,
    pub visible_in_preview: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FanCurveModel {
    Linear,
    Exponential,
    Polynomial,
    LookupTable,
    Custom,
}

impl FanCurveModel {
    pub fn as_str(self) -> &'static str {
        match self {
            FanCurveModel::Linear => "Linear",
            FanCurveModel::Exponential => "Exponential",
            FanCurveModel::Polynomial => "Polynomial",
            FanCurveModel::LookupTable => "LookupTable",
            FanCurveModel::Custom => "Custom",
        }
    }

    pub fn from_str(value: &str) -> Self {
        match value.to_ascii_lowercase().as_str() {
            "exponential" => FanCurveModel::Exponential,
            "polynomial" => FanCurveModel::Polynomial,
            "lookuptable" | "lookup_table" | "table" => FanCurveModel::LookupTable,
            "custom" => FanCurveModel::Custom,
            _ => FanCurveModel::Linear,
        }
    }
}

#[derive(Debug, Clone)]
pub enum DownforceModel {
    None,
    Constant {
        force_n: f64,
    },
    LinearVoltage {
        k_n_per_v: f64,
        offset_n: f64,
        max_force_n: f64,
    },
    LinearCurrent {
        k_n_per_a: f64,
        offset_n: f64,
        max_force_n: f64,
    },
    Exponential {
        a: f64,
        b: f64,
        max_force_n: f64,
    },
    Polynomial {
        coefficients: Vec<f64>,
        max_force_n: f64,
    },
    LookupTable {
        points: Vec<DownforcePoint>,
    },
    Fan {
        nominal_voltage_v: f64,
        nominal_current_a: f64,
        max_force_n: f64,
        curve: FanCurveModel,
        response_time_s: f64,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct DownforcePoint {
    pub input: f64,
    pub force_n: f64,
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
    /// Schema of the file that was loaded/saved. `rtsim-track-v2` enables the
    /// parametric segment-based track editor; `rtsim-track-v1` remains supported
    /// as a sampled polyline cache.
    pub schema: String,
    pub name: String,
    pub model: String,
    pub line_width_m: f64,
    pub base_reflectance: f64,
    pub line_reflectance: f64,
    pub surface_mu: f64,
    /// Sampled centerline cache used by the current simulation/runtime path.
    /// For v2 files this is derived from `parametric` and is not the source of truth.
    pub centerline: Vec<Vec2>,
    pub parametric: Option<TrackV2>,
}

#[derive(Debug, Clone)]
pub struct SurfaceProfile {
    pub schema: String,
    pub name: String,
    pub rules_mode: TrackRulesMode,
    pub line_width_mm: Option<f64>,
    pub background_reflectance: f64,
    pub line_reflectance: f64,
    pub base_color: String,
    pub line_color: String,
    pub surface_mu: f64,
    pub marker_profile: String,
    pub overrides: TrackRuleOverrides,
}

#[derive(Debug, Clone)]
pub struct MotorProfile {
    pub schema: String,
    pub name: String,
    pub motor: MotorConfig,
}

#[derive(Debug, Clone)]
pub struct DriverProfile {
    pub schema: String,
    pub name: String,
    pub driver: DriverConfig,
}

#[derive(Debug, Clone)]
pub struct BatteryProfile {
    pub schema: String,
    pub name: String,
    pub battery: BatteryConfig,
}

#[derive(Debug, Clone)]
pub struct TireProfile {
    pub schema: String,
    pub name: String,
    pub tire: TireConfig,
}

#[derive(Debug, Clone)]
pub struct EncoderProfile {
    pub schema: String,
    pub name: String,
    pub encoder: EncoderConfig,
}

#[derive(Debug, Clone)]
pub struct GyroProfile {
    pub schema: String,
    pub name: String,
    pub gyro: GyroConfig,
}

#[derive(Debug, Clone)]
pub struct FanProfile {
    pub schema: String,
    pub name: String,
    pub fan: FanConfig,
}

impl TrackConfig {
    pub fn from_parametric(mut track: TrackV2) -> Self {
        track.schema = "rtsim-track-v2".to_string();
        let mut cfg = Self {
            schema: track.schema.clone(),
            name: track.name.clone(),
            model: "ParametricTrack".to_string(),
            line_width_m: 0.019,
            base_reflectance: track.surface.base_reflectance,
            line_reflectance: track.surface.line_reflectance,
            surface_mu: track.surface.surface_mu,
            centerline: Vec::new(),
            parametric: Some(track),
        };
        refresh_track_cache(&mut cfg);
        cfg
    }
}

pub fn refresh_track_cache(track: &mut TrackConfig) {
    if let Some(parametric) = &mut track.parametric {
        let rules = resolve_rules(&parametric.rules);
        let geometry = build_geometry(parametric);
        track.schema = parametric.schema.clone();
        track.name = parametric.name.clone();
        track.model = "ParametricTrack".to_string();
        track.line_width_m = rules.line_width_mm / 1000.0;
        track.base_reflectance = parametric.surface.base_reflectance;
        track.line_reflectance = parametric.surface.line_reflectance;
        track.surface_mu = parametric.surface.surface_mu;
        track.centerline = geometry.centerline_m;
    }
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
    let mut project = project;
    if let Some(parametric) = &track.parametric {
        if let Some(start_pose) = resolve_robot_start_pose(parametric) {
            project.start_pose = Pose2::new(
                start_pose.x_mm / 1000.0,
                start_pose.y_mm / 1000.0,
                start_pose.heading_deg.to_radians(),
            );
        }
    }

    Ok(LoadedConfig {
        project_path,
        project,
        robot,
        track,
    })
}

pub fn load_track_from_file(path: impl AsRef<Path>) -> Result<TrackConfig, String> {
    let path = path.as_ref();
    let track_json = read_json(path).map_err(|err| err.to_string())?;
    parse_track_config(path, &track_json).map_err(|err| err.to_string())
}

pub fn load_surface_profile_from_file(path: impl AsRef<Path>) -> Result<SurfaceProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_surface_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_robot_from_file(path: impl AsRef<Path>) -> Result<RobotConfig, String> {
    let path = path.as_ref();
    let robot_json = read_json(path).map_err(|err| err.to_string())?;
    parse_robot_config(path, &robot_json).map_err(|err| err.to_string())
}

pub fn load_motor_profile_from_file(path: impl AsRef<Path>) -> Result<MotorProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_motor_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_driver_profile_from_file(path: impl AsRef<Path>) -> Result<DriverProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_driver_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_battery_profile_from_file(path: impl AsRef<Path>) -> Result<BatteryProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_battery_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_tire_profile_from_file(path: impl AsRef<Path>) -> Result<TireProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_tire_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_encoder_profile_from_file(path: impl AsRef<Path>) -> Result<EncoderProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_encoder_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_gyro_profile_from_file(path: impl AsRef<Path>) -> Result<GyroProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_gyro_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_fan_profile_from_file(path: impl AsRef<Path>) -> Result<FanProfile, String> {
    let path = path.as_ref();
    let profile_json = read_json(path).map_err(|err| err.to_string())?;
    parse_fan_profile_config(path, &profile_json).map_err(|err| err.to_string())
}

pub fn load_sensor_asset_from_file(path: impl AsRef<Path>) -> Result<SensorAsset, String> {
    let path = path.as_ref();
    let sensor_json = read_json(path).map_err(|err| err.to_string())?;
    parse_sensor_asset_config(path, &sensor_json).map_err(|err| err.to_string())
}

pub fn apply_surface_profile(track: &mut TrackV2, profile: &SurfaceProfile) {
    track.rules.profile = profile.name.clone();
    track.rules.mode = profile.rules_mode;
    track.rules.overrides = profile.overrides;
    track.rules.overrides.line_width_mm = profile.line_width_mm;
    track.surface.base_color = profile.base_color.clone();
    track.surface.line_color = profile.line_color.clone();
    track.surface.base_reflectance = profile.background_reflectance;
    track.surface.line_reflectance = profile.line_reflectance;
    track.surface.surface_mu = profile.surface_mu;
}

pub fn surface_profile_from_track(track: &TrackV2) -> SurfaceProfile {
    SurfaceProfile {
        schema: "rtsim-surface-profile-v1".to_string(),
        name: track.rules.profile.clone(),
        rules_mode: track.rules.mode,
        line_width_mm: Some(resolve_rules(&track.rules).line_width_mm),
        background_reflectance: track.surface.base_reflectance,
        line_reflectance: track.surface.line_reflectance,
        base_color: track.surface.base_color.clone(),
        line_color: track.surface.line_color.clone(),
        surface_mu: track.surface.surface_mu,
        marker_profile: track.rules.profile.clone(),
        overrides: track.rules.overrides,
    }
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
    let motor_left = parse_motor(left_json);
    let motor_right = motor_left.clone();

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

    let mut sensors = parse_robot_sensor_instances(path, root.get("sensors"))?;
    if let Some(shared) = sensors.first().cloned() {
        for sensor in sensors.iter_mut().skip(1) {
            sensor.asset_path = shared.asset_path.clone();
            sensor.asset = shared.asset.clone();
        }
    }
    let line_validity_areas =
        parse_robot_line_validity_areas(path, root.get("line_validity_areas"), &chassis)?;

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
        sensors,
        line_validity_areas,
        encoder,
        gyro,
        controller,
    })
}

fn parse_robot_line_validity_areas(
    path: &Path,
    value: Option<&JsonValue>,
    chassis: &ChassisConfig,
) -> CfgResult<Vec<RobotLineValidityArea>> {
    let Some(value) = value else {
        return Ok(vec![RobotLineValidityArea {
            name: "Main body".to_string(),
            position_m: Vec2::new(0.0, 0.0),
            length_m: chassis.length_m,
            width_m: chassis.width_m,
            angle_deg: 0.0,
            enabled: true,
        }]);
    };
    let arr = value.as_array().ok_or_else(|| ConfigError::Invalid {
        path: path.to_path_buf(),
        field: "line_validity_areas".to_string(),
        message: "expected array".to_string(),
    })?;
    let mut areas = Vec::with_capacity(arr.len());
    for (index, item) in arr.iter().enumerate() {
        let name = item
            .get("name")
            .and_then(JsonValue::as_str)
            .map(str::to_string)
            .unwrap_or_else(|| format!("Area {}", index + 1));
        areas.push(RobotLineValidityArea {
            name,
            position_m: vec2_mm_field(item, "position_mm", Vec2::new(0.0, 0.0)),
            length_m: num_field(item, "length_mm", chassis.length_m * 1000.0).max(0.1) / 1000.0,
            width_m: num_field(item, "width_mm", chassis.width_m * 1000.0).max(0.1) / 1000.0,
            angle_deg: num_field(item, "angle_deg", 0.0),
            enabled: nested_bool(Some(item), "enabled", true),
        });
    }
    Ok(areas)
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
    let model_kind = root
        .and_then(|v| v.get("downforce_model"))
        .map(|v| {
            parse_downforce_model(
                path,
                v,
                &model,
                max_force_n,
                response_time_s,
                &force_curve,
                &fans,
            )
        })
        .transpose()?
        .unwrap_or_else(|| {
            default_downforce_model(&model, max_force_n, response_time_s, &force_curve, &fans)
        });

    Ok(NormalForceConfig {
        model,
        model_kind,
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
        fans.push(parse_fan_config(
            path,
            fan,
            &format!("normal_force.fans[{i}]"),
        )?);
    }
    Ok(fans)
}

fn parse_fan_config(path: &Path, fan: &JsonValue, field: &str) -> CfgResult<FanConfig> {
    let position_m = vec2_mm_field(fan, "position_mm", Vec2::new(0.0, 0.0));
    let force_curve = fan
        .get("force_curve")
        .or_else(|| fan.get("thrust_curve"))
        .or_else(|| fan.get("measured_curve"))
        .map(|v| parse_curve(path, v, &format!("{field}.force_curve")))
        .transpose()?
        .unwrap_or_default();
    let max_current_a = num_field(fan, "max_current_a", 0.0);
    let nominal_voltage_v = num_field(fan, "nominal_voltage_v", 7.4);
    let nominal_current_a = num_field(fan, "nominal_current_a", max_current_a);
    Ok(FanConfig {
        position_m,
        visual_radius_m: num_field(fan, "visual_radius_mm", 12.0) / 1000.0,
        action_radius_m: num_field(fan, "action_radius_mm", 20.0) / 1000.0,
        max_force_n: num_field(fan, "max_force_n", 0.0),
        max_current_a,
        nominal_voltage_v,
        nominal_current_a,
        power_w: num_field(fan, "power_w", nominal_voltage_v * nominal_current_a),
        min_pwm: num_field(fan, "min_pwm", 0.0),
        max_pwm: num_field(fan, "max_pwm", 1.0),
        response_time_s: num_field(fan, "response_time_s", 0.0),
        pwm_scale: num_field(fan, "pwm_scale", 1.0),
        enabled_pwm: num_field(fan, "pwm", 1.0),
        curve_model: FanCurveModel::from_str(nested_str(Some(fan), "curve_model", "LookupTable")),
        force_curve,
    })
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
    if schema == "rtsim-track-v2" {
        let parametric = parse_track_v2(path, root)?;
        return Ok(TrackConfig::from_parametric(parametric));
    }

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
        parametric: None,
    })
}

fn parse_track_v2(path: &Path, root: &JsonValue) -> CfgResult<TrackV2> {
    let schema = str_field(path, root, "track_schema", "rtsim-track-v2")?.to_string();
    let name = str_field(path, root, "name", "unnamed-track")?.to_string();
    let units = str_field(path, root, "units", "mm")?.to_string();

    let area_json = root.get("area");
    let area = TrackArea {
        width_mm: nested_num(path, area_json, "width_mm", 3000.0)?,
        height_mm: nested_num(path, area_json, "height_mm", 2000.0)?,
        grid_mm: nested_num(path, area_json, "grid_mm", 100.0)?,
    };

    let origin_json = root.get("origin");
    let origin = TrackPose {
        x_mm: nested_num(path, origin_json, "x_mm", 500.0)?,
        y_mm: nested_num(path, origin_json, "y_mm", 500.0)?,
        heading_deg: nested_num(path, origin_json, "heading_deg", 0.0)?,
    };

    let rules = parse_track_rules(path, root.get("rules"))?;
    let surface = parse_track_surface(path, root.get("surface"))?;
    let segments = parse_track_segments(path, root)?;
    let closure_json = root.get("closure");
    let closure = TrackClosureConfig {
        required: nested_bool(closure_json, "required", true),
        position_tolerance_mm: nested_num(path, closure_json, "position_tolerance_mm", 0.5)?,
        heading_tolerance_deg: nested_num(path, closure_json, "heading_tolerance_deg", 0.1)?,
    };
    let markings = parse_track_markings(path, root.get("markings"))?;

    Ok(TrackV2 {
        schema,
        name,
        units,
        area,
        origin,
        rules,
        surface,
        segments,
        closure,
        markings,
    })
}

fn parse_track_rules(path: &Path, value: Option<&JsonValue>) -> CfgResult<TrackRulesConfig> {
    let profile = nested_str(value, "profile", "robotrace official").to_string();
    let mode = TrackRulesMode::from_str(nested_str(value, "mode", "warning"));
    let overrides = parse_track_rule_overrides(path, value.and_then(|v| v.get("overrides")))?;
    Ok(TrackRulesConfig {
        profile,
        mode,
        overrides,
    })
}

fn parse_track_rule_overrides(
    path: &Path,
    overrides_json: Option<&JsonValue>,
) -> CfgResult<TrackRuleOverrides> {
    Ok(TrackRuleOverrides {
        line_width_mm: optional_nested_num(path, overrides_json, "line_width_mm")?,
        max_total_length_mm: optional_nested_num(path, overrides_json, "max_total_length_mm")?,
        min_arc_radius_mm: optional_nested_num(path, overrides_json, "min_arc_radius_mm")?,
        min_distance_between_curvature_changes_mm: optional_nested_num(
            path,
            overrides_json,
            "min_distance_between_curvature_changes_mm",
        )?,
        intersection_angle_deg: optional_nested_num(
            path,
            overrides_json,
            "intersection_angle_deg",
        )?,
        intersection_angle_tolerance_deg: optional_nested_num(
            path,
            overrides_json,
            "intersection_angle_tolerance_deg",
        )?,
        min_straight_around_intersection_mm: optional_nested_num(
            path,
            overrides_json,
            "min_straight_around_intersection_mm",
        )?,
        start_finish_must_be_on_straight: optional_nested_bool(
            overrides_json,
            "start_finish_must_be_on_straight",
        ),
        min_straight_around_start_finish_mm: optional_nested_num(
            path,
            overrides_json,
            "min_straight_around_start_finish_mm",
        )?,
        start_goal_distance_mm: optional_nested_num(
            path,
            overrides_json,
            "start_goal_distance_mm",
        )?,
        start_goal_area_half_width_mm: optional_nested_num(
            path,
            overrides_json,
            "start_goal_area_half_width_mm",
        )?,
        min_table_edge_clearance_mm: optional_nested_num(
            path,
            overrides_json,
            "min_table_edge_clearance_mm",
        )?,
        max_slope_deg: optional_nested_num(path, overrides_json, "max_slope_deg")?,
    })
}

fn parse_surface_profile_config(path: &Path, root: &JsonValue) -> CfgResult<SurfaceProfile> {
    let schema = str_field(
        path,
        root,
        "surface_profile_schema",
        "rtsim-surface-profile-v1",
    )?
    .to_string();
    let rules_json = root.get("rules");
    let surface_json = root.get("surface");
    let overrides_json = rules_json
        .and_then(|v| v.get("overrides"))
        .or_else(|| root.get("overrides"));
    let mut overrides = parse_track_rule_overrides(path, overrides_json)?;
    let top_level_line_width = optional_nested_num(path, Some(root), "line_width_mm")?;
    if top_level_line_width.is_some() {
        overrides.line_width_mm = top_level_line_width;
    }

    let name = str_field(
        path,
        root,
        "name",
        nested_str(rules_json, "profile", "surface profile"),
    )?
    .to_string();
    let rules_mode = root
        .get("rules_mode")
        .and_then(JsonValue::as_str)
        .map(TrackRulesMode::from_str)
        .unwrap_or_else(|| TrackRulesMode::from_str(nested_str(rules_json, "mode", "warning")));

    let background_reflectance = optional_nested_num(path, Some(root), "background_reflectance")?
        .or(optional_nested_num(path, Some(root), "base_reflectance")?)
        .unwrap_or(nested_num(path, surface_json, "base_reflectance", 0.08)?);
    let line_reflectance = optional_nested_num(path, Some(root), "line_reflectance")?
        .unwrap_or(nested_num(path, surface_json, "line_reflectance", 0.86)?);

    Ok(SurfaceProfile {
        schema,
        name: name.clone(),
        rules_mode,
        line_width_mm: overrides.line_width_mm,
        background_reflectance,
        line_reflectance,
        base_color: root
            .get("base_color")
            .and_then(JsonValue::as_str)
            .unwrap_or_else(|| nested_str(surface_json, "base_color", "black"))
            .to_string(),
        line_color: root
            .get("line_color")
            .and_then(JsonValue::as_str)
            .unwrap_or_else(|| nested_str(surface_json, "line_color", "white"))
            .to_string(),
        surface_mu: optional_nested_num(path, Some(root), "surface_mu")?.unwrap_or(nested_num(
            path,
            surface_json,
            "surface_mu",
            1.20,
        )?),
        marker_profile: root
            .get("marker_profile")
            .and_then(JsonValue::as_str)
            .unwrap_or(&name)
            .to_string(),
        overrides,
    })
}

fn profile_schema(root: &JsonValue, field: &str, default: &str) -> String {
    root.get(field)
        .or_else(|| root.get("profile_schema"))
        .and_then(JsonValue::as_str)
        .unwrap_or(default)
        .to_string()
}

fn profile_name(root: &JsonValue, item: &JsonValue, fallback: &str) -> String {
    root.get("name")
        .or_else(|| item.get("name"))
        .or_else(|| item.get("model"))
        .and_then(JsonValue::as_str)
        .unwrap_or(fallback)
        .to_string()
}

fn parse_motor_profile_config(_path: &Path, root: &JsonValue) -> CfgResult<MotorProfile> {
    let motor_json = root.get("motor").unwrap_or(root);
    let motor = parse_motor(motor_json);
    Ok(MotorProfile {
        schema: profile_schema(root, "motor_profile_schema", "rtsim-motor-profile-v1"),
        name: profile_name(root, motor_json, &motor.model),
        motor,
    })
}

fn parse_driver_profile_config(path: &Path, root: &JsonValue) -> CfgResult<DriverProfile> {
    let driver_json = root.get("driver").unwrap_or(root);
    let driver = parse_driver_config(path, Some(driver_json))?;
    Ok(DriverProfile {
        schema: profile_schema(root, "driver_profile_schema", "rtsim-driver-profile-v1"),
        name: profile_name(root, driver_json, &driver.model),
        driver,
    })
}

fn parse_battery_profile_config(path: &Path, root: &JsonValue) -> CfgResult<BatteryProfile> {
    let battery_json = root.get("battery").unwrap_or(root);
    let battery = parse_battery_config(path, Some(battery_json))?;
    Ok(BatteryProfile {
        schema: profile_schema(root, "battery_profile_schema", "rtsim-battery-profile-v1"),
        name: profile_name(root, battery_json, &battery.model),
        battery,
    })
}

fn parse_tire_profile_config(path: &Path, root: &JsonValue) -> CfgResult<TireProfile> {
    let tire_json = root.get("tire").unwrap_or(root);
    let tire = parse_tire_config(path, tire_json)?;
    Ok(TireProfile {
        schema: profile_schema(root, "tire_profile_schema", "rtsim-tire-profile-v1"),
        name: profile_name(root, tire_json, &tire.model),
        tire,
    })
}

fn parse_encoder_profile_config(path: &Path, root: &JsonValue) -> CfgResult<EncoderProfile> {
    let value = root.get("encoder").unwrap_or(root);
    let encoder = EncoderConfig {
        model: nested_str(Some(value), "model", "QuantizedEncoder").to_string(),
        ticks_per_rev: nested_num(path, Some(value), "ticks_per_rev", 360.0)? as u32,
        invert_left: nested_bool(Some(value), "invert_left", false),
        invert_right: nested_bool(Some(value), "invert_right", false),
    };
    Ok(EncoderProfile {
        schema: profile_schema(root, "encoder_profile_schema", "rtsim-encoder-profile-v1"),
        name: profile_name(root, value, &encoder.model),
        encoder,
    })
}

fn parse_gyro_profile_config(path: &Path, root: &JsonValue) -> CfgResult<GyroProfile> {
    let value = root.get("gyro").unwrap_or(root);
    let gyro = GyroConfig {
        model: nested_str(Some(value), "model", "NoisyGyro").to_string(),
        noise_std_rad_s: nested_num(path, Some(value), "noise_std_rad_s", 0.01)?,
        bias_rad_s: nested_num(path, Some(value), "bias_rad_s", 0.0)?,
        saturation_rad_s: nested_num(path, Some(value), "saturation_rad_s", 34.906585)?,
        seed: nested_num(path, Some(value), "seed", 0x9A17_0002u64 as f64)? as u64,
    };
    Ok(GyroProfile {
        schema: profile_schema(root, "gyro_profile_schema", "rtsim-gyro-profile-v1"),
        name: profile_name(root, value, &gyro.model),
        gyro,
    })
}

fn parse_fan_profile_config(path: &Path, root: &JsonValue) -> CfgResult<FanProfile> {
    let fan_json = root.get("fan").unwrap_or(root);
    let fan = parse_fan_config(path, fan_json, "fan")?;
    Ok(FanProfile {
        schema: profile_schema(root, "fan_profile_schema", "rtsim-fan-profile-v1"),
        name: profile_name(root, fan_json, "fan"),
        fan,
    })
}

fn parse_tire_config(path: &Path, tire_json: &JsonValue) -> CfgResult<TireConfig> {
    Ok(TireConfig {
        model: str_field(path, tire_json, "model", "SlipRatioWheel")?.to_string(),
        mu_longitudinal: num_field(tire_json, "mu_longitudinal", 1.2),
        mu_lateral: num_field(tire_json, "mu_lateral", 1.0),
        rolling_resistance: num_field(tire_json, "rolling_resistance", 0.015),
        slip_velocity_epsilon_m_s: num_field(tire_json, "slip_velocity_epsilon_m_s", 0.05),
    })
}

fn parse_driver_config(path: &Path, driver_json: Option<&JsonValue>) -> CfgResult<DriverConfig> {
    Ok(DriverConfig {
        model: nested_str(driver_json, "model", "PwmHBridge").to_string(),
        pwm_frequency_hz: nested_num(path, driver_json, "pwm_frequency_hz", 20_000.0)?,
        mode: nested_str(driver_json, "mode", "brake").to_string(),
        voltage_drop_v: nested_num(path, driver_json, "voltage_drop_v", 0.2)?,
        pwm_resolution_bits: nested_num(path, driver_json, "pwm_resolution_bits", 10.0)? as u32,
        command_deadband: nested_num(path, driver_json, "command_deadband", 0.001)?,
        current_limit_a: nested_num(path, driver_json, "current_limit_a", 1000.0)?,
    })
}

fn parse_battery_config(path: &Path, battery_json: Option<&JsonValue>) -> CfgResult<BatteryConfig> {
    let cells = nested_num(path, battery_json, "cells", 2.0)? as u32;
    let nominal_voltage_v = nested_num(path, battery_json, "nominal_voltage_v", 7.4)?;
    Ok(BatteryConfig {
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
    })
}

fn default_sensor_asset() -> SensorAsset {
    SensorAsset {
        name: "Default Line Sensor".to_string(),
        model: "GenericAnalogLineSensor".to_string(),
        sensor_type: SensorType::LineAnalog,
        visual_width_m: 8.0 / 1000.0,
        visual_height_m: 8.0 / 1000.0,
        visual_radius_m: 4.0 / 1000.0,
        detection_area: SensorDetectionArea::Rectangle {
            width_m: 5.0 / 1000.0,
            height_m: 2.0 / 1000.0,
        },
        response_model: SensorResponseModel::Ideal,
        notes: String::new(),
    }
}

fn default_robot_sensor_instance() -> RobotSensorInstance {
    RobotSensorInstance {
        name: "Front line sensor".to_string(),
        asset_path: PathBuf::from("RobotAssets/Sensors/default_line_sensor.json"),
        asset: default_sensor_asset(),
        position_m: Vec2::new(0.055, 0.0),
        angle_deg: 0.0,
        enabled: true,
        visible_in_preview: true,
    }
}

fn parse_robot_sensor_instances(
    path: &Path,
    value: Option<&JsonValue>,
) -> CfgResult<Vec<RobotSensorInstance>> {
    let Some(value) = value else {
        return Ok(vec![default_robot_sensor_instance()]);
    };
    let arr = value.as_array().ok_or_else(|| ConfigError::Invalid {
        path: path.to_path_buf(),
        field: "sensors".to_string(),
        message: "expected array".to_string(),
    })?;
    let mut sensors = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        sensors.push(parse_robot_sensor_instance(path, item, i)?);
    }
    Ok(sensors)
}

fn parse_robot_sensor_instance(
    path: &Path,
    item: &JsonValue,
    index: usize,
) -> CfgResult<RobotSensorInstance> {
    let name = item
        .get("name")
        .or_else(|| item.get("instance_name"))
        .and_then(JsonValue::as_str)
        .unwrap_or("Sensor")
        .to_string();
    let asset_path = item
        .get("asset_path")
        .and_then(JsonValue::as_str)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("RobotAssets/Sensors/default_line_sensor.json"));
    let asset = if let Some(asset_json) = item.get("asset") {
        parse_sensor_asset_config(path, asset_json)?
    } else {
        let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
        let resolved = if asset_path.is_absolute() || asset_path.exists() {
            asset_path.clone()
        } else {
            normalize_child_path(base_dir, &asset_path)
        };
        read_json(&resolved)
            .ok()
            .and_then(|json| parse_sensor_asset_config(&resolved, &json).ok())
            .unwrap_or_else(default_sensor_asset)
    };
    let position_m = item
        .get("position_mm")
        .map(|_| vec2_mm_field(item, "position_mm", Vec2::new(0.055, 0.0)))
        .unwrap_or_else(|| {
            Vec2::new(
                num_field(item, "position_x_mm", 55.0) / 1000.0,
                num_field(item, "position_y_mm", 0.0) / 1000.0,
            )
        });
    Ok(RobotSensorInstance {
        name: if name == "Sensor" {
            format!("Sensor {}", index + 1)
        } else {
            name
        },
        asset_path,
        asset,
        position_m,
        angle_deg: num_field(item, "angle_deg", 0.0),
        enabled: nested_bool(Some(item), "enabled", true),
        visible_in_preview: nested_bool(Some(item), "visible_in_preview", true),
    })
}

fn parse_sensor_asset_config(path: &Path, root: &JsonValue) -> CfgResult<SensorAsset> {
    let sensor_json = root
        .get("sensor_asset")
        .or_else(|| root.get("sensor"))
        .unwrap_or(root);
    Ok(SensorAsset {
        name: sensor_json
            .get("name")
            .or_else(|| root.get("name"))
            .and_then(JsonValue::as_str)
            .unwrap_or("Default Line Sensor")
            .to_string(),
        model: nested_str(Some(sensor_json), "model", "GenericAnalogLineSensor").to_string(),
        sensor_type: SensorType::from_str(nested_str(
            Some(sensor_json),
            "sensor_type",
            "LineAnalog",
        )),
        visual_width_m: nested_num(path, Some(sensor_json), "visual_width_mm", 8.0)? / 1000.0,
        visual_height_m: nested_num(path, Some(sensor_json), "visual_height_mm", 8.0)? / 1000.0,
        visual_radius_m: nested_num(path, Some(sensor_json), "visual_radius_mm", 0.0)? / 1000.0,
        detection_area: parse_sensor_detection_area(
            path,
            sensor_json.get("detection_area"),
            "sensor_asset.detection_area",
        )?,
        response_model: parse_sensor_response_model(
            path,
            sensor_json.get("response_model"),
            "sensor_asset.response_model",
        )?,
        notes: nested_str(Some(sensor_json), "notes", "").to_string(),
    })
}

fn parse_sensor_detection_area(
    path: &Path,
    value: Option<&JsonValue>,
    field: &str,
) -> CfgResult<SensorDetectionArea> {
    let Some(value) = value else {
        return Ok(SensorDetectionArea::Point {
            radius_m: 1.0 / 1000.0,
        });
    };
    if let Some(kind) = value.get("kind").and_then(JsonValue::as_str) {
        return parse_sensor_detection_area_by_kind(path, value, field, kind);
    }
    if let Some(rect) = value.get("Rectangle").or_else(|| value.get("rectangle")) {
        return Ok(SensorDetectionArea::Rectangle {
            width_m: nested_num(path, Some(rect), "width_mm", 5.0)? / 1000.0,
            height_m: nested_num(path, Some(rect), "height_mm", 2.0)? / 1000.0,
        });
    }
    if let Some(circle) = value.get("Circle").or_else(|| value.get("circle")) {
        return Ok(SensorDetectionArea::Circle {
            radius_m: nested_num(path, Some(circle), "radius_mm", 2.5)? / 1000.0,
        });
    }
    if let Some(point) = value.get("Point").or_else(|| value.get("point")) {
        return Ok(SensorDetectionArea::Point {
            radius_m: nested_num(path, Some(point), "radius_mm", 1.0)? / 1000.0,
        });
    }
    if let Some(cone) = value.get("Cone").or_else(|| value.get("cone")) {
        return Ok(SensorDetectionArea::Cone {
            range_m: nested_num(path, Some(cone), "range_mm", 80.0)? / 1000.0,
            angle_deg: nested_num(path, Some(cone), "angle_deg", 25.0)?,
        });
    }
    if let Some(poly) = value
        .get("CustomPolygon")
        .or_else(|| value.get("custom_polygon"))
    {
        return parse_sensor_polygon(path, poly, field);
    }
    Err(ConfigError::Invalid {
        path: path.to_path_buf(),
        field: field.to_string(),
        message: "unsupported detection area".to_string(),
    })
}

fn parse_sensor_detection_area_by_kind(
    path: &Path,
    value: &JsonValue,
    field: &str,
    kind: &str,
) -> CfgResult<SensorDetectionArea> {
    match kind.to_ascii_lowercase().as_str() {
        "point" => Ok(SensorDetectionArea::Point {
            radius_m: nested_num(path, Some(value), "radius_mm", 1.0)? / 1000.0,
        }),
        "rectangle" => Ok(SensorDetectionArea::Rectangle {
            width_m: nested_num(path, Some(value), "width_mm", 5.0)? / 1000.0,
            height_m: nested_num(path, Some(value), "height_mm", 2.0)? / 1000.0,
        }),
        "circle" => Ok(SensorDetectionArea::Circle {
            radius_m: nested_num(path, Some(value), "radius_mm", 2.5)? / 1000.0,
        }),
        "cone" => Ok(SensorDetectionArea::Cone {
            range_m: nested_num(path, Some(value), "range_mm", 80.0)? / 1000.0,
            angle_deg: nested_num(path, Some(value), "angle_deg", 25.0)?,
        }),
        "custompolygon" | "custom_polygon" => parse_sensor_polygon(path, value, field),
        other => Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: format!("unsupported detection area kind '{other}'"),
        }),
    }
}

fn parse_sensor_polygon(
    path: &Path,
    value: &JsonValue,
    field: &str,
) -> CfgResult<SensorDetectionArea> {
    let arr = value
        .get("points_mm")
        .unwrap_or(value)
        .as_array()
        .ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: "expected points_mm array".to_string(),
        })?;
    let mut points = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        let pair = item.as_array().ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: format!("{field}[{i}]"),
            message: "expected [x_mm, y_mm]".to_string(),
        })?;
        if pair.len() != 2 {
            return Err(ConfigError::Invalid {
                path: path.to_path_buf(),
                field: format!("{field}[{i}]"),
                message: "expected [x_mm, y_mm]".to_string(),
            });
        }
        let x = pair[0].as_f64().unwrap_or(0.0) / 1000.0;
        let y = pair[1].as_f64().unwrap_or(0.0) / 1000.0;
        points.push(Vec2::new(x, y));
    }
    Ok(SensorDetectionArea::CustomPolygon { points_m: points })
}

fn parse_sensor_response_model(
    path: &Path,
    value: Option<&JsonValue>,
    field: &str,
) -> CfgResult<SensorResponseModel> {
    let Some(value) = value else {
        return Ok(SensorResponseModel::Ideal);
    };
    if let Some(kind) = value.as_str() {
        return Ok(match kind.to_ascii_lowercase().as_str() {
            "ideal" => SensorResponseModel::Ideal,
            other => SensorResponseModel::Custom {
                description: other.to_string(),
            },
        });
    }
    let kind = value
        .get("kind")
        .and_then(JsonValue::as_str)
        .or_else(|| value.get("type").and_then(JsonValue::as_str))
        .unwrap_or("Ideal");
    match kind.to_ascii_lowercase().as_str() {
        "ideal" => Ok(SensorResponseModel::Ideal),
        "threshold" => Ok(SensorResponseModel::Threshold {
            threshold: nested_num(path, Some(value), "threshold", 0.5)?,
        }),
        "linear" => Ok(SensorResponseModel::Linear {
            gain: nested_num(path, Some(value), "gain", 1.0)?,
            offset: nested_num(path, Some(value), "offset", 0.0)?,
        }),
        "polynomial" => Ok(SensorResponseModel::Polynomial {
            coefficients: parse_number_array(value.get("coefficients"))
                .unwrap_or_else(|| vec![0.0, 1.0]),
        }),
        "lookuptable" | "lookup_table" => Ok(SensorResponseModel::LookupTable {
            points: parse_sensor_response_points(path, value.get("points"), field)?,
        }),
        "custom" => Ok(SensorResponseModel::Custom {
            description: nested_str(Some(value), "description", "").to_string(),
        }),
        other => Err(ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: format!("unsupported response model '{other}'"),
        }),
    }
}

fn parse_sensor_response_points(
    path: &Path,
    value: Option<&JsonValue>,
    field: &str,
) -> CfgResult<Vec<SensorResponsePoint>> {
    let Some(value) = value else {
        return Ok(Vec::new());
    };
    let arr = value.as_array().ok_or_else(|| ConfigError::Invalid {
        path: path.to_path_buf(),
        field: format!("{field}.points"),
        message: "expected array".to_string(),
    })?;
    let mut points = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        if let Some(pair) = item.as_array() {
            if pair.len() == 2 {
                points.push(SensorResponsePoint {
                    input: pair[0].as_f64().unwrap_or(0.0),
                    output: pair[1].as_f64().unwrap_or(0.0),
                });
                continue;
            }
        }
        points.push(SensorResponsePoint {
            input: nested_num(path, Some(item), "input", i as f64)?,
            output: nested_num(path, Some(item), "output", 0.0)?,
        });
    }
    Ok(points)
}

fn parse_number_array(value: Option<&JsonValue>) -> Option<Vec<f64>> {
    let arr = value?.as_array()?;
    Some(arr.iter().filter_map(JsonValue::as_f64).collect())
}

fn default_downforce_model(
    model: &str,
    max_force_n: f64,
    response_time_s: f64,
    force_curve: &[(f64, f64)],
    fans: &[FanConfig],
) -> DownforceModel {
    match model.to_ascii_lowercase().as_str() {
        "constantdownforce" => DownforceModel::Constant {
            force_n: max_force_n,
        },
        "fandownforce" => {
            let fan = fans.first();
            DownforceModel::Fan {
                nominal_voltage_v: fan.map(|f| f.nominal_voltage_v).unwrap_or(7.4),
                nominal_current_a: fan.map(|f| f.nominal_current_a).unwrap_or(0.0),
                max_force_n: fan.map(|f| f.max_force_n).unwrap_or(max_force_n),
                curve: fan
                    .map(|f| f.curve_model)
                    .unwrap_or(FanCurveModel::LookupTable),
                response_time_s: fan.map(|f| f.response_time_s).unwrap_or(response_time_s),
            }
        }
        "measureddownforcecurve" => DownforceModel::LookupTable {
            points: force_curve
                .iter()
                .map(|(input, force_n)| DownforcePoint {
                    input: *input,
                    force_n: *force_n,
                })
                .collect(),
        },
        _ => DownforceModel::None,
    }
}

fn parse_downforce_model(
    path: &Path,
    value: &JsonValue,
    fallback_model: &str,
    max_force_n: f64,
    response_time_s: f64,
    force_curve: &[(f64, f64)],
    fans: &[FanConfig],
) -> CfgResult<DownforceModel> {
    let kind = value
        .as_str()
        .or_else(|| {
            value
                .get("kind")
                .or_else(|| value.get("type"))
                .and_then(JsonValue::as_str)
        })
        .unwrap_or(fallback_model);
    Ok(match kind.to_ascii_lowercase().as_str() {
        "none" | "nodownforce" => DownforceModel::None,
        "constant" | "constantdownforce" => DownforceModel::Constant {
            force_n: nested_num(path, Some(value), "force_n", max_force_n)?,
        },
        "linearvoltage" | "linear_voltage" => DownforceModel::LinearVoltage {
            k_n_per_v: nested_num(path, Some(value), "k_n_per_v", 0.0)?,
            offset_n: nested_num(path, Some(value), "offset_n", 0.0)?,
            max_force_n: nested_num(path, Some(value), "max_force_n", max_force_n)?,
        },
        "linearcurrent" | "linear_current" => DownforceModel::LinearCurrent {
            k_n_per_a: nested_num(path, Some(value), "k_n_per_a", 0.0)?,
            offset_n: nested_num(path, Some(value), "offset_n", 0.0)?,
            max_force_n: nested_num(path, Some(value), "max_force_n", max_force_n)?,
        },
        "exponential" => DownforceModel::Exponential {
            a: nested_num(path, Some(value), "a", 0.0)?,
            b: nested_num(path, Some(value), "b", 1.0)?,
            max_force_n: nested_num(path, Some(value), "max_force_n", max_force_n)?,
        },
        "polynomial" => DownforceModel::Polynomial {
            coefficients: parse_number_array(value.get("coefficients"))
                .unwrap_or_else(|| vec![0.0, 1.0]),
            max_force_n: nested_num(path, Some(value), "max_force_n", max_force_n)?,
        },
        "lookuptable" | "lookup_table" | "measureddownforcecurve" => DownforceModel::LookupTable {
            points: value
                .get("points")
                .map(|points| parse_downforce_points(path, points, "downforce_model.points"))
                .transpose()?
                .unwrap_or_else(|| {
                    force_curve
                        .iter()
                        .map(|(input, force_n)| DownforcePoint {
                            input: *input,
                            force_n: *force_n,
                        })
                        .collect()
                }),
        },
        "fan" | "fandownforce" => {
            let fan = fans.first();
            DownforceModel::Fan {
                nominal_voltage_v: nested_num(
                    path,
                    Some(value),
                    "nominal_voltage_v",
                    fan.map(|f| f.nominal_voltage_v).unwrap_or(7.4),
                )?,
                nominal_current_a: nested_num(
                    path,
                    Some(value),
                    "nominal_current_a",
                    fan.map(|f| f.nominal_current_a).unwrap_or(0.0),
                )?,
                max_force_n: nested_num(
                    path,
                    Some(value),
                    "max_force_n",
                    fan.map(|f| f.max_force_n).unwrap_or(max_force_n),
                )?,
                curve: FanCurveModel::from_str(nested_str(Some(value), "curve", "LookupTable")),
                response_time_s: nested_num(
                    path,
                    Some(value),
                    "response_time_s",
                    fan.map(|f| f.response_time_s).unwrap_or(response_time_s),
                )?,
            }
        }
        _ => default_downforce_model(
            fallback_model,
            max_force_n,
            response_time_s,
            force_curve,
            fans,
        ),
    })
}

fn parse_downforce_points(
    path: &Path,
    value: &JsonValue,
    field: &str,
) -> CfgResult<Vec<DownforcePoint>> {
    let arr = value.as_array().ok_or_else(|| ConfigError::Invalid {
        path: path.to_path_buf(),
        field: field.to_string(),
        message: "expected array".to_string(),
    })?;
    let mut points = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        if let Some(pair) = item.as_array() {
            if pair.len() == 2 {
                points.push(DownforcePoint {
                    input: pair[0].as_f64().unwrap_or(0.0),
                    force_n: pair[1].as_f64().unwrap_or(0.0),
                });
                continue;
            }
        }
        points.push(DownforcePoint {
            input: nested_num(path, Some(item), "input", i as f64)?,
            force_n: nested_num(path, Some(item), "force_n", 0.0)?,
        });
    }
    Ok(points)
}

fn parse_track_surface(path: &Path, value: Option<&JsonValue>) -> CfgResult<TrackSurfaceConfig> {
    Ok(TrackSurfaceConfig {
        base_color: nested_str(value, "base_color", "black").to_string(),
        line_color: nested_str(value, "line_color", "white").to_string(),
        base_reflectance: nested_num(path, value, "base_reflectance", 0.08)?,
        line_reflectance: nested_num(path, value, "line_reflectance", 0.86)?,
        surface_mu: nested_num(path, value, "surface_mu", 1.20)?,
    })
}

fn parse_track_segments(path: &Path, root: &JsonValue) -> CfgResult<Vec<TrackSegment>> {
    let arr = root
        .get("segments")
        .and_then(JsonValue::as_array)
        .ok_or_else(|| ConfigError::Missing {
            path: path.to_path_buf(),
            field: "segments".to_string(),
        })?;
    let mut segments = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        let kind = str_field(path, item, "kind", "straight")?;
        let id = str_field(path, item, "id", "")?.to_string();
        match kind {
            "straight" | "reta" => segments.push(TrackSegment::Straight(StraightSegment {
                id: if id.is_empty() {
                    format!("R{}", i + 1)
                } else {
                    id
                },
                length_mm: required_num(path, item, "length_mm")?,
            })),
            "arc" | "arco" => segments.push(TrackSegment::Arc(ArcSegment {
                id: if id.is_empty() {
                    format!("C{}", i + 1)
                } else {
                    id
                },
                radius_mm: required_num(path, item, "radius_mm")?,
                sweep_deg: required_num(path, item, "sweep_deg")?,
            })),
            other => {
                return Err(ConfigError::Invalid {
                    path: path.to_path_buf(),
                    field: format!("segments[{i}].kind"),
                    message: format!("unsupported segment kind '{other}'"),
                })
            }
        }
    }
    Ok(segments)
}

fn parse_track_markings(path: &Path, value: Option<&JsonValue>) -> CfgResult<TrackMarkings> {
    let start_finish_json = value.and_then(|v| v.get("start_finish"));
    let robot_start_json = start_finish_json.and_then(|v| v.get("robot_start"));
    let corner_json = value.and_then(|v| v.get("corner_markers"));
    Ok(TrackMarkings {
        start_finish: StartFinishMarking {
            enabled: nested_bool(start_finish_json, "enabled", true),
            segment_id: nested_str(start_finish_json, "segment_id", "R1").to_string(),
            start_s_mm: nested_num(path, start_finish_json, "start_s_mm", 100.0)?,
            distance_mm: nested_num(path, start_finish_json, "distance_mm", 1000.0)?,
            margin_mm: nested_num(path, start_finish_json, "margin_mm", 100.0)?,
            exit_direction: StartExitDirection::from_str(nested_str(
                start_finish_json,
                "exit_direction",
                "to_increasing_s",
            )),
            robot_start: RobotStartConfig {
                delta_x_mm: nested_num(path, robot_start_json, "delta_x_mm", 125.0)?,
                delta_y_mm: nested_num(path, robot_start_json, "delta_y_mm", 0.0)?,
                heading_deg: nested_num(path, robot_start_json, "heading_deg", 0.0)?,
            },
        },
        corner_markers: TrackCornerMarkersConfig {
            auto_generate: nested_bool(corner_json, "auto_generate", true),
        },
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

fn required_num(path: &Path, root: &JsonValue, field: &str) -> CfgResult<f64> {
    root.get(field)
        .and_then(JsonValue::as_f64)
        .ok_or_else(|| ConfigError::Missing {
            path: path.to_path_buf(),
            field: field.to_string(),
        })
}

fn optional_nested_num(
    path: &Path,
    root: Option<&JsonValue>,
    field: &str,
) -> CfgResult<Option<f64>> {
    match root.and_then(|v| v.get(field)) {
        Some(v) => v.as_f64().map(Some).ok_or_else(|| ConfigError::Invalid {
            path: path.to_path_buf(),
            field: field.to_string(),
            message: "expected number".to_string(),
        }),
        None => Ok(None),
    }
}

fn optional_nested_bool(root: Option<&JsonValue>, field: &str) -> Option<bool> {
    root.and_then(|v| v.get(field)).and_then(JsonValue::as_bool)
}

fn optional_nested_str(root: Option<&JsonValue>, field: &str) -> Option<String> {
    root.and_then(|v| v.get(field))
        .and_then(JsonValue::as_str)
        .map(str::to_string)
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

#[cfg(test)]
mod robot_line_validity_tests {
    use super::*;

    fn area() -> RobotLineValidityArea {
        RobotLineValidityArea {
            name: "side arm".to_string(),
            position_m: Vec2::new(0.0, 0.050),
            length_m: 0.100,
            width_m: 0.010,
            angle_deg: 0.0,
            enabled: true,
        }
    }

    #[test]
    fn validity_rectangle_overlaps_a_line_segment() {
        assert!(area().overlaps_line_segment(
            Pose2::new(1.0, 2.0, 0.0),
            Vec2::new(0.9, 2.050),
            Vec2::new(1.1, 2.050),
            0.020,
        ));
    }

    #[test]
    fn disabled_or_distant_rectangle_does_not_overlap() {
        let mut validity_area = area();
        assert!(!validity_area.overlaps_line_segment(
            Pose2::new(0.0, 0.0, 0.0),
            Vec2::new(-0.1, -0.1),
            Vec2::new(0.1, -0.1),
            0.020,
        ));
        validity_area.enabled = false;
        assert!(!validity_area.overlaps_line_segment(
            Pose2::new(0.0, 0.0, 0.0),
            Vec2::new(-0.1, 0.050),
            Vec2::new(0.1, 0.050),
            0.020,
        ));
    }
}
