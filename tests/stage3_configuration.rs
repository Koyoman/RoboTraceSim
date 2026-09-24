use robotrace_sim::io::{experiment::*, persistence::*};
use robotrace_sim::{
    config::*,
    json::{parse_json, JsonValue},
    math::{Pose2, Vec2},
    sim::SimulationCore,
};
use std::path::{Path, PathBuf};

struct Temp(PathBuf);
impl Temp {
    fn new() -> Self {
        let p = std::env::temp_dir().join(robotrace_sim::io::assets::new_instance_id());
        std::fs::create_dir_all(&p).unwrap();
        Self(p)
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn config() -> LoadedConfig {
    load_project(Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/basic/projeto_suction.rtsim"))
        .unwrap()
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-12, "{a} != {b}");
}

#[test]
fn frozen_config_records_overrides_and_expands_track_defaults() {
    let cfg =
        load_project(Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/basic/projeto.rtsim"))
            .unwrap();
    let core = SimulationCore::with_time_override(cfg, Some(2150), Some(50)).unwrap();
    let resolved = core.resolved_experiment();
    assert_eq!(resolved.config().project.time.physics_dt_us, 50);
    near(resolved.config().project.duration_s, 0.00215);
    assert!(resolved
        .config()
        .track
        .parametric
        .as_ref()
        .unwrap()
        .rules
        .overrides
        .line_width_mm
        .is_some());
}

#[test]
fn fan_instances_and_track_roundtrip_preserve_distinct_parameters() {
    let dir = Temp::new();
    let cfg =
        load_project(Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/basic/projeto.rtsim"))
            .unwrap();
    save_project_bundle(&cfg, &dir.0.join("project.rtsim")).unwrap();
    let actual = load_project(dir.0.join("project.rtsim")).unwrap();
    assert_eq!(
        cfg.robot.normal_force.fans.len(),
        actual.robot.normal_force.fans.len()
    );
    for (a, b) in cfg
        .robot
        .normal_force
        .fans
        .iter()
        .zip(&actual.robot.normal_force.fans)
    {
        assert_eq!(a.id, b.id);
        near(a.max_force_n, b.max_force_n);
        near(a.response_time_s, b.response_time_s);
        near(a.position_m.x, b.position_m.x);
        near(a.position_m.y, b.position_m.y);
    }
    assert_eq!(
        parse_json(&track_json(&cfg.track)).unwrap(),
        parse_json(&track_json(&actual.track)).unwrap()
    );
}

#[test]
fn explicit_array_conversion_preserves_acquisition_and_individual_geometry() {
    let robot = load_robot_from_file(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/stage3/robot_array_legacy.json"),
    )
    .unwrap();
    assert_eq!(robot.schema, "rtsim-robot-v8");
    assert_eq!(robot.sensors.len(), 16);
    near(robot.sensors[0].position_m.y, 0.036);
    near(robot.sensors[15].position_m.y, -0.036);
    for (i, s) in robot.sensors.iter().enumerate() {
        near(s.position_m.x, 0.055);
        assert_eq!(s.acquisition.adc_bits, 12);
        assert_eq!(s.acquisition.seed, 1371 + i as u64);
        near(s.acquisition.reflectance_noise_std, 0.01);
        near(s.acquisition.adc_noise_lsb, 1.0);
    }
}

#[test]
fn json_unicode_controls_surrogates_and_strict_syntax() {
    let original = "Robô 日本 🤖\u{0008}\u{000c}\n\t\\\"";
    let json = JsonValue::String(original.into()).to_json().unwrap();
    assert_eq!(parse_json(&json).unwrap().as_str(), Some(original));
    assert_eq!(
        parse_json(r#""\uD83E\uDD16""#).unwrap().as_str(),
        Some("🤖")
    );
    for bad in [
        "[1,]",
        "{\"x\":1,}",
        "{\"x\":1,\"x\":2}",
        "1e999",
        "NaN",
        "01",
        "1.",
        r#""\uD800""#,
        r#""\uDC00""#,
    ] {
        assert!(parse_json(bad).is_err(), "accepted {bad}");
    }
}

#[test]
fn roundtrip_preserves_individual_motors_sensors_ids_and_disabled_logs() {
    let dir = Temp::new();
    let mut cfg = config();
    cfg.robot.name = "Robô 日本 🤖\u{0008}".into();
    cfg.robot.motor_right.gear_ratio = 12.345678901234;
    cfg.robot.sensors[0].asset.name = "sensor esquerdo 日本".into();
    cfg.robot.sensors[1].asset.name = "sensor direito".into();
    cfg.robot.sensors[1].asset.response_model = SensorResponseModel::Polynomial {
        coefficients: vec![0.13, 0.7654321098765],
    };
    cfg.robot.sensors[1].angle_deg = 37.123456789;
    cfg.robot.sensors[1].acquisition.adc_bits = 10;
    cfg.robot.sensors[1].acquisition.seed = 98765;
    cfg.project.csv_output = None;
    cfg.project.replay_output = None;
    save_project_bundle(&cfg, &dir.0.join("project.rtsim")).unwrap();
    let loaded = load_project(dir.0.join("project.rtsim")).unwrap();
    assert_eq!(loaded.robot.name, cfg.robot.name);
    near(
        loaded.robot.motor_right.gear_ratio,
        cfg.robot.motor_right.gear_ratio,
    );
    assert_ne!(
        loaded.robot.motor_left.gear_ratio,
        loaded.robot.motor_right.gear_ratio
    );
    assert_eq!(loaded.robot.sensors.len(), 16);
    for (a, b) in cfg.robot.sensors.iter().zip(&loaded.robot.sensors) {
        assert_eq!(a.id, b.id);
        assert_eq!(a.asset.name, b.asset.name);
        near(a.position_m.x, b.position_m.x);
        near(a.position_m.y, b.position_m.y);
        near(a.angle_deg, b.angle_deg);
        assert_eq!(a.acquisition.seed, b.acquisition.seed);
        assert_eq!(a.acquisition.adc_bits, b.acquisition.adc_bits);
        assert_eq!(
            sensor_asset_json(&a.asset, 0),
            sensor_asset_json(&b.asset, 0)
        );
    }
    assert!(loaded.project.csv_output.is_none() && loaded.project.replay_output.is_none());
}

#[test]
fn portable_project_runs_after_directory_move() {
    let dir = Temp::new();
    let from = dir.0.join("old");
    let to = dir.0.join("moved");
    save_project_bundle(&config(), &from.join("project.rtsim")).unwrap();
    std::fs::rename(&from, &to).unwrap();
    let mut core =
        SimulationCore::new(load_project(to.join("project.rtsim")).unwrap(), Some(1000)).unwrap();
    core.advance_until(1000).unwrap();
    assert_eq!(core.time_us(), 1000);
    assert_eq!(core.sensor_ids().len(), 16);
}

#[test]
fn missing_asset_unknown_model_bad_schema_and_duplicate_id_fail() {
    let dir = Temp::new();
    let path = dir.0.join("robot.json");
    let cfg = config();
    let mut json = parse_json(&robot_json(&cfg.robot)).unwrap();
    if let JsonValue::Object(root) = &mut json {
        if let Some(JsonValue::Array(sensors)) = root.get_mut("sensors") {
            if let JsonValue::Object(first) = &mut sensors[0] {
                first.remove("asset");
                first.insert(
                    "asset_path".into(),
                    JsonValue::String("missing.json".into()),
                );
            }
        }
    }
    std::fs::write(&path, json.to_json().unwrap()).unwrap();
    assert!(load_robot_from_file(&path)
        .unwrap_err()
        .contains("missing.json"));
    if let JsonValue::Object(root) = &mut json {
        if let Some(JsonValue::Array(sensors)) = root.get_mut("sensors") {
            if let JsonValue::Object(first) = &mut sensors[0] {
                first.remove("asset_path");
            }
        }
    }
    std::fs::write(&path, json.to_json().unwrap()).unwrap();
    assert!(load_robot_from_file(&path)
        .unwrap_err()
        .contains("asset ou asset_path"));
    for text in [
        robot_json(&cfg.robot).replace("DcMotorSimple", "TypoMotor"),
        robot_json(&cfg.robot).replace("rtsim-robot-v8", "rtsim-robot-v99"),
    ] {
        std::fs::write(&path, text).unwrap();
        assert!(load_robot_from_file(&path).is_err());
    }
    let mut cfg = cfg;
    cfg.robot.sensors[1].id = cfg.robot.sensors[0].id.clone();
    assert!(save_robot_to_file(&cfg.robot, &path).is_err());
}

#[test]
fn invalid_numeric_types_ranges_and_unordered_curves_fail() {
    for text in [
        r#"{"mass_g":null}"#,
        r#"{"mass_g":"180"}"#,
        r#"{"mass_g":[180]}"#,
        r#"{"seed":1.5}"#,
        r#"{"adc_bits":32}"#,
        r#"{"mass_g":-1}"#,
        r#"{"force_curve":[[0.5,1],[0.2,2]]}"#,
        r#"{"sensor_type":"Typo"}"#,
        r#"{"response_model":"Typo"}"#,
    ] {
        assert!(
            robotrace_sim::io::validation::validate_document(&parse_json(text).unwrap()).is_err(),
            "accepted {text}"
        );
    }
    let mut cfg = config();
    cfg.robot.motor_left.efficiency = f64::NAN;
    assert!(ResolvedExperiment::new(&cfg).is_err());
}

#[test]
fn resolved_snapshot_is_immutable_and_has_content_fingerprints() {
    let mut cfg = config();
    let frozen = ResolvedExperiment::new(&cfg).unwrap();
    let json = frozen.to_json().to_string();
    let old = frozen.config().robot.motor_right.gear_ratio;
    cfg.robot.motor_right.gear_ratio = 2.0;
    cfg.robot.sensors[0].position_m.x = 0.25;
    assert_eq!(json, frozen.to_json());
    assert_eq!(old, frozen.config().robot.motor_right.gear_ratio);
    assert!(frozen.sources().len() >= 3);
    assert!(!frozen.warnings().is_empty());
    for source in frozen.sources() {
        assert_eq!(
            source.fnv1a64,
            fingerprint(&std::fs::read(&source.path).unwrap())
        );
    }
    assert_eq!(fingerprint(b"hello"), "a430d84680aabd0b");
    for connection in frozen.connections() {
        assert!(frozen.components().contains(&connection.source));
        assert!(frozen.components().contains(&connection.target));
    }
    parse_json(frozen.to_json()).unwrap();
}

fn independent_setup() -> LoadedConfig {
    let mut cfg = config();
    cfg.robot.sensors.truncate(2);
    cfg.project.start_pose = Pose2::default();
    cfg.track = TrackConfig {
        environment: Default::default(),
        schema: "rtsim-track-v1".into(),
        name: "line".into(),
        model: "VectorTrack".into(),
        line_width_m: 0.02,
        base_reflectance: 0.9,
        line_reflectance: 0.1,
        surface_mu: 1.0,
        centerline: vec![Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0)],
        parametric: None,
    };
    for sensor in &mut cfg.robot.sensors {
        sensor.asset.response_model = SensorResponseModel::Ideal;
        sensor.acquisition.reflectance_noise_std = 0.0;
        sensor.acquisition.adc_noise_lsb = 0.0;
    }
    cfg.robot.sensors[0].position_m = Vec2::new(0.0, 0.0);
    cfg.robot.sensors[1].position_m = Vec2::new(0.0, 0.1);
    cfg
}
#[test]
fn each_sensor_reads_its_own_world_position_and_robot_orientation() {
    let mut cfg = independent_setup();
    let sample = SimulationCore::new(cfg.clone(), Some(0)).unwrap().sample();
    assert!(sample.sensor_adc[0] < 1000 && sample.sensor_adc[1] > 3000);
    cfg.project.start_pose.y = -0.1;
    let sample = SimulationCore::new(cfg.clone(), Some(0)).unwrap().sample();
    assert!(sample.sensor_adc[0] > 3000 && sample.sensor_adc[1] < 1000);
    cfg.project.start_pose = Pose2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2);
    let sample = SimulationCore::new(cfg, Some(0)).unwrap().sample();
    assert!(sample.sensor_adc.iter().all(|v| *v < 1000));
}

#[test]
fn sensor_rng_is_independent_of_array_order_and_other_sensor_disable() {
    let mut cfg = independent_setup();
    for s in &mut cfg.robot.sensors {
        s.acquisition.reflectance_noise_std = 0.1;
    }
    let a = SimulationCore::new(cfg.clone(), Some(0))
        .unwrap()
        .sample()
        .sensor_adc;
    cfg.robot.sensors.reverse();
    let b = SimulationCore::new(cfg.clone(), Some(0))
        .unwrap()
        .sample()
        .sensor_adc;
    assert_eq!(a, vec![b[1], b[0]]);
    cfg.robot.sensors[0].enabled = false;
    let b = SimulationCore::new(cfg, Some(0))
        .unwrap()
        .sample()
        .sensor_adc;
    assert_eq!(b, vec![a[0]]);
}
