use super::{assets::resolve_from_file, models::ResolvedModels, persistence::*, validation::*};
use crate::config::{LoadedConfig, SensorResponseModel, SensorType};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComponentId(pub String);
#[derive(Debug, Clone)]
pub struct Connection {
    pub id: ComponentId,
    pub source: ComponentId,
    pub target: ComponentId,
}
#[derive(Debug, Clone)]
pub struct SourceFingerprint {
    pub path: PathBuf,
    pub fnv1a64: String,
}
#[derive(Debug, Clone)]
pub struct ResolvedExperiment {
    config: LoadedConfig,
    models: ResolvedModels,
    components: Vec<ComponentId>,
    connections: Vec<Connection>,
    sources: Vec<SourceFingerprint>,
    warnings: Vec<String>,
    snapshot: String,
}

/// Stable content fingerprint, not a cryptographic signature.
pub fn fingerprint(bytes: &[u8]) -> String {
    let hash = bytes.iter().fold(0xcbf29ce484222325u64, |h, b| {
        (h ^ *b as u64).wrapping_mul(0x100000001b3)
    });
    format!("{hash:016x}")
}
impl ResolvedExperiment {
    pub fn new(cfg: &LoadedConfig) -> Result<Self, String> {
        let mut cfg = cfg.clone();
        let track_warnings = crate::track::definition::validate_definition(&cfg.track)?;
        cfg.project.start_pose =
            crate::track::definition::effective_start_pose(&cfg.track, cfg.project.start_pose)?;
        let assembly_warnings = crate::models::robot::resolve_robot(&mut cfg.robot)?;
        if let Some(track) = &mut cfg.track.parametric {
            let rules = crate::rtsim_track::resolve_rules(&track.rules);
            track.rules.overrides.line_width_mm = Some(rules.line_width_mm);
            track.rules.overrides.max_total_length_mm = Some(rules.max_total_length_mm);
            track.rules.overrides.min_arc_radius_mm = Some(rules.min_arc_radius_mm);
            track
                .rules
                .overrides
                .min_distance_between_curvature_changes_mm =
                Some(rules.min_distance_between_curvature_changes_mm);
            track.rules.overrides.intersection_angle_deg = Some(rules.intersection_angle_deg);
            track.rules.overrides.intersection_angle_tolerance_deg =
                Some(rules.intersection_angle_tolerance_deg);
            track.rules.overrides.min_straight_around_intersection_mm =
                Some(rules.min_straight_around_intersection_mm);
            track.rules.overrides.start_finish_must_be_on_straight =
                Some(rules.start_finish_must_be_on_straight);
            track.rules.overrides.min_straight_around_start_finish_mm =
                Some(rules.min_straight_around_start_finish_mm);
            track.rules.overrides.start_goal_distance_mm = Some(rules.start_goal_distance_mm);
            track.rules.overrides.start_goal_area_half_width_mm =
                Some(rules.start_goal_area_half_width_mm);
            track.rules.overrides.min_table_edge_clearance_mm =
                Some(rules.min_table_edge_clearance_mm);
            track.rules.overrides.max_slope_deg = Some(rules.max_slope_deg);
        }

        validate_robot(&cfg.robot)?;
        validate_document(
            &crate::json::parse_json(&track_json(&cfg.track)).map_err(|e| e.to_string())?,
        )?;
        validate_document(
            &crate::json::parse_json(&project_json(&cfg.project)).map_err(|e| e.to_string())?,
        )?;
        crate::config::refresh_track_cache(&mut cfg.track);
        crate::core::scheduler::validate_time(&cfg.project.time)?;
        if let Some(p) = &cfg.robot.powertrain {
            if p.command_latency_us % cfg.project.time.physics_dt_us != 0 {
                return Err("driver latency must be a multiple of physics_dt_us".into());
            }
        }
        cfg.robot.sensing.validate(&cfg.robot, cfg.project.time)?;
        let models = ResolvedModels::from_robot(&cfg.robot)?;
        for sensor in cfg.robot.sensors.iter().filter(|s| s.enabled) {
            let n = cfg
                .robot
                .sensing
                .optical
                .get(&sensor.id)
                .cloned()
                .unwrap_or_default()
                .area_samples;
            if crate::sensor::footprint(&sensor.asset.detection_area, n).is_empty() {
                return Err(format!(
                    "{}: optical quadrature does not resolve polygon; increase area_samples",
                    sensor.id
                ));
            }

            if !matches!(
                sensor.asset.sensor_type,
                SensorType::LineAnalog | SensorType::LineDigital
            ) || matches!(
                sensor.asset.response_model,
                SensorResponseModel::Custom { .. }
            ) {
                return Err(format!("sensor {}: this runtime supports line sensors with a defined response; disable unsupported instances for this run",sensor.id));
            }
        }
        let mut components = vec![
            ComponentId("motor:left".into()),
            ComponentId("motor:right".into()),
            ComponentId("chassis".into()),
            ComponentId("battery".into()),
            ComponentId("driver".into()),
            ComponentId("controller".into()),
            ComponentId("encoder".into()),
            ComponentId("gyro".into()),
        ];
        components.extend(
            cfg.robot
                .assembly
                .as_ref()
                .unwrap()
                .wheels
                .iter()
                .map(|w| ComponentId(w.id.clone())),
        );
        components.extend(cfg.robot.sensors.iter().map(|s| ComponentId(s.id.clone())));
        components.extend(
            cfg.robot
                .normal_force
                .fans
                .iter()
                .map(|s| ComponentId(s.id.clone())),
        );
        let mut unique = std::collections::BTreeSet::new();
        if components.iter().any(|id| !unique.insert(id.0.clone())) {
            return Err("component ID conflicts with a reserved runtime component".into());
        }
        let mut connections = [("supply", "battery", "driver")]
            .map(|(id, source, target)| Connection {
                id: ComponentId(id.into()),
                source: ComponentId(source.into()),
                target: ComponentId(target.into()),
            })
            .to_vec();
        for w in &cfg.robot.assembly.as_ref().unwrap().wheels {
            if let Some(motor) = &w.motor {
                connections.push(Connection {
                    id: ComponentId(format!("drive:{}", w.id)),
                    source: ComponentId(motor.clone()),
                    target: ComponentId(w.id.clone()),
                });
            }
        }
        for sensor in &cfg.robot.sensors {
            connections.push(Connection {
                id: ComponentId(format!("readout:{}", sensor.id)),
                source: ComponentId(sensor.id.clone()),
                target: ComponentId("controller".into()),
            });
        }
        for fan in &cfg.robot.normal_force.fans {
            connections.push(Connection {
                id: ComponentId(format!("supply:{}", fan.id)),
                source: ComponentId("battery".into()),
                target: ComponentId(fan.id.clone()),
            });
        }
        let mut paths = vec![
            cfg.project_path.clone(),
            resolve_from_file(&cfg.project_path, &cfg.project.robot_path),
            resolve_from_file(&cfg.project_path, &cfg.project.track_path),
        ];
        let robot_path = paths[1].clone();
        paths.extend(
            cfg.robot
                .sensors
                .iter()
                .filter(|s| !s.asset_path.as_os_str().is_empty())
                .map(|s| resolve_from_file(&robot_path, &s.asset_path)),
        );
        paths.sort();
        paths.dedup();
        let mut sources = Vec::new();
        let mut warnings=vec!["Physical parameters have no experimental calibration certificate; fidelity is not guaranteed by schema validation.".into(),
            "Optical areas use deterministic numerical quadrature; finite area resolution should be checked for each experiment.".into()];
        warnings.extend(assembly_warnings);
        warnings.extend(track_warnings);
        for path in paths {
            match std::fs::read(&path) {
                Ok(bytes)=>sources.push(SourceFingerprint {path,fnv1a64:fingerprint(&bytes)}),
                Err(error) if error.kind()==std::io::ErrorKind::NotFound =>warnings.push(format!("No source file for {}: snapshot contains the in-memory definition/embedded asset.",path.display())),
                Err(error)=>return Err(format!("failed to fingerprint {}: {error}",path.display()))
            }
        }
        let source_json = sources
            .iter()
            .map(|s| {
                format!(
                    "{{\"path\":\"{}\",\"fnv1a64\":\"{}\"}}",
                    escape_json(&s.path.to_string_lossy()),
                    s.fnv1a64
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let connections_json = connections
            .iter()
            .map(|c| {
                format!(
                    "{{\"id\":\"{}\",\"source\":\"{}\",\"target\":\"{}\"}}",
                    escape_json(&c.id.0),
                    escape_json(&c.source.0),
                    escape_json(&c.target.0)
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let snapshot=format!("{{\"schema\":\"rtsim-resolved-experiment-v1\",\"app_version\":\"{}\",\"project\":{},\"robot\":{},\"track\":{},\"sources\":[{}],\"connections\":[{}]}}",env!("CARGO_PKG_VERSION"),project_json(&cfg.project),robot_json(&cfg.robot),track_json(&cfg.track),source_json,connections_json);
        crate::json::parse_json(&snapshot).map_err(|e| e.to_string())?;
        Ok(Self {
            config: cfg.clone(),
            models,
            components,
            connections,
            sources,
            warnings,
            snapshot,
        })
    }
    pub fn config(&self) -> &LoadedConfig {
        &self.config
    }
    pub fn models(&self) -> &ResolvedModels {
        &self.models
    }
    pub fn components(&self) -> &[ComponentId] {
        &self.components
    }
    pub fn connections(&self) -> &[Connection] {
        &self.connections
    }
    pub fn sources(&self) -> &[SourceFingerprint] {
        &self.sources
    }
    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }
    pub fn to_json(&self) -> &str {
        &self.snapshot
    }
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), String> {
        std::fs::write(path, &self.snapshot).map_err(|e| e.to_string())
    }
}

/// Embed effective assets, then use local robot/track references in a transportable folder.
pub fn save_project_bundle(cfg: &LoadedConfig, project_path: &Path) -> Result<(), String> {
    let mut copy = cfg.clone();
    copy.project_path = project_path.into();
    copy.project.robot_path = PathBuf::from("robot.json");
    copy.project.track_path = PathBuf::from("track.json");
    for sensor in &mut copy.robot.sensors {
        sensor.asset_path = PathBuf::new();
    }
    copy.project.csv_output = cfg
        .project
        .csv_output
        .as_ref()
        .map(|_| PathBuf::from("resultado.csv"));
    copy.project.replay_output = cfg
        .project
        .replay_output
        .as_ref()
        .map(|_| PathBuf::from("resultado.rtlog"));
    save_loaded_config(&copy)
}
