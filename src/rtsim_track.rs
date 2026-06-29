//! Parametric closed-track geometry for Robotrace/Robotrace-style tracks.
//!
//! This module is intentionally independent from the legacy `VectorTrack` runtime
//! model.  The parametric track is the authoritative geometry; sampled polylines
//! and raster maps are caches derived from it for rendering and simulation.

use crate::math::{clamp, distance_point_segment, wrap_angle, Pose2, Vec2};
use std::collections::HashSet;

const DEFAULT_SAMPLE_STEP_MM: f64 = 5.0;

#[derive(Debug, Clone)]
pub struct TrackV2 {
    pub schema: String,
    pub name: String,
    pub units: String,
    pub area: TrackArea,
    pub origin: TrackPose,
    pub rules: TrackRulesConfig,
    pub surface: TrackSurfaceConfig,
    pub segments: Vec<TrackSegment>,
    pub closure: TrackClosureConfig,
    pub markings: TrackMarkings,
}

#[derive(Debug, Clone, Copy)]
pub struct TrackArea {
    pub width_mm: f64,
    pub height_mm: f64,
    pub grid_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackPose {
    pub x_mm: f64,
    pub y_mm: f64,
    pub heading_deg: f64,
}

#[derive(Debug, Clone)]
pub struct TrackRulesConfig {
    pub profile: String,
    pub mode: TrackRulesMode,
    pub overrides: TrackRuleOverrides,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackRulesMode {
    Strict,
    Warning,
    Free,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct TrackRuleOverrides {
    pub line_width_mm: Option<f64>,
    pub max_total_length_mm: Option<f64>,
    pub min_arc_radius_mm: Option<f64>,
    pub min_distance_between_curvature_changes_mm: Option<f64>,
    pub intersection_angle_deg: Option<f64>,
    pub intersection_angle_tolerance_deg: Option<f64>,
    pub min_straight_around_intersection_mm: Option<f64>,
    pub start_finish_must_be_on_straight: Option<bool>,
    pub min_straight_around_start_finish_mm: Option<f64>,
    pub start_goal_distance_mm: Option<f64>,
    pub start_goal_area_half_width_mm: Option<f64>,
    pub min_table_edge_clearance_mm: Option<f64>,
    pub max_slope_deg: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct TrackRuleSet {
    pub line_width_mm: f64,
    pub max_total_length_mm: f64,
    pub min_arc_radius_mm: f64,
    pub min_distance_between_curvature_changes_mm: f64,
    pub intersection_angle_deg: f64,
    pub intersection_angle_tolerance_deg: f64,
    pub min_straight_around_intersection_mm: f64,
    pub start_finish_must_be_on_straight: bool,
    pub min_straight_around_start_finish_mm: f64,
    pub start_goal_distance_mm: f64,
    pub start_goal_area_half_width_mm: f64,
    pub min_table_edge_clearance_mm: f64,
    pub max_slope_deg: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct TrackClosureConfig {
    pub required: bool,
    pub position_tolerance_mm: f64,
    pub heading_tolerance_deg: f64,
}

#[derive(Debug, Clone)]
pub struct TrackSurfaceConfig {
    pub base_color: String,
    pub line_color: String,
    pub base_reflectance: f64,
    pub line_reflectance: f64,
    pub surface_mu: f64,
}

#[derive(Debug, Clone)]
pub enum TrackSegment {
    Straight(StraightSegment),
    Arc(ArcSegment),
}

#[derive(Debug, Clone)]
pub struct StraightSegment {
    pub id: String,
    pub length_mm: f64,
}

#[derive(Debug, Clone)]
pub struct ArcSegment {
    pub id: String,
    pub radius_mm: f64,
    pub sweep_deg: f64,
}

#[derive(Debug, Clone)]
pub struct TrackMarkings {
    pub start_finish: StartFinishMarking,
    pub corner_markers: TrackCornerMarkersConfig,
}

#[derive(Debug, Clone)]
pub struct StartFinishMarking {
    pub enabled: bool,
    pub segment_id: String,
    pub start_s_mm: f64,
    pub distance_mm: f64,
    pub margin_mm: f64,
    pub exit_direction: StartExitDirection,
    pub robot_start: RobotStartConfig,
}

#[derive(Debug, Clone, Copy)]
pub struct RobotStartConfig {
    pub delta_x_mm: f64,
    pub delta_y_mm: f64,
    pub heading_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartExitDirection {
    ToIncreasingS,
    ToDecreasingS,
}

#[derive(Debug, Clone)]
pub struct TrackCornerMarkersConfig {
    pub auto_generate: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkerSide {
    Left,
    Right,
    Center,
}

#[derive(Debug, Clone)]
pub struct TrackGeometry {
    pub segment_poses: Vec<SegmentPose>,
    pub samples: Vec<CenterlineSample>,
    pub centerline_m: Vec<Vec2>,
    pub total_length_mm: f64,
    pub final_pose: TrackPose,
    pub closure_error: TrackClosureError,
}

#[derive(Debug, Clone)]
pub struct SegmentPose {
    pub id: String,
    pub kind: &'static str,
    pub start_s_mm: f64,
    pub end_s_mm: f64,
    pub start: TrackPose,
    pub end: TrackPose,
    pub curvature_1_per_mm: f64,
}

#[derive(Debug, Clone)]
pub struct CenterlineSample {
    pub s_mm: f64,
    pub pose: TrackPose,
    pub segment_id: Option<String>,
}

#[derive(Debug, Clone, Copy)]
pub struct TrackClosureError {
    pub dx_mm: f64,
    pub dy_mm: f64,
    pub distance_mm: f64,
    pub heading_error_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Error,
    Warning,
    Info,
}

#[derive(Debug, Clone)]
pub struct TrackValidationIssue {
    pub severity: Severity,
    pub rule_id: String,
    pub segment_id: Option<String>,
    pub message: String,
}

#[derive(Debug, Clone)]
pub struct ClosestPoint {
    pub point: Vec2,
    pub distance_m: f64,
    pub s_mm: f64,
}

#[derive(Debug, Clone)]
pub struct StartFinishSegmentOption {
    pub id: String,
    pub length_mm: f64,
}

#[derive(Debug, Clone)]
pub struct ResolvedStartFinish {
    pub segment_id: String,
    pub start_local_s_mm: f64,
    pub finish_local_s_mm: f64,
    pub start_abs_s_mm: f64,
    pub finish_abs_s_mm: f64,
    pub start_pose: TrackPose,
    pub finish_pose: TrackPose,
    pub travel_heading_deg: f64,
}

pub const ROBOT_START_MAX_LATERAL_OFFSET_MM: f64 = 250.0;
pub const ROBOT_START_MARKER_CLEARANCE_MM: f64 = 125.0;

impl TrackV2 {
    pub fn default_closed_rectangle() -> Self {
        Self {
            schema: "rtsim-track-v2".to_string(),
            name: "Track v2 fechado".to_string(),
            units: "mm".to_string(),
            area: TrackArea {
                width_mm: 3000.0,
                height_mm: 2000.0,
                grid_mm: 100.0,
            },
            origin: TrackPose {
                x_mm: 500.0,
                y_mm: 500.0,
                heading_deg: 0.0,
            },
            rules: TrackRulesConfig {
                profile: "robotrace official".to_string(),
                mode: TrackRulesMode::Warning,
                overrides: TrackRuleOverrides::default(),
            },
            surface: TrackSurfaceConfig {
                base_color: "black".to_string(),
                line_color: "white".to_string(),
                base_reflectance: 0.08,
                line_reflectance: 0.86,
                surface_mu: 1.20,
            },
            segments: vec![
                TrackSegment::Straight(StraightSegment {
                    id: "R1".to_string(),
                    length_mm: 1200.0,
                }),
                TrackSegment::Arc(ArcSegment {
                    id: "C1".to_string(),
                    radius_mm: 300.0,
                    sweep_deg: 90.0,
                }),
                TrackSegment::Straight(StraightSegment {
                    id: "R2".to_string(),
                    length_mm: 600.0,
                }),
                TrackSegment::Arc(ArcSegment {
                    id: "C2".to_string(),
                    radius_mm: 300.0,
                    sweep_deg: 90.0,
                }),
                TrackSegment::Straight(StraightSegment {
                    id: "R3".to_string(),
                    length_mm: 1200.0,
                }),
                TrackSegment::Arc(ArcSegment {
                    id: "C3".to_string(),
                    radius_mm: 300.0,
                    sweep_deg: 90.0,
                }),
                TrackSegment::Straight(StraightSegment {
                    id: "R4".to_string(),
                    length_mm: 600.0,
                }),
                TrackSegment::Arc(ArcSegment {
                    id: "C4".to_string(),
                    radius_mm: 300.0,
                    sweep_deg: 90.0,
                }),
            ],
            closure: TrackClosureConfig {
                required: true,
                position_tolerance_mm: 0.5,
                heading_tolerance_deg: 0.1,
            },
            markings: TrackMarkings {
                start_finish: StartFinishMarking {
                    enabled: true,
                    segment_id: "R1".to_string(),
                    start_s_mm: 100.0,
                    distance_mm: 1000.0,
                    margin_mm: 100.0,
                    exit_direction: StartExitDirection::ToIncreasingS,
                    robot_start: RobotStartConfig {
                        delta_x_mm: ROBOT_START_MARKER_CLEARANCE_MM,
                        delta_y_mm: 0.0,
                        heading_deg: 0.0,
                    },
                },
                corner_markers: TrackCornerMarkersConfig {
                    auto_generate: true,
                },
            },
        }
    }
}

impl TrackRulesMode {
    pub fn as_str(self) -> &'static str {
        match self {
            TrackRulesMode::Strict => "strict",
            TrackRulesMode::Warning => "warning",
            TrackRulesMode::Free => "free",
        }
    }

    pub fn from_str(value: &str) -> Self {
        match value.trim().to_ascii_lowercase().as_str() {
            "strict" => TrackRulesMode::Strict,
            "free" => TrackRulesMode::Free,
            _ => TrackRulesMode::Warning,
        }
    }
}

impl StartExitDirection {
    pub fn as_str(self) -> &'static str {
        match self {
            StartExitDirection::ToIncreasingS => "to_increasing_s",
            StartExitDirection::ToDecreasingS => "to_decreasing_s",
        }
    }

    pub fn from_str(value: &str) -> Self {
        match value.trim().to_ascii_lowercase().as_str() {
            "to_decreasing_s" | "decreasing" | "decreasing_s" | "left" | "esquerda" => {
                StartExitDirection::ToDecreasingS
            }
            _ => StartExitDirection::ToIncreasingS,
        }
    }

    pub fn is_increasing_s(self) -> bool {
        self == StartExitDirection::ToIncreasingS
    }

    pub fn ui_label(self) -> &'static str {
        match self {
            StartExitDirection::ToIncreasingS => "sair para a direita (+s)",
            StartExitDirection::ToDecreasingS => "sair para a esquerda (-s)",
        }
    }
}

impl MarkerSide {
    pub fn as_str(self) -> &'static str {
        match self {
            MarkerSide::Left => "left",
            MarkerSide::Right => "right",
            MarkerSide::Center => "center",
        }
    }

    pub fn from_str(value: &str) -> Self {
        match value.trim().to_ascii_lowercase().as_str() {
            "left" | "esquerda" => MarkerSide::Left,
            "center" | "centre" | "centro" => MarkerSide::Center,
            _ => MarkerSide::Right,
        }
    }
}

impl TrackSegment {
    pub fn id(&self) -> &str {
        match self {
            TrackSegment::Straight(s) => &s.id,
            TrackSegment::Arc(a) => &a.id,
        }
    }

    pub fn id_mut(&mut self) -> &mut String {
        match self {
            TrackSegment::Straight(s) => &mut s.id,
            TrackSegment::Arc(a) => &mut a.id,
        }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            TrackSegment::Straight(_) => "straight",
            TrackSegment::Arc(_) => "arc",
        }
    }

    pub fn length_mm(&self) -> f64 {
        match self {
            TrackSegment::Straight(s) => s.length_mm.max(0.0),
            TrackSegment::Arc(a) => a.radius_mm.max(0.0) * a.sweep_deg.to_radians().abs(),
        }
    }

    pub fn curvature_1_per_mm(&self) -> f64 {
        match self {
            TrackSegment::Straight(_) => 0.0,
            TrackSegment::Arc(a) if a.radius_mm.abs() > 1e-9 => {
                a.sweep_deg.signum() / a.radius_mm.abs()
            }
            TrackSegment::Arc(_) => 0.0,
        }
    }
}

pub fn official_rules() -> TrackRuleSet {
    TrackRuleSet {
        line_width_mm: 19.0,
        max_total_length_mm: 60_000.0,
        min_arc_radius_mm: 100.0,
        min_distance_between_curvature_changes_mm: 100.0,
        intersection_angle_deg: 90.0,
        intersection_angle_tolerance_deg: 5.0,
        min_straight_around_intersection_mm: 100.0,
        start_finish_must_be_on_straight: true,
        min_straight_around_start_finish_mm: 100.0,
        start_goal_distance_mm: 1000.0,
        start_goal_area_half_width_mm: 200.0,
        min_table_edge_clearance_mm: 200.0,
        max_slope_deg: 5.0,
    }
}

pub fn resolve_rules(cfg: &TrackRulesConfig) -> TrackRuleSet {
    let base = official_rules();
    let o = cfg.overrides;
    TrackRuleSet {
        line_width_mm: o.line_width_mm.unwrap_or(base.line_width_mm),
        max_total_length_mm: o.max_total_length_mm.unwrap_or(base.max_total_length_mm),
        min_arc_radius_mm: o.min_arc_radius_mm.unwrap_or(base.min_arc_radius_mm),
        min_distance_between_curvature_changes_mm: o
            .min_distance_between_curvature_changes_mm
            .unwrap_or(base.min_distance_between_curvature_changes_mm),
        intersection_angle_deg: o
            .intersection_angle_deg
            .unwrap_or(base.intersection_angle_deg),
        intersection_angle_tolerance_deg: o
            .intersection_angle_tolerance_deg
            .unwrap_or(base.intersection_angle_tolerance_deg),
        min_straight_around_intersection_mm: o
            .min_straight_around_intersection_mm
            .unwrap_or(base.min_straight_around_intersection_mm),
        start_finish_must_be_on_straight: o
            .start_finish_must_be_on_straight
            .unwrap_or(base.start_finish_must_be_on_straight),
        min_straight_around_start_finish_mm: o
            .min_straight_around_start_finish_mm
            .unwrap_or(base.min_straight_around_start_finish_mm),
        start_goal_distance_mm: o
            .start_goal_distance_mm
            .unwrap_or(base.start_goal_distance_mm),
        start_goal_area_half_width_mm: o
            .start_goal_area_half_width_mm
            .unwrap_or(base.start_goal_area_half_width_mm),
        min_table_edge_clearance_mm: o
            .min_table_edge_clearance_mm
            .unwrap_or(base.min_table_edge_clearance_mm),
        max_slope_deg: o.max_slope_deg.unwrap_or(base.max_slope_deg),
    }
}

pub fn build_geometry(track: &TrackV2) -> TrackGeometry {
    build_geometry_with_step(track, DEFAULT_SAMPLE_STEP_MM)
}

pub fn build_geometry_with_step(track: &TrackV2, sample_step_mm: f64) -> TrackGeometry {
    let mut pose = track.origin;
    let mut s_total = 0.0;
    let mut segment_poses = Vec::with_capacity(track.segments.len());
    let mut samples = vec![CenterlineSample {
        s_mm: 0.0,
        pose,
        segment_id: None,
    }];
    let mut centerline_m = vec![Vec2::new(pose.x_mm / 1000.0, pose.y_mm / 1000.0)];

    let step_mm = sample_step_mm.max(0.25);
    for seg in &track.segments {
        let start = pose;
        let start_s = s_total;
        append_segment_samples(
            seg,
            start,
            start_s,
            step_mm,
            &mut samples,
            &mut centerline_m,
        );
        let len = seg.length_mm();
        pose = end_pose_of_segment(start, seg);
        s_total += len;
        segment_poses.push(SegmentPose {
            id: seg.id().to_string(),
            kind: seg.kind(),
            start_s_mm: start_s,
            end_s_mm: s_total,
            start,
            end: pose,
            curvature_1_per_mm: seg.curvature_1_per_mm(),
        });
    }

    let dx = pose.x_mm - track.origin.x_mm;
    let dy = pose.y_mm - track.origin.y_mm;
    let heading_error_deg = wrap_degrees(pose.heading_deg - track.origin.heading_deg);
    TrackGeometry {
        segment_poses,
        samples,
        centerline_m,
        total_length_mm: s_total,
        final_pose: pose,
        closure_error: TrackClosureError {
            dx_mm: dx,
            dy_mm: dy,
            distance_mm: (dx * dx + dy * dy).sqrt(),
            heading_error_deg,
        },
    }
}

pub fn sample_centerline(track: &TrackV2, step_mm: f64) -> Vec<Vec2> {
    build_geometry_with_step(track, step_mm).centerline_m
}

pub fn pose_at_s(track: &TrackV2, s_mm: f64) -> Option<TrackPose> {
    let mut pose = track.origin;
    let mut cursor = 0.0;
    for seg in &track.segments {
        let len = seg.length_mm();
        if s_mm <= cursor + len || seg.id() == track.segments.last().map(|s| s.id()).unwrap_or("") {
            let local_s = clamp(s_mm - cursor, 0.0, len);
            return Some(pose_at_local_s(pose, seg, local_s));
        }
        pose = end_pose_of_segment(pose, seg);
        cursor += len;
    }
    if track.segments.is_empty() {
        Some(track.origin)
    } else {
        Some(pose)
    }
}

pub fn curvature_at_s(track: &TrackV2, s_mm: f64) -> f64 {
    let mut cursor = 0.0;
    for seg in &track.segments {
        let len = seg.length_mm();
        if s_mm <= cursor + len {
            return seg.curvature_1_per_mm();
        }
        cursor += len;
    }
    0.0
}

pub fn closest_point_on_centerline(track: &TrackV2, point_m: Vec2) -> Option<ClosestPoint> {
    let geometry = build_geometry_with_step(track, DEFAULT_SAMPLE_STEP_MM);
    closest_point_on_samples(&geometry.centerline_m, point_m, geometry.total_length_mm)
}

pub fn distance_to_centerline_m(track: &TrackV2, point_m: Vec2) -> f64 {
    closest_point_on_centerline(track, point_m)
        .map(|p| p.distance_m)
        .unwrap_or(f64::INFINITY)
}

pub fn reflectance_at(track: &TrackV2, point_m: Vec2) -> f64 {
    let rules = resolve_rules(&track.rules);
    if distance_to_centerline_m(track, point_m) <= rules.line_width_mm / 2000.0 {
        track.surface.line_reflectance
    } else {
        track.surface.base_reflectance
    }
}

pub fn surface_mu_at(track: &TrackV2, _point_m: Vec2) -> f64 {
    track.surface.surface_mu
}

pub fn is_inside_track_area(track: &TrackV2, point_m: Vec2) -> bool {
    let x_mm = point_m.x * 1000.0;
    let y_mm = point_m.y * 1000.0;
    x_mm >= 0.0 && x_mm <= track.area.width_mm && y_mm >= 0.0 && y_mm <= track.area.height_mm
}

pub fn validate_track(track: &TrackV2) -> Vec<TrackValidationIssue> {
    let mut issues = Vec::new();
    let rules = resolve_rules(&track.rules);
    let geometry = build_geometry(track);
    validate_segments(track, &rules, &mut issues);
    validate_closure(track, &geometry, &mut issues);
    validate_rules(track, &geometry, &rules, &mut issues);
    validate_markings(track, &geometry, &rules, &mut issues);
    validate_crossings(track, &geometry, &rules, &mut issues);
    issues
}

pub fn has_blocking_errors(track: &TrackV2) -> bool {
    track.rules.mode == TrackRulesMode::Strict
        && validate_track(track)
            .iter()
            .any(|issue| issue.severity == Severity::Error)
}

pub fn auto_close_with_straight(track: &mut TrackV2) -> Result<(), String> {
    let geometry = build_geometry(track);
    let final_pose = geometry.final_pose;
    let heading_rad = final_pose.heading_deg.to_radians();
    let target = Vec2::new(track.origin.x_mm, track.origin.y_mm);
    let current = Vec2::new(final_pose.x_mm, final_pose.y_mm);
    let delta = target - current;
    let forward = Vec2::new(heading_rad.cos(), heading_rad.sin());
    let lateral_error = (delta.x * -forward.y + delta.y * forward.x).abs();
    let length = delta.dot(forward);
    if lateral_error > track.closure.position_tolerance_mm.max(0.5) || length <= 0.0 {
        return Err(format!(
            "reta final não fecha a pista: erro lateral {:.2} mm, avanço {:.2} mm",
            lateral_error, length
        ));
    }
    let id = next_segment_id(&track.segments, "R");
    track.segments.push(TrackSegment::Straight(StraightSegment {
        id,
        length_mm: length,
    }));
    Ok(())
}

pub fn auto_close_adjust_last_straight(track: &mut TrackV2) -> Result<(), String> {
    let Some(TrackSegment::Straight(last)) = track.segments.last().cloned() else {
        return Err("último segmento não é uma reta".to_string());
    };
    let old_len = last.length_mm;
    for candidate_len in [old_len + 1.0, old_len - 1.0, old_len] {
        let mut trial = track.clone();
        if let Some(TrackSegment::Straight(last_trial)) = trial.segments.last_mut() {
            last_trial.length_mm = candidate_len.max(0.001);
        }
        let geom = build_geometry(&trial);
        let final_pose = geom.final_pose;
        let heading_rad = final_pose.heading_deg.to_radians();
        let target = Vec2::new(trial.origin.x_mm, trial.origin.y_mm);
        let current = Vec2::new(final_pose.x_mm, final_pose.y_mm);
        let delta = target - current;
        let forward = Vec2::new(heading_rad.cos(), heading_rad.sin());
        let correction = delta.dot(forward);
        if let Some(TrackSegment::Straight(last_trial)) = trial.segments.last_mut() {
            last_trial.length_mm = (candidate_len + correction).max(0.001);
        }
        let closure = build_geometry(&trial).closure_error;
        if closure.distance_mm <= trial.closure.position_tolerance_mm
            && closure.heading_error_deg.abs() <= trial.closure.heading_tolerance_deg
        {
            *track = trial;
            return Ok(());
        }
    }
    Err("não foi possível ajustar apenas a última reta".to_string())
}

pub fn next_segment_id(segments: &[TrackSegment], prefix: &str) -> String {
    let used: HashSet<&str> = segments.iter().map(|s| s.id()).collect();
    for idx in 1..10_000 {
        let candidate = format!("{prefix}{idx}");
        if !used.contains(candidate.as_str()) {
            return candidate;
        }
    }
    format!("{prefix}{}", segments.len() + 1)
}

fn append_segment_samples(
    seg: &TrackSegment,
    start: TrackPose,
    start_s: f64,
    step_mm: f64,
    samples: &mut Vec<CenterlineSample>,
    centerline_m: &mut Vec<Vec2>,
) {
    let len = seg.length_mm();
    let n = (len / step_mm).ceil().max(1.0) as usize;
    for i in 1..=n {
        let local_s = len * (i as f64 / n as f64);
        let pose = pose_at_local_s(start, seg, local_s);
        samples.push(CenterlineSample {
            s_mm: start_s + local_s,
            pose,
            segment_id: Some(seg.id().to_string()),
        });
        centerline_m.push(Vec2::new(pose.x_mm / 1000.0, pose.y_mm / 1000.0));
    }
}

fn pose_at_local_s(start: TrackPose, seg: &TrackSegment, local_s_mm: f64) -> TrackPose {
    match seg {
        TrackSegment::Straight(s) => {
            let theta = start.heading_deg.to_radians();
            let l = clamp(local_s_mm, 0.0, s.length_mm.max(0.0));
            TrackPose {
                x_mm: start.x_mm + theta.cos() * l,
                y_mm: start.y_mm + theta.sin() * l,
                heading_deg: start.heading_deg,
            }
        }
        TrackSegment::Arc(a) => {
            let radius = a.radius_mm.abs().max(1e-9);
            let sweep = a.sweep_deg.to_radians();
            let arc_len = radius * sweep.abs();
            let frac = if arc_len > 1e-12 {
                clamp(local_s_mm / arc_len, 0.0, 1.0)
            } else {
                0.0
            };
            let partial_sweep = sweep * frac;
            pose_after_arc(start, radius, partial_sweep)
        }
    }
}

fn end_pose_of_segment(start: TrackPose, seg: &TrackSegment) -> TrackPose {
    pose_at_local_s(start, seg, seg.length_mm())
}

fn pose_after_arc(start: TrackPose, radius_mm: f64, sweep_rad: f64) -> TrackPose {
    let theta = start.heading_deg.to_radians();
    let sign = if sweep_rad >= 0.0 { 1.0 } else { -1.0 };
    let left = Vec2::new(-theta.sin(), theta.cos());
    let center = Vec2::new(start.x_mm, start.y_mm) + left * (sign * radius_mm);
    let local = Vec2::new(start.x_mm, start.y_mm) - center;
    let c = sweep_rad.cos();
    let s = sweep_rad.sin();
    let rotated = Vec2::new(local.x * c - local.y * s, local.x * s + local.y * c);
    let p = center + rotated;
    TrackPose {
        x_mm: p.x,
        y_mm: p.y,
        heading_deg: normalize_degrees(start.heading_deg + sweep_rad.to_degrees()),
    }
}

fn validate_segments(
    track: &TrackV2,
    rules: &TrackRuleSet,
    issues: &mut Vec<TrackValidationIssue>,
) {
    let mut ids = HashSet::new();
    for seg in &track.segments {
        if seg.id().trim().is_empty() {
            push_issue(
                issues,
                Severity::Error,
                "segment.id.empty",
                Some(seg.id()),
                "segmento com ID vazio",
            );
        }
        if !ids.insert(seg.id().to_string()) {
            push_issue(
                issues,
                Severity::Error,
                "segment.id.unique",
                Some(seg.id()),
                format!("ID de segmento duplicado: {}", seg.id()),
            );
        }
        match seg {
            TrackSegment::Straight(s) if s.length_mm <= 0.0 => push_issue(
                issues,
                Severity::Error,
                "segment.straight.length",
                Some(&s.id),
                "reta precisa ter comprimento maior que zero",
            ),
            TrackSegment::Arc(a) => {
                if a.radius_mm <= 0.0 {
                    push_issue(
                        issues,
                        Severity::Error,
                        "segment.arc.radius.positive",
                        Some(&a.id),
                        "arco precisa ter raio maior que zero",
                    );
                }
                if a.sweep_deg.abs() <= 1e-9 {
                    push_issue(
                        issues,
                        Severity::Error,
                        "segment.arc.sweep.nonzero",
                        Some(&a.id),
                        "arco precisa ter ângulo diferente de zero",
                    );
                }
                if a.radius_mm > 0.0 && a.radius_mm < rules.min_arc_radius_mm {
                    push_issue(
                        issues,
                        rule_severity(track.rules.mode),
                        "official.min_arc_radius_mm",
                        Some(&a.id),
                        format!(
                            "raio da curva {} = {:.1} mm, mínimo permitido = {:.1} mm",
                            a.id, a.radius_mm, rules.min_arc_radius_mm
                        ),
                    );
                }
            }
            _ => {}
        }
    }
    if track.segments.is_empty() {
        push_issue(
            issues,
            Severity::Error,
            "segments.nonempty",
            None::<&str>,
            "pista precisa ter pelo menos um segmento",
        );
    }
}

fn validate_closure(
    track: &TrackV2,
    geometry: &TrackGeometry,
    issues: &mut Vec<TrackValidationIssue>,
) {
    let err = geometry.closure_error;
    if track.closure.required
        && (err.distance_mm > track.closure.position_tolerance_mm
            || err.heading_error_deg.abs() > track.closure.heading_tolerance_deg)
    {
        push_issue(
            issues,
            rule_severity(track.rules.mode),
            "closure.required",
            None::<&str>,
            format!(
                "pista não fecha; erro final = {:.2} mm, dx = {:.2} mm, dy = {:.2} mm, dθ = {:.3}°",
                err.distance_mm, err.dx_mm, err.dy_mm, err.heading_error_deg
            ),
        );
    }
}

fn validate_rules(
    track: &TrackV2,
    geometry: &TrackGeometry,
    rules: &TrackRuleSet,
    issues: &mut Vec<TrackValidationIssue>,
) {
    if geometry.total_length_mm > rules.max_total_length_mm {
        push_issue(
            issues,
            rule_severity(track.rules.mode),
            "official.max_total_length_mm",
            None::<&str>,
            format!(
                "comprimento total = {:.2} m, limite oficial = {:.2} m",
                geometry.total_length_mm / 1000.0,
                rules.max_total_length_mm / 1000.0
            ),
        );
    }

    let official = official_rules();
    if track
        .rules
        .profile
        .to_ascii_lowercase()
        .contains("official")
        || track.rules.profile.to_ascii_lowercase().contains("rob")
    {
        if (rules.line_width_mm - official.line_width_mm).abs() > 0.01 {
            push_issue(
                issues,
                Severity::Warning,
                "official.line_width_mm.override",
                None::<&str>,
                format!(
                    "largura da linha = {:.1} mm, regra oficial = {:.1} mm",
                    rules.line_width_mm, official.line_width_mm
                ),
            );
        }
    }

    let mut last_change_s: Option<f64> = None;
    let mut last_curvature = 0.0;
    for seg in &geometry.segment_poses {
        if (seg.curvature_1_per_mm - last_curvature).abs() > 1e-12 {
            if let Some(prev_s) = last_change_s {
                let ds = (seg.start_s_mm - prev_s).abs();
                if ds < rules.min_distance_between_curvature_changes_mm {
                    push_issue(
                        issues,
                        rule_severity(track.rules.mode),
                        "official.min_distance_between_curvature_changes_mm",
                        Some(&seg.id),
                        format!(
                            "mudança de curvatura muito próxima: {:.1} mm, mínimo = {:.1} mm",
                            ds, rules.min_distance_between_curvature_changes_mm
                        ),
                    );
                }
            }
            last_change_s = Some(seg.start_s_mm);
            last_curvature = seg.curvature_1_per_mm;
        }
    }

    for sample in &geometry.samples {
        let clearance = min_edge_clearance_mm(track, sample.pose.x_mm, sample.pose.y_mm);
        if clearance < rules.min_table_edge_clearance_mm {
            push_issue(
                issues,
                rule_severity(track.rules.mode),
                "official.min_table_edge_clearance_mm",
                sample.segment_id.as_deref(),
                format!(
                    "pista muito próxima da borda: {:.1} mm, mínimo = {:.1} mm",
                    clearance, rules.min_table_edge_clearance_mm
                ),
            );
            break;
        }
    }
}

fn validate_markings(
    track: &TrackV2,
    geometry: &TrackGeometry,
    _rules: &TrackRuleSet,
    issues: &mut Vec<TrackValidationIssue>,
) {
    let sf = &track.markings.start_finish;
    let valid_segments = valid_start_finish_segments(track);

    if sf.enabled && valid_segments.is_empty() {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.no_valid_straight",
            None::<&str>,
            format!(
                "nenhuma reta comporta Start/Finish: mínimo = {:.1} mm",
                start_finish_required_length_mm(sf)
            ),
        );
    }

    if sf.distance_mm <= 0.0 {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.distance_positive",
            Some(&sf.segment_id),
            "distância Start/Finish precisa ser maior que zero",
        );
    }
    if sf.margin_mm < 0.0 {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.margin_non_negative",
            Some(&sf.segment_id),
            "margem Start/Finish não pode ser negativa",
        );
    }

    if !sf.enabled {
        return;
    }

    let Some(seg) = geometry
        .segment_poses
        .iter()
        .find(|seg| seg.id == sf.segment_id)
    else {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.segment_exists",
            Some(&sf.segment_id),
            format!(
                "Start/Finish aponta para segmento inexistente: {}",
                sf.segment_id
            ),
        );
        return;
    };

    if seg.kind != "straight" {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.must_be_straight",
            Some(&sf.segment_id),
            "Start/Finish só pode ser configurado em reta; arcos não são opções válidas",
        );
        return;
    }

    let seg_len = seg.end_s_mm - seg.start_s_mm;
    let required_len = start_finish_required_length_mm(sf);
    if seg_len + 1e-6 < required_len {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.straight_length",
            Some(&sf.segment_id),
            format!(
                "reta tem {:.1} mm; necessário >= margem {:.1} + distância {:.1} + margem {:.1} = {:.1} mm",
                seg_len, sf.margin_mm, sf.distance_mm, sf.margin_mm, required_len
            ),
        );
    }

    if sf.start_s_mm + 1e-6 < sf.margin_mm {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.start_margin",
            Some(&sf.segment_id),
            format!(
                "start_s_mm = {:.1} mm viola margem mínima de {:.1} mm",
                sf.start_s_mm, sf.margin_mm
            ),
        );
    }

    if sf.start_s_mm + sf.distance_mm > seg_len - sf.margin_mm + 1e-6 {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.finish_margin",
            Some(&sf.segment_id),
            format!(
                "start_s_mm + distance_mm = {:.1} mm precisa ser <= comprimento {:.1} - margem {:.1} = {:.1} mm",
                sf.start_s_mm + sf.distance_mm,
                seg_len,
                sf.margin_mm,
                seg_len - sf.margin_mm
            ),
        );
    }

    let (start_local, finish_local) = start_finish_local_positions(sf);
    let local_distance = (start_local - finish_local).abs();
    if (local_distance - sf.distance_mm).abs() > 1e-3 {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.distance_exact",
            Some(&sf.segment_id),
            format!(
                "distância calculada Start/Finish = {:.3} mm; esperada = {:.3} mm",
                local_distance, sf.distance_mm
            ),
        );
    }

    let robot_min_x = ROBOT_START_MARKER_CLEARANCE_MM;
    let robot_max_x = sf.distance_mm - ROBOT_START_MARKER_CLEARANCE_MM;
    if robot_max_x < robot_min_x {
        push_issue(
            issues,
            Severity::Error,
            "marking.start_finish.robot_area_length",
            Some(&sf.segment_id),
            format!(
                "a área de início do robô exige pelo menos {:.1} mm entre START e FINISH",
                ROBOT_START_MARKER_CLEARANCE_MM * 2.0
            ),
        );
    } else if sf.robot_start.delta_x_mm < robot_min_x - 1e-6
        || sf.robot_start.delta_x_mm > robot_max_x + 1e-6
    {
        push_issue(
            issues,
            Severity::Warning,
            "marking.start_finish.robot_delta_x",
            Some(&sf.segment_id),
            format!(
                "ΔX do robô deve ficar entre {:.1} mm e {:.1} mm para manter {:.1} mm dos marcadores",
                robot_min_x, robot_max_x, ROBOT_START_MARKER_CLEARANCE_MM
            ),
        );
    }

    if sf.robot_start.delta_y_mm.abs() > ROBOT_START_MAX_LATERAL_OFFSET_MM + 1e-6 {
        push_issue(
            issues,
            Severity::Warning,
            "marking.start_finish.robot_delta_y",
            Some(&sf.segment_id),
            format!(
                "ΔY do robô deve ficar dentro de ±{:.1} mm da linha",
                ROBOT_START_MAX_LATERAL_OFFSET_MM
            ),
        );
    }
}

pub fn start_finish_required_length_mm(sf: &StartFinishMarking) -> f64 {
    sf.margin_mm.max(0.0) + sf.distance_mm.max(0.0) + sf.margin_mm.max(0.0)
}

pub fn valid_start_finish_segments(track: &TrackV2) -> Vec<StartFinishSegmentOption> {
    let geometry = build_geometry(track);
    let required_len = start_finish_required_length_mm(&track.markings.start_finish);
    geometry
        .segment_poses
        .iter()
        .filter(|seg| seg.kind == "straight")
        .filter_map(|seg| {
            let length_mm = seg.end_s_mm - seg.start_s_mm;
            (length_mm + 1e-6 >= required_len).then(|| StartFinishSegmentOption {
                id: seg.id.clone(),
                length_mm,
            })
        })
        .collect()
}

pub fn center_start_finish_on_segment(track: &mut TrackV2, segment_length_mm: f64) {
    let sf = &mut track.markings.start_finish;
    let max_start = (segment_length_mm - sf.margin_mm - sf.distance_mm).max(sf.margin_mm);
    sf.start_s_mm = ((segment_length_mm - sf.distance_mm) * 0.5).clamp(sf.margin_mm, max_start);
}

pub fn clamp_start_finish_to_segment(track: &mut TrackV2, segment_length_mm: f64) {
    let sf = &mut track.markings.start_finish;
    let max_start = (segment_length_mm - sf.margin_mm - sf.distance_mm).max(sf.margin_mm);
    sf.start_s_mm = sf.start_s_mm.clamp(sf.margin_mm, max_start);
}

pub fn start_finish_local_positions(sf: &StartFinishMarking) -> (f64, f64) {
    let a = sf.start_s_mm;
    let b = sf.start_s_mm + sf.distance_mm;
    match sf.exit_direction {
        StartExitDirection::ToIncreasingS => (a, b),
        StartExitDirection::ToDecreasingS => (b, a),
    }
}

pub fn resolve_start_finish_markers(track: &TrackV2) -> Option<ResolvedStartFinish> {
    let sf = &track.markings.start_finish;
    if !sf.enabled {
        return None;
    }
    let geometry = build_geometry(track);
    let seg = geometry
        .segment_poses
        .iter()
        .find(|seg| seg.id == sf.segment_id && seg.kind == "straight")?;
    let (start_local, finish_local) = start_finish_local_positions(sf);
    let start_abs = seg.start_s_mm + start_local;
    let finish_abs = seg.start_s_mm + finish_local;
    let mut start_pose = pose_at_s(track, start_abs)?;
    let mut finish_pose = pose_at_s(track, finish_abs)?;
    let travel_heading_deg = if sf.exit_direction.is_increasing_s() {
        seg.start.heading_deg
    } else {
        normalize_degrees(seg.start.heading_deg + 180.0)
    };
    start_pose.heading_deg = travel_heading_deg;
    finish_pose.heading_deg = travel_heading_deg;
    Some(ResolvedStartFinish {
        segment_id: seg.id.clone(),
        start_local_s_mm: start_local,
        finish_local_s_mm: finish_local,
        start_abs_s_mm: start_abs,
        finish_abs_s_mm: finish_abs,
        start_pose,
        finish_pose,
        travel_heading_deg,
    })
}

pub fn resolve_robot_start_pose(track: &TrackV2) -> Option<TrackPose> {
    let resolved = resolve_start_finish_markers(track)?;
    let robot_start = track.markings.start_finish.robot_start;

    // The robot placement uses the START -> FINISH axis so delta_x_mm is easy to
    // reason about inside the permitted rectangle. The heading reference is the
    // opposite direction, because heading_deg = 0.0 means "point at START".
    let theta = resolved.travel_heading_deg.to_radians();
    let forward_start_to_finish = Vec2::new(theta.cos(), theta.sin());
    let left = Vec2::new(-theta.sin(), theta.cos());
    let base = Vec2::new(resolved.start_pose.x_mm, resolved.start_pose.y_mm);
    let p = base + forward_start_to_finish * robot_start.delta_x_mm + left * robot_start.delta_y_mm;

    Some(TrackPose {
        x_mm: p.x,
        y_mm: p.y,
        heading_deg: normalize_degrees(
            resolved.travel_heading_deg + 180.0 + robot_start.heading_deg,
        ),
    })
}

pub fn robot_start_allowed_area_corners(track: &TrackV2) -> Option<[Vec2; 4]> {
    let resolved = resolve_start_finish_markers(track)?;
    let distance = track.markings.start_finish.distance_mm;
    let min_x = ROBOT_START_MARKER_CLEARANCE_MM;
    let max_x = distance - ROBOT_START_MARKER_CLEARANCE_MM;
    if max_x < min_x {
        return None;
    }

    let theta = resolved.travel_heading_deg.to_radians();
    let forward = Vec2::new(theta.cos(), theta.sin());
    let left = Vec2::new(-theta.sin(), theta.cos());
    let base = Vec2::new(resolved.start_pose.x_mm, resolved.start_pose.y_mm);
    let y = ROBOT_START_MAX_LATERAL_OFFSET_MM;

    Some([
        base + forward * min_x + left * -y,
        base + forward * max_x + left * -y,
        base + forward * max_x + left * y,
        base + forward * min_x + left * y,
    ])
}

fn validate_crossings(
    track: &TrackV2,
    geometry: &TrackGeometry,
    rules: &TrackRuleSet,
    issues: &mut Vec<TrackValidationIssue>,
) {
    let pts = &geometry.centerline_m;
    if pts.len() < 4 {
        return;
    }
    let mut found = 0usize;
    for i in 0..pts.len() - 1 {
        for j in (i + 2)..pts.len() - 1 {
            if i == 0 && j + 2 >= pts.len() {
                continue;
            }
            let Some(_p) = segment_intersection(pts[i], pts[i + 1], pts[j], pts[j + 1]) else {
                continue;
            };
            let a = pts[i + 1] - pts[i];
            let b = pts[j + 1] - pts[j];
            let angle = angle_between_deg(a, b);
            let target = rules.intersection_angle_deg;
            let diff = (angle - target).abs().min((180.0 - angle - target).abs());
            if diff > rules.intersection_angle_tolerance_deg {
                push_issue(
                    issues,
                    rule_severity(track.rules.mode),
                    "official.intersection_angle_deg",
                    None::<&str>,
                    format!(
                        "cruzamento detectado com ângulo {:.1}°, esperado {:.1}° ± {:.1}°",
                        angle, target, rules.intersection_angle_tolerance_deg
                    ),
                );
            } else {
                push_issue(
                    issues,
                    Severity::Info,
                    "geometry.crossing.detected",
                    None::<&str>,
                    format!("cruzamento detectado com ângulo {:.1}°", angle),
                );
            }
            found += 1;
            if found >= 8 {
                return;
            }
        }
    }
}

fn closest_point_on_samples(
    samples: &[Vec2],
    p: Vec2,
    total_length_mm: f64,
) -> Option<ClosestPoint> {
    if samples.len() < 2 {
        return None;
    }
    let mut best_dist = f64::INFINITY;
    let mut best_point = samples[0];
    let mut best_s_mm = 0.0;
    let mut accumulated_m = 0.0;
    for w in samples.windows(2) {
        let a = w[0];
        let b = w[1];
        let ab = b - a;
        let denom = ab.norm2();
        let t = if denom > 1e-18 {
            clamp((p - a).dot(ab) / denom, 0.0, 1.0)
        } else {
            0.0
        };
        let candidate = a + ab * t;
        let dist = (p - candidate).norm();
        if dist < best_dist {
            best_dist = dist;
            best_point = candidate;
            best_s_mm = (accumulated_m + ab.norm() * t) * 1000.0;
        }
        accumulated_m += ab.norm();
    }
    if total_length_mm.is_finite() && total_length_mm > 0.0 {
        best_s_mm = best_s_mm.min(total_length_mm);
    }
    Some(ClosestPoint {
        point: best_point,
        distance_m: best_dist,
        s_mm: best_s_mm,
    })
}

fn min_edge_clearance_mm(track: &TrackV2, x_mm: f64, y_mm: f64) -> f64 {
    x_mm.min(y_mm)
        .min(track.area.width_mm - x_mm)
        .min(track.area.height_mm - y_mm)
}

fn segment_intersection(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Option<Vec2> {
    let r = b - a;
    let s = d - c;
    let denom = cross(r, s);
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = cross(c - a, s) / denom;
    let u = cross(c - a, r) / denom;
    if (0.001..=0.999).contains(&t) && (0.001..=0.999).contains(&u) {
        Some(a + r * t)
    } else {
        None
    }
}

fn angle_between_deg(a: Vec2, b: Vec2) -> f64 {
    let denom = (a.norm() * b.norm()).max(1e-12);
    let c = clamp(a.dot(b).abs() / denom, 0.0, 1.0);
    c.acos().to_degrees()
}

fn cross(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}

fn rule_severity(mode: TrackRulesMode) -> Severity {
    match mode {
        TrackRulesMode::Strict => Severity::Error,
        TrackRulesMode::Warning => Severity::Warning,
        TrackRulesMode::Free => Severity::Info,
    }
}

fn push_issue(
    issues: &mut Vec<TrackValidationIssue>,
    severity: Severity,
    rule_id: impl Into<String>,
    segment_id: Option<impl AsRef<str>>,
    message: impl Into<String>,
) {
    issues.push(TrackValidationIssue {
        severity,
        rule_id: rule_id.into(),
        segment_id: segment_id.map(|s| s.as_ref().to_string()),
        message: message.into(),
    });
}

fn normalize_degrees(mut a: f64) -> f64 {
    while a > 180.0 {
        a -= 360.0;
    }
    while a <= -180.0 {
        a += 360.0;
    }
    a
}

fn wrap_degrees(a: f64) -> f64 {
    wrap_angle(a.to_radians()).to_degrees()
}

#[allow(dead_code)]
fn _pose2_from_track_pose(pose: TrackPose) -> Pose2 {
    Pose2::new(
        pose.x_mm / 1000.0,
        pose.y_mm / 1000.0,
        pose.heading_deg.to_radians(),
    )
}

#[allow(dead_code)]
fn _distance_point_track_samples(samples: &[Vec2], p: Vec2) -> f64 {
    samples
        .windows(2)
        .map(|w| distance_point_segment(p, w[0], w[1]))
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_closes_with_four_arcs() {
        let track = TrackV2::default_closed_rectangle();
        let geom = build_geometry(&track);
        assert!(
            geom.closure_error.distance_mm < 1e-9,
            "{:?}",
            geom.closure_error
        );
        assert!(geom.closure_error.heading_error_deg.abs() < 1e-9);
        assert!(validate_track(&track)
            .iter()
            .all(|i| i.severity != Severity::Error));
    }

    #[test]
    fn right_arc_uses_negative_sweep() {
        let track = TrackV2 {
            segments: vec![TrackSegment::Arc(ArcSegment {
                id: "C1".to_string(),
                radius_mm: 100.0,
                sweep_deg: -90.0,
            })],
            ..TrackV2::default_closed_rectangle()
        };
        let geom = build_geometry(&track);
        assert!((geom.final_pose.x_mm - 600.0).abs() < 1e-9);
        assert!((geom.final_pose.y_mm - 400.0).abs() < 1e-9);
        assert!((geom.final_pose.heading_deg + 90.0).abs() < 1e-9);
    }

    #[test]
    fn validator_reports_open_track() {
        let mut track = TrackV2::default_closed_rectangle();
        track.segments.pop();
        let issues = validate_track(&track);
        assert!(issues.iter().any(|i| i.rule_id == "closure.required"));
    }

    #[test]
    fn pose_query_inside_segment() {
        let track = TrackV2::default_closed_rectangle();
        let pose = pose_at_s(&track, 600.0).unwrap();
        assert!((pose.x_mm - 1100.0).abs() < 1e-9);
        assert!((pose.y_mm - 500.0).abs() < 1e-9);
    }
}
