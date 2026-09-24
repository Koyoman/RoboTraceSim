use crate::{
    config::TrackConfig,
    math::{Pose2, Vec2},
};
#[derive(Debug, Clone)]
pub struct RectArea {
    pub center_m: Vec2,
    pub size_m: Vec2,
    pub angle_deg: f64,
}
impl RectArea {
    pub fn contains(&self, p: Vec2) -> bool {
        let d = p - self.center_m;
        let (s, c) = self.angle_deg.to_radians().sin_cos();
        (d.x * c + d.y * s).abs() <= self.size_m.x / 2.
            && (-d.x * s + d.y * c).abs() <= self.size_m.y / 2.
    }
    pub fn corners(&self) -> [Vec2; 4] {
        let pose = Pose2::new(
            self.center_m.x,
            self.center_m.y,
            self.angle_deg.to_radians(),
        );
        [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)].map(|(x, y)| {
            pose.transform_point(Vec2::new(x * self.size_m.x / 2., y * self.size_m.y / 2.))
        })
    }
    pub fn valid(&self) -> bool {
        [
            self.center_m.x,
            self.center_m.y,
            self.size_m.x,
            self.size_m.y,
            self.angle_deg,
        ]
        .iter()
        .all(|v| v.is_finite())
            && self.size_m.x > 0.
            && self.size_m.y > 0.
    }
}
#[derive(Debug, Clone)]
pub struct SurfaceRegion {
    pub id: String,
    pub area: RectArea,
    pub material: String,
    pub mu: f64,
    pub reflectance: f64,
    pub color: [u8; 3],
    pub height_m: f64,
    pub roughness_m: f64,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarkKind {
    Paint,
    Gap,
}
#[derive(Debug, Clone)]
pub struct OpticalMark {
    pub id: String,
    pub area: RectArea,
    pub kind: MarkKind,
    pub reflectance: f64,
    pub color: [u8; 3],
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateKind {
    Start,
    Checkpoint,
    Finish,
}
#[derive(Debug, Clone)]
pub struct RaceGate {
    pub id: String,
    pub kind: GateKind,
    pub center_m: Vec2,
    pub heading_deg: f64,
    pub half_width_m: f64,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartSource {
    Project,
    Track,
}
#[derive(Debug, Clone)]
pub struct TrackEnvironment {
    pub regions: Vec<SurfaceRegion>,
    pub marks: Vec<OpticalMark>,
    pub gates: Vec<RaceGate>,
    pub outside_material: String,
    pub outside_mu: f64,
    pub outside_reflectance: f64,
    pub race_enabled: bool,
    pub laps: u32,
    pub stop_on_exit: bool,
    pub start_source: StartSource,
    pub relief_enabled: bool,
}
impl Default for TrackEnvironment {
    fn default() -> Self {
        Self {
            regions: vec![],
            marks: vec![],
            gates: vec![],
            outside_material: "outside".into(),
            outside_mu: 0.,
            outside_reflectance: 0.,
            race_enabled: false,
            laps: 1,
            stop_on_exit: false,
            start_source: StartSource::Track,
            relief_enabled: false,
        }
    }
}
pub fn effective_start_pose(track: &TrackConfig, project_pose: Pose2) -> Result<Pose2, String> {
    if track.environment.start_source == StartSource::Project || track.parametric.is_none() {
        return Ok(project_pose);
    }
    let p = crate::rtsim_track::resolve_robot_start_pose(track.parametric.as_ref().unwrap())
        .ok_or("Track start selected but no valid start marking exists; choose Project source")?;
    Ok(Pose2::new(
        p.x_mm / 1000.,
        p.y_mm / 1000.,
        p.heading_deg.to_radians(),
    ))
}
pub fn validate_definition(track: &TrackConfig) -> Result<Vec<String>, String> {
    let e = &track.environment;
    for n in [e.outside_mu, e.outside_reflectance] {
        if !n.is_finite() || n < 0. {
            return Err("invalid outside surface".into());
        }
    }
    if e.outside_material.trim().is_empty() || e.outside_reflectance > 1. || e.laps == 0 {
        return Err("invalid outside material/reflectance/laps".into());
    }
    let mut ids = std::collections::BTreeSet::new();
    for r in &e.regions {
        if !ids.insert(r.id.clone())
            || r.id.trim().is_empty()
            || !r.area.valid()
            || r.material.trim().is_empty()
            || !r.mu.is_finite()
            || r.mu < 0.
            || !r.reflectance.is_finite()
            || !(0.0..=1.).contains(&r.reflectance)
            || !r.height_m.is_finite()
            || !r.roughness_m.is_finite()
            || r.roughness_m < 0.
        {
            return Err(format!("invalid region {}", r.id));
        }
    }
    for m in &e.marks {
        if !ids.insert(m.id.clone())
            || m.id.trim().is_empty()
            || !m.area.valid()
            || !m.reflectance.is_finite()
            || !(0.0..=1.).contains(&m.reflectance)
        {
            return Err(format!("invalid optical mark {}", m.id));
        }
    }
    for g in &e.gates {
        if !ids.insert(g.id.clone())
            || g.id.trim().is_empty()
            || [g.center_m.x, g.center_m.y, g.heading_deg, g.half_width_m]
                .iter()
                .any(|v| !v.is_finite())
            || g.half_width_m <= 0.
        {
            return Err(format!("invalid race gate {}", g.id));
        }
    }
    let mut warnings = Vec::new();
    if e.relief_enabled {
        return Err("vertical relief is not supported by planar-impulse-v2; disable relief to preserve it as metadata".into());
    }
    if e.regions
        .iter()
        .any(|r| r.height_m != 0. || r.roughness_m != 0.)
    {
        warnings.push("Height/roughness stored as metadata; vertical forces are disabled.".into());
    }

    if let Some(p) = &track.parametric {
        if p.rules.source.trim().is_empty() || p.rules.edition.trim().is_empty() {
            warnings
                .push("Rule profile has no source/edition; no official compliance claim.".into());
        }
        if p.units != "mm"
            || [p.area.width_mm, p.area.height_mm, p.area.grid_mm]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.)
            || [p.origin.x_mm, p.origin.y_mm, p.origin.heading_deg]
                .iter()
                .any(|v| !v.is_finite())
        {
            return Err("invalid parametric area/origin/units".into());
        }
        let rules = crate::rtsim_track::resolve_rules(&p.rules);
        if [
            rules.max_total_length_mm,
            rules.min_arc_radius_mm,
            rules.min_distance_between_curvature_changes_mm,
            rules.intersection_angle_deg,
            rules.intersection_angle_tolerance_deg,
            rules.min_straight_around_intersection_mm,
            rules.min_straight_around_start_finish_mm,
            rules.start_goal_distance_mm,
            rules.start_goal_area_half_width_mm,
            rules.min_table_edge_clearance_mm,
            rules.max_slope_deg,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.)
        {
            return Err("invalid numeric track rule".into());
        }

        if !rules.line_width_mm.is_finite()
            || rules.line_width_mm <= 0.
            || [
                p.closure.position_tolerance_mm,
                p.closure.heading_tolerance_deg,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.)
        {
            return Err("invalid track width/closure tolerance".into());
        }
        if p.segments.is_empty() {
            return Err("track needs segments".into());
        }
        let mut segids = std::collections::BTreeSet::new();
        let mut length = 0.;
        for s in &p.segments {
            if s.id().trim().is_empty() || !segids.insert(s.id()) {
                return Err("duplicate/empty segment ID".into());
            }
            match s {
                crate::rtsim_track::TrackSegment::Straight(s) => {
                    if !s.length_mm.is_finite() || s.length_mm <= 0. {
                        return Err("invalid straight length".into());
                    }
                }
                crate::rtsim_track::TrackSegment::Arc(a) => {
                    if !a.radius_mm.is_finite()
                        || a.radius_mm <= 0.
                        || !a.sweep_deg.is_finite()
                        || a.sweep_deg == 0.
                        || a.sweep_deg.abs() > 360.
                    {
                        return Err(
                            "arc needs positive radius and nonzero sweep <= 360 degrees".into()
                        );
                    }
                }
            }
            length += s.length_mm();
        }
        if length > 5_000_000. || p.segments.len() > 100_000 {
            return Err("track exceeds bounded geometry cache budget".into());
        }
        let sf = &p.markings.start_finish;
        if [
            sf.start_s_mm,
            sf.distance_mm,
            sf.margin_mm,
            sf.robot_start.delta_x_mm,
            sf.robot_start.delta_y_mm,
            sf.robot_start.heading_deg,
        ]
        .iter()
        .any(|v| !v.is_finite())
            || sf.margin_mm < 0.
            || sf.distance_mm <= 0.
        {
            return Err("invalid start/finish numeric configuration".into());
        }
        if sf.enabled {
            let segment = p
                .segments
                .iter()
                .find(|s| {
                    s.id() == sf.segment_id
                        && matches!(s, crate::rtsim_track::TrackSegment::Straight(_))
                })
                .ok_or("invalid start/finish segment reference")?;
            if sf.start_s_mm < 0. || sf.start_s_mm + sf.distance_mm > segment.length_mm() + 1e-8 {
                return Err("start/finish must lie on the referenced straight".into());
            }
        }
        for value in [p.surface.base_reflectance, p.surface.line_reflectance] {
            if !value.is_finite() || !(0.0..=1.).contains(&value) {
                return Err("invalid reflectance".into());
            }
        }
        if !p.surface.surface_mu.is_finite() || p.surface.surface_mu < 0. {
            return Err("invalid friction".into());
        }
    } else {
        if track.centerline.len() < 2
            || track
                .centerline
                .iter()
                .any(|p| !p.x.is_finite() || !p.y.is_finite())
            || !track.line_width_m.is_finite()
            || track.line_width_m <= 0.
        {
            return Err("invalid polyline geometry".into());
        }
        if [track.base_reflectance, track.line_reflectance]
            .iter()
            .any(|v| !v.is_finite() || !(0.0..=1.).contains(v))
            || !track.surface_mu.is_finite()
            || track.surface_mu < 0.
        {
            return Err("invalid polyline surface".into());
        }
    }
    Ok(warnings)
}
/// Move an item by index without changing its stable identity/references.
pub fn move_item<T>(items: &mut Vec<T>, from: usize, to: usize) -> Result<(), String> {
    if from >= items.len() || to >= items.len() {
        return Err("invalid reorder index".into());
    }
    let value = items.remove(from);
    items.insert(to, value);
    Ok(())
}
/// Convex polygon intersection used by the canvas to restore substrate colors under gaps.
pub fn clip_polygon(
    subject: &[crate::math::Vec2],
    clip: &[crate::math::Vec2],
) -> Vec<crate::math::Vec2> {
    let cross = |a: crate::math::Vec2, b: crate::math::Vec2| a.x * b.y - a.y * b.x;
    let mut out = subject.to_vec();
    for (a, b) in clip
        .iter()
        .zip(clip.iter().cycle().skip(1))
        .take(clip.len())
    {
        let input = std::mem::take(&mut out);
        if input.is_empty() {
            break;
        }
        let mut previous = *input.last().unwrap();
        for point in input {
            let dp = cross(*b - *a, previous - *a);
            let dc = cross(*b - *a, point - *a);
            if (dp >= 0.) != (dc >= 0.) {
                let t = dp / (dp - dc);
                out.push(previous + (point - previous) * t);
            }
            if dc >= 0. {
                out.push(point);
            }
            previous = point;
        }
    }
    out
}
#[derive(Debug, Default)]
pub struct TrackEditHistory {
    undo: Vec<(TrackConfig, Pose2)>,
    redo: Vec<(TrackConfig, Pose2)>,
}
impl TrackEditHistory {
    pub fn record(&mut self, before: (TrackConfig, Pose2), cfg: &crate::config::LoadedConfig) {
        if crate::io::persistence::track_json(&before.0)
            != crate::io::persistence::track_json(&cfg.track)
            || before.1 != cfg.project.start_pose
        {
            self.undo.push(before);
            if self.undo.len() > 100 {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
    }
    pub fn undo(&mut self, cfg: &mut crate::config::LoadedConfig) -> bool {
        if let Some((track, pose)) = self.undo.pop() {
            self.redo.push((
                std::mem::replace(&mut cfg.track, track),
                std::mem::replace(&mut cfg.project.start_pose, pose),
            ));
            true
        } else {
            false
        }
    }
    pub fn redo(&mut self, cfg: &mut crate::config::LoadedConfig) -> bool {
        if let Some((track, pose)) = self.redo.pop() {
            self.undo.push((
                std::mem::replace(&mut cfg.track, track),
                std::mem::replace(&mut cfg.project.start_pose, pose),
            ));
            true
        } else {
            false
        }
    }
    pub fn clear(&mut self) {
        self.undo.clear();
        self.redo.clear();
    }
}
pub fn display_color(name: &str) -> [u8; 3] {
    match name.to_ascii_lowercase().as_str() {
        "white" => [245, 245, 245],
        "black" => [8, 8, 8],
        "red" => [220, 40, 40],
        "blue" => [40, 70, 220],
        "green" => [40, 180, 70],
        _ => {
            let hex = name.strip_prefix('#').unwrap_or("");
            if hex.len() == 6 {
                if let Ok(n) = u32::from_str_radix(hex, 16) {
                    return [(n >> 16) as u8, (n >> 8) as u8, n as u8];
                }
            }
            [128, 128, 128]
        }
    }
}
