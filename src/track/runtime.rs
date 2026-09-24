use super::spatial::{Bounds, SpatialIndex};
use super::{definition::*, TrackModel};
use crate::{
    config::{RobotConfig, TrackConfig},
    math::{distance_point_segment, Pose2, Vec2},
    rtsim_track::*,
};
use std::sync::Arc;
#[derive(Debug, Clone)]
enum Primitive {
    Line(Vec2, Vec2),
    Arc {
        center: Vec2,
        radius: f64,
        start: f64,
        sweep: f64,
        first: Vec2,
        last: Vec2,
    },
}
impl Primitive {
    fn distance(&self, p: Vec2) -> f64 {
        match *self {
            Self::Line(a, b) => distance_point_segment(p, a, b),
            Self::Arc {
                center,
                radius,
                start,
                sweep,
                first,
                last,
            } => {
                let delta = p - center;
                let angle = delta.y.atan2(delta.x);
                let progress = ((angle - start) * sweep.signum()).rem_euclid(std::f64::consts::TAU);
                if progress <= sweep.abs() + 1e-12 {
                    (delta.norm() - radius).abs()
                } else {
                    (p - first).norm().min((p - last).norm())
                }
            }
        }
    }
}
#[derive(Debug)]
struct RuntimeData {
    line_index: SpatialIndex,
    region_index: SpatialIndex,
    mark_index: SpatialIndex,
    geometry: Option<TrackGeometry>,
    definition: TrackConfig,
    primitives: Vec<Primitive>,
    polyline: Vec<Vec2>,
    marks: Vec<OpticalMark>,
    gates: Vec<RaceGate>,
    bounds: Option<Vec2>,
    width: f64,
    base: f64,
    line: f64,
    mu: f64,
}
/// Immutable, cheap-to-clone snapshot. No geometry construction in sensor/contact queries.
#[derive(Debug, Clone)]
pub struct TrackRuntime {
    data: Arc<RuntimeData>,
}
pub type VectorTrack = TrackRuntime;
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSample {
    pub material: String,
    pub mu: f64,
    pub substrate_reflectance: f64,
    pub height_m: f64,
    pub roughness_m: f64,
    pub normal: [f64; 3],
    pub inside: bool,
}
impl TrackRuntime {
    pub fn try_new(cfg: TrackConfig) -> Result<Self, String> {
        validate_definition(&cfg)?;
        let mut primitives = Vec::new();
        let mut marks = Vec::new();
        let mut gates = cfg.environment.gates.clone();
        let mut polyline = cfg.centerline.clone();
        let mut width = cfg.line_width_m;
        let mut base = cfg.base_reflectance;
        let mut line = cfg.line_reflectance;
        let mut mu = cfg.surface_mu;
        let mut bounds = None;
        let mut geometry = None;
        if let Some(p) = &cfg.parametric {
            let geo = build_geometry(p);
            polyline = geo.centerline_m.clone();
            width = resolve_rules(&p.rules).line_width_mm / 1000.;
            base = p.surface.base_reflectance;
            line = p.surface.line_reflectance;
            mu = p.surface.surface_mu;
            bounds = Some(Vec2::new(p.area.width_mm / 1000., p.area.height_mm / 1000.));
            for (segment, pose) in p.segments.iter().zip(&geo.segment_poses) {
                let first = Vec2::new(pose.start.x_mm / 1000., pose.start.y_mm / 1000.);
                let last = Vec2::new(pose.end.x_mm / 1000., pose.end.y_mm / 1000.);
                match segment {
                    TrackSegment::Straight(_) => primitives.push(Primitive::Line(first, last)),
                    TrackSegment::Arc(a) => {
                        let h = pose.start.heading_deg.to_radians();
                        let radius = a.radius_mm / 1000.;
                        let center =
                            first + Vec2::new(-h.sin(), h.cos()) * (a.sweep_deg.signum() * radius);
                        let d = first - center;
                        primitives.push(Primitive::Arc {
                            center,
                            radius,
                            start: d.y.atan2(d.x),
                            sweep: a.sweep_deg.to_radians(),
                            first,
                            last,
                        });
                    }
                }
            }
            if let Some(sf) = resolve_start_finish_markers(p) {
                for (id, pose, kind) in [
                    ("auto:start", sf.start_pose, GateKind::Start),
                    ("auto:finish", sf.finish_pose, GateKind::Finish),
                ] {
                    marks.push(side_mark(
                        id,
                        pose,
                        -1.,
                        width,
                        line,
                        display_color(&p.surface.line_color),
                    ));
                    if cfg.environment.gates.is_empty() {
                        gates.push(RaceGate {
                            id: id.into(),
                            kind,
                            center_m: Vec2::new(pose.x_mm / 1000., pose.y_mm / 1000.),
                            heading_deg: sf.travel_heading_deg + 180.,
                            half_width_m: resolve_rules(&p.rules).start_goal_area_half_width_mm
                                / 1000.,
                        });
                    }
                }
            }
            geometry = Some(geo.clone());
            if p.markings.corner_markers.auto_generate {
                for seg in &geo.segment_poses {
                    if seg.kind == "arc" {
                        for (end, mut pose) in [("start", seg.start), ("end", seg.end)] {
                            if !p.markings.start_finish.exit_direction.is_increasing_s() {
                                pose.heading_deg += 180.;
                            }
                            marks.push(side_mark(
                                &format!("auto:{}:{end}", seg.id),
                                pose,
                                1.,
                                width,
                                line,
                                display_color(&p.surface.line_color),
                            ));
                        }
                    }
                }
            }
        } else {
            for pair in cfg.centerline.windows(2) {
                primitives.push(Primitive::Line(pair[0], pair[1]));
            }
        }
        marks.extend(cfg.environment.marks.clone());
        if cfg.environment.race_enabled {
            if gates.iter().filter(|g| g.kind == GateKind::Start).count() != 1
                || gates.iter().filter(|g| g.kind == GateKind::Finish).count() != 1
            {
                return Err("race requires exactly one start and finish gate".into());
            }
        }
        Ok(Self {
            data: Arc::new(RuntimeData {
                line_index: SpatialIndex::new(
                    primitives
                        .iter()
                        .map(|p| match p {
                            Primitive::Line(a, b) => Bounds::points(&[*a, *b]),
                            Primitive::Arc { center, radius, .. } => Bounds::points(&[
                                *center - Vec2::new(*radius, *radius),
                                *center + Vec2::new(*radius, *radius),
                            ]),
                        })
                        .collect(),
                ),
                region_index: SpatialIndex::new(
                    cfg.environment
                        .regions
                        .iter()
                        .map(|r| Bounds::points(&r.area.corners()))
                        .collect(),
                ),
                mark_index: SpatialIndex::new(
                    marks
                        .iter()
                        .map(|m| Bounds::points(&m.area.corners()))
                        .collect(),
                ),
                geometry,
                definition: cfg,
                primitives,
                polyline,
                marks,
                gates,
                bounds,
                width,
                base,
                line,
                mu,
            }),
        })
    }
    pub fn new(cfg: TrackConfig) -> Self {
        Self::try_new(cfg).expect("valid track configuration")
    }
    pub fn shares_geometry_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }
    pub fn geometry(&self) -> Option<&TrackGeometry> {
        self.data.geometry.as_ref()
    }
    pub fn cached_polyline(&self) -> &[Vec2] {
        &self.data.polyline
    }
    pub fn marks(&self) -> &[OpticalMark] {
        &self.data.marks
    }
    pub fn gates(&self) -> &[RaceGate] {
        &self.data.gates
    }
    pub fn definition(&self) -> &TrackConfig {
        &self.data.definition
    }
    pub fn inside(&self, p: Vec2) -> bool {
        self.data
            .bounds
            .is_none_or(|b| p.x >= 0. && p.y >= 0. && p.x <= b.x && p.y <= b.y)
    }
    pub fn surface_at(&self, p: Vec2) -> SurfaceSample {
        let e = &self.data.definition.environment;
        if !self.inside(p) {
            return SurfaceSample {
                material: e.outside_material.clone(),
                mu: e.outside_mu,
                substrate_reflectance: e.outside_reflectance,
                height_m: 0.,
                roughness_m: 0.,
                normal: [0., 0., 1.],
                inside: false,
            };
        }
        if let Some(r) = self.region_at(p) {
            return SurfaceSample {
                material: r.material.clone(),
                mu: r.mu,
                substrate_reflectance: r.reflectance,
                height_m: r.height_m,
                roughness_m: r.roughness_m,
                normal: [0., 0., 1.],
                inside: true,
            };
        }
        SurfaceSample {
            material: "base".into(),
            mu: self.data.mu,
            substrate_reflectance: self.data.base,
            height_m: 0.,
            roughness_m: 0.,
            normal: [0., 0., 1.],
            inside: true,
        }
    }
    pub fn robot_inside(&self, robot: &RobotConfig, pose: Pose2) -> bool {
        [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
            .into_iter()
            .all(|(x, y)| {
                self.inside(pose.transform_point(Vec2::new(
                    x * robot.chassis.length_m / 2.,
                    y * robot.chassis.width_m / 2.,
                )))
            })
            && robot.assembly.as_ref().is_none_or(|a| {
                a.wheels.iter().all(|w| {
                    crate::models::robot::wheel_corners(w, pose)
                        .iter()
                        .all(|p| self.inside(*p))
                })
            })
    }
    pub fn robot_over_line(&self, robot: &RobotConfig, pose: Pose2) -> bool {
        robot.line_validity_areas.iter().any(|a| {
            self.data
                .polyline
                .windows(2)
                .any(|w| a.overlaps_line_segment(pose, w[0], w[1], self.data.width))
        })
    }
    fn region_at(&self, p: Vec2) -> Option<&SurfaceRegion> {
        self.data
            .region_index
            .last(p, |i| {
                self.data.definition.environment.regions[i].area.contains(p)
            })
            .map(|i| &self.data.definition.environment.regions[i])
    }
    pub fn region_color_at(&self, p: Vec2) -> [u8; 3] {
        self.region_at(p).map(|r| r.color).unwrap_or([8, 8, 8])
    }
}
impl TrackModel for TrackRuntime {
    fn reflectance_at(&self, p: Vec2) -> f64 {
        if !self.inside(p) {
            return self.data.definition.environment.outside_reflectance;
        }
        let substrate = self
            .region_at(p)
            .map(|r| r.reflectance)
            .unwrap_or(self.data.base);
        if let Some(i) = self
            .data
            .mark_index
            .last(p, |i| self.data.marks[i].area.contains(p))
        {
            let m = &self.data.marks[i];
            return if m.kind == MarkKind::Gap {
                substrate
            } else {
                m.reflectance
            };
        }
        if self.distance_to_line_m(p) <= self.data.width / 2. {
            self.data.line
        } else {
            substrate
        }
    }
    fn surface_mu_at(&self, p: Vec2) -> f64 {
        if !self.inside(p) {
            self.data.definition.environment.outside_mu
        } else {
            self.region_at(p).map(|r| r.mu).unwrap_or(self.data.mu)
        }
    }
    fn distance_to_line_m(&self, p: Vec2) -> f64 {
        self.data
            .line_index
            .nearest(p, |i| self.data.primitives[i].distance(p))
    }
    fn base_reflectance(&self) -> f64 {
        self.data.base
    }
    fn line_reflectance(&self) -> f64 {
        self.data.line
    }
}
fn side_mark(
    id: &str,
    p: TrackPose,
    side: f64,
    width: f64,
    reflectance: f64,
    color: [u8; 3],
) -> OpticalMark {
    let h = p.heading_deg.to_radians();
    OpticalMark {
        id: id.into(),
        area: RectArea {
            center_m: Vec2::new(p.x_mm / 1000., p.y_mm / 1000.)
                + Vec2::new(-h.sin(), h.cos()) * (side * (width / 2. + 0.04)),
            size_m: Vec2::new(0.01, 0.04),
            angle_deg: p.heading_deg,
        },
        kind: MarkKind::Paint,
        reflectance,
        color,
    }
}

#[derive(Debug, Clone, Default)]
pub struct TrackRuntimeCache {
    key: String,
    runtime: Option<TrackRuntime>,
    builds: u64,
}
impl TrackRuntimeCache {
    pub fn get(&mut self, cfg: &TrackConfig) -> Result<TrackRuntime, String> {
        let key = crate::io::persistence::track_json(cfg);
        if self.runtime.is_none() || key != self.key {
            let runtime = TrackRuntime::try_new(cfg.clone())?;
            self.runtime = Some(runtime);
            self.key = key;
            self.builds += 1;
        }
        Ok(self.runtime.as_ref().unwrap().clone())
    }
    pub fn build_count(&self) -> u64 {
        self.builds
    }
}
