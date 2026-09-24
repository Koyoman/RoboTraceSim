//! Assembly geometry and mass accounting, independent of the graphical editor.
use crate::{
    config::*,
    json::JsonValue,
    math::{Pose2, Vec2},
};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WheelKind {
    Driven,
    Passive,
    Caster,
}
#[derive(Debug, Clone)]
pub struct WheelInstance {
    pub id: String,
    /// Contact projection in the body frame; z is surface height (planar runtime: zero).
    pub position_m: Vec2,
    pub height_m: f64,
    pub angle_deg: f64,
    pub radius_m: f64,
    pub width_m: f64,
    pub inertia_kg_m2: f64,
    pub material: String,
    pub tire: TireConfig,
    pub motor: Option<String>,
    pub kind: WheelKind,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MassMode {
    Measured,
    Components,
}
#[derive(Debug, Clone)]
pub struct MassElement {
    pub id: String,
    /// A known component, or None for a separate structural part/ballast.
    pub component: Option<String>,
    pub mass_kg: f64,
    pub position_m: Vec2,
    pub height_m: f64,
    /// Intrinsic yaw inertia about this element's own center of mass.
    pub inertia_kg_m2: f64,
}
#[derive(Debug, Clone)]
pub struct RobotAssembly {
    pub wheels: Vec<WheelInstance>,
    pub mass_mode: MassMode,
    pub measured_com_height_m: f64,
    pub masses: Vec<MassElement>,
}
#[derive(Debug, Clone, Copy)]
pub struct MassProperties {
    pub mass_kg: f64,
    pub center_m: Vec2,
    pub height_m: f64,
    pub inertia_kg_m2: f64,
}

pub fn sensor_world_position(sensor: &RobotSensorInstance, pose: Pose2) -> Vec2 {
    pose.transform_point(sensor.position_m)
}
pub fn wheel_world_position(wheel: &WheelInstance, pose: Pose2) -> Vec2 {
    pose.transform_point(wheel.position_m)
}
pub fn wheel_corners(w: &WheelInstance, pose: Pose2) -> [Vec2; 4] {
    let frame = Pose2::new(w.position_m.x, w.position_m.y, w.angle_deg.to_radians());
    [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)].map(|(x, y)| {
        pose.transform_point(frame.transform_point(Vec2::new(x * w.radius_m, y * w.width_m / 2.)))
    })
}
impl RobotAssembly {
    pub fn from_legacy(robot: &RobotConfig) -> Self {
        let d = robot.drivetrain;
        let mut wheels = Vec::new();
        for (side, y) in [
            ("left", d.track_width_m / 2.),
            ("right", -d.track_width_m / 2.),
        ] {
            for (end, x) in [("front", d.wheelbase_m / 2.), ("rear", -d.wheelbase_m / 2.)] {
                wheels.push(WheelInstance {
                    id: format!("wheel:{side}:{end}"),
                    position_m: Vec2::new(x, y),
                    height_m: 0.,
                    angle_deg: 0.,
                    radius_m: d.wheel_radius_m,
                    width_m: d.wheel_width_m,
                    // The existing solver stores one equivalent inertia per side.
                    inertia_kg_m2: d.wheel_inertia_kg_m2 / 2.,
                    material: "default".into(),
                    tire: robot.tire.clone(),
                    motor: Some(format!("motor:{side}")),
                    kind: WheelKind::Driven,
                });
            }
        }
        let mut masses = vec![MassElement {
            id: "mass:chassis".into(),
            component: Some("chassis".into()),
            mass_kg: robot.chassis.mass_kg,
            position_m: robot.chassis.center_of_mass_m,
            height_m: 0.,
            inertia_kg_m2: robot.chassis.inertia_kg_m2,
        }];
        for id in ["battery", "motor:left", "motor:right"]
            .into_iter()
            .chain(robot.normal_force.fans.iter().map(|f| f.id.as_str()))
        {
            masses.push(MassElement {
                id: format!("mass:{id}"),
                component: Some(id.into()),
                mass_kg: 0.,
                position_m: Vec2::default(),
                height_m: 0.,
                inertia_kg_m2: 0.,
            });
        }
        Self {
            wheels,
            mass_mode: MassMode::Measured,
            measured_com_height_m: 0.,
            masses,
        }
    }
    pub fn effective(robot: &RobotConfig) -> Self {
        robot
            .assembly
            .clone()
            .unwrap_or_else(|| Self::from_legacy(robot))
    }
    pub fn element_position(&self, robot: &RobotConfig, e: &MassElement) -> (Vec2, f64) {
        if let Some(id) = &e.component {
            if let Some(w) = self.wheels.iter().find(|w| &w.id == id) {
                return (w.position_m, w.height_m + w.radius_m);
            }
            if let Some(s) = robot.sensors.iter().find(|s| &s.id == id) {
                return (s.position_m, s.height_m);
            }
            if let Some(f) = robot.normal_force.fans.iter().find(|f| &f.id == id) {
                return (f.position_m, e.height_m);
            }
        }
        (e.position_m, e.height_m)
    }
    pub fn mass_properties(&self, robot: &RobotConfig) -> Result<MassProperties, String> {
        if self.mass_mode == MassMode::Measured {
            return Ok(MassProperties {
                mass_kg: robot.chassis.mass_kg,
                center_m: robot.chassis.center_of_mass_m,
                height_m: self.measured_com_height_m,
                inertia_kg_m2: robot.chassis.inertia_kg_m2,
            });
        }
        let mass: f64 = self.masses.iter().map(|e| e.mass_kg).sum();
        if !mass.is_finite() || mass <= 0. {
            return Err("assembly: component mass sum must be positive".into());
        }
        let mut center = Vec2::default();
        let mut height = 0.;
        for e in &self.masses {
            let (p, z) = self.element_position(robot, e);
            center = center + p * (e.mass_kg / mass);
            height += z * e.mass_kg / mass;
        }
        let inertia = self
            .masses
            .iter()
            .map(|e| {
                let (p, _) = self.element_position(robot, e);
                e.inertia_kg_m2 + e.mass_kg * (p - center).norm2()
            })
            .sum();
        if !f64::is_finite(inertia) || inertia <= 0. {
            return Err("assembly: yaw inertia must be positive".into());
        }
        Ok(MassProperties {
            mass_kg: mass,
            center_m: center,
            height_m: height,
            inertia_kg_m2: inertia,
        })
    }
    pub fn validate(&self, robot: &RobotConfig) -> Result<Vec<String>, String> {
        let mut ids: BTreeSet<String> = [
            "chassis",
            "battery",
            "motor:left",
            "motor:right",
            "driver",
            "controller",
            "encoder",
            "gyro",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect();
        for id in robot
            .sensors
            .iter()
            .map(|s| &s.id)
            .chain(robot.normal_force.fans.iter().map(|f| &f.id))
            .chain(self.wheels.iter().map(|w| &w.id))
        {
            if id.trim().is_empty() || !ids.insert(id.clone()) {
                return Err(format!("assembly: duplicate/empty component ID {id}"));
            }
        }
        if self.wheels.is_empty() {
            return Err("assembly: at least one support is required".into());
        }
        for w in &self.wheels {
            if [w.radius_m, w.width_m, w.inertia_kg_m2]
                .iter()
                .any(|x| !x.is_finite() || *x <= 0.)
                || [w.position_m.x, w.position_m.y, w.height_m, w.angle_deg]
                    .iter()
                    .any(|x| !x.is_finite())
                || w.height_m < 0.
            {
                return Err(format!("{}: invalid geometry/inertia", w.id));
            }
            if w.material.trim().is_empty() {
                return Err(format!("{}: material name required", w.id));
            }
            if !["SlipRatioWheel", "CoulombFrictionWheel"].contains(&w.tire.model.as_str()) {
                return Err(format!("{}: unknown tire model", w.id));
            }
            if [
                w.tire.mu_longitudinal,
                w.tire.mu_lateral,
                w.tire.rolling_resistance,
                w.tire.slip_velocity_epsilon_m_s,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.)
            {
                return Err(format!("{}: invalid tire", w.id));
            }
            match (&w.motor, w.kind) {
                (Some(m), WheelKind::Driven) if m == "motor:left" || m == "motor:right" => {}
                (None, WheelKind::Passive | WheelKind::Caster) => {}
                _ => return Err(format!("{}: invalid motor association", w.id)),
            }
        }
        let mut mass_ids = BTreeSet::new();
        let mut refs = BTreeSet::new();
        for e in &self.masses {
            if e.id.trim().is_empty() || !mass_ids.insert(&e.id) {
                return Err("assembly: duplicate mass ID".into());
            }
            if [e.mass_kg, e.inertia_kg_m2, e.height_m]
                .iter()
                .any(|v| !v.is_finite() || *v < 0.)
                || !e.position_m.x.is_finite()
                || !e.position_m.y.is_finite()
            {
                return Err(format!("{}: invalid mass properties", e.id));
            }
            if let Some(id) = &e.component {
                if !ids.contains(id) || !refs.insert(id) {
                    return Err(format!(
                        "{}: absent or multiply counted component {id}",
                        e.id
                    ));
                }
            }
        }
        if !self.measured_com_height_m.is_finite() || self.measured_com_height_m < 0. {
            return Err("assembly: invalid COM height".into());
        }
        for s in &robot.sensors {
            if !s.height_m.is_finite() || s.height_m < 0. {
                return Err(format!("{}: invalid sensor height", s.id));
            }
        }
        let m = self.mass_properties(robot)?;
        let mut warnings = vec![if robot.physics.is_some() {
            "Per-wheel planar solver: COM height enters quasi-static load transfer; no pitch/roll or optical height dynamics. Shared motors use equal torque split and mean shaft speed.".into()
        } else {
            "Planar reference solver: heights are metadata; no pitch/roll or optical height effect."
                .into()
        }];
        let hull = self.support_polygon();
        if hull.len() < 3 {
            return Err("assembly: degenerate support polygon".into());
        }
        if hull
            .iter()
            .zip(hull.iter().cycle().skip(1))
            .take(hull.len())
            .any(|(a, b)| cross(*b - *a, m.center_m - *a) < -1e-12)
        {
            warnings.push(
                "COM projection is outside support polygon; tipping requires a vertical solver."
                    .into(),
            );
        }
        if robot.physics.is_none() {
            if let Err(e) = self.solver_geometry() {
                warnings.push(e);
            }
        }
        Ok(warnings)
    }
    /// Adapter to the current symmetric four-wheel planar solver. Never averages incompatible geometry.
    pub fn solver_geometry(&self) -> Result<DrivetrainConfig, String> {
        let fail = || {
            "assembly incompatible with planar-impulse-v2: requires four aligned driven wheels in a centered rectangle, equal radii/width/tire and equal inertia sums per side; passive/caster and independent contacts need stage 6".to_string()
        };
        if self.wheels.len() != 4 {
            return Err(fail());
        }
        let a = &self.wheels[0];
        let x = a.position_m.x.abs();
        let y = a.position_m.y.abs();
        let close = |a: f64, b: f64| (a - b).abs() < 1e-10;
        let mut corners = BTreeSet::new();
        let mut il = 0.;
        let mut ir = 0.;
        for w in &self.wheels {
            if w.kind != WheelKind::Driven
                || !close(w.angle_deg.rem_euclid(360.), 0.)
                || !close(w.height_m, 0.)
                || !close(w.radius_m, a.radius_m)
                || !close(w.width_m, a.width_m)
                || !close(w.position_m.x.abs(), x)
                || !close(w.position_m.y.abs(), y)
                || w.material != a.material
                || !same_tire(&w.tire, &a.tire)
            {
                return Err(fail());
            }
            let left = w.position_m.y > 0.;
            if w.motor.as_deref() != Some(if left { "motor:left" } else { "motor:right" }) {
                return Err(fail());
            }
            corners.insert((w.position_m.x > 0., left));
            if left {
                il += w.inertia_kg_m2;
            } else {
                ir += w.inertia_kg_m2;
            }
        }
        if x <= 0. || y <= 0. || corners.len() != 4 || !close(il, ir) {
            return Err(fail());
        }
        Ok(DrivetrainConfig {
            wheel_radius_m: a.radius_m,
            wheel_width_m: a.width_m,
            track_width_m: 2. * y,
            wheelbase_m: 2. * x,
            wheel_inertia_kg_m2: il,
        })
    }
    pub fn support_polygon(&self) -> Vec<Vec2> {
        let mut pts: Vec<_> = self.wheels.iter().map(|w| w.position_m).collect();
        pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
        pts.dedup();
        if pts.len() < 3 {
            return pts;
        }
        // Standard monotone chains, kept separate to preserve both halves.
        let chain = |points: Vec<Vec2>| {
            let mut h: Vec<Vec2> = Vec::new();
            for p in points {
                while h.len() >= 2
                    && cross(h[h.len() - 1] - h[h.len() - 2], p - h[h.len() - 1]) <= 0.
                {
                    h.pop();
                }
                h.push(p);
            }
            h
        };
        let mut low = chain(pts.clone());
        let mut high = chain(pts.into_iter().rev().collect());
        low.pop();
        high.pop();
        low.extend(high);
        low
    }
}
fn cross(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}
fn same_tire(a: &TireConfig, b: &TireConfig) -> bool {
    a.model == b.model
        && a.mu_longitudinal == b.mu_longitudinal
        && a.mu_lateral == b.mu_lateral
        && a.rolling_resistance == b.rolling_resistance
        && a.slip_velocity_epsilon_m_s == b.slip_velocity_epsilon_m_s
}
pub fn resolve_robot(robot: &mut RobotConfig) -> Result<Vec<String>, String> {
    let a = RobotAssembly::effective(robot);
    let warnings = a.validate(robot)?;
    let d = if let Some(f) = &robot.physics {
        f.validate()?;
        if !(3..=8).contains(&a.wheels.len()) || a.wheels.iter().any(|w| w.height_m != 0.) {
            return Err("per-wheel planar solver requires 3..8 supports at zero height".into());
        }
        let mut d = robot.drivetrain;
        d.wheelbase_m = 2.
            * a.wheels
                .iter()
                .map(|w| w.position_m.x.abs())
                .fold(0., f64::max);
        d.track_width_m = 2.
            * a.wheels
                .iter()
                .map(|w| w.position_m.y.abs())
                .fold(0., f64::max);
        if f.contact == "ideal" {
            for id in ["motor:left", "motor:right"] {
                let group: Vec<_> = a
                    .wheels
                    .iter()
                    .filter(|w| w.motor.as_deref() == Some(id))
                    .collect();
                let first = group.first().ok_or("ideal drive requires both motors")?;
                if group.iter().any(|w| {
                    w.angle_deg != 0.
                        || (w.position_m.y - first.position_m.y).abs() > 1e-10
                        || (w.radius_m - first.radius_m).abs() > 1e-10
                }) {
                    return Err("ideal drive requires parallel wheels with one lateral offset/radius per motor".into());
                }
            }
            let yl = a
                .wheels
                .iter()
                .find(|w| w.motor.as_deref() == Some("motor:left"))
                .unwrap()
                .position_m
                .y;
            let yr = a
                .wheels
                .iter()
                .find(|w| w.motor.as_deref() == Some("motor:right"))
                .unwrap()
                .position_m
                .y;
            if (yl - yr).abs() < 1e-6 {
                return Err("ideal axle has zero track width".into());
            }
        }
        d
    } else {
        a.solver_geometry()?
    };
    if let Some(p) = &robot.powertrain {
        p.validate()?;
        if robot.physics.as_ref().is_none_or(|f| f.contact == "ideal") {
            return Err("electrical powertrain requires per-wheel dynamic physics".into());
        }
        for id in ["motor:left", "motor:right"] {
            if a.wheels
                .iter()
                .filter(|w| w.motor.as_deref() == Some(id))
                .count()
                != 1
            {
                return Err("electrical rotor/transmission currently requires exactly one driven wheel per motor; use passive/caster supports".into());
            }
        }
    }
    let m = a.mass_properties(robot)?;
    if m.center_m.x.abs() > d.wheelbase_m / 2. || m.center_m.y.abs() > d.track_width_m / 2. {
        return Err("assembly: COM outside support rectangle; tipping is not implemented".into());
    }
    robot.drivetrain = d;
    if robot.physics.is_none() {
        robot.tire = a.wheels[0].tire.clone();
    }
    robot.chassis.mass_kg = m.mass_kg;
    robot.chassis.center_of_mass_m = m.center_m;
    robot.chassis.inertia_kg_m2 = m.inertia_kg_m2;
    robot.assembly = Some(a);
    Ok(warnings)
}

// History stores whole definitions, so assets and IDs survive undo/redo together.
#[derive(Debug, Default)]
pub struct RobotHistory {
    undo: Vec<RobotConfig>,
    redo: Vec<RobotConfig>,
}
impl RobotHistory {
    pub fn record(&mut self, before: RobotConfig, after: &RobotConfig) {
        if crate::io::persistence::robot_json(&before) != crate::io::persistence::robot_json(after)
        {
            self.undo.push(before);
            if self.undo.len() > 100 {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
    }
    pub fn undo(&mut self, robot: &mut RobotConfig) -> bool {
        if let Some(old) = self.undo.pop() {
            self.redo.push(std::mem::replace(robot, old));
            true
        } else {
            false
        }
    }
    pub fn redo(&mut self, robot: &mut RobotConfig) -> bool {
        if let Some(next) = self.redo.pop() {
            self.undo.push(std::mem::replace(robot, next));
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

fn num(v: &JsonValue, k: &str) -> Result<f64, String> {
    v.get(k)
        .and_then(JsonValue::as_f64)
        .filter(|v| v.is_finite())
        .ok_or_else(|| format!("assembly.{k}: finite number required"))
}
fn string(v: &JsonValue, k: &str) -> Result<String, String> {
    v.get(k)
        .and_then(JsonValue::as_str)
        .map(str::to_owned)
        .ok_or_else(|| format!("assembly.{k}: string required"))
}
fn reference(v: &JsonValue, k: &str) -> Result<Option<String>, String> {
    match v.get(k) {
        Some(JsonValue::Null) => Ok(None),
        Some(JsonValue::String(s)) => Ok(Some(s.clone())),
        _ => Err(format!("assembly.{k}: string or null required")),
    }
}
fn point(v: &JsonValue) -> Result<Vec2, String> {
    let p = v
        .get("position_mm")
        .and_then(JsonValue::as_array)
        .ok_or("assembly.position_mm: pair required")?;
    if p.len() != 2 {
        return Err("assembly.position_mm: pair required".into());
    }
    Ok(Vec2::new(
        p[0].as_f64().ok_or("invalid x")? / 1000.,
        p[1].as_f64().ok_or("invalid y")? / 1000.,
    ))
}
impl RobotAssembly {
    pub fn from_json(v: &JsonValue) -> Result<Self, String> {
        let mass_mode = match string(v, "mass_mode")?.as_str() {
            "measured" => MassMode::Measured,
            "components" => MassMode::Components,
            _ => return Err("unknown mass_mode".into()),
        };
        let wheels = v
            .get("wheels")
            .and_then(JsonValue::as_array)
            .ok_or("assembly.wheels required")?
            .iter()
            .map(|w| {
                let t = w.get("tire").ok_or("wheel.tire required")?;
                Ok(WheelInstance {
                    id: string(w, "id")?,
                    position_m: point(w)?,
                    height_m: num(w, "height_mm")? / 1000.,
                    angle_deg: num(w, "angle_deg")?,
                    radius_m: num(w, "radius_mm")? / 1000.,
                    width_m: num(w, "width_mm")? / 1000.,
                    inertia_kg_m2: num(w, "inertia_kg_m2")?,
                    material: string(w, "material")?,
                    motor: reference(w, "motor")?,
                    kind: match string(w, "kind")?.as_str() {
                        "driven" => WheelKind::Driven,
                        "passive" => WheelKind::Passive,
                        "caster" => WheelKind::Caster,
                        _ => return Err("unknown wheel kind".into()),
                    },
                    tire: TireConfig {
                        model: string(t, "model")?,
                        mu_longitudinal: num(t, "mu_longitudinal")?,
                        mu_lateral: num(t, "mu_lateral")?,
                        rolling_resistance: num(t, "rolling_resistance")?,
                        slip_velocity_epsilon_m_s: num(t, "slip_velocity_epsilon_m_s")?,
                    },
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let masses = v
            .get("masses")
            .and_then(JsonValue::as_array)
            .ok_or("assembly.masses required")?
            .iter()
            .map(|m| {
                Ok(MassElement {
                    id: string(m, "id")?,
                    component: reference(m, "component")?,
                    mass_kg: num(m, "mass_g")? / 1000.,
                    position_m: point(m)?,
                    height_m: num(m, "height_mm")? / 1000.,
                    inertia_kg_m2: num(m, "inertia_kg_m2")?,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        Ok(Self {
            wheels,
            masses,
            mass_mode,
            measured_com_height_m: num(v, "measured_com_height_mm")? / 1000.,
        })
    }
    pub fn to_json(&self) -> String {
        use crate::io::persistence::escape_json as esc;
        let reference = |v: &Option<String>| {
            v.as_ref()
                .map(|v| format!("\"{}\"", esc(v)))
                .unwrap_or("null".into())
        };
        let wheels=self.wheels.iter().map(|w|format!(r#"{{"id":"{}","position_mm":[{},{}],"height_mm":{},"angle_deg":{},"radius_mm":{},"width_mm":{},"inertia_kg_m2":{},"material":"{}","motor":{},"kind":"{}","tire":{{"model":"{}","mu_longitudinal":{},"mu_lateral":{},"rolling_resistance":{},"slip_velocity_epsilon_m_s":{}}}}}"#,esc(&w.id),w.position_m.x*1000.,w.position_m.y*1000.,w.height_m*1000.,w.angle_deg,w.radius_m*1000.,w.width_m*1000.,w.inertia_kg_m2,esc(&w.material),reference(&w.motor),match w.kind{WheelKind::Driven=>"driven",WheelKind::Passive=>"passive",WheelKind::Caster=>"caster"},esc(&w.tire.model),w.tire.mu_longitudinal,w.tire.mu_lateral,w.tire.rolling_resistance,w.tire.slip_velocity_epsilon_m_s)).collect::<Vec<_>>().join(",");
        let masses=self.masses.iter().map(|m|format!(r#"{{"id":"{}","component":{},"mass_g":{},"position_mm":[{},{}],"height_mm":{},"inertia_kg_m2":{}}}"#,esc(&m.id),reference(&m.component),m.mass_kg*1000.,m.position_m.x*1000.,m.position_m.y*1000.,m.height_m*1000.,m.inertia_kg_m2)).collect::<Vec<_>>().join(",");
        format!(
            r#"{{"mass_mode":"{}","measured_com_height_mm":{},"wheels":[{}],"masses":[{}]}}"#,
            if self.mass_mode == MassMode::Measured {
                "measured"
            } else {
                "components"
            },
            self.measured_com_height_m * 1000.,
            wheels,
            masses
        )
    }
}

pub fn component_ids(robot: &RobotConfig) -> Vec<String> {
    RobotAssembly::effective(robot)
        .wheels
        .iter()
        .map(|w| w.id.clone())
        .chain(robot.sensors.iter().map(|s| s.id.clone()))
        .chain(robot.normal_force.fans.iter().map(|f| f.id.clone()))
        .collect()
}
pub fn component_pose(robot: &RobotConfig, id: &str) -> Option<Pose2> {
    if let Some(w) = RobotAssembly::effective(robot)
        .wheels
        .iter()
        .find(|w| w.id == id)
    {
        return Some(Pose2::new(
            w.position_m.x,
            w.position_m.y,
            w.angle_deg.to_radians(),
        ));
    }
    if let Some(s) = robot.sensors.iter().find(|s| s.id == id) {
        return Some(Pose2::new(
            s.position_m.x,
            s.position_m.y,
            s.angle_deg.to_radians(),
        ));
    }
    robot
        .normal_force
        .fans
        .iter()
        .find(|f| f.id == id)
        .map(|f| Pose2::new(f.position_m.x, f.position_m.y, 0.))
}
pub fn set_component_pose(robot: &mut RobotConfig, id: &str, pose: Pose2) -> Result<(), String> {
    if !pose.x.is_finite() || !pose.y.is_finite() || !pose.yaw.is_finite() {
        return Err("nonfinite component pose".into());
    }
    if robot.assembly.is_none() {
        robot.assembly = Some(RobotAssembly::from_legacy(robot));
    }
    if let Some(w) = robot
        .assembly
        .as_mut()
        .unwrap()
        .wheels
        .iter_mut()
        .find(|w| w.id == id)
    {
        w.position_m = Vec2::new(pose.x, pose.y);
        w.angle_deg = pose.yaw.to_degrees();
        return Ok(());
    }
    if let Some(s) = robot.sensors.iter_mut().find(|s| s.id == id) {
        s.position_m = Vec2::new(pose.x, pose.y);
        s.angle_deg = pose.yaw.to_degrees();
        return Ok(());
    }
    if let Some(f) = robot.normal_force.fans.iter_mut().find(|f| f.id == id) {
        f.position_m = Vec2::new(pose.x, pose.y);
        return Ok(());
    }
    Err(format!("unknown component {id}"))
}
pub fn duplicate_component(robot: &mut RobotConfig, id: &str) -> Result<String, String> {
    let new = crate::io::assets::new_instance_id();
    if robot.assembly.is_none() {
        robot.assembly = Some(RobotAssembly::from_legacy(robot));
    }
    let a = robot.assembly.as_mut().unwrap();
    if let Some(w) = a.wheels.iter().find(|w| w.id == id) {
        let mut w = w.clone();
        w.id = new.clone();
        w.position_m.x += 0.01;
        a.wheels.push(w);
    } else if let Some(s) = robot.sensors.iter().find(|s| s.id == id) {
        let mut s = s.clone();
        s.id = new.clone();
        s.position_m.x += 0.01;
        if let Some(settings) = robot.sensing.optical.get(id).cloned() {
            robot.sensing.optical.insert(new.clone(), settings);
        }
        robot.sensors.push(s);
    } else if let Some(f) = robot.normal_force.fans.iter().find(|f| f.id == id) {
        let mut f = f.clone();
        f.id = new.clone();
        f.position_m.x += 0.01;
        robot.normal_force.fans.push(f);
    } else {
        return Err(format!("unknown component {id}"));
    }
    if let Some(m) = a.masses.iter().find(|m| m.component.as_deref() == Some(id)) {
        let mut m = m.clone();
        m.id = crate::io::assets::new_instance_id();
        m.component = Some(new.clone());
        a.masses.push(m);
    }
    Ok(new)
}
pub fn remove_component(robot: &mut RobotConfig, id: &str) -> Result<(), String> {
    if !component_ids(robot).iter().any(|i| i == id) {
        return Err(format!("unknown component {id}"));
    }
    if robot.assembly.is_none() {
        robot.assembly = Some(RobotAssembly::from_legacy(robot));
    }
    let a = robot.assembly.as_mut().unwrap();
    a.wheels.retain(|w| w.id != id);
    a.masses.retain(|m| m.component.as_deref() != Some(id));
    robot.sensors.retain(|s| s.id != id);
    robot.sensing.optical.remove(id);
    robot.normal_force.fans.retain(|s| s.id != id);
    Ok(())
}
