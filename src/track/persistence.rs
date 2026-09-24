use super::definition::*;
use crate::{json::JsonValue as J, math::Vec2};
fn obj(v: Vec<(&str, J)>) -> J {
    J::Object(v.into_iter().map(|(k, v)| (k.to_owned(), v)).collect())
}
fn n(v: f64) -> J {
    J::Number(v)
}
fn s(v: &str) -> J {
    J::String(v.into())
}
fn array(v: impl IntoIterator<Item = J>) -> J {
    J::Array(v.into_iter().collect())
}
fn rect(r: &RectArea) -> J {
    obj(vec![
        (
            "center_mm",
            array([n(r.center_m.x * 1000.), n(r.center_m.y * 1000.)]),
        ),
        (
            "size_mm",
            array([n(r.size_m.x * 1000.), n(r.size_m.y * 1000.)]),
        ),
        ("angle_deg", n(r.angle_deg)),
    ])
}
fn color(c: [u8; 3]) -> J {
    array(c.map(|v| n(v as f64)))
}
impl TrackEnvironment {
    pub fn to_value(&self) -> J {
        obj(vec![
            ("schema", s("rtsim-track-environment-v1")),
            ("outside_material", s(&self.outside_material)),
            ("outside_mu", n(self.outside_mu)),
            ("outside_reflectance", n(self.outside_reflectance)),
            ("race_enabled", J::Bool(self.race_enabled)),
            ("laps", n(self.laps as f64)),
            ("stop_on_exit", J::Bool(self.stop_on_exit)),
            (
                "start_source",
                s(if self.start_source == StartSource::Track {
                    "track"
                } else {
                    "project"
                }),
            ),
            ("relief_enabled", J::Bool(self.relief_enabled)),
            (
                "regions",
                array(self.regions.iter().map(|r| {
                    obj(vec![
                        ("id", s(&r.id)),
                        ("area", rect(&r.area)),
                        ("material", s(&r.material)),
                        ("mu", n(r.mu)),
                        ("reflectance", n(r.reflectance)),
                        ("color", color(r.color)),
                        ("height_mm", n(r.height_m * 1000.)),
                        ("roughness_mm", n(r.roughness_m * 1000.)),
                    ])
                })),
            ),
            (
                "marks",
                array(self.marks.iter().map(|m| {
                    obj(vec![
                        ("id", s(&m.id)),
                        ("area", rect(&m.area)),
                        (
                            "kind",
                            s(if m.kind == MarkKind::Paint {
                                "paint"
                            } else {
                                "gap"
                            }),
                        ),
                        ("reflectance", n(m.reflectance)),
                        ("color", color(m.color)),
                    ])
                })),
            ),
            (
                "gates",
                array(self.gates.iter().map(|g| {
                    obj(vec![
                        ("id", s(&g.id)),
                        (
                            "kind",
                            s(match g.kind {
                                GateKind::Start => "start",
                                GateKind::Checkpoint => "checkpoint",
                                GateKind::Finish => "finish",
                            }),
                        ),
                        (
                            "center_mm",
                            array([n(g.center_m.x * 1000.), n(g.center_m.y * 1000.)]),
                        ),
                        ("heading_deg", n(g.heading_deg)),
                        ("half_width_mm", n(g.half_width_m * 1000.)),
                    ])
                })),
            ),
        ])
    }
    pub fn from_value(v: &J) -> Result<Self, String> {
        if strv(v, "schema")? != "rtsim-track-environment-v1" {
            return Err("unknown track environment schema".into());
        }
        let laps = num(v, "laps")?;
        if laps.fract() != 0. || laps < 1. || laps > u32::MAX as f64 {
            return Err("laps must be positive integer".into());
        }
        Ok(Self {
            outside_material: strv(v, "outside_material")?,
            outside_mu: num(v, "outside_mu")?,
            outside_reflectance: num(v, "outside_reflectance")?,
            race_enabled: boolean(v, "race_enabled")?,
            laps: laps as u32,
            stop_on_exit: boolean(v, "stop_on_exit")?,
            start_source: match strv(v, "start_source")?.as_str() {
                "project" => StartSource::Project,
                "track" => StartSource::Track,
                _ => return Err("unknown start_source".into()),
            },
            relief_enabled: boolean(v, "relief_enabled")?,
            regions: list(v, "regions")?
                .iter()
                .map(|r| {
                    Ok(SurfaceRegion {
                        id: strv(r, "id")?,
                        area: read_rect(field(r, "area")?)?,
                        material: strv(r, "material")?,
                        mu: num(r, "mu")?,
                        reflectance: num(r, "reflectance")?,
                        color: read_color(r)?,
                        height_m: num(r, "height_mm")? / 1000.,
                        roughness_m: num(r, "roughness_mm")? / 1000.,
                    })
                })
                .collect::<Result<_, String>>()?,
            marks: list(v, "marks")?
                .iter()
                .map(|m| {
                    Ok(OpticalMark {
                        id: strv(m, "id")?,
                        area: read_rect(field(m, "area")?)?,
                        kind: match strv(m, "kind")?.as_str() {
                            "paint" => MarkKind::Paint,
                            "gap" => MarkKind::Gap,
                            _ => return Err("unknown mark kind".into()),
                        },
                        reflectance: num(m, "reflectance")?,
                        color: read_color(m)?,
                    })
                })
                .collect::<Result<_, String>>()?,
            gates: list(v, "gates")?
                .iter()
                .map(|g| {
                    Ok(RaceGate {
                        id: strv(g, "id")?,
                        kind: match strv(g, "kind")?.as_str() {
                            "start" => GateKind::Start,
                            "checkpoint" => GateKind::Checkpoint,
                            "finish" => GateKind::Finish,
                            _ => return Err("unknown gate kind".into()),
                        },
                        center_m: pair(g, "center_mm")? * 0.001,
                        heading_deg: num(g, "heading_deg")?,
                        half_width_m: num(g, "half_width_mm")? / 1000.,
                    })
                })
                .collect::<Result<_, String>>()?,
        })
    }
}
fn field<'a>(v: &'a J, k: &str) -> Result<&'a J, String> {
    v.get(k).ok_or_else(|| format!("environment.{k} required"))
}
fn num(v: &J, k: &str) -> Result<f64, String> {
    field(v, k)?
        .as_f64()
        .filter(|n| n.is_finite())
        .ok_or_else(|| format!("environment.{k}: finite number required"))
}
fn strv(v: &J, k: &str) -> Result<String, String> {
    field(v, k)?
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("environment.{k}: string required"))
}
fn boolean(v: &J, k: &str) -> Result<bool, String> {
    match field(v, k)? {
        J::Bool(b) => Ok(*b),
        _ => Err(format!("environment.{k}: boolean required")),
    }
}
fn list<'a>(v: &'a J, k: &str) -> Result<&'a [J], String> {
    field(v, k)?
        .as_array()
        .ok_or_else(|| format!("environment.{k}: array required"))
}
fn pair(v: &J, k: &str) -> Result<Vec2, String> {
    let a = list(v, k)?;
    if a.len() != 2 {
        return Err("coordinate pair required".into());
    }
    Ok(Vec2::new(
        a[0].as_f64().ok_or("invalid coordinate")?,
        a[1].as_f64().ok_or("invalid coordinate")?,
    ))
}
fn read_rect(v: &J) -> Result<RectArea, String> {
    Ok(RectArea {
        center_m: pair(v, "center_mm")? * 0.001,
        size_m: pair(v, "size_mm")? * 0.001,
        angle_deg: num(v, "angle_deg")?,
    })
}
fn read_color(v: &J) -> Result<[u8; 3], String> {
    let c = list(v, "color")?;
    if c.len() != 3 {
        return Err("RGB requires 3 bytes".into());
    }
    let mut out = [0; 3];
    for (i, v) in c.iter().enumerate() {
        let n = v.as_f64().ok_or("invalid RGB")?;
        if !n.is_finite() || n.fract() != 0. || !(0.0..=255.).contains(&n) {
            return Err("RGB channel must be a byte".into());
        }
        out[i] = n as u8;
    }
    Ok(out)
}
