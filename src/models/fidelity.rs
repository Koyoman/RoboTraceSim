//! Explicit model selection. Parameters are SI and validated before a run.
use crate::json::JsonValue as J;
#[derive(Debug, Clone, PartialEq)]
pub struct FidelityConfig {
    pub preset: String,
    pub contact: String,
    pub normal: String,
    pub rolling: bool,
    pub load_exponent: f64,
    pub reference_load_n: f64,
    pub longitudinal_stiffness: f64,
    pub lateral_stiffness: f64,
    pub relaxation_s: f64,
    pub radial_stiffness_n_m: f64,
    pub caster_trail_m: f64,
}
pub struct ModelDescriptor {
    pub id: &'static str,
    pub parameters: &'static str,
    pub state: &'static str,
    pub dependencies: &'static str,
    pub limitations: &'static str,
}
pub const MODELS: &[ModelDescriptor] = &[
    ModelDescriptor {
        id: "rolling",
        parameters: "coefficient, normal load, radius",
        state: "shaft velocity",
        dependencies: "per-wheel impulse solve",
        limitations: "bounded dissipative shaft torque; no speed-dependent bearing law",
    },
    ModelDescriptor {
        id: "caster",
        parameters: "positive trail length",
        state: "swivel angle, shaft velocity",
        dependencies: "passive contact with no motor",
        limitations: "overdamped kinematic alignment; no swivel inertia or shimmy",
    },
    ModelDescriptor {
        id: "ideal",
        parameters: "wheel geometry",
        state: "pose, wheel angle",
        dependencies: "parallel driven wheels, one radius per motor",
        limitations: "kinematic differential axle; no scrub, slip or force prediction",
    },
    ModelDescriptor {
        id: "coulomb",
        parameters: "mu longitudinal/lateral, rolling coefficient",
        state: "body velocity, each wheel speed",
        dependencies: "planar rigid body, positive inertias",
        limitations: "combined elliptical friction; no vertical dynamics",
    },
    ModelDescriptor {
        id: "brush",
        parameters: "slip stiffness, relaxation, load exponent, radial stiffness",
        state: "wheel speed, relaxed slip per contact",
        dependencies: "planar body, measured positive stiffness",
        limitations:
            "reduced empirical law, requires calibration; radial compression is quasi-static",
    },
    ModelDescriptor {
        id: "static",
        parameters: "mass, COM, support geometry, applied downforce",
        state: "none",
        dependencies: "at least three non-collinear contacts",
        limitations: "minimum squared load solution; no pitch/roll",
    },
    ModelDescriptor {
        id: "quasi_static",
        parameters: "static inputs plus COM height",
        state: "previous planar acceleration",
        dependencies: "feasible nonnegative support equilibrium",
        limitations: "one-tick acceleration lag; infeasible equilibrium stops run",
    },
];
impl FidelityConfig {
    pub fn preset(name: &str) -> Result<Self, String> {
        let (contact, normal, rolling) = match name {
            "ideal" => ("ideal", "static", false),
            "simplified" => ("coulomb", "quasi_static", true),
            "realistic" => ("brush", "quasi_static", true),
            _ => return Err(format!("unknown physics preset: {name}")),
        };
        Ok(Self {
            preset: name.into(),
            contact: contact.into(),
            normal: normal.into(),
            rolling,
            load_exponent: 0.,
            reference_load_n: 1.,
            longitudinal_stiffness: 20.,
            lateral_stiffness: 20.,
            relaxation_s: 0.,
            radial_stiffness_n_m: 0.,
            caster_trail_m: 0.01,
        })
    }
    pub fn validate(&self) -> Result<(), String> {
        Self::preset(&self.preset)?;
        if !["ideal", "coulomb", "brush"].contains(&self.contact.as_str())
            || !["static", "quasi_static"].contains(&self.normal.as_str())
        {
            return Err("unknown contact/normal model".into());
        }
        for (name, v) in [
            ("reference_load_n", self.reference_load_n),
            ("longitudinal_stiffness", self.longitudinal_stiffness),
            ("lateral_stiffness", self.lateral_stiffness),
            ("caster_trail_m", self.caster_trail_m),
        ] {
            if !v.is_finite() || v <= 0. {
                return Err(format!("physics.{name} must be positive and finite"));
            }
        }
        for (name, v) in [
            ("load_exponent", self.load_exponent),
            ("relaxation_s", self.relaxation_s),
            ("radial_stiffness_n_m", self.radial_stiffness_n_m),
        ] {
            if !v.is_finite() || v < 0. {
                return Err(format!("physics.{name} must be nonnegative and finite"));
            }
        }
        if self.load_exponent > 1. {
            return Err("load_exponent must be <= 1".into());
        }
        if self.contact != "brush"
            && (self.load_exponent != 0.
                || self.relaxation_s != 0.
                || self.radial_stiffness_n_m != 0.)
        {
            return Err("tire refinements require brush contact".into());
        }
        if self.contact == "ideal" && (self.rolling || self.normal != "static") {
            return Err("ideal kinematics requires static normal and rolling disabled".into());
        }
        Ok(())
    }
    pub fn from_value(v: &J) -> Result<Self, String> {
        let allowed = [
            "preset",
            "contact",
            "normal",
            "rolling",
            "load_exponent",
            "reference_load_n",
            "longitudinal_stiffness",
            "lateral_stiffness",
            "relaxation_s",
            "radial_stiffness_n_m",
            "caster_trail_m",
        ];
        if let J::Object(fields) = v {
            for k in fields.keys() {
                if !allowed.contains(&k.as_str()) {
                    return Err(format!("unsupported physics option: {k}"));
                }
            }
        } else {
            return Err("physics must be an object".into());
        }
        let name = v
            .get("preset")
            .and_then(J::as_str)
            .ok_or("physics.preset required")?;
        let mut f = Self::preset(name)?;
        for (key, dst) in [("contact", &mut f.contact), ("normal", &mut f.normal)] {
            if let Some(v) = v.get(key) {
                *dst = v
                    .as_str()
                    .ok_or(format!("physics.{key} must be a string"))?
                    .into();
            }
        }
        if let Some(v) = v.get("rolling") {
            f.rolling = v.as_bool().ok_or("physics.rolling must be boolean")?;
        }
        for (key, dst) in [
            ("load_exponent", &mut f.load_exponent),
            ("reference_load_n", &mut f.reference_load_n),
            ("longitudinal_stiffness", &mut f.longitudinal_stiffness),
            ("lateral_stiffness", &mut f.lateral_stiffness),
            ("relaxation_s", &mut f.relaxation_s),
            ("radial_stiffness_n_m", &mut f.radial_stiffness_n_m),
            ("caster_trail_m", &mut f.caster_trail_m),
        ] {
            if let Some(v) = v.get(key) {
                *dst = v.as_f64().ok_or(format!("physics.{key} must be numeric"))?;
            }
        }
        f.validate()?;
        Ok(f)
    }
    pub fn to_json(&self) -> String {
        let mut fields = vec![
            ("preset", J::String(self.preset.clone())),
            ("contact", J::String(self.contact.clone())),
            ("normal", J::String(self.normal.clone())),
            ("rolling", J::Bool(self.rolling)),
        ];
        for (k, v) in [
            ("load_exponent", self.load_exponent),
            ("reference_load_n", self.reference_load_n),
            ("longitudinal_stiffness", self.longitudinal_stiffness),
            ("lateral_stiffness", self.lateral_stiffness),
            ("relaxation_s", self.relaxation_s),
            ("radial_stiffness_n_m", self.radial_stiffness_n_m),
            ("caster_trail_m", self.caster_trail_m),
        ] {
            fields.push((k, J::Number(v)));
        }
        J::Object(fields.into_iter().map(|(k, v)| (k.into(), v)).collect())
            .to_json()
            .unwrap_or_else(|_| "null".into())
    }
}
