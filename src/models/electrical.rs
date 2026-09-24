//! SI electrical and transmission definitions for the coupled power solver.
use crate::json::JsonValue as J;
#[derive(Debug, Clone, PartialEq)]
pub struct ElectricalMotor {
    pub model: String,
    pub parameter_shaft: String,
    pub resistance_ohm: f64,
    pub inductance_h: f64,
    pub ke_v_s_rad: f64,
    pub kt_nm_a: f64,
    pub rotor_inertia_kg_m2: f64,
    pub viscous_nm_s: f64,
    pub ratio: f64,
    pub efficiency: f64,
}
impl Default for ElectricalMotor {
    fn default() -> Self {
        Self {
            model: "dc_electrical".into(),
            parameter_shaft: "motor".into(),
            resistance_ohm: 4.,
            inductance_h: 0.0005,
            ke_v_s_rad: 0.003,
            kt_nm_a: 0.003,
            rotor_inertia_kg_m2: 1e-8,
            viscous_nm_s: 1e-7,
            ratio: 10.,
            efficiency: 0.85,
        }
    }
}
#[derive(Debug, Clone, PartialEq)]
pub struct PowertrainConfig {
    pub motors: [ElectricalMotor; 2],
    pub source: String,
    pub ocv_curve: Vec<(f64, f64)>,
    pub resistance_curve: Vec<(f64, f64)>,
    pub rc_resistance_ohm: f64,
    pub rc_time_s: f64,
    pub wiring_resistance_ohm: f64,
    pub auxiliary_power_w: f64,
    pub regulator_efficiency: f64,
    pub bridge_resistance_ohm: f64,
    pub regeneration: bool,
    pub charge_limit_a: f64,
    pub undervoltage_v: f64,
    pub command_latency_us: u64,
    pub thermal_capacity_j_k: f64,
    pub thermal_resistance_k_w: f64,
    pub ambient_c: f64,
    pub max_temperature_c: f64,
    pub resistance_temp_coefficient: f64,
    pub suction_gap_m: f64,
    pub suction_leak_per_m: f64,
    pub voltage_tolerance_v: f64,
    pub max_iterations: u32,
}
impl Default for PowertrainConfig {
    fn default() -> Self {
        Self {
            motors: std::array::from_fn(|_| ElectricalMotor::default()),
            source: "thevenin".into(),
            ocv_curve: vec![],
            resistance_curve: vec![],
            rc_resistance_ohm: 0.,
            rc_time_s: 1.,
            wiring_resistance_ohm: 0.,
            auxiliary_power_w: 0.,
            regulator_efficiency: 1.,
            bridge_resistance_ohm: 0.05,
            regeneration: false,
            charge_limit_a: 0.,
            undervoltage_v: 4.,
            command_latency_us: 0,
            thermal_capacity_j_k: 10.,
            thermal_resistance_k_w: 20.,
            ambient_c: 25.,
            max_temperature_c: 120.,
            resistance_temp_coefficient: 0.0039,
            suction_gap_m: 0.,
            suction_leak_per_m: 0.,
            voltage_tolerance_v: 1e-7,
            max_iterations: 80,
        }
    }
}
fn number(v: &J, k: &str, d: f64) -> Result<f64, String> {
    v.get(k)
        .map(|v| v.as_f64().ok_or(format!("powertrain.{k} must be numeric")))
        .unwrap_or(Ok(d))
}
fn string(v: &J, k: &str, d: &str) -> Result<String, String> {
    v.get(k)
        .map(|v| {
            v.as_str()
                .map(str::to_owned)
                .ok_or(format!("powertrain.{k} must be a string"))
        })
        .unwrap_or(Ok(d.into()))
}
fn keys(v: &J, allowed: &[&str]) -> Result<(), String> {
    if let J::Object(o) = v {
        for k in o.keys() {
            if !allowed.contains(&k.as_str()) {
                return Err(format!("unsupported powertrain field {k}"));
            }
        }
        Ok(())
    } else {
        Err("powertrain definition must be an object".into())
    }
}
fn curve(v: &J, k: &str) -> Result<Vec<(f64, f64)>, String> {
    let Some(v) = v.get(k) else { return Ok(vec![]) };
    v.as_array()
        .ok_or("curve must be array")?
        .iter()
        .map(|v| {
            let a = v.as_array().ok_or("curve point must be array")?;
            if a.len() != 2 {
                return Err("curve point needs two numbers".into());
            }
            Ok((
                a[0].as_f64().ok_or("invalid curve x")?,
                a[1].as_f64().ok_or("invalid curve y")?,
            ))
        })
        .collect()
}
fn object(fields: Vec<(&str, J)>) -> J {
    J::Object(fields.into_iter().map(|(k, v)| (k.into(), v)).collect())
}
impl ElectricalMotor {
    pub fn validate(&self) -> Result<(), String> {
        if !["dc_simple", "dc_electrical"].contains(&self.model.as_str()) {
            return Err("unsupported motor: explicit switching/BLDC/FOC not implemented".into());
        }
        if !["motor", "output"].contains(&self.parameter_shaft.as_str())
            || (self.parameter_shaft == "output" && self.ratio != 1.)
        {
            return Err("output-shaft parameters require ratio=1; do not reduce catalog output parameters twice".into());
        }
        for v in [
            self.resistance_ohm,
            self.ke_v_s_rad,
            self.kt_nm_a,
            self.ratio,
            self.efficiency,
        ] {
            if !v.is_finite() || v <= 0. {
                return Err("invalid positive motor parameter".into());
            }
        }
        for v in [
            self.inductance_h,
            self.rotor_inertia_kg_m2,
            self.viscous_nm_s,
        ] {
            if !v.is_finite() || v < 0. {
                return Err("invalid nonnegative motor parameter".into());
            }
        }
        if self.efficiency > 1. || (self.kt_nm_a - self.ke_v_s_rad).abs() > 1e-9 * self.ke_v_s_rad {
            return Err("passive SI DC model requires Kt=Ke and efficiency <=1".into());
        }
        if self.model == "dc_electrical" && self.inductance_h == 0. {
            return Err("dc_electrical requires L>0; use dc_simple for algebraic current".into());
        }
        if self.model == "dc_simple" && self.inductance_h != 0. {
            return Err("dc_simple requires L=0".into());
        }
        Ok(())
    }
    fn from_value(v: &J) -> Result<Self, String> {
        keys(
            v,
            &[
                "model",
                "parameter_shaft",
                "resistance_ohm",
                "inductance_h",
                "ke_v_s_rad",
                "kt_nm_a",
                "rotor_inertia_kg_m2",
                "viscous_nm_s",
                "ratio",
                "efficiency",
            ],
        )?;
        let mut m = Self::default();
        m.model = string(v, "model", &m.model)?;
        m.parameter_shaft = string(v, "parameter_shaft", &m.parameter_shaft)?;
        for (k, x) in [
            ("resistance_ohm", &mut m.resistance_ohm),
            ("inductance_h", &mut m.inductance_h),
            ("ke_v_s_rad", &mut m.ke_v_s_rad),
            ("kt_nm_a", &mut m.kt_nm_a),
            ("rotor_inertia_kg_m2", &mut m.rotor_inertia_kg_m2),
            ("viscous_nm_s", &mut m.viscous_nm_s),
            ("ratio", &mut m.ratio),
            ("efficiency", &mut m.efficiency),
        ] {
            *x = number(v, k, *x)?;
        }
        m.validate()?;
        Ok(m)
    }
    fn value(&self) -> J {
        let mut f = vec![
            ("model", J::String(self.model.clone())),
            ("parameter_shaft", J::String(self.parameter_shaft.clone())),
        ];
        for (k, x) in [
            ("resistance_ohm", self.resistance_ohm),
            ("inductance_h", self.inductance_h),
            ("ke_v_s_rad", self.ke_v_s_rad),
            ("kt_nm_a", self.kt_nm_a),
            ("rotor_inertia_kg_m2", self.rotor_inertia_kg_m2),
            ("viscous_nm_s", self.viscous_nm_s),
            ("ratio", self.ratio),
            ("efficiency", self.efficiency),
        ] {
            f.push((k, J::Number(x)));
        }
        object(f)
    }
}
impl PowertrainConfig {
    pub fn validate(&self) -> Result<(), String> {
        for m in &self.motors {
            m.validate()?;
        }
        if !["ideal", "thevenin"].contains(&self.source.as_str()) {
            return Err("unsupported source; separate dynamic buses are not implemented".into());
        }
        for v in [
            self.rc_time_s,
            self.regulator_efficiency,
            self.thermal_capacity_j_k,
            self.thermal_resistance_k_w,
            self.voltage_tolerance_v,
        ] {
            if !v.is_finite() || v <= 0. {
                return Err("invalid positive power parameter".into());
            }
        }
        for v in [
            self.rc_resistance_ohm,
            self.wiring_resistance_ohm,
            self.auxiliary_power_w,
            self.bridge_resistance_ohm,
            self.charge_limit_a,
            self.undervoltage_v,
            self.resistance_temp_coefficient,
            self.suction_gap_m,
            self.suction_leak_per_m,
        ] {
            if !v.is_finite() || v < 0. {
                return Err("invalid nonnegative power parameter".into());
            }
        }
        if !self.ambient_c.is_finite()
            || !self.max_temperature_c.is_finite()
            || self.max_temperature_c <= self.ambient_c
            || self.regulator_efficiency > 1.
            || self.max_iterations < 8
            || self.max_iterations > 256
        {
            return Err("invalid thermal/regulator/iteration configuration".into());
        }
        if self.regeneration && self.charge_limit_a <= 0. {
            return Err("regeneration requires positive charge_limit_a".into());
        }
        for c in [&self.ocv_curve, &self.resistance_curve] {
            if !c.is_empty()
                && (c.len() < 2
                    || c[0].0 != 0.
                    || c.last().unwrap().0 != 1.
                    || c.iter()
                        .any(|(x, y)| !x.is_finite() || !y.is_finite() || *y < 0.)
                    || c.windows(2).any(|w| w[1].0 <= w[0].0))
            {
                return Err("SoC curves require ordered finite points covering 0..1".into());
            }
        }
        Ok(())
    }
    pub fn from_value(v: &J) -> Result<Self, String> {
        keys(
            v,
            &[
                "motors",
                "source",
                "ocv_curve",
                "resistance_curve",
                "rc_resistance_ohm",
                "rc_time_s",
                "wiring_resistance_ohm",
                "auxiliary_power_w",
                "regulator_efficiency",
                "bridge_resistance_ohm",
                "regeneration",
                "charge_limit_a",
                "undervoltage_v",
                "command_latency_us",
                "thermal_capacity_j_k",
                "thermal_resistance_k_w",
                "ambient_c",
                "max_temperature_c",
                "resistance_temp_coefficient",
                "suction_gap_m",
                "suction_leak_per_m",
                "voltage_tolerance_v",
                "max_iterations",
            ],
        )?;
        let mut c = Self::default();
        let ms = v
            .get("motors")
            .and_then(J::as_array)
            .ok_or("powertrain.motors requires left/right definitions")?;
        if ms.len() != 2 {
            return Err("exactly two motors required".into());
        }
        c.motors = [
            ElectricalMotor::from_value(&ms[0])?,
            ElectricalMotor::from_value(&ms[1])?,
        ];
        c.source = string(v, "source", &c.source)?;
        c.ocv_curve = curve(v, "ocv_curve")?;
        c.resistance_curve = curve(v, "resistance_curve")?;
        for (k, x) in [
            ("rc_resistance_ohm", &mut c.rc_resistance_ohm),
            ("rc_time_s", &mut c.rc_time_s),
            ("wiring_resistance_ohm", &mut c.wiring_resistance_ohm),
            ("auxiliary_power_w", &mut c.auxiliary_power_w),
            ("regulator_efficiency", &mut c.regulator_efficiency),
            ("bridge_resistance_ohm", &mut c.bridge_resistance_ohm),
            ("charge_limit_a", &mut c.charge_limit_a),
            ("undervoltage_v", &mut c.undervoltage_v),
            ("thermal_capacity_j_k", &mut c.thermal_capacity_j_k),
            ("thermal_resistance_k_w", &mut c.thermal_resistance_k_w),
            ("ambient_c", &mut c.ambient_c),
            ("max_temperature_c", &mut c.max_temperature_c),
            (
                "resistance_temp_coefficient",
                &mut c.resistance_temp_coefficient,
            ),
            ("suction_gap_m", &mut c.suction_gap_m),
            ("suction_leak_per_m", &mut c.suction_leak_per_m),
            ("voltage_tolerance_v", &mut c.voltage_tolerance_v),
        ] {
            *x = number(v, k, *x)?;
        }
        if let Some(b) = v.get("regeneration") {
            c.regeneration = b.as_bool().ok_or("regeneration must be boolean")?;
        }
        for (k, max) in [
            ("command_latency_us", u32::MAX as u64),
            ("max_iterations", 256),
        ] {
            if let Some(v) = v.get(k) {
                let n = v
                    .as_u64()
                    .filter(|n| *n <= max)
                    .ok_or("integer power parameter out of range")?;
                if k == "command_latency_us" {
                    c.command_latency_us = n
                } else {
                    c.max_iterations = n as u32
                }
            }
        }
        c.validate()?;
        Ok(c)
    }
    pub fn to_json(&self) -> String {
        let mut f = vec![
            ("source", J::String(self.source.clone())),
            (
                "motors",
                J::Array(self.motors.iter().map(ElectricalMotor::value).collect()),
            ),
            ("regeneration", J::Bool(self.regeneration)),
        ];
        for (k, c) in [
            ("ocv_curve", &self.ocv_curve),
            ("resistance_curve", &self.resistance_curve),
        ] {
            f.push((
                k,
                J::Array(
                    c.iter()
                        .map(|(x, y)| J::Array(vec![J::Number(*x), J::Number(*y)]))
                        .collect(),
                ),
            ));
        }
        for (k, x) in [
            ("rc_resistance_ohm", self.rc_resistance_ohm),
            ("rc_time_s", self.rc_time_s),
            ("wiring_resistance_ohm", self.wiring_resistance_ohm),
            ("auxiliary_power_w", self.auxiliary_power_w),
            ("regulator_efficiency", self.regulator_efficiency),
            ("bridge_resistance_ohm", self.bridge_resistance_ohm),
            ("charge_limit_a", self.charge_limit_a),
            ("undervoltage_v", self.undervoltage_v),
            ("thermal_capacity_j_k", self.thermal_capacity_j_k),
            ("thermal_resistance_k_w", self.thermal_resistance_k_w),
            ("ambient_c", self.ambient_c),
            ("max_temperature_c", self.max_temperature_c),
            (
                "resistance_temp_coefficient",
                self.resistance_temp_coefficient,
            ),
            ("suction_gap_m", self.suction_gap_m),
            ("suction_leak_per_m", self.suction_leak_per_m),
            ("voltage_tolerance_v", self.voltage_tolerance_v),
            ("command_latency_us", self.command_latency_us as f64),
            ("max_iterations", self.max_iterations as f64),
        ] {
            f.push((k, J::Number(x)));
        }
        object(f).to_json().unwrap_or_else(|_| "null".into())
    }
}
/// Backward Euler RL current at held terminal voltage and solved rotor speed.
pub fn current_step(
    previous: f64,
    voltage: f64,
    omega: f64,
    m: &ElectricalMotor,
    r: f64,
    dt: f64,
) -> f64 {
    (m.inductance_h / dt * previous + voltage - m.ke_v_s_rad * omega) / (r + m.inductance_h / dt)
}
pub fn interpolate(curve: &[(f64, f64)], x: f64, fallback: f64) -> f64 {
    if curve.is_empty() {
        return fallback;
    }
    if x <= curve[0].0 {
        return curve[0].1;
    }
    for p in curve.windows(2) {
        if x <= p[1].0 {
            return p[0].1 + (p[1].1 - p[0].1) * (x - p[0].0) / (p[1].0 - p[0].0);
        }
    }
    curve.last().unwrap().1
}
