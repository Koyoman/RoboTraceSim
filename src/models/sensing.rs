//! Persisted stage-8 acquisition/control settings. Times are integer microseconds.
use crate::json::JsonValue as J;
use std::collections::BTreeMap;
macro_rules! numeric_config {
    ($name:ident { $($field:ident : $ty:ty = $default:expr),* $(,)? }) => {
        #[derive(Debug,Clone,PartialEq)] pub struct $name { $(pub $field:$ty),* }
        impl Default for $name { fn default()->Self { Self { $($field:$default),* } } }
        impl $name {
            pub fn from_value(v:&J)->Result<Self,String>{
                let J::Object(map)=v else{return Err(concat!(stringify!($name)," must be an object").into())};
                let mut out=Self::default();
                for (key,value) in map { match key.as_str() { $(stringify!($field)=>{let n=value.as_f64().ok_or("expected number")?;if !n.is_finite() || (stringify!($ty)!="f64" && !(0.0..=9007199254740991.0).contains(&n)) || (n as $ty) as f64 != n {return Err(format!("invalid {}",key));}out.$field=n as $ty;}),*, _=>return Err(format!("unknown {} field {}",stringify!($name),key)) } }
                Ok(out)
            }
            pub fn value(&self)->J {J::Object([$( (stringify!($field).into(),J::Number(self.$field as f64)) ),*].into_iter().collect())}
        }
    }
}
numeric_config!(Acquisition {
    period_us: u64 = 0,
    phase_us: u64 = 0,
    mux_group: u32 = 0,
    conversion_us: u64 = 0,
    latency_us: u64 = 0,
    filter_tau_s: f64 = 0.,
    threshold: f64 = 0.5,
    hysteresis: f64 = 0.,
    area_samples: u32 = 9
});
numeric_config!(EncoderSettings {
    shaft_ratio: f64 = 1.,
    quadrature: u32 = 1,
    latency_us: u64 = 0,
    filter_tau_s: f64 = 0.,
    loss_probability: f64 = 0.,
    seed: u64 = 311
});
numeric_config!(ImuSettings {
    latency_us: u64 = 0,
    filter_tau_s: f64 = 0.,
    drift_std_rad_s_sqrt_s: f64 = 0.,
    yaw_misalignment_deg: f64 = 0.,
    accel_noise_std_m_s2: f64 = 0.,
    accel_bias_x_m_s2: f64 = 0.,
    accel_bias_y_m_s2: f64 = 0.,
    accel_limit_m_s2: f64 = 100.,
    seed: u64 = 419
});
numeric_config!(ControlSettings {
    derivative_tau_s: f64 = 0.,
    integral_limit: f64 = 1.,
    speed_mode: u32 = 0,
    target_speed_m_s: f64 = 0.3,
    speed_kp: f64 = 1.,
    speed_ki: f64 = 1.,
    recovery_pwm: f64 = 0.15,
    loss_timeout_us: u64 = 100000,
    gyro_weight: f64 = 0.5,
    mark_threshold: f64 = 0.8,
    mark_min_channels: u32 = 2,
    mark_refractory_us: u64 = 100000,
    lap_min_distance_m: f64 = 0.5,
    profile_enabled: u32 = 0,
    curve_speed_factor: f64 = 0.6
});
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SensingConfig {
    pub replay_csv: Option<String>,
    pub optical: BTreeMap<String, Acquisition>,
    pub encoder: EncoderSettings,
    pub imu: ImuSettings,
    pub control: ControlSettings,
}
impl SensingConfig {
    pub fn from_value(v: &J) -> Result<Self, String> {
        let J::Object(map) = v else {
            return Err("sensing must be object".into());
        };
        let mut s = Self::default();
        for (k, v) in map {
            match k.as_str() {
                "replay_csv" => {
                    s.replay_csv = {
                        let text = v.as_str().ok_or("replay_csv must be string")?;
                        if text.is_empty() {
                            None
                        } else {
                            Some(text.into())
                        }
                    }
                }
                "optical" => {
                    let J::Object(items) = v else {
                        return Err("optical must map sensor IDs to settings".into());
                    };
                    for (id, v) in items {
                        s.optical.insert(id.clone(), Acquisition::from_value(v)?);
                    }
                }
                "encoder" => s.encoder = EncoderSettings::from_value(v)?,
                "imu" => s.imu = ImuSettings::from_value(v)?,
                "control" => s.control = ControlSettings::from_value(v)?,
                _ => return Err(format!("unknown sensing field {k}")),
            }
        }
        Ok(s)
    }
    pub fn to_json(&self) -> String {
        J::Object(
            [
                (
                    "replay_csv".into(),
                    self.replay_csv
                        .clone()
                        .map(J::String)
                        .unwrap_or(J::String(String::new())),
                ),
                (
                    "optical".into(),
                    J::Object(
                        self.optical
                            .iter()
                            .map(|(k, v)| (k.clone(), v.value()))
                            .collect(),
                    ),
                ),
                ("encoder".into(), self.encoder.value()),
                ("imu".into(), self.imu.value()),
                ("control".into(), self.control.value()),
            ]
            .into_iter()
            .collect(),
        )
        .to_json()
        .unwrap_or("null".into())
    }
    pub fn validate(
        &self,
        robot: &crate::config::RobotConfig,
        time: crate::config::TimeConfig,
    ) -> Result<(), String> {
        if let Some(text) = &self.replay_csv {
            let replay = crate::control::replay_controller::ReplayController::from_csv(
                text,
                time.physics_dt_us,
            )?;
            if robot.powertrain.is_none() && replay.requires_powertrain() {
                return Err("brake/coast replay requires coupled powertrain".into());
            }
        }
        let grid = |n: u64| n % time.physics_dt_us == 0;
        for (id, a) in &self.optical {
            if !robot.sensors.iter().any(|s| s.id == *id) {
                return Err(format!("unknown optical ID {id}"));
            }
            let period = if a.period_us == 0 {
                time.sensor_period_us
            } else {
                a.period_us
            };
            if !grid(period)
                || !grid(a.phase_us)
                || !grid(a.conversion_us)
                || !grid(a.latency_us)
                || a.phase_us >= period
                || a.conversion_us > period
                || a.conversion_us.checked_add(a.latency_us).is_none()
                || a.latency_us / period > 10000
                || !(1..=64).contains(&a.area_samples)
                || a.filter_tau_s < 0.
                || !(0.0..=1.0).contains(&a.threshold)
                || !(0.0..=1.0).contains(&a.hysteresis)
            {
                return Err(format!("invalid optical timing/response for {id}"));
            }
        }
        let active: Vec<_> = self
            .optical
            .iter()
            .filter(|(id, a)| {
                a.mux_group > 0 && robot.sensors.iter().any(|s| s.enabled && s.id == **id)
            })
            .collect();
        for (index, (id, a)) in active.iter().enumerate() {
            let period = if a.period_us == 0 {
                time.sensor_period_us
            } else {
                a.period_us
            };
            if a.conversion_us == 0
                || a.phase_us
                    .checked_add(a.conversion_us)
                    .is_none_or(|end| end > period)
            {
                return Err(format!("multiplexer {id}: conversion must fit its slot"));
            }
            for (other, b) in active.iter().skip(index + 1) {
                if a.mux_group != b.mux_group {
                    continue;
                }
                let other_period = if b.period_us == 0 {
                    time.sensor_period_us
                } else {
                    b.period_us
                };
                if period != other_period
                    || (a.phase_us < b.phase_us.saturating_add(b.conversion_us)
                        && b.phase_us < a.phase_us.saturating_add(a.conversion_us))
                {
                    return Err(format!(
                        "multiplexer slots overlap or periods differ: {id}, {other}"
                    ));
                }
            }
        }
        let e = &self.encoder;
        let i = &self.imu;
        let c = &self.control;
        if e.shaft_ratio <= 0.
            || e.shaft_ratio > 10000.
            || ![1, 2, 4].contains(&e.quadrature)
            || robot
                .encoder
                .ticks_per_rev
                .checked_mul(e.quadrature)
                .is_none()
            || !(0.0..=1.0).contains(&e.loss_probability)
            || e.filter_tau_s < 0.
            || !grid(e.latency_us)
            || e.latency_us / time.encoder_period_us > 10000
        {
            return Err("invalid encoder settings".into());
        }
        if !grid(i.latency_us)
            || i.latency_us / time.imu_period_us > 10000
            || i.filter_tau_s < 0.
            || i.drift_std_rad_s_sqrt_s < 0.
            || i.accel_noise_std_m_s2 < 0.
            || i.accel_limit_m_s2 <= 0.
        {
            return Err("invalid IMU settings".into());
        }
        if c.derivative_tau_s < 0.
            || c.integral_limit < 0.
            || c.speed_mode > 1
            || c.profile_enabled > 1
            || (c.profile_enabled == 1 && c.speed_mode != 1)
            || c.speed_kp < 0.
            || c.speed_ki < 0.
            || c.target_speed_m_s < 0.
            || !(0.0..=1.0).contains(&c.gyro_weight)
            || !(0.0..=1.0).contains(&c.mark_threshold)
            || !(0.0..=1.0).contains(&c.curve_speed_factor)
            || !(0.0..=1.0).contains(&c.recovery_pwm)
            || c.lap_min_distance_m <= 0.
            || c.mark_min_channels == 0
        {
            return Err("invalid control settings".into());
        }
        let finite = |v: J| -> bool {
            if let J::Object(m) = v {
                m.values().all(|v| v.as_f64().is_some_and(f64::is_finite))
            } else {
                false
            }
        };
        if !finite(e.value())
            || !finite(i.value())
            || !finite(c.value())
            || self.optical.values().any(|a| !finite(a.value()))
        {
            return Err("non-finite sensing setting".into());
        }
        Ok(())
    }
}
