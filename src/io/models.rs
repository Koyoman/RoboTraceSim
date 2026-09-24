//! Canonical selectors. Serialized labels are parsed explicitly, never defaulted on typos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NormalForceKind {
    None,
    Constant,
    Fan,
    Measured,
    ClosedLoop,
    Suction,
}
impl NormalForceKind {
    pub fn parse(s: &str) -> Result<Self, String> {
        match s.to_ascii_lowercase().as_str() {
            "none" | "nodownforce" | "weightonly" => Ok(Self::None),
            "constantdownforce" => Ok(Self::Constant),
            "fandownforce" => Ok(Self::Fan),
            "measureddownforcecurve" => Ok(Self::Measured),
            "closedloopdownforce" => Ok(Self::ClosedLoop),
            "suctiondownforce" => Ok(Self::Suction),
            _ => Err(format!("unknown downforce model: {s}")),
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "NoDownforce",
            Self::Constant => "ConstantDownforce",
            Self::Fan => "FanDownforce",
            Self::Measured => "MeasuredDownforceCurve",
            Self::ClosedLoop => "ClosedLoopDownforce",
            Self::Suction => "SuctionDownforce",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorKind {
    DcElectrical,
    DcSimple,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TireKind {
    RigidSlip,
    RigidCoulomb,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverMode {
    Brake,
    Coast,
}
#[derive(Debug, Clone)]
pub struct ResolvedModels {
    pub motors: [MotorKind; 2],
    pub tire: TireKind,
    pub driver: DriverMode,
    pub downforce: NormalForceKind,
}
impl ResolvedModels {
    pub fn from_robot(r: &crate::config::RobotConfig) -> Result<Self, String> {
        for m in [&r.motor_left, &r.motor_right] {
            if m.model != "DcMotorSimple" {
                return Err(format!("unsupported motor model: {}", m.model));
            }
        }
        let tire = match r.tire.model.as_str() {
            "SlipRatioWheel" => TireKind::RigidSlip,
            "CoulombFrictionWheel" => TireKind::RigidCoulomb,
            _ => return Err(format!("unsupported tire model: {}", r.tire.model)),
        };
        let driver = match r.driver.mode.to_ascii_lowercase().as_str() {
            "brake" => DriverMode::Brake,
            "coast" | "free" | "hi-z" | "hiz" => DriverMode::Coast,
            _ => return Err(format!("unknown driver mode: {}", r.driver.mode)),
        };
        for (actual, expected) in [
            (&r.driver.model, "PwmHBridge"),
            (&r.battery.model, "VoltageSagBattery"),
            (&r.encoder.model, "QuantizedEncoder"),
            (&r.gyro.model, "NoisyGyro"),
        ] {
            if actual != expected {
                return Err(format!("unsupported model {actual}, expected {expected}"));
            }
        }
        Ok(Self {
            motors: r
                .powertrain
                .as_ref()
                .map(|p| {
                    std::array::from_fn(|i| {
                        if p.motors[i].model == "dc_electrical" {
                            MotorKind::DcElectrical
                        } else {
                            MotorKind::DcSimple
                        }
                    })
                })
                .unwrap_or([MotorKind::DcSimple; 2]),
            tire,
            driver,
            downforce: r.normal_force.model,
        })
    }
}
