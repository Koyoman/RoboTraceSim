use super::TimedCommand;
use crate::models::power::ActuatorMode;
/// Immutable measured commands. Zero-order hold, no interpolation or future lookahead.
#[derive(Debug, Clone)]
pub struct ReplayController {
    commands: Vec<TimedCommand>,
}
impl ReplayController {
    pub fn new(commands: Vec<TimedCommand>, physics_dt_us: u64) -> Result<Self, String> {
        if physics_dt_us == 0 || commands.first().is_none_or(|c| c.t_us != 0) {
            return Err("replay requires an initial command at t=0".into());
        }
        let mut previous = None;
        for c in &commands {
            if c.t_us % physics_dt_us != 0
                || previous.is_some_and(|p| c.t_us <= p)
                || c.pwm.iter().any(|x| !x.is_finite() || x.abs() > 1.)
                || !c.downforce_pwm.is_finite()
                || !(0.0..=1.0).contains(&c.downforce_pwm)
            {
                return Err("invalid replay command/order/grid".into());
            }
            previous = Some(c.t_us);
        }
        Ok(Self { commands })
    }
    pub fn from_csv(text: &str, dt: u64) -> Result<Self, String> {
        let mut lines = text.lines();
        if lines.next() != Some("t_us,pwm_left,pwm_right,downforce_pwm,mode_left,mode_right") {
            return Err("invalid command CSV header".into());
        }
        let mut out = Vec::new();
        for line in lines {
            let v: Vec<_> = line.split(',').collect();
            if v.len() != 6 {
                return Err("invalid command CSV row".into());
            }
            let mode = |s: &str| match s {
                "drive" => Ok(ActuatorMode::Drive),
                "brake" => Ok(ActuatorMode::Brake),
                "coast" => Ok(ActuatorMode::Coast),
                _ => Err("invalid actuator mode"),
            };
            out.push(TimedCommand {
                t_us: v[0].parse().map_err(|_| "invalid time")?,
                pwm: [
                    v[1].parse().map_err(|_| "invalid PWM")?,
                    v[2].parse().map_err(|_| "invalid PWM")?,
                ],
                downforce_pwm: v[3].parse().map_err(|_| "invalid PWM")?,
                modes: [mode(v[4])?, mode(v[5])?],
            });
        }
        Self::new(out, dt)
    }
    pub fn requires_powertrain(&self) -> bool {
        self.commands
            .iter()
            .any(|c| c.modes.iter().any(|m| *m != ActuatorMode::Drive))
    }
    pub fn at(&self, t_us: u64) -> TimedCommand {
        let index = self
            .commands
            .partition_point(|c| c.t_us <= t_us)
            .saturating_sub(1);
        let mut c = self.commands[index];
        c.t_us = t_us;
        c
    }
    pub fn csv(&self) -> String {
        let mut out = String::from("t_us,pwm_left,pwm_right,downforce_pwm,mode_left,mode_right\n");
        let mode = |m| match m {
            ActuatorMode::Drive => "drive",
            ActuatorMode::Brake => "brake",
            ActuatorMode::Coast => "coast",
        };
        for c in &self.commands {
            out.push_str(&format!(
                "{},{},{},{},{},{}\n",
                c.t_us,
                c.pwm[0],
                c.pwm[1],
                c.downforce_pwm,
                mode(c.modes[0]),
                mode(c.modes[1])
            ));
        }
        out
    }
}
