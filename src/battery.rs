use crate::config::BatteryConfig;
use crate::math::clamp;

#[derive(Debug, Clone, Copy, Default)]
pub struct BatteryOutput {
    pub terminal_voltage_v: f64,
    pub open_circuit_voltage_v: f64,
    pub current_a: f64,
    pub soc: f64,
}

#[derive(Debug, Clone)]
pub struct VoltageSagBattery {
    cfg: BatteryConfig,
    soc: f64,
    terminal_voltage_v: f64,
}

impl VoltageSagBattery {
    pub fn new(cfg: BatteryConfig) -> Self {
        let soc = clamp(cfg.initial_soc, 0.0, 1.0);
        let open = open_circuit_voltage(&cfg, soc);
        Self {
            cfg,
            soc,
            terminal_voltage_v: open,
        }
    }

    pub fn terminal_voltage_v(&self) -> f64 {
        self.terminal_voltage_v
    }

    pub fn output(&self) -> BatteryOutput {
        BatteryOutput {
            terminal_voltage_v: self.terminal_voltage_v,
            open_circuit_voltage_v: open_circuit_voltage(&self.cfg, self.soc),
            current_a: 0.0,
            soc: self.soc,
        }
    }

    pub fn step(&mut self, load_current_a: f64, dt_us: u64) -> BatteryOutput {
        let current_a = clamp(
            load_current_a.max(0.0),
            0.0,
            self.cfg.current_limit_a.max(0.0),
        );
        let dt_s = dt_us as f64 / 1_000_000.0;
        let capacity_coulomb = (self.cfg.capacity_mah.max(1e-9) / 1000.0) * 3600.0;
        self.soc = clamp(self.soc - current_a * dt_s / capacity_coulomb, 0.0, 1.0);

        let open = open_circuit_voltage(&self.cfg, self.soc);
        let sag = current_a * self.cfg.internal_resistance_ohm.max(0.0);
        self.terminal_voltage_v = (open - sag).max(0.0);

        BatteryOutput {
            terminal_voltage_v: self.terminal_voltage_v,
            open_circuit_voltage_v: open,
            current_a,
            soc: self.soc,
        }
    }
}

fn open_circuit_voltage(cfg: &BatteryConfig, soc: f64) -> f64 {
    let soc = clamp(soc, 0.0, 1.0);
    cfg.empty_voltage_v + (cfg.full_voltage_v - cfg.empty_voltage_v) * soc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> BatteryConfig {
        BatteryConfig {
            model: "VoltageSagBattery".to_string(),
            cells: 2,
            nominal_voltage_v: 7.4,
            full_voltage_v: 7.4,
            empty_voltage_v: 6.0,
            capacity_mah: 300.0,
            internal_resistance_ohm: 0.1,
            initial_soc: 1.0,
            current_limit_a: 100.0,
        }
    }

    #[test]
    fn load_current_causes_voltage_sag() {
        let mut b = VoltageSagBattery::new(cfg());
        let no_load = b.terminal_voltage_v();
        let loaded = b.step(5.0, 500).terminal_voltage_v;
        assert!(loaded < no_load);
    }
}
