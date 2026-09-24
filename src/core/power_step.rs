// Included in sim.rs to share the single mechanical executor and its output contract.
type PowerTrial = (
    RobotState,
    crate::models::contact::ContactState,
    ConfiguredNormalForce,
    crate::models::power::PowerState,
    LastPhysics,
);
#[allow(clippy::too_many_arguments)]
fn power_trial(
    cfg: &LoadedConfig,
    track: &dyn TrackModel,
    body: RobotState,
    contacts: &crate::models::contact::ContactState,
    normal_model: &ConfiguredNormalForce,
    old: &crate::models::power::PowerState,
    command: ControllerOutput,
    voltage: f64,
    dt_us: u64,
) -> Result<PowerTrial, String> {
    use crate::models::{electrical::*, power::*};
    let p = cfg.robot.powertrain.as_ref().unwrap();
    let dt = dt_us as f64 * 1e-6;
    let mut normals = normal_model.clone();
    let normal = normals.step(NormalForceInput {
        mass_kg: cfg.robot.chassis.mass_kg,
        center_of_mass_m: cfg.robot.chassis.center_of_mass_m,
        wheelbase_m: cfg.robot.drivetrain.wheelbase_m,
        track_width_m: cfg.robot.drivetrain.track_width_m,
        battery_voltage_v: voltage,
        command_pwm: command.pwm_downforce,
        speed_m_s: body.vx_body_m_s,
        dt_us,
    });
    let aux = p.auxiliary_power_w / p.regulator_efficiency;
    let capacity = cfg.robot.battery.current_limit_a;
    let motor_budget = ((capacity - normal.current_a - aux / voltage.max(0.01)) / 2.).max(0.);
    let current_limit = cfg.robot.driver.current_limit_a.min(motor_budget);
    let mut guess = [body.wheel_omega_left_rad_s, body.wheel_omega_right_rad_s];
    let mut gain_override = [None; 2];
    let mut gain_bounds = std::array::from_fn::<_, 2, _>(|side| {
        let m = &p.motors[side];
        (
            m.kt_nm_a * m.ratio * m.efficiency,
            m.kt_nm_a * m.ratio / m.efficiency,
        )
    });
    for _ in 0..80 {
        let mut gains = [0.; 2];
        let mut resistances = [0.; 2];
        let mut voltages = [0.; 2];
        let mut drop_signs = [0.; 2];
        let drives = std::array::from_fn(|side| {
            let m = &p.motors[side];
            let state = &old.motors[side];
            let mode = old.applied.modes[side];
            let r = m.resistance_ohm
                * (1. + p.resistance_temp_coefficient * (state.temperature_c - p.ambient_c));
            resistances[side] = r;
            let duty = duty(
                old.applied.pwm[side],
                cfg.robot.driver.pwm_resolution_bits,
                cfg.robot.driver.command_deadband,
            );
            let source = duty * voltage + m.inductance_h / dt * state.current_a
                - m.ke_v_s_rad * m.ratio * guess[side];
            let drop_sign = if source.abs() <= cfg.robot.driver.voltage_drop_v {
                0.
            } else {
                source.signum()
            };
            drop_signs[side] = drop_sign;
            let applied = match mode {
                ActuatorMode::Drive => duty * voltage - drop_sign * cfg.robot.driver.voltage_drop_v,
                ActuatorMode::Brake => 0.,
                ActuatorMode::Coast => -state.current_a.signum() * voltage,
            };
            voltages[side] = applied;
            let i = current_step(
                state.current_a,
                applied,
                m.ratio * guess[side],
                m,
                r + p.bridge_resistance_ohm,
                dt,
            );
            let gain = m.kt_nm_a
                * m.ratio
                * if i * guess[side] >= 0. {
                    m.efficiency
                } else {
                    1. / m.efficiency
                };
            let gain = gain_override[side].unwrap_or(gain);
            gains[side] = gain;
            let re = r + p.bridge_resistance_ohm + m.inductance_h / dt;
            let damping = gain * m.ke_v_s_rad * m.ratio / re;
            let target =
                (m.inductance_h / dt * state.current_a + applied) / (m.ke_v_s_rad * m.ratio);
            let (lo, hi) = if mode == ActuatorMode::Drive && drop_sign == 0. {
                (0., 0.)
            } else if mode == ActuatorMode::Coast {
                if state.current_a > 0. {
                    (0., current_limit)
                } else if state.current_a < 0. {
                    (-current_limit, 0.)
                } else {
                    (0., 0.)
                }
            } else {
                (-current_limit, current_limit)
            };
            crate::motor::MotorDrive::affine(
                target,
                damping,
                lo * gain,
                hi * gain,
                m.viscous_nm_s * m.ratio * m.ratio,
            )
        });
        let mut next_body = body;
        let mut next_contacts = contacts.clone();
        let mut unused_battery = VoltageSagBattery::new(cfg.robot.battery.clone());
        let mut result = individual_physics(
            &mut next_body,
            body,
            cfg,
            track,
            &mut next_contacts,
            cfg.robot.physics.as_ref().unwrap(),
            normal,
            drives,
            &mut unused_battery,
            voltage,
            f64::INFINITY,
            dt_us,
        )?;
        let speeds = [
            next_body.wheel_omega_left_rad_s,
            next_body.wheel_omega_right_rad_s,
        ];
        let torques = [
            result.motor_left.wheel_torque_nm,
            result.motor_right.wheel_torque_nm,
        ];
        let mut changed = false;
        for side in 0..2 {
            let m = &p.motors[side];
            let i = torques[side] / gains[side];
            let gain = m.kt_nm_a
                * m.ratio
                * if i * speeds[side] >= 0. {
                    m.efficiency
                } else {
                    1. / m.efficiency
                };
            if old.applied.modes[side] == ActuatorMode::Drive {
                let duty = duty(
                    old.applied.pwm[side],
                    cfg.robot.driver.pwm_resolution_bits,
                    cfg.robot.driver.command_deadband,
                );
                let source = duty * voltage + m.inductance_h / dt * old.motors[side].current_a
                    - m.ke_v_s_rad * m.ratio * speeds[side];
                let sign = if source.abs() <= cfg.robot.driver.voltage_drop_v {
                    0.
                } else {
                    source.signum()
                };
                if sign != drop_signs[side] {
                    changed = true;
                }
            }
            if (torques[side] * speeds[side]).abs() > 1e-10 && (gain - gains[side]).abs() > 1e-12 {
                if gain < gains[side] {
                    gain_bounds[side].1 = gains[side];
                } else {
                    gain_bounds[side].0 = gains[side];
                }
                gain_override[side] = Some(0.5 * (gain_bounds[side].0 + gain_bounds[side].1));
                changed = true;
            }
        }
        if changed {
            guess = speeds;
            continue;
        }
        let mut next = old.clone();
        let mut bus_power = aux + normal.current_a * voltage;
        let mut outputs = [MotorOutput::default(); 2];
        let mut returned = 0.;
        let mut dumped = 0.;
        for side in 0..2 {
            let m = &p.motors[side];
            let previous = &old.motors[side];
            let i = torques[side] / gains[side];
            let omega = speeds[side] * m.ratio;
            let r = resistances[side];
            let terminal = (r + p.bridge_resistance_ohm) * i
                + m.inductance_h * (i - previous.current_a) / dt
                + m.ke_v_s_rad * omega;
            let fixed_drop = if old.applied.modes[side] == ActuatorMode::Drive {
                cfg.robot.driver.voltage_drop_v * i.abs()
            } else {
                0.
            };
            let copper = r * i * i;
            let bridge = p.bridge_resistance_ohm * i * i + fixed_drop;
            let magnetic = 0.5 * m.inductance_h * i * i;
            let derivative = (magnetic - previous.magnetic_energy_j) / dt;
            let numerical = 0.5 * m.inductance_h * (i - previous.current_a).powi(2) / dt;
            let bearing = m.viscous_nm_s * omega * omega;
            let shaft = torques[side] * speeds[side] - bearing;
            let gear = (m.kt_nm_a * i * omega - torques[side] * speeds[side]).max(0.);
            let input = terminal * i + fixed_drop;
            let regeneration = p.regeneration
                && old.soc < 1. - 1e-10
                && old.applied.modes[side] != ActuatorMode::Brake;
            let accepted = if input < 0. && regeneration {
                input.max(-p.charge_limit_a * voltage / 2.)
            } else {
                input.max(0.)
            };
            let dump = (accepted - input).max(0.);
            dumped += dump;
            returned += (-accepted).max(0.);
            bus_power += accepted;
            let balance = input - copper - bridge - derivative - numerical - gear - bearing - shaft;
            if !balance.is_finite() || balance.abs() > 1e-6 * (1. + input.abs()) {
                return Err(format!("electrical power balance residual {balance} W"));
            }
            let limited =
                (i.abs() - current_limit).abs() < 1e-7 && (voltages[side] - terminal).abs() > 1e-6;
            let temperature = temperature_step(
                previous.temperature_c,
                copper + bridge + gear + bearing,
                p,
                dt,
            );
            next.motors[side] = MotorElectricalState {
                current_a: i,
                rotor_speed_rad_s: omega,
                temperature_c: temperature,
                applied_voltage_v: terminal,
                copper_loss_w: copper,
                bridge_loss_w: bridge,
                gear_loss_w: gear,
                bearing_loss_w: bearing,
                magnetic_energy_j: magnetic,
                numerical_loss_w: numerical,
                shaft_power_w: shaft,
                bus_power_w: accepted,
                dump_power_w: dump,
                balance_residual_w: balance,
                current_limited: limited,
            };
            outputs[side] = MotorOutput {
                wheel_torque_nm: torques[side],
                motor_torque_nm: m.kt_nm_a * i,
                current_a: i,
                supply_current_a: accepted / voltage,
                voltage_v: terminal,
                applied_pwm: (terminal / voltage).clamp(-1., 1.),
                braking: old.applied.modes[side] == ActuatorMode::Brake,
                coasting: old.applied.modes[side] == ActuatorMode::Coast,
            };
        }
        let current = bus_power / voltage;
        let (target, soc, rc) = battery_trial(p, &cfg.robot.battery, old, current, dt);
        next.voltage_v = voltage;
        next.current_a = current;
        next.soc = soc;
        next.polarization_v = rc;
        next.source_power_w = open_voltage(p, &cfg.robot.battery, soc) * current;
        next.battery_loss_w = resistance(p, &cfg.robot.battery, soc) * current * current
            + if p.rc_resistance_ohm > 0. {
                rc * rc / p.rc_resistance_ohm
            } else {
                0.
            };
        next.auxiliary_power_w = aux;
        next.downforce_power_w = normal.current_a * voltage;
        next.bus_recovered_energy_j += returned * dt;
        next.regenerated_energy_j += (-bus_power).max(0.) * dt;
        let capacitance = if p.source != "ideal" && p.rc_resistance_ohm > 0. {
            p.rc_time_s / p.rc_resistance_ohm
        } else {
            0.
        };
        next.polarization_energy_j = 0.5 * capacitance * rc * rc;
        next.battery_numerical_loss_w = 0.5 * capacitance * (rc - old.polarization_v).powi(2) / dt;
        next.battery_balance_residual_w = next.source_power_w
            - bus_power
            - next.battery_loss_w
            - (next.polarization_energy_j - old.polarization_energy_j) / dt
            - next.battery_numerical_loss_w;
        next.dumped_energy_j += dumped * dt;
        next.residual_v = voltage - target;
        result.motor_left = outputs[0];
        result.motor_right = outputs[1];
        result.battery = BatteryOutput {
            terminal_voltage_v: voltage,
            open_circuit_voltage_v: open_voltage(p, &cfg.robot.battery, soc),
            current_a: current,
            soc,
        };
        result.diagnostics.electrical_energy_j = voltage * current * dt;
        return Ok((next_body, next_contacts, normals, next, result));
    }
    Err("electrical transmission branch did not converge".into())
}
#[allow(clippy::too_many_arguments)]
fn coupled_power_step(
    body: &mut RobotState,
    cfg: &LoadedConfig,
    track: &dyn TrackModel,
    contacts: &mut crate::models::contact::ContactState,
    normal_model: &mut ConfiguredNormalForce,
    power: &mut crate::models::power::PowerState,
    command: ControllerOutput,
    t_us: u64,
    dt_us: u64,
) -> Result<LastPhysics, String> {
    use crate::models::power::*;
    let p = cfg.robot.powertrain.as_ref().unwrap();
    let modes = [command.pwm_left, command.pwm_right].map(|v| {
        if v.abs() > cfg.robot.driver.command_deadband {
            ActuatorMode::Drive
        } else if cfg.robot.driver.mode == "brake" {
            ActuatorMode::Brake
        } else {
            ActuatorMode::Coast
        }
    });
    if power.manual_command.is_none() {
        power.request(
            ActuatorCommand {
                t_us,
                pwm: [command.pwm_left, command.pwm_right],
                modes,
            },
            p.command_latency_us,
        )?;
    }
    power.deliver(t_us);
    let trial = |voltage: f64| {
        power_trial(
            cfg,
            track,
            *body,
            contacts,
            normal_model,
            power,
            command,
            voltage,
            dt_us,
        )
    };
    let mut evaluations = 0;
    let selected = if p.source == "ideal" {
        evaluations += 1;
        Some(trial(cfg.robot.battery.nominal_voltage_v)?)
    } else {
        let max_ocv = p
            .ocv_curve
            .iter()
            .map(|(_, v)| *v)
            .fold(cfg.robot.battery.full_voltage_v, f64::max);
        let max_r = p
            .resistance_curve
            .iter()
            .map(|(_, r)| *r)
            .fold(cfg.robot.battery.internal_resistance_ohm, f64::max)
            + p.wiring_resistance_ohm
            + p.rc_resistance_ohm;
        let upper = max_ocv + max_r * p.charge_limit_a + power.polarization_v.abs() + 0.1;
        let lower = p.undervoltage_v.max(0.01);
        let mut hi = upper;
        let mut bracket = None;
        // Search downward for the highest-voltage branch (constant-power loads can have two roots).
        for index in 1..=32 {
            let v = upper - (upper - lower) * index as f64 / 32.;
            let sample = trial(v)?;
            evaluations += 1;
            if sample.3.residual_v <= 0. {
                bracket = Some((v, hi));
                break;
            }
            hi = v;
        }
        if let Some((mut lo, mut hi)) = bracket {
            let mut solution = None;
            for _ in 0..p.max_iterations {
                let v = (lo + hi) / 2.;
                let sample = trial(v)?;
                evaluations += 1;
                let residual = sample.3.residual_v;
                if residual.abs() <= p.voltage_tolerance_v {
                    solution = Some(sample);
                    break;
                }
                if residual > 0. {
                    hi = v
                } else {
                    lo = v
                }
            }
            solution
        } else {
            power.trip(t_us, "undervoltage_or_voltage_collapse");
            return Err("power protection: no feasible bus voltage".into());
        }
    };
    let Some((next_body, next_contacts, next_normal, mut next, result)) = selected else {
        return Err("power circuit did not converge within configured iteration/tolerance".into());
    };
    if next.voltage_v < p.undervoltage_v {
        power.trip(t_us, "undervoltage");
        return Err("undervoltage protection".into());
    }
    if next.current_a > cfg.robot.battery.current_limit_a + 1e-6
        || next.current_a < -p.charge_limit_a - 1e-6
    {
        power.trip(t_us, "overcurrent");
        return Err("battery current protection".into());
    }
    if !(0.0..=1.0).contains(&next.soc) {
        power.trip(t_us, "state_of_charge_limit");
        return Err("battery state of charge protection".into());
    }
    if next
        .motors
        .iter()
        .any(|m| m.temperature_c > p.max_temperature_c)
    {
        power.trip(t_us, "overtemperature");
        return Err("motor thermal protection".into());
    }
    for side in 0..2 {
        if next.motors[side].current_limited && !power.motors[side].current_limited {
            next.events
                .push((t_us + dt_us, format!("motor_{side}_current_limited")));
        }
    }
    next.iterations = evaluations;
    *body = next_body;
    *contacts = next_contacts;
    *normal_model = next_normal;
    *power = next;
    Ok(result)
}
