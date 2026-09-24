use crate::config::*;
use crate::rtsim_track::*;
use std::fs;
use std::path::{Path, PathBuf};
pub fn save_track_to_file(track: &TrackConfig, path: &Path) -> Result<(), String> {
    crate::track::definition::validate_definition(track)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, track_json(track)).map_err(|e| e.to_string())
}

pub fn save_surface_profile_to_file(profile: &SurfaceProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, surface_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_robot_to_file(robot: &RobotConfig, path: &Path) -> Result<(), String> {
    crate::io::validation::validate_robot(robot)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, robot_json(robot)).map_err(|e| e.to_string())
}

pub fn save_motor_profile_to_file(profile: &MotorProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, motor_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_driver_profile_to_file(profile: &DriverProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, driver_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_battery_profile_to_file(profile: &BatteryProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, battery_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_tire_profile_to_file(profile: &TireProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, tire_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_encoder_profile_to_file(profile: &EncoderProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, encoder_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_gyro_profile_to_file(profile: &GyroProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, gyro_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_sensor_asset_to_file(asset: &SensorAsset, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, sensor_asset_json(asset, 0)).map_err(|e| e.to_string())
}

pub fn save_fan_profile_to_file(profile: &FanProfile, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(path, fan_profile_json(profile)).map_err(|e| e.to_string())
}

pub fn save_loaded_config(cfg: &LoadedConfig) -> Result<(), String> {
    crate::track::definition::validate_definition(&cfg.track)?;
    crate::io::validation::validate_robot(&cfg.robot)?;
    crate::core::scheduler::validate_time(&cfg.project.time)?;
    for text in [project_json(&cfg.project), track_json(&cfg.track)] {
        let json = crate::json::parse_json(&text).map_err(|e| e.to_string())?;
        crate::io::validation::validate_document(&json)?;
    }
    if let Some(track) = &cfg.track.parametric {
        if track.rules.mode == TrackRulesMode::Strict {
            let errors: Vec<_> = validate_track(track)
                .into_iter()
                .filter(|issue| issue.severity == Severity::Error)
                .collect();
            if !errors.is_empty() {
                return Err(format!(
                    "modo strict bloqueou o salvamento: {}",
                    errors
                        .iter()
                        .take(3)
                        .map(|issue| issue.message.as_str())
                        .collect::<Vec<_>>()
                        .join("; ")
                ));
            }
        }
    }
    let base_dir = cfg.project_path.parent().unwrap_or_else(|| Path::new("."));
    fs::create_dir_all(base_dir).map_err(|e| e.to_string())?;
    let robot_path =
        crate::io::assets::resolve_from_file(&cfg.project_path, &cfg.project.robot_path);
    let track_path =
        crate::io::assets::resolve_from_file(&cfg.project_path, &cfg.project.track_path);
    if let Some(parent) = robot_path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    if let Some(parent) = track_path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    write_json(&cfg.project_path, project_json(&cfg.project)).map_err(|e| e.to_string())?;
    write_json(&robot_path, robot_json(&cfg.robot)).map_err(|e| e.to_string())?;
    write_json(&track_path, track_json(&cfg.track)).map_err(|e| e.to_string())?;
    Ok(())
}

pub fn project_json(project: &ProjectConfig) -> String {
    let mut out = String::new();
    out.push_str("{\n");
    out.push_str(&format!(
        "  \"rtsim_schema\": \"{}\",\n",
        escape_json(&project.schema)
    ));
    out.push_str(&format!(
        "  \"name\": \"{}\",\n",
        escape_json(&project.name)
    ));
    out.push_str(&format!(
        "  \"robot\": \"{}\",\n",
        escape_json(&project.robot_path.to_string_lossy().replace('\\', "/"))
    ));
    out.push_str(&format!(
        "  \"track\": \"{}\",\n",
        escape_json(&project.track_path.to_string_lossy().replace('\\', "/"))
    ));
    out.push_str("  \"time\": {\n");
    out.push_str(&format!(
        "    \"physics_dt_us\": {},\n",
        project.time.physics_dt_us
    ));
    out.push_str(&format!(
        "    \"controller_period_us\": {},\n",
        project.time.controller_period_us
    ));
    out.push_str(&format!(
        "    \"sensor_period_us\": {},\n",
        project.time.sensor_period_us
    ));
    out.push_str(&format!(
        "    \"imu_period_us\": {},\n",
        project.time.imu_period_us
    ));
    out.push_str(&format!(
        "    \"encoder_period_us\": {},\n",
        project.time.encoder_period_us
    ));
    out.push_str(&format!(
        "    \"log_period_us\": {},\n",
        project.time.log_period_us
    ));
    out.push_str(&format!(
        "    \"render_period_us\": {}\n",
        project.time.render_period_us
    ));
    out.push_str("  },\n");
    out.push_str("  \"simulation\": {\n");
    out.push_str(&format!("    \"duration_s\": {},\n", project.duration_s));
    out.push_str(&format!(
        "    \"start_pose_m\": [{}, {}, {}]\n",
        project.start_pose.x, project.start_pose.y, project.start_pose.yaw
    ));
    out.push_str("  },\n");
    out.push_str(&format!(
        "  \"log\": {{\n    \"csv\": {},\n    \"replay\": {}\n",
        optional_path(&project.csv_output),
        optional_path(&project.replay_output)
    ));
    out.push_str("  }\n}\n");
    out
}

pub fn motor_profile_json(profile: &MotorProfile) -> String {
    format!(
        "{{\n  \"motor_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"motor\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        motor_json(&profile.motor, 2)
    )
}

pub fn driver_profile_json(profile: &DriverProfile) -> String {
    format!(
        "{{\n  \"driver_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"driver\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        driver_json(&profile.driver, 2)
    )
}

pub fn battery_profile_json(profile: &BatteryProfile) -> String {
    format!(
        "{{\n  \"battery_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"battery\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        battery_json(&profile.battery, 2)
    )
}

pub fn tire_profile_json(profile: &TireProfile) -> String {
    format!(
        "{{\n  \"tire_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"tire\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        tire_json(&profile.tire, 2)
    )
}

pub fn encoder_profile_json(profile: &EncoderProfile) -> String {
    format!(
        "{{\n  \"encoder_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"encoder\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        encoder_json(&profile.encoder, 2)
    )
}

pub fn gyro_profile_json(profile: &GyroProfile) -> String {
    format!(
        "{{\n  \"gyro_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"gyro\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        gyro_json(&profile.gyro, 2)
    )
}

pub fn fan_profile_json(profile: &FanProfile) -> String {
    format!(
        "{{\n  \"fan_profile_schema\": \"{}\",\n  \"name\": \"{}\",\n  \"fan\": {}\n}}\n",
        escape_json(&profile.schema),
        escape_json(&profile.name),
        fan_json(&profile.fan, 2)
    )
}

pub fn robot_json(robot: &RobotConfig) -> String {
    let mut out = String::new();
    out.push_str("{\n");

    out.push_str(&format!("  \"robot_schema\": \"{}\",\n", "rtsim-robot-v8"));
    out.push_str(&format!("  \"name\": \"{}\",\n", escape_json(&robot.name)));
    out.push_str(&format!(
        "  \"assembly\": {},\n",
        crate::models::robot::RobotAssembly::effective(robot).to_json()
    ));
    out.push_str(&format!("  \"sensing\": {},\n", robot.sensing.to_json()));
    if let Some(p) = &robot.powertrain {
        out.push_str(&format!("  \"powertrain\": {},\n", p.to_json()));
    }
    if let Some(f) = &robot.physics {
        out.push_str(&format!("  \"physics\": {},\n", f.to_json()));
    }
    out.push_str("  \"chassis\": {\n");
    out.push_str(&format!("    \"model\": \"RigidBody2DChassis\",\n    \"mass_g\": {},\n    \"inertia_kg_m2\": {},\n    \"center_of_mass_mm\": [{}, {}],\n    \"length_mm\": {},\n    \"width_mm\": {}\n  }},\n",
        robot.chassis.mass_kg * 1000.0,
        robot.chassis.inertia_kg_m2,
        robot.chassis.center_of_mass_m.x * 1000.0,
        robot.chassis.center_of_mass_m.y * 1000.0,
        robot.chassis.length_m * 1000.0,
        robot.chassis.width_m * 1000.0));
    out.push_str(&normal_force_json(&robot.normal_force));
    out.push_str(",\n  \"drivetrain\": {\n");
    out.push_str(&format!("    \"type\": \"DifferentialDrive4Wheel\",\n    \"wheel_radius_mm\": {},\n    \"wheel_width_mm\": {},\n    \"track_width_mm\": {},\n    \"wheelbase_mm\": {},\n    \"wheel_inertia_g_cm2\": {}\n  }},\n",
        robot.drivetrain.wheel_radius_m * 1000.0,
        robot.drivetrain.wheel_width_m * 1000.0,
        robot.drivetrain.track_width_m * 1000.0,
        robot.drivetrain.wheelbase_m * 1000.0,
        robot.drivetrain.wheel_inertia_kg_m2 / 1e-7));
    out.push_str(&format!("  \"tire\": {{\n    \"model\": \"{}\",\n    \"mu_longitudinal\": {},\n    \"mu_lateral\": {},\n    \"rolling_resistance\": {},\n    \"slip_velocity_epsilon_m_s\": {}\n  }},\n",
        escape_json(&robot.tire.model), robot.tire.mu_longitudinal, robot.tire.mu_lateral, robot.tire.rolling_resistance, robot.tire.slip_velocity_epsilon_m_s));
    out.push_str("  \"motors\": {\n");
    out.push_str(&format!(
        "    \"left\": {},\n",
        motor_json(&robot.motor_left, 4)
    ));
    out.push_str(&format!(
        "    \"right\": {}\n  }},\n",
        motor_json(&robot.motor_right, 4)
    ));
    out.push_str(&format!("  \"driver\": {{\n    \"model\": \"{}\",\n    \"pwm_frequency_hz\": {},\n    \"mode\": \"{}\",\n    \"voltage_drop_v\": {},\n    \"pwm_resolution_bits\": {},\n    \"command_deadband\": {},\n    \"current_limit_a\": {}\n  }},\n",
        escape_json(&robot.driver.model), robot.driver.pwm_frequency_hz, escape_json(&robot.driver.mode), robot.driver.voltage_drop_v, robot.driver.pwm_resolution_bits, robot.driver.command_deadband, robot.driver.current_limit_a));
    out.push_str(&format!("  \"battery\": {{\n    \"model\": \"{}\",\n    \"nominal_voltage_v\": {},\n    \"full_voltage_v\": {},\n    \"empty_voltage_v\": {},\n    \"cells\": {},\n    \"capacity_mah\": {},\n    \"internal_resistance_ohm\": {},\n    \"initial_soc\": {},\n    \"current_limit_a\": {}\n  }},\n",
        escape_json(&robot.battery.model), robot.battery.nominal_voltage_v, robot.battery.full_voltage_v, robot.battery.empty_voltage_v, robot.battery.cells, robot.battery.capacity_mah, robot.battery.internal_resistance_ohm, robot.battery.initial_soc, robot.battery.current_limit_a));
    out.push_str(&format!(
        "  \"sensors\": {},\n",
        robot_sensors_json(&robot.sensors, 2)
    ));
    out.push_str(&format!(
        "  \"line_validity_areas\": {},\n",
        robot_line_validity_areas_json(&robot.line_validity_areas, 2)
    ));
    out.push_str(&format!("  \"encoder\": {{\n    \"model\": \"{}\",\n    \"ticks_per_rev\": {},\n    \"invert_left\": {},\n    \"invert_right\": {}\n  }},\n",
        escape_json(&robot.encoder.model), robot.encoder.ticks_per_rev, robot.encoder.invert_left, robot.encoder.invert_right));
    out.push_str(&format!("  \"gyro\": {{\n    \"model\": \"{}\",\n    \"noise_std_rad_s\": {},\n    \"bias_rad_s\": {},\n    \"saturation_rad_s\": {},\n    \"seed\": {}\n  }},\n",
        escape_json(&robot.gyro.model), robot.gyro.noise_std_rad_s, robot.gyro.bias_rad_s, robot.gyro.saturation_rad_s, robot.gyro.seed));
    out.push_str(&format!("  \"controller\": {{\n    \"model\": \"BuiltInPid\",\n    \"kp\": {},\n    \"ki\": {},\n    \"kd\": {},\n    \"base_pwm\": {},\n    \"max_pwm\": {},\n    \"target_position_mm\": {},\n    \"downforce_pwm\": {}\n  }}\n}}\n",
        robot.controller.kp, robot.controller.ki, robot.controller.kd, robot.controller.base_pwm, robot.controller.max_pwm, robot.controller.target_position_m * 1000.0, robot.controller.downforce_pwm));
    out
}

pub fn normal_force_json(normal: &NormalForceConfig) -> String {
    let mut out = String::new();
    out.push_str("  \"normal_force\": {\n");
    out.push_str(&format!(
        "    \"model\": \"{}\",\n",
        escape_json(normal.model.as_str())
    ));
    out.push_str(&format!(
        "    \"default_pwm\": {},\n",
        normal.command_pwm_default
    ));
    out.push_str(&format!(
        "    \"position_mm\": [{}, {}],\n",
        normal.position_m.x * 1000.0,
        normal.position_m.y * 1000.0
    ));
    out.push_str(&format!("    \"max_force_n\": {},\n", normal.max_force_n));
    out.push_str(&format!(
        "    \"max_current_a\": {},\n",
        normal.max_current_a
    ));
    out.push_str(&format!(
        "    \"response_time_s\": {},\n",
        normal.response_time_s
    ));
    out.push_str(&format!(
        "    \"chamber_area_m2\": {},\n",
        normal.chamber_area_m2
    ));
    out.push_str(&format!(
        "    \"max_delta_pressure_pa\": {},\n",
        normal.max_delta_pressure_pa
    ));
    out.push_str(&format!(
        "    \"leakage_factor\": {},\n",
        normal.leakage_factor
    ));
    out.push_str(&format!(
        "    \"speed_sensitivity\": {},\n",
        normal.speed_sensitivity
    ));
    out.push_str(&format!(
        "    \"force_curve\": {},\n",
        curve_json(&normal.force_curve)
    ));
    out.push_str("    \"fans\": [");
    if !normal.fans.is_empty() {
        out.push('\n');
        for (idx, fan) in normal.fans.iter().enumerate() {
            if idx > 0 {
                out.push_str(",\n");
            }
            out.push_str(&fan_json(fan, 6));
        }
        out.push('\n');
        out.push_str("    ");
    }
    out.push_str("]\n  }");
    out
}

pub fn motor_json(motor: &MotorConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"nominal_voltage_v\": {},\n{pad}  \"gear_ratio\": {},\n{pad}  \"efficiency\": {},\n{pad}  \"no_load_rpm\": {},\n{pad}  \"stall_torque_mnm\": {},\n{pad}  \"stall_current_a\": {}\n{pad}}}",
        escape_json(&motor.model),
        motor.nominal_voltage_v,
        motor.gear_ratio,
        motor.efficiency,
        motor.no_load_rpm,
        motor.stall_torque_nm * 1000.0,
        motor.stall_current_a,
    )
}

pub fn driver_json(driver: &DriverConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"pwm_frequency_hz\": {},\n{pad}  \"mode\": \"{}\",\n{pad}  \"voltage_drop_v\": {},\n{pad}  \"pwm_resolution_bits\": {},\n{pad}  \"command_deadband\": {},\n{pad}  \"current_limit_a\": {}\n{pad}}}",
        escape_json(&driver.model),
        driver.pwm_frequency_hz,
        escape_json(&driver.mode),
        driver.voltage_drop_v,
        driver.pwm_resolution_bits,
        driver.command_deadband,
        driver.current_limit_a
    )
}

pub fn battery_json(battery: &BatteryConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"nominal_voltage_v\": {},\n{pad}  \"full_voltage_v\": {},\n{pad}  \"empty_voltage_v\": {},\n{pad}  \"cells\": {},\n{pad}  \"capacity_mah\": {},\n{pad}  \"internal_resistance_ohm\": {},\n{pad}  \"initial_soc\": {},\n{pad}  \"current_limit_a\": {}\n{pad}}}",
        escape_json(&battery.model),
        battery.nominal_voltage_v,
        battery.full_voltage_v,
        battery.empty_voltage_v,
        battery.cells,
        battery.capacity_mah,
        battery.internal_resistance_ohm,
        battery.initial_soc,
        battery.current_limit_a
    )
}

pub fn tire_json(tire: &TireConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"mu_longitudinal\": {},\n{pad}  \"mu_lateral\": {},\n{pad}  \"rolling_resistance\": {},\n{pad}  \"slip_velocity_epsilon_m_s\": {}\n{pad}}}",
        escape_json(&tire.model),
        tire.mu_longitudinal,
        tire.mu_lateral,
        tire.rolling_resistance,
        tire.slip_velocity_epsilon_m_s
    )
}

pub fn encoder_json(encoder: &EncoderConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"ticks_per_rev\": {},\n{pad}  \"invert_left\": {},\n{pad}  \"invert_right\": {}\n{pad}}}",
        escape_json(&encoder.model),
        encoder.ticks_per_rev,
        encoder.invert_left,
        encoder.invert_right,
    )
}

pub fn gyro_json(gyro: &GyroConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"noise_std_rad_s\": {},\n{pad}  \"bias_rad_s\": {},\n{pad}  \"saturation_rad_s\": {},\n{pad}  \"seed\": {}\n{pad}}}",
        escape_json(&gyro.model),
        gyro.noise_std_rad_s,
        gyro.bias_rad_s,
        gyro.saturation_rad_s,
        gyro.seed,
    )
}

pub fn robot_sensors_json(sensors: &[RobotSensorInstance], indent: usize) -> String {
    let pad = " ".repeat(indent);
    let mut out = String::from("[");
    if !sensors.is_empty() {
        out.push('\n');
        for (idx, sensor) in sensors.iter().enumerate() {
            if idx > 0 {
                out.push_str(",\n");
            }
            out.push_str(&robot_sensor_instance_json(sensor, indent + 2));
        }
        out.push('\n');
        out.push_str(&pad);
    }
    out.push(']');
    out
}

pub fn robot_sensor_instance_json(sensor: &RobotSensorInstance, indent: usize) -> String {
    format!("{{\"acquisition\":{{\"adc_bits\":{},\"reflectance_noise_std\":{},\"adc_noise_lsb\":{},\"seed\":{}}},\"id\":\"{}\",\"name\":\"{}\",\"asset_path\":\"{}\",\"asset\":{},\"position_mm\":[{},{}],\"height_mm\":{},\"angle_deg\":{},\"enabled\":{},\"visible_in_preview\":{}}}",
        sensor.acquisition.adc_bits,sensor.acquisition.reflectance_noise_std,sensor.acquisition.adc_noise_lsb,sensor.acquisition.seed,
        escape_json(&sensor.id),escape_json(&sensor.name),escape_json(&sensor.asset_path.to_string_lossy().replace('\\',"/")),sensor_asset_json(&sensor.asset,indent+2),
        sensor.position_m.x*1000.0,sensor.position_m.y*1000.0,sensor.height_m*1000.0,sensor.angle_deg,sensor.enabled,sensor.visible_in_preview)
}

pub fn robot_line_validity_areas_json(areas: &[RobotLineValidityArea], indent: usize) -> String {
    let pad = " ".repeat(indent);
    let item_pad = " ".repeat(indent + 2);
    let mut out = String::from("[");
    if !areas.is_empty() {
        out.push('\n');
        for (idx, area) in areas.iter().enumerate() {
            if idx > 0 {
                out.push_str(",\n");
            }
            out.push_str(&format!(
                "{item_pad}{{\n{item_pad}  \"name\": \"{}\",\n{item_pad}  \"position_mm\": [{}, {}],\n{item_pad}  \"length_mm\": {},\n{item_pad}  \"width_mm\": {},\n{item_pad}  \"angle_deg\": {},\n{item_pad}  \"enabled\": {}\n{item_pad}}}",
                escape_json(&area.name),
                area.position_m.x * 1000.0,
                area.position_m.y * 1000.0,
                area.length_m * 1000.0,
                area.width_m * 1000.0,
                area.angle_deg,
                area.enabled,
            ));
        }
        out.push('\n');
        out.push_str(&pad);
    }
    out.push(']');
    out
}

pub fn sensor_asset_json(asset: &SensorAsset, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"name\": \"{}\",\n{pad}  \"model\": \"{}\",\n{pad}  \"sensor_type\": \"{}\",\n{pad}  \"visual_width_mm\": {},\n{pad}  \"visual_height_mm\": {},\n{pad}  \"visual_radius_mm\": {},\n{pad}  \"detection_area\": {},\n{pad}  \"response_model\": {},\n{pad}  \"notes\": \"{}\"\n{pad}}}",
        escape_json(&asset.name),
        escape_json(&asset.model),
        asset.sensor_type.as_str(),
        asset.visual_width_m * 1000.0,
        asset.visual_height_m * 1000.0,
        asset.visual_radius_m * 1000.0,
        sensor_detection_area_json(&asset.detection_area, indent + 2),
        sensor_response_model_json(&asset.response_model, indent + 2),
        escape_json(&asset.notes)
    )
}

pub fn sensor_detection_area_json(area: &SensorDetectionArea, indent: usize) -> String {
    let pad = " ".repeat(indent);
    match area {
        SensorDetectionArea::Point { radius_m } => format!(
            "{{\n{pad}  \"kind\": \"Point\",\n{pad}  \"radius_mm\": {}\n{pad}}}",
            radius_m * 1000.0
        ),
        SensorDetectionArea::Rectangle { width_m, height_m } => format!(
            "{{\n{pad}  \"kind\": \"Rectangle\",\n{pad}  \"width_mm\": {},\n{pad}  \"height_mm\": {}\n{pad}}}",
            width_m * 1000.0,
            height_m * 1000.0
        ),
        SensorDetectionArea::Circle { radius_m } => format!(
            "{{\n{pad}  \"kind\": \"Circle\",\n{pad}  \"radius_mm\": {}\n{pad}}}",
            radius_m * 1000.0
        ),
        SensorDetectionArea::Cone { range_m, angle_deg } => format!(
            "{{\n{pad}  \"kind\": \"Cone\",\n{pad}  \"range_mm\": {},\n{pad}  \"angle_deg\": {}\n{pad}}}",
            range_m * 1000.0,
            angle_deg
        ),
        SensorDetectionArea::CustomPolygon { points_m } => {
            let mut out = format!("{{\n{pad}  \"kind\": \"CustomPolygon\",\n{pad}  \"points_mm\": [");
            for (idx, p) in points_m.iter().enumerate() {
                if idx > 0 {
                    out.push_str(", ");
                }
                out.push_str(&format!("[{}, {}]", p.x * 1000.0, p.y * 1000.0));
            }
            out.push_str(&format!("]\n{pad}}}"));
            out
        }
    }
}

pub fn sensor_response_model_json(model: &SensorResponseModel, indent: usize) -> String {
    let pad = " ".repeat(indent);
    match model {
        SensorResponseModel::Ideal => "\"Ideal\"".to_string(),
        SensorResponseModel::Threshold { threshold } => format!(
            "{{\n{pad}  \"kind\": \"Threshold\",\n{pad}  \"threshold\": {}\n{pad}}}",
            threshold
        ),
        SensorResponseModel::Linear { gain, offset } => format!(
            "{{\n{pad}  \"kind\": \"Linear\",\n{pad}  \"gain\": {},\n{pad}  \"offset\": {}\n{pad}}}",
            gain,
            offset
        ),
        SensorResponseModel::Polynomial { coefficients } => format!(
            "{{\n{pad}  \"kind\": \"Polynomial\",\n{pad}  \"coefficients\": {}\n{pad}}}",
            number_vec_json(coefficients)
        ),
        SensorResponseModel::LookupTable { points } => {
            let mut out = format!("{{\n{pad}  \"kind\": \"LookupTable\",\n{pad}  \"points\": [");
            for (idx, point) in points.iter().enumerate() {
                if idx > 0 {
                    out.push_str(", ");
                }
                out.push_str(&format!("[{}, {}]", point.input, point.output));
            }
            out.push_str(&format!("]\n{pad}}}"));
            out
        }
        SensorResponseModel::Custom { description } => format!(
            "{{\n{pad}  \"kind\": \"Custom\",\n{pad}  \"description\": \"{}\"\n{pad}}}",
            escape_json(description)
        ),
    }
}

pub fn number_vec_json(values: &[f64]) -> String {
    let mut out = String::from("[");
    for (idx, value) in values.iter().enumerate() {
        if idx > 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("{}", value));
    }
    out.push(']');
    out
}

pub fn fan_json(fan: &FanConfig, indent: usize) -> String {
    let pad = " ".repeat(indent);
    format!(
        "{{\n{pad}  \"id\": \"{}\",\n{pad}  \"position_mm\": [{}, {}],\n{pad}  \"visual_radius_mm\": {},\n{pad}  \"action_radius_mm\": {},\n{pad}  \"max_force_n\": {},\n{pad}  \"max_current_a\": {},\n{pad}  \"nominal_voltage_v\": {},\n{pad}  \"nominal_current_a\": {},\n{pad}  \"nominal_rpm\": {},\n{pad}  \"power_w\": {},\n{pad}  \"min_pwm\": {},\n{pad}  \"max_pwm\": {},\n{pad}  \"response_time_s\": {},\n{pad}  \"pwm_scale\": {},\n{pad}  \"pwm\": {},\n{pad}  \"curve_model\": \"{}\",\n{pad}  \"force_curve\": {}\n{pad}}}",
        escape_json(&fan.id),
        fan.position_m.x * 1000.0,
        fan.position_m.y * 1000.0,
        fan.visual_radius_m * 1000.0,
        fan.action_radius_m * 1000.0,
        fan.max_force_n,
        fan.max_current_a,
        fan.nominal_voltage_v,
        fan.nominal_current_a,
        fan.nominal_rpm,
        fan.power_w,
        fan.min_pwm,
        fan.max_pwm,
        fan.response_time_s,
        fan.pwm_scale,
        fan.enabled_pwm,
        fan.curve_model.as_str(),
        curve_json(&fan.force_curve)
    )
}

pub fn track_json(track: &TrackConfig) -> String {
    let base = track_base_json(track);
    let environment = track
        .environment
        .to_value()
        .to_json()
        .unwrap_or_else(|_| "null".into());
    let base = base.trim_end();
    format!(
        "{},\n  \"environment\": {}\n}}\n",
        &base[..base.len() - 1],
        environment
    )
}
fn track_base_json(track: &TrackConfig) -> String {
    if let Some(v2) = &track.parametric {
        return track_v2_json(v2);
    }
    let mut out = String::new();
    out.push_str("{\n");
    out.push_str(&format!(
        "  \"track_schema\": \"{}\",\n",
        escape_json(&track.schema)
    ));
    out.push_str(&format!("  \"name\": \"{}\",\n", escape_json(&track.name)));
    out.push_str(&format!(
        "  \"model\": \"{}\",\n",
        escape_json(&track.model)
    ));
    out.push_str(&format!(
        "  \"line_width_mm\": {},\n",
        track.line_width_m * 1000.0
    ));
    out.push_str(&format!(
        "  \"base_reflectance\": {},\n",
        track.base_reflectance
    ));
    out.push_str(&format!(
        "  \"line_reflectance\": {},\n",
        track.line_reflectance
    ));
    out.push_str(&format!("  \"surface_mu\": {},\n", track.surface_mu));
    out.push_str("  \"centerline_m\": [\n");
    for (i, p) in track.centerline.iter().enumerate() {
        let suffix = if i + 1 == track.centerline.len() {
            ""
        } else {
            ","
        };
        out.push_str(&format!("    [{}, {}]{}\n", p.x, p.y, suffix));
    }
    out.push_str("  ]\n}\n");
    out
}

pub fn track_v2_json(track: &TrackV2) -> String {
    let mut out = String::new();
    out.push_str("{\n");
    out.push_str(&format!(
        "  \"track_schema\": \"{}\",\n",
        escape_json(&track.schema)
    ));
    out.push_str(&format!("  \"name\": \"{}\",\n", escape_json(&track.name)));
    out.push_str(&format!(
        "  \"units\": \"{}\",\n",
        escape_json(&track.units)
    ));
    out.push_str("  \"area\": {\n");
    out.push_str(&format!("    \"width_mm\": {},\n", track.area.width_mm));
    out.push_str(&format!("    \"height_mm\": {},\n", track.area.height_mm));
    out.push_str(&format!("    \"grid_mm\": {}\n", track.area.grid_mm));
    out.push_str("  },\n");
    out.push_str("  \"origin\": {\n");
    out.push_str(&format!("    \"x_mm\": {},\n", track.origin.x_mm));
    out.push_str(&format!("    \"y_mm\": {},\n", track.origin.y_mm));
    out.push_str(&format!(
        "    \"heading_deg\": {}\n",
        track.origin.heading_deg
    ));
    out.push_str("  },\n");
    out.push_str("  \"rules\": {\n");
    out.push_str(&format!(
        "    \"source\": \"{}\", \"edition\": \"{}\",\n",
        escape_json(&track.rules.source),
        escape_json(&track.rules.edition)
    ));
    out.push_str(&format!(
        "    \"profile\": \"{}\",\n",
        escape_json(&track.rules.profile)
    ));
    out.push_str(&format!(
        "    \"mode\": \"{}\",\n",
        track.rules.mode.as_str()
    ));
    out.push_str("    \"overrides\": {");
    let mut fields = Vec::new();
    push_opt_num(
        &mut fields,
        "line_width_mm",
        track.rules.overrides.line_width_mm,
    );
    push_opt_num(
        &mut fields,
        "max_total_length_mm",
        track.rules.overrides.max_total_length_mm,
    );
    push_opt_num(
        &mut fields,
        "min_arc_radius_mm",
        track.rules.overrides.min_arc_radius_mm,
    );
    push_opt_num(
        &mut fields,
        "min_distance_between_curvature_changes_mm",
        track
            .rules
            .overrides
            .min_distance_between_curvature_changes_mm,
    );
    push_opt_num(
        &mut fields,
        "intersection_angle_deg",
        track.rules.overrides.intersection_angle_deg,
    );
    push_opt_num(
        &mut fields,
        "intersection_angle_tolerance_deg",
        track.rules.overrides.intersection_angle_tolerance_deg,
    );
    push_opt_num(
        &mut fields,
        "min_straight_around_intersection_mm",
        track.rules.overrides.min_straight_around_intersection_mm,
    );
    push_opt_bool(
        &mut fields,
        "start_finish_must_be_on_straight",
        track.rules.overrides.start_finish_must_be_on_straight,
    );
    push_opt_num(
        &mut fields,
        "min_straight_around_start_finish_mm",
        track.rules.overrides.min_straight_around_start_finish_mm,
    );
    push_opt_num(
        &mut fields,
        "start_goal_distance_mm",
        track.rules.overrides.start_goal_distance_mm,
    );
    push_opt_num(
        &mut fields,
        "start_goal_area_half_width_mm",
        track.rules.overrides.start_goal_area_half_width_mm,
    );
    push_opt_num(
        &mut fields,
        "min_table_edge_clearance_mm",
        track.rules.overrides.min_table_edge_clearance_mm,
    );
    push_opt_num(
        &mut fields,
        "max_slope_deg",
        track.rules.overrides.max_slope_deg,
    );
    if fields.is_empty() {
        out.push_str("}\n");
    } else {
        out.push('\n');
        for (i, field) in fields.iter().enumerate() {
            let suffix = if i + 1 == fields.len() { "" } else { "," };
            out.push_str(&format!("      {}{}\n", field, suffix));
        }
        out.push_str("    }\n");
    }
    out.push_str("  },\n");
    out.push_str("  \"surface\": {\n");
    out.push_str(&format!(
        "    \"base_color\": \"{}\",\n",
        escape_json(&track.surface.base_color)
    ));
    out.push_str(&format!(
        "    \"line_color\": \"{}\",\n",
        escape_json(&track.surface.line_color)
    ));
    out.push_str(&format!(
        "    \"base_reflectance\": {},\n",
        track.surface.base_reflectance
    ));
    out.push_str(&format!(
        "    \"line_reflectance\": {},\n",
        track.surface.line_reflectance
    ));
    out.push_str(&format!(
        "    \"surface_mu\": {}\n",
        track.surface.surface_mu
    ));
    out.push_str("  },\n");
    out.push_str("  \"segments\": [\n");
    for (i, segment) in track.segments.iter().enumerate() {
        let suffix = if i + 1 == track.segments.len() {
            ""
        } else {
            ","
        };
        match segment {
            TrackSegment::Straight(straight) => {
                out.push_str(&format!(
                    "    {{ \"id\": \"{}\", \"kind\": \"straight\", \"length_mm\": {} }}{}\n",
                    escape_json(&straight.id),
                    straight.length_mm,
                    suffix
                ));
            }
            TrackSegment::Arc(arc) => {
                out.push_str(&format!(
                    "    {{ \"id\": \"{}\", \"kind\": \"arc\", \"radius_mm\": {}, \"sweep_deg\": {} }}{}\n",
                    escape_json(&arc.id), arc.radius_mm, arc.sweep_deg, suffix
                ));
            }
        }
    }
    out.push_str("  ],\n");
    out.push_str("  \"closure\": {\n");
    out.push_str(&format!("    \"required\": {},\n", track.closure.required));
    out.push_str(&format!(
        "    \"position_tolerance_mm\": {},\n",
        track.closure.position_tolerance_mm
    ));
    out.push_str(&format!(
        "    \"heading_tolerance_deg\": {}\n",
        track.closure.heading_tolerance_deg
    ));
    out.push_str("  },\n");
    out.push_str("  \"markings\": {\n");
    out.push_str("    \"start_finish\": {\n");
    out.push_str(&format!(
        "      \"enabled\": {},\n",
        track.markings.start_finish.enabled
    ));
    out.push_str(&format!(
        "      \"segment_id\": \"{}\",\n",
        escape_json(&track.markings.start_finish.segment_id)
    ));
    out.push_str(&format!(
        "      \"start_s_mm\": {},\n",
        track.markings.start_finish.start_s_mm
    ));
    out.push_str(&format!(
        "      \"distance_mm\": {},\n",
        track.markings.start_finish.distance_mm
    ));
    out.push_str(&format!(
        "      \"margin_mm\": {},\n",
        track.markings.start_finish.margin_mm
    ));
    out.push_str(&format!(
        "      \"exit_direction\": \"{}\",\n",
        track.markings.start_finish.exit_direction.as_str()
    ));
    out.push_str("      \"robot_start\": {\n");
    out.push_str(&format!(
        "        \"delta_x_mm\": {},\n",
        track.markings.start_finish.robot_start.delta_x_mm
    ));
    out.push_str(&format!(
        "        \"delta_y_mm\": {},\n",
        track.markings.start_finish.robot_start.delta_y_mm
    ));
    out.push_str(&format!(
        "        \"heading_deg\": {}\n",
        track.markings.start_finish.robot_start.heading_deg
    ));
    out.push_str("      }\n");
    out.push_str("    },\n");
    out.push_str("    \"corner_markers\": {\n");
    out.push_str(&format!(
        "      \"auto_generate\": {}\n",
        track.markings.corner_markers.auto_generate
    ));
    out.push_str("    }\n");
    out.push_str("  }\n");
    out.push_str("}\n");
    out
}

pub fn surface_profile_json(profile: &SurfaceProfile) -> String {
    let mut out = String::new();
    out.push_str("{\n");
    out.push_str(&format!(
        "  \"surface_profile_schema\": \"{}\",\n",
        escape_json(&profile.schema)
    ));
    out.push_str(&format!(
        "  \"name\": \"{}\",\n",
        escape_json(&profile.name)
    ));
    if let Some(line_width_mm) = profile.line_width_mm {
        out.push_str(&format!("  \"line_width_mm\": {},\n", line_width_mm));
    }
    out.push_str(&format!(
        "  \"background_reflectance\": {},\n",
        profile.background_reflectance
    ));
    out.push_str(&format!(
        "  \"line_reflectance\": {},\n",
        profile.line_reflectance
    ));
    out.push_str(&format!(
        "  \"marker_profile\": \"{}\",\n",
        escape_json(&profile.marker_profile)
    ));
    out.push_str("  \"rules\": {\n");
    out.push_str(&format!(
        "    \"source\": \"{}\", \"edition\": \"{}\",\n",
        escape_json(&profile.rule_source),
        escape_json(&profile.rule_edition)
    ));
    out.push_str(&format!(
        "    \"mode\": \"{}\",\n",
        profile.rules_mode.as_str()
    ));
    out.push_str("    \"overrides\": {");
    let mut fields = Vec::new();
    push_opt_num(
        &mut fields,
        "line_width_mm",
        profile.overrides.line_width_mm,
    );
    push_opt_num(
        &mut fields,
        "max_total_length_mm",
        profile.overrides.max_total_length_mm,
    );
    push_opt_num(
        &mut fields,
        "min_arc_radius_mm",
        profile.overrides.min_arc_radius_mm,
    );
    push_opt_num(
        &mut fields,
        "min_distance_between_curvature_changes_mm",
        profile.overrides.min_distance_between_curvature_changes_mm,
    );
    push_opt_num(
        &mut fields,
        "intersection_angle_deg",
        profile.overrides.intersection_angle_deg,
    );
    push_opt_num(
        &mut fields,
        "intersection_angle_tolerance_deg",
        profile.overrides.intersection_angle_tolerance_deg,
    );
    push_opt_num(
        &mut fields,
        "min_straight_around_intersection_mm",
        profile.overrides.min_straight_around_intersection_mm,
    );
    push_opt_bool(
        &mut fields,
        "start_finish_must_be_on_straight",
        profile.overrides.start_finish_must_be_on_straight,
    );
    push_opt_num(
        &mut fields,
        "min_straight_around_start_finish_mm",
        profile.overrides.min_straight_around_start_finish_mm,
    );
    push_opt_num(
        &mut fields,
        "start_goal_distance_mm",
        profile.overrides.start_goal_distance_mm,
    );
    push_opt_num(
        &mut fields,
        "start_goal_area_half_width_mm",
        profile.overrides.start_goal_area_half_width_mm,
    );
    push_opt_num(
        &mut fields,
        "min_table_edge_clearance_mm",
        profile.overrides.min_table_edge_clearance_mm,
    );
    push_opt_num(
        &mut fields,
        "max_slope_deg",
        profile.overrides.max_slope_deg,
    );
    if fields.is_empty() {
        out.push_str("}\n");
    } else {
        out.push('\n');
        for (i, field) in fields.iter().enumerate() {
            let suffix = if i + 1 == fields.len() { "" } else { "," };
            out.push_str(&format!("      {}{}\n", field, suffix));
        }
        out.push_str("    }\n");
    }
    out.push_str("  },\n");
    out.push_str("  \"surface\": {\n");
    out.push_str(&format!(
        "    \"base_color\": \"{}\",\n",
        escape_json(&profile.base_color)
    ));
    out.push_str(&format!(
        "    \"line_color\": \"{}\",\n",
        escape_json(&profile.line_color)
    ));
    out.push_str(&format!(
        "    \"base_reflectance\": {},\n",
        profile.background_reflectance
    ));
    out.push_str(&format!(
        "    \"line_reflectance\": {},\n",
        profile.line_reflectance
    ));
    out.push_str(&format!("    \"surface_mu\": {}\n", profile.surface_mu));
    out.push_str("  }\n");
    out.push_str("}\n");
    out
}

pub fn push_opt_num(fields: &mut Vec<String>, key: &str, value: Option<f64>) {
    if let Some(value) = value {
        fields.push(format!("\"{}\": {}", key, value));
    }
}

pub fn push_opt_bool(fields: &mut Vec<String>, key: &str, value: Option<bool>) {
    if let Some(value) = value {
        fields.push(format!("\"{}\": {}", key, value));
    }
}

pub fn curve_json(curve: &[(f64, f64)]) -> String {
    let mut out = String::from("[");
    for (idx, (x, y)) in curve.iter().enumerate() {
        if idx > 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("[{}, {}]", x, y));
    }
    out.push(']');
    out
}

pub fn escape_json(input: &str) -> String {
    let mut s = String::new();
    for ch in input.chars() {
        match ch {
            '"' => s.push_str("\\\""),
            '\\' => s.push_str("\\\\"),
            c if c < ' ' => s.push_str(&format!("\\u{:04x}", c as u32)),
            c => s.push(c),
        }
    }
    s
}

fn write_json(path: impl AsRef<Path>, text: impl AsRef<str>) -> std::io::Result<()> {
    let value = crate::json::parse_json(text.as_ref())
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
    crate::io::validation::validate_document(&value)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e))?;
    fs::write(path, text.as_ref())
}

fn optional_path(path: &Option<PathBuf>) -> String {
    path.as_ref()
        .map(|p| {
            format!(
                "\"{}\"",
                escape_json(&p.to_string_lossy().replace('\\', "/"))
            )
        })
        .unwrap_or("null".into())
}
