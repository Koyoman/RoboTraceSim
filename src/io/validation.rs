use super::models::ResolvedModels;
use crate::config::*;
use crate::json::JsonValue;

/// Validate before lossy numeric conversions/default application in the catalog readers.
pub fn validate_document(root: &JsonValue) -> Result<(), String> {
    fn visit(v: &JsonValue, path: &str) -> Result<(), String> {
        match v {
            JsonValue::Object(obj) => {
                for (key, value) in obj {
                    let field = format!("{path}.{key}");
                    if matches!(key.as_str(), "id" | "asset_path" | "source" | "edition")
                        && value.as_str().is_none()
                    {
                        return Err(format!("{field}: expected string"));
                    }
                    if matches!(
                        key.as_str(),
                        "enabled"
                            | "visible_in_preview"
                            | "invert_left"
                            | "invert_right"
                            | "start_finish_must_be_on_straight"
                    ) && !matches!(value, JsonValue::Bool(_) | JsonValue::Null)
                    {
                        return Err(format!("{field}: expected boolean"));
                    }
                    if key == "exit_direction" {
                        let name = value
                            .as_str()
                            .ok_or_else(|| format!("{field}: expected direction"))?;
                        if ![
                            "to_increasing_s",
                            "increasing",
                            "increasing_s",
                            "right",
                            "direita",
                            "to_decreasing_s",
                            "decreasing",
                            "decreasing_s",
                            "left",
                            "esquerda",
                        ]
                        .contains(&name)
                        {
                            return Err(format!("{field}: unknown direction {name}"));
                        }
                    }
                    if key.ends_with("_schema") {
                        let schema = value
                            .as_str()
                            .ok_or_else(|| format!("{field}: expected schema string"))?;
                        let valid = if key == "robot_schema" {
                            (1..=8).any(|n| schema == format!("rtsim-robot-v{n}"))
                        } else if key == "track_schema" {
                            matches!(schema, "rtsim-track-v1" | "rtsim-track-v2")
                        } else {
                            let expected = match key.as_str() {
                                "rtsim_schema" => "rtsim-project-v1".into(),
                                "surface_profile_schema" => "rtsim-surface-profile-v1".into(),
                                other => format!(
                                    "rtsim-{}-v1",
                                    other.trim_end_matches("_schema").replace('_', "-")
                                ),
                            };
                            schema == expected
                        };
                        if !valid {
                            return Err(format!("{field}: unsupported schema {schema}"));
                        }
                    }
                    let integer = matches!(
                        key.as_str(),
                        "seed"
                            | "count"
                            | "adc_bits"
                            | "ticks_per_rev"
                            | "cells"
                            | "pwm_resolution_bits"
                    ) || key.ends_with("_us");
                    let numeric = integer
                        || [
                            "_mm", "_m2", "_kg_m2", "_g_cm2", "_g", "_nm", "_mnm", "_rpm", "_v",
                            "_a", "_w", "_n", "_pa", "_ohm", "_mah", "_hz", "_s", "_deg",
                        ]
                        .iter()
                        .any(|suffix| key.ends_with(suffix))
                        || matches!(
                            key.as_str(),
                            "gain"
                                | "offset"
                                | "kp"
                                | "ki"
                                | "kd"
                                | "efficiency"
                                | "gear_ratio"
                                | "initial_soc"
                                | "mu_longitudinal"
                                | "mu_lateral"
                                | "surface_mu"
                                | "rolling_resistance"
                                | "leakage_factor"
                                | "speed_sensitivity"
                                | "reflectance_noise_std"
                                | "adc_noise_lsb"
                                | "threshold"
                                | "command_deadband"
                                | "pwm"
                                | "base_pwm"
                                | "max_pwm"
                                | "min_pwm"
                                | "default_pwm"
                                | "downforce_pwm"
                                | "pwm_scale"
                                | "base_reflectance"
                                | "background_reflectance"
                                | "line_reflectance"
                        );
                    if numeric
                        && value == &JsonValue::Null
                        && !path.ends_with("overrides")
                        && key != "line_width_mm"
                    {
                        return Err(format!("{field}: number cannot be null"));
                    }
                    if numeric
                        && matches!(value, JsonValue::Array(_))
                        && !matches!(
                            key.as_str(),
                            "position_mm"
                                | "center_mm"
                                | "size_mm"
                                | "center_of_mass_mm"
                                | "points_mm"
                                | "centerline_mm"
                                | "polyline_mm"
                        )
                    {
                        return Err(format!("{field}: expected scalar"));
                    }
                    if key == "response_model" {
                        if let Some(name) = value.as_str() {
                            if !name.eq_ignore_ascii_case("ideal") {
                                return Err(format!(
                                    "{field}: use an explicit supported response object"
                                ));
                            }
                        }
                    }
                    if key == "sensor_type" {
                        let name = value
                            .as_str()
                            .ok_or_else(|| format!("{field}: expected sensor type"))?;
                        if matches!(SensorType::from_str(name), SensorType::Custom)
                            && !name.eq_ignore_ascii_case("custom")
                        {
                            return Err(format!("{field}: unknown sensor type {name}"));
                        }
                    }
                    if key == "mode" {
                        if let Some(mode) = value.as_str() {
                            if !["brake", "coast", "free", "hi-z", "hiz", "warning", "strict"]
                                .contains(&mode.to_ascii_lowercase().as_str())
                            {
                                return Err(format!("{field}: unknown mode {mode}"));
                            }
                        }
                    }
                    // Coordinate arrays and point tables are checked separately.
                    if numeric && !matches!(value, JsonValue::Array(_) | JsonValue::Null) {
                        let n = value
                            .as_f64()
                            .ok_or_else(|| format!("{field}: expected number"))?;
                        if !n.is_finite() {
                            return Err(format!("{field}: non-finite number"));
                        }
                        if integer
                            && (n < 0.0
                                || n.fract() != 0.0
                                || n > crate::core::clock::MAX_EXACT_US as f64)
                        {
                            return Err(format!("{field}: expected safe nonnegative integer"));
                        }
                        if key == "count" && !(1.0..=4096.0).contains(&n) {
                            return Err(format!("{field}: count must be 1..4096"));
                        }
                        if matches!(key.as_str(), "adc_bits" | "pwm_resolution_bits")
                            && !(1.0..=24.0).contains(&n)
                        {
                            return Err(format!("{field}: resolution must be 1..24 bits"));
                        }
                        if matches!(key.as_str(), "cells" | "ticks_per_rev")
                            && !(1.0..=1e9).contains(&n)
                        {
                            return Err(format!("{field}: integer outside supported range"));
                        }
                        if key.starts_with("mu_")
                            || key.starts_with("max_")
                            || key.starts_with("nominal_")
                            || key.starts_with("stall_")
                            || key.starts_with("visual_")
                            || matches!(
                                key.as_str(),
                                "rolling_resistance"
                                    | "reflectance_noise_std"
                                    | "adc_noise_lsb"
                                    | "response_time_s"
                                    | "internal_resistance_ohm"
                                    | "surface_mu"
                            )
                        {
                            if n < 0.0 {
                                return Err(format!("{field}: must be nonnegative"));
                            }
                        }
                        if matches!(
                            key.as_str(),
                            "mass_g"
                                | "inertia_kg_m2"
                                | "wheel_radius_mm"
                                | "wheel_inertia_g_cm2"
                                | "gear_ratio"
                                | "capacity_mah"
                                | "no_load_rpm"
                                | "nominal_voltage_v"
                                | "stall_current_a"
                                | "stall_torque_mnm"
                                | "length_mm"
                                | "width_mm"
                                | "radius_mm"
                        ) && n <= 0.0
                            && !(path.contains(".assembly.masses[")
                                && matches!(key.as_str(), "mass_g" | "inertia_kg_m2")
                                && n == 0.0)
                        {
                            return Err(format!("{field}: must be positive"));
                        }
                        if matches!(
                            key.as_str(),
                            "efficiency"
                                | "initial_soc"
                                | "base_reflectance"
                                | "background_reflectance"
                                | "line_reflectance"
                                | "leakage_factor"
                                | "command_deadband"
                                | "max_pwm"
                                | "min_pwm"
                                | "default_pwm"
                                | "downforce_pwm"
                        ) && !(0.0..=1.0).contains(&n)
                        {
                            return Err(format!("{field}: must be in [0,1]"));
                        }
                    }
                    if integer && value == &JsonValue::Null {
                        return Err(format!("{field}: integer cannot be null"));
                    }
                    if matches!(
                        key.as_str(),
                        "position_mm" | "center_of_mass_mm" | "start_pose_m"
                    ) {
                        let n = if key == "start_pose_m" { 3 } else { 2 };
                        if value
                            .as_array()
                            .is_none_or(|a| a.len() != n || a.iter().any(|v| v.as_f64().is_none()))
                        {
                            return Err(format!("{field}: expected {n} numeric coordinates"));
                        }
                    }
                    if key == "model" {
                        let name = value
                            .as_str()
                            .ok_or_else(|| format!("{field}: expected model string"))?;
                        let known = [
                            "DcMotorSimple",
                            "dc_electrical",
                            "dc_simple",
                            "PwmHBridge",
                            "VoltageSagBattery",
                            "QuantizedEncoder",
                            "NoisyGyro",
                            "BuiltInPid",
                            "RigidBody2DChassis",
                            "VectorTrack",
                            "ParametricTrack",
                            "SlipRatioWheel",
                            "CoulombFrictionWheel",
                            "GenericAnalogLineSensor",
                            "GenericDigitalLineSensor",
                            "QTRAnalogSingle",
                            "GenericToF",
                            "Custom",
                            "NoisyAdcSensor",
                        ];
                        if !known.contains(&name)
                            && super::models::NormalForceKind::parse(name).is_err()
                        {
                            return Err(format!("{field}: unknown model {name}"));
                        }
                    }
                    if key == "curve_model" || key == "curve" {
                        if let Some(name) = value.as_str() {
                            if ![
                                "linear",
                                "exponential",
                                "polynomial",
                                "lookuptable",
                                "lookup_table",
                                "table",
                                "custom",
                            ]
                            .contains(&name.to_ascii_lowercase().as_str())
                            {
                                return Err(format!("{field}: unknown curve {name}"));
                            }
                        }
                    }
                    if matches!(
                        key.as_str(),
                        "force_curve"
                            | "measured_curve"
                            | "thrust_curve"
                            | "points"
                            | "points_mm"
                            | "centerline_mm"
                            | "coefficients"
                    ) {
                        let items = value
                            .as_array()
                            .ok_or_else(|| format!("{field}: expected array"))?;
                        let mut previous = None;
                        for entry in items {
                            if key == "coefficients" {
                                if entry.as_f64().is_none() {
                                    return Err(format!("{field}: nonnumeric coefficient"));
                                }
                                continue;
                            }
                            let pair = entry.as_array();
                            if let Some(pair) = pair {
                                if pair.len() != 2 || pair.iter().any(|n| n.as_f64().is_none()) {
                                    return Err(format!("{field}: expected numeric pairs"));
                                }
                                if !matches!(key.as_str(), "points_mm" | "centerline_mm") {
                                    let x = pair[0].as_f64().unwrap();
                                    if previous.is_some_and(|p| p >= x) {
                                        return Err(format!(
                                            "{field}: inputs must increase strictly"
                                        ));
                                    }
                                    previous = Some(x);
                                }
                            }
                        }
                    }
                    if key == "downforce_model" {
                        // Explicit migration of the formerly unused duplicate selector.
                        let kind = value.get("kind").and_then(JsonValue::as_str).unwrap_or("");
                        let model = obj
                            .get("model")
                            .and_then(JsonValue::as_str)
                            .unwrap_or("NoDownforce");
                        let compatible = matches!(
                            (model, kind),
                            ("FanDownforce", "Fan")
                                | ("NoDownforce", "None")
                                | ("ConstantDownforce", "Constant")
                                | ("MeasuredDownforceCurve", "LookupTable")
                        );
                        if !compatible {
                            return Err(format!("{field}: conflicting or unsupported duplicate selector; use normal_force.model and its physical parameters"));
                        }
                    }
                    visit(value, &field)?;
                }
            }
            JsonValue::Array(arr) => {
                for (i, item) in arr.iter().enumerate() {
                    visit(item, &format!("{path}[{i}]"))?;
                }
            }
            JsonValue::Number(n) if !n.is_finite() => {
                return Err(format!("{path}: non-finite number"))
            }
            _ => {}
        }
        Ok(())
    }
    visit(root, "document")
}

pub fn validate_robot(robot: &RobotConfig) -> Result<(), String> {
    if let Some(p) = &robot.powertrain {
        p.validate()?;
    }
    if let Some(f) = &robot.physics {
        f.validate()?;
    }
    crate::models::robot::RobotAssembly::effective(robot).validate(robot)?;
    ResolvedModels::from_robot(robot)?;
    validate_robot_physics(robot)?;
    // Same catalog validation also covers programmatic/GUI edits before persistence/run.
    let json = crate::json::parse_json(&super::persistence::robot_json(robot))
        .map_err(|e| e.to_string())?;
    validate_document(&json)?;
    let mut ids = std::collections::BTreeSet::new();
    for id in robot
        .sensors
        .iter()
        .map(|s| &s.id)
        .chain(robot.normal_force.fans.iter().map(|f| &f.id))
    {
        if id.trim().is_empty() || !ids.insert(id) {
            return Err(format!("empty or duplicate component id: {id}"));
        }
    }
    for sensor in &robot.sensors {
        use crate::config::SensorDetectionArea as A;
        let positive = |v: f64| v.is_finite() && v > 0.;
        let valid = match &sensor.asset.detection_area {
            A::Point { radius_m } => radius_m.is_finite() && *radius_m >= 0.,
            A::Circle { radius_m } => positive(*radius_m),
            A::Rectangle { width_m, height_m } => positive(*width_m) && positive(*height_m),
            A::Cone { range_m, angle_deg } => {
                positive(*range_m) && positive(*angle_deg) && *angle_deg <= 180.
            }
            A::CustomPolygon { points_m } => {
                points_m.len() >= 3
                    && points_m.iter().all(|p| p.x.is_finite() && p.y.is_finite())
                    && points_m
                        .iter()
                        .zip(points_m.iter().cycle().skip(1))
                        .take(points_m.len())
                        .map(|(a, b)| a.x * b.y - a.y * b.x)
                        .sum::<f64>()
                        .abs()
                        > 1e-18
            }
        };
        if !valid {
            return Err(format!("{}: invalid optical footprint", sensor.id));
        }
        if let SensorResponseModel::LookupTable { points } = &sensor.asset.response_model {
            if points.len() < 2 || points.windows(2).any(|p| p[0].input >= p[1].input) {
                return Err(format!("{}: unordered response table", sensor.id));
            }
        }
    }
    Ok(())
}

pub fn validate_robot_physics(r: &RobotConfig) -> Result<(), String> {
    for (name, value) in [
        ("mass_kg", r.chassis.mass_kg),
        ("inertia_kg_m2", r.chassis.inertia_kg_m2),
        ("wheel_radius_m", r.drivetrain.wheel_radius_m),
        ("wheel_inertia_kg_m2", r.drivetrain.wheel_inertia_kg_m2),
        ("track_width_m", r.drivetrain.track_width_m),
        ("wheelbase_m", r.drivetrain.wheelbase_m),
        ("capacity_mah", r.battery.capacity_mah),
        (
            "slip_velocity_epsilon_m_s",
            r.tire.slip_velocity_epsilon_m_s,
        ),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(format!(
                "physics config: {name} must be finite and positive"
            ));
        }
    }
    for (name, value) in [
        ("mu_longitudinal", r.tire.mu_longitudinal),
        ("mu_lateral", r.tire.mu_lateral),
        ("rolling_resistance", r.tire.rolling_resistance),
        ("battery.current_limit_a", r.battery.current_limit_a),
        ("driver.current_limit_a", r.driver.current_limit_a),
        ("voltage_drop_v", r.driver.voltage_drop_v),
        ("internal_resistance_ohm", r.battery.internal_resistance_ohm),
        ("empty_voltage_v", r.battery.empty_voltage_v),
        ("full_voltage_v", r.battery.full_voltage_v),
    ] {
        if !value.is_finite() || value < 0.0 {
            return Err(format!(
                "physics config: {name} must be finite and non-negative"
            ));
        }
    }
    if !r.battery.initial_soc.is_finite()
        || !(0.0..=1.0).contains(&r.battery.initial_soc)
        || r.battery.full_voltage_v < r.battery.empty_voltage_v
    {
        return Err("physics config: invalid battery SOC/voltage range".into());
    }
    if !r.chassis.center_of_mass_m.x.is_finite() || !r.chassis.center_of_mass_m.y.is_finite() {
        return Err("physics config: invalid center of mass".into());
    }
    if !r.driver.command_deadband.is_finite() || !(0.0..=1.0).contains(&r.driver.command_deadband) {
        return Err("driver: command_deadband must be in [0, 1]".into());
    }
    let normal = &r.normal_force;
    for (name, value) in [
        ("max_force_n", normal.max_force_n),
        ("max_current_a", normal.max_current_a),
        ("response_time_s", normal.response_time_s),
        ("chamber_area_m2", normal.chamber_area_m2),
        ("max_delta_pressure_pa", normal.max_delta_pressure_pa),
        ("speed_sensitivity", normal.speed_sensitivity),
    ] {
        if !value.is_finite() || value < 0.0 {
            return Err(format!("normal_force: invalid {name}"));
        }
    }
    if !normal.leakage_factor.is_finite()
        || !(0.0..=1.0).contains(&normal.leakage_factor)
        || !normal.position_m.x.is_finite()
        || !normal.position_m.y.is_finite()
    {
        return Err("normal_force: invalid position or leakage".into());
    }
    for curve in
        std::iter::once(&normal.force_curve).chain(normal.fans.iter().map(|f| &f.force_curve))
    {
        if curve
            .iter()
            .any(|(x, y)| !x.is_finite() || !y.is_finite() || !(0.0..=1.0).contains(x) || *y < 0.0)
            || curve
                .windows(2)
                .any(|w| w[0].0 >= w[1].0 || w[0].1 > w[1].1)
        {
            return Err(
                "normal_force: curve must have increasing PWM and nondecreasing finite force"
                    .into(),
            );
        }
    }
    for fan in &normal.fans {
        if ![fan.position_m.x, fan.position_m.y]
            .iter()
            .all(|x| x.is_finite())
            || ![
                fan.max_force_n,
                fan.max_current_a,
                fan.response_time_s,
                fan.pwm_scale,
                fan.enabled_pwm,
            ]
            .iter()
            .all(|x| x.is_finite() && *x >= 0.0)
            || !fan.nominal_voltage_v.is_finite()
            || fan.nominal_voltage_v <= 0.0
        {
            return Err("normal_force: invalid fan parameters".into());
        }
    }
    for motor in [&r.motor_left, &r.motor_right] {
        for (name, value) in [
            ("nominal_voltage_v", motor.nominal_voltage_v),
            ("gear_ratio", motor.gear_ratio),
            ("efficiency", motor.efficiency),
            ("no_load_rpm", motor.no_load_rpm),
            ("stall_torque_nm", motor.stall_torque_nm),
            ("stall_current_a", motor.stall_current_a),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("motor config: {name} must be finite and positive"));
            }
        }
        let ke = motor.nominal_voltage_v / (motor.no_load_rpm * std::f64::consts::TAU / 60.0);
        let kt = motor.stall_torque_nm / motor.stall_current_a;
        if motor.efficiency > 1.0 || kt > ke * (1.0 + 1e-12) {
            return Err("motor config: efficiency must be <= 1 and stall torque/current must not exceed back-EMF constant (passivity)".into());
        }
    }
    Ok(())
}
