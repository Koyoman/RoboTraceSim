use robotrace_sim::{
    experiments::{calibration::*, metrics::*},
    json::JsonValue as J,
};
use std::{collections::BTreeMap, path::PathBuf};
fn dir() -> PathBuf {
    let p = PathBuf::from("target").join(format!("stage10-{}", robotrace_sim::sim::new_run_id()));
    std::fs::create_dir(&p).unwrap();
    p
}
fn series(rows: &[(i64, f64)]) -> Series {
    Series {
        rows: rows
            .iter()
            .map(|(t, v)| Row {
                t_us: *t,
                values: [("vx_body_m_s".into(), *v)].into_iter().collect(),
            })
            .collect(),
    }
}
#[test]
fn strict_import_preserves_missing_and_rejects_ambiguous_data() {
    let p = dir().join("log.csv");
    std::fs::write(&p, "t_us,x_m,y_m\n1000,1,\n3000,,2\n").unwrap();
    let s = Series::read(&p).unwrap();
    assert!(!s.rows[0].values.contains_key("y_m"));
    assert_eq!(s.rows[0].t_us, 1000);
    for bad in [
        "t_us,x_m\n0,1\n0,2\n",
        "t_us,x_m\n2,1\n1,2\n",
        "t_us,x_m\n0,NaN\n",
        "t_us,x_m,x_m\n0,1,2\n",
    ] {
        std::fs::write(&p, bad).unwrap();
        assert!(Series::read(&p).is_err());
    }
}
#[test]
fn interpolation_respects_gaps_yaw_and_discrete_observations() {
    let mut s = series(&[(0, 0.), (100, 1.), (1000, 2.)]);
    for (r, (yaw, adc)) in s.rows.iter_mut().zip([(3.1, 10.), (-3.1, 20.), (0., 30.)]) {
        r.values.insert("yaw_rad".into(), yaw);
        r.values.insert("sensor_00_adc".into(), adc);
    }
    assert_eq!(s.at(50, "vx_body_m_s", 100), Some(0.5));
    assert_eq!(s.at(500, "vx_body_m_s", 100), None);
    assert_eq!(s.at(50, "sensor_00_adc", 100), Some(10.));
    assert!((s.at(50, "yaw_rad", 100).unwrap().abs() - std::f64::consts::PI).abs() < 1e-12);
    s.rows[0]
        .values
        .insert("sensor_00_available_us".into(), 10.);
    assert_eq!(s.at(0, "sensor_00_adc", 100), None);
}
#[test]
fn frame_and_time_alignment_preserve_acquisition_delivery_delay() {
    let mut s = series(&[(1000, 0.)]);
    s.rows[0].values.extend([
        ("x_m".into(), 1.),
        ("y_m".into(), 0.),
        ("yaw_rad".into(), 0.),
        ("sensor_00_acquired_us".into(), 900.),
        ("sensor_00_available_us".into(), 1000.),
    ]);
    s.align(1000, 50, [2., 3.], std::f64::consts::FRAC_PI_2)
        .unwrap();
    let r = &s.rows[0];
    assert_eq!(r.t_us, 50);
    assert!((r.values["x_m"] - 2.).abs() < 1e-12);
    assert_eq!(r.values["y_m"], 4.);
    assert_eq!(
        r.values["sensor_00_available_us"] - r.values["sensor_00_acquired_us"],
        100.
    );
}
#[test]
fn missing_observations_are_null_with_explicit_coverage() {
    let a = series(&[(0, 1.), (100, 2.)]);
    let mut b = a.clone();
    b.rows[1].values.clear();
    let m = compare(&a, &b, "vx_body_m_s", 100);
    assert_eq!(m.coverage, 0.5);
    assert_eq!(m.rms, Some(0.));
    let missing = compare(&a, &b, "battery_current_a", 100);
    assert_eq!(missing.matched, 0);
    assert_eq!(missing.json().get("rms"), Some(&J::Null));
}
#[test]
fn offset_estimation_requires_excitation_and_finds_known_shift() {
    let a = series(&[(0, 0.), (100, 1.), (200, 0.2), (300, 3.)]);
    let mut b = a.clone();
    b.align(0, 50, [0.; 2], 0.).unwrap();
    assert_eq!(
        estimate_offset(&a, &b, "vx_body_m_s", &[-100, -50, 0, 50], 100, 1.).unwrap(),
        -50
    );
    assert!(estimate_offset(
        &a,
        &series(&[(0, 1.), (100, 1.), (200, 1.)]),
        "vx_body_m_s",
        &[0],
        100,
        1.
    )
    .is_err());
}
#[test]
fn energy_line_loss_and_braking_have_coverage() {
    let mut s = series(&[(0, 1.), (1_000_000, 0.5), (2_000_000, 0.)]);
    for (i, r) in s.rows.iter_mut().enumerate() {
        r.values.extend([
            ("battery_voltage_v".into(), 2.),
            ("battery_current_a".into(), 3.),
            ("line_visible".into(), 0.),
            ("saturated".into(), 1.),
            ("brake_active".into(), 1.),
            ("x_m".into(), i as f64 * 0.25),
            ("y_m".into(), 0.),
        ]);
    }
    let d = derived(&s, 1_000_000);
    assert_eq!(d.get("energy_j").and_then(J::as_f64), Some(12.));
    assert_eq!(d.get("line_loss_s").and_then(J::as_f64), Some(2.));
    assert_eq!(d.get("braking_distance_m").and_then(J::as_f64), Some(0.5));
    assert_eq!(derived(&s, 100).get("energy_j"), Some(&J::Null));
}
fn fixture() -> (PathBuf, J) {
    let root = dir();
    let project = std::path::Path::new("examples/physics/realistic.rtsim")
        .canonicalize()
        .unwrap()
        .to_string_lossy()
        .replace('\\', "/");
    let protocol = object(
        [
            "robot_components",
            "instrument",
            "instrument_accuracy",
            "mounting",
            "surface",
            "lighting",
            "temperature",
            "battery_state",
            "units",
            "acquisition_rate",
            "date",
            "operator",
        ]
        .map(|k| (k, J::String("synthetic fixture".into()))),
    );
    let mut datasets = Vec::new();
    for (name, split, values) in [
        ("train", "calibration", "0,0\n1000,0.01\n"),
        ("test", "validation", "0,0\n1000,0.02\n"),
    ] {
        std::fs::write(
            root.join(format!("{name}.csv")),
            format!("t_us,vx_body_m_s\n{values}"),
        )
        .unwrap();
        datasets.push(object([
            ("id", J::String(name.into())),
            ("split", J::String(split.into())),
            ("kind", J::String("synthetic".into())),
            ("condition_group", J::String(name.into())),
            ("project", J::String(project.clone())),
            ("csv", J::String(format!("{name}.csv"))),
            ("origin_us", J::Number(0.)),
            ("offset_us", J::Number(0.)),
            ("max_gap_us", J::Number(1000.)),
            ("time_basis", J::String("delivery".into())),
            ("protocol", protocol.clone()),
        ]));
    }
    let study = object([
        ("schema", J::String("rtsim-study-v1".into())),
        ("preset", J::String("realistic".into())),
        ("duration_us", J::Number(1000.)),
        (
            "tolerance_basis",
            J::String("synthetic pipeline threshold fixed a priori".into()),
        ),
        (
            "objectives",
            J::Array(vec![object([
                ("signal", J::String("vx_body_m_s".into())),
                ("scale", J::Number(1.)),
                ("weight", J::Number(1.)),
                ("max_rms", J::Number(100.)),
                ("min_coverage", J::Number(1.)),
                ("unit", J::String("m/s".into())),
            ])]),
        ),
        ("parameters", J::Array(vec![])),
        ("datasets", J::Array(datasets)),
    ]);
    (root, study)
}
fn save(root: &std::path::Path, j: &J) -> PathBuf {
    let p = root.join("study.json");
    std::fs::write(&p, j.to_json().unwrap()).unwrap();
    p
}
#[test]
fn synthetic_qualification_cannot_claim_physical_validation() {
    let (root, j) = fixture();
    let report = qualify(&save(&root, &j), &root.join("report.json")).unwrap();
    assert_eq!(
        report.get("status").and_then(J::as_str),
        Some("synthetic_or_unknown_not_physically_qualified")
    );
    assert_eq!(
        report
            .get("evaluation")
            .and_then(J::as_array)
            .unwrap()
            .len(),
        4
    );
}
#[test]
fn split_refuses_reused_content() {
    let (root, j) = fixture();
    std::fs::copy(root.join("train.csv"), root.join("test.csv")).unwrap();
    let e = Study::read(&save(&root, &j)).err().unwrap();
    assert!(e.contains("leakage"));
}
#[test]
fn split_refuses_shared_conditions() {
    let (root, mut j) = fixture();
    if let J::Object(m) = &mut j {
        if let J::Array(a) = m.get_mut("datasets").unwrap() {
            for d in a {
                if let J::Object(m) = d {
                    m.insert("condition_group".into(), J::String("same".into()));
                }
            }
        }
    }
    assert!(Study::read(&save(&root, &j))
        .err()
        .unwrap()
        .contains("condition_group"));
}
#[test]
fn changing_holdout_does_not_change_fitted_parameters() {
    let (root, mut j) = fixture();
    if let J::Object(m) = &mut j {
        m.insert(
            "parameters".into(),
            J::Array(vec![object([
                ("name", J::String("motor_torque_scale".into())),
                ("min", J::Number(0.9)),
                ("max", J::Number(1.1)),
                ("steps", J::Number(3.)),
            ])]),
        );
    }
    let path = save(&root, &j);
    let a = qualify(&path, &root.join("a.json")).unwrap();
    std::fs::write(
        root.join("test.csv"),
        "t_us,vx_body_m_s\n0,10000\n1000,10000\n",
    )
    .unwrap();
    let b = qualify(&path, &root.join("b.json")).unwrap();
    assert_ne!(a.get("parameters"), Some(&J::Null));
    assert_eq!(a.get("parameters"), b.get("parameters"));
    assert_eq!(a.get("candidates"), b.get("candidates"));
    assert_eq!(
        b.get("status").and_then(J::as_str),
        Some("validation_criteria_failed")
    );
}
#[test]
fn parameter_scales_apply_per_contact_and_do_not_modify_original() {
    let cfg = robotrace_sim::config::load_project(std::path::Path::new(
        "examples/physics/realistic.rtsim",
    ))
    .unwrap();
    let mut changed = cfg.clone();
    apply(
        &mut changed,
        &[("mu_longitudinal_scale".into(), 0.8)]
            .into_iter()
            .collect::<BTreeMap<_, _>>(),
    )
    .unwrap();
    assert_eq!(
        changed.robot.assembly.as_ref().unwrap().wheels[0]
            .tire
            .mu_longitudinal,
        cfg.robot.assembly.as_ref().unwrap().wheels[0]
            .tire
            .mu_longitudinal
            * 0.8
    );
    assert!(apply(&mut changed, &[("bad".into(), 1.)].into_iter().collect()).is_err());
}

#[test]
fn unexcited_parameter_is_not_fitted() {
    let (root, mut j) = fixture();
    std::fs::write(
        root.join("train.csv"),
        "t_us,line_visible,x_m\n0,1,0\n1000,1,0\n",
    )
    .unwrap();
    std::fs::write(
        root.join("test.csv"),
        "t_us,line_visible,x_m\n0,1,1\n1000,1,1\n",
    )
    .unwrap();
    if let J::Object(m) = &mut j {
        m.insert(
            "parameters".into(),
            J::Array(vec![object([
                ("name", J::String("motor_torque_scale".into())),
                ("min", J::Number(0.9)),
                ("max", J::Number(1.1)),
                ("steps", J::Number(3.)),
            ])]),
        );
        if let J::Array(o) = m.get_mut("objectives").unwrap() {
            if let J::Object(o) = &mut o[0] {
                o.insert("signal".into(), J::String("line_visible".into()));
                o.insert("unit".into(), J::String("boolean".into()));
            }
        }
    }
    let report = qualify(&save(&root, &j), &root.join("report.json")).unwrap();
    assert_eq!(
        report.get("status").and_then(J::as_str),
        Some("identifiability_screen_failed")
    );
    assert_eq!(report.get("parameters"), Some(&J::Null));
}
#[test]
fn units_are_not_silently_converted() {
    let (root, mut j) = fixture();
    if let J::Object(m) = &mut j {
        if let J::Array(o) = m.get_mut("objectives").unwrap() {
            if let J::Object(o) = &mut o[0] {
                o.insert("unit".into(), J::String("km/h".into()));
            }
        }
    }
    assert!(Study::read(&save(&root, &j))
        .err()
        .unwrap()
        .contains("unit m/s"));
}
#[test]
fn robustness_keeps_observation_and_controller_periods_fixed() {
    let root = dir();
    let project = std::path::Path::new("examples/physics/simplified.rtsim");
    let report =
        robotrace_sim::experiments::robustness::run(project, &root.join("robustness.json"), 1000)
            .unwrap();
    let attempts = report.get("attempts").and_then(J::as_array).unwrap();
    assert_eq!(report.get("duration_us").and_then(J::as_u64), Some(1000));
    assert_eq!(attempts.len(), 27);
    for a in attempts {
        assert!(a.get("error").is_none());
        let time = a
            .get("snapshot")
            .unwrap()
            .get("project")
            .unwrap()
            .get("time")
            .unwrap();
        assert_eq!(
            time.get("controller_period_us").and_then(J::as_u64),
            Some(1000)
        );
        assert_eq!(time.get("sensor_period_us").and_then(J::as_u64), Some(500));
    }
    assert_eq!(
        report
            .get("refinement")
            .and_then(J::as_array)
            .unwrap()
            .len(),
        18
    );
}

#[test]
fn trajectory_requires_both_coordinates() {
    let mut a = series(&[(0, 0.)]);
    a.rows[0]
        .values
        .extend([("x_m".into(), 3.), ("y_m".into(), 4.)]);
    let mut b = a.clone();
    b.rows[0].values.insert("x_m".into(), 0.);
    b.rows[0].values.insert("y_m".into(), 0.);
    assert_eq!(trajectory(&a, &b, 100).rms, Some(5.));
    b.rows[0].values.remove("y_m");
    assert_eq!(trajectory(&a, &b, 100).rms, None);
}

#[test]
fn frozen_numerical_baseline_detects_model_drift() {
    let baseline =
        robotrace_sim::json::parse_json(include_str!("scenarios/stage10-numerical-v1.json"))
            .unwrap();
    let mut cfg = robotrace_sim::config::load_project(std::path::Path::new(
        baseline.get("project").unwrap().as_str().unwrap(),
    ))
    .unwrap();
    cfg.project.time.physics_dt_us = 50;
    for (i, s) in cfg.robot.sensors.iter_mut().enumerate() {
        s.acquisition.seed = 1371 + i as u64 * 97;
    }
    cfg.robot.gyro.seed = 1671;
    cfg.robot.sensing.encoder.seed = 1771;
    cfg.robot.sensing.imu.seed = 1871;
    let prediction = simulate(cfg, 100000).unwrap();
    let last = prediction.series.rows.last().unwrap();
    let expected = match baseline.get("expected_final").unwrap() {
        J::Object(m) => m,
        _ => panic!(),
    };
    for (k, v) in expected {
        assert!(
            (last.values[k] - v.as_f64().unwrap()).abs()
                <= baseline
                    .get("absolute_tolerance")
                    .unwrap()
                    .as_f64()
                    .unwrap(),
            "model drift in {k}"
        );
    }
}
