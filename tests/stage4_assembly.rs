use robotrace_sim::{
    config::{load_project, load_robot_from_file, LoadedConfig},
    io::{
        experiment::ResolvedExperiment,
        persistence::{robot_json, save_robot_to_file},
    },
    math::{Pose2, Vec2},
    models::robot::*,
    sim::SimulationCore,
};
use std::path::Path;
fn config() -> LoadedConfig {
    let mut c = load_project(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/basic/projeto_suction.rtsim"),
    )
    .unwrap();
    c.robot.assembly = Some(RobotAssembly::from_legacy(&c.robot));
    c
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-11, "{a} != {b}");
}
fn mass(id: &str, m: f64, x: f64, y: f64, z: f64, i: f64) -> MassElement {
    MassElement {
        id: id.into(),
        component: None,
        mass_kg: m,
        position_m: Vec2::new(x, y),
        height_m: z,
        inertia_kg_m2: i,
    }
}
#[test]
fn aggregate_conversion_preserves_geometry_and_equivalent_inertia() {
    let c = config();
    let a = c.robot.assembly.as_ref().unwrap();
    let d = a.solver_geometry().unwrap();
    near(d.wheel_radius_m, c.robot.drivetrain.wheel_radius_m);
    near(d.track_width_m, c.robot.drivetrain.track_width_m);
    near(d.wheelbase_m, c.robot.drivetrain.wheelbase_m);
    near(
        d.wheel_inertia_kg_m2,
        c.robot.drivetrain.wheel_inertia_kg_m2,
    );
    assert_eq!(a.support_polygon().len(), 4);
    assert!(a.validate(&c.robot).is_ok());
}
#[test]
fn parallel_axis_mass_sum_and_exclusive_measured_override() {
    let mut c = config();
    let a = c.robot.assembly.as_mut().unwrap();
    a.mass_mode = MassMode::Components;
    a.masses = vec![
        mass("a", 1., -0.01, 0., 0.02, 0.001),
        mass("b", 3., 0.03, 0.02, 0.04, 0.002),
    ];
    let a = a.clone();
    let m = a.mass_properties(&c.robot).unwrap();
    near(m.mass_kg, 4.);
    near(m.center_m.x, 0.02);
    near(m.center_m.y, 0.015);
    near(m.height_m, 0.035);
    near(m.inertia_kg_m2, 0.0045);
    let mut a = a;
    a.mass_mode = MassMode::Measured;
    a.measured_com_height_m = 0.1;
    c.robot.chassis.mass_kg = 0.2;
    c.robot.chassis.inertia_kg_m2 = 0.01;
    let m = a.mass_properties(&c.robot).unwrap();
    near(m.mass_kg, 0.2);
    near(m.inertia_kg_m2, 0.01);
    near(m.height_m, 0.1);
}
#[test]
fn mass_references_follow_components_and_reject_double_accounting() {
    let mut c = config();
    let id = c.robot.sensors[0].id.clone();
    c.robot.sensors[0].position_m = Vec2::new(0.023, -0.012);
    c.robot.sensors[0].height_m = 0.005;
    let a = c.robot.assembly.as_mut().unwrap();
    a.mass_mode = MassMode::Components;
    let mut m = mass("sensor-mass", 0.1, 10., 10., 10., 1e-5);
    m.component = Some(id);
    a.masses = vec![m.clone()];
    let a = a.clone();
    let result = a.mass_properties(&c.robot).unwrap();
    near(result.center_m.x, 0.023);
    near(result.center_m.y, -0.012);
    near(result.height_m, 0.005);
    let mut a = a;
    m.id = "another".into();
    a.masses.push(m);
    assert!(a
        .validate(&c.robot)
        .unwrap_err()
        .contains("multiply counted"));
}
#[test]
fn world_geometry_matches_analytic_translation_rotation_and_preview() {
    let c = config();
    let w = &c.robot.assembly.as_ref().unwrap().wheels[0];
    let mut sensor = c.robot.sensors[0].clone();
    sensor.position_m = Vec2::new(0.013, -0.027);
    sensor.angle_deg = 37.;
    for yaw in [0., std::f64::consts::FRAC_PI_2, -1.3] {
        let pose = Pose2::new(1.2, -0.7, yaw);
        let actual = sensor_world_position(&sensor, pose);
        near(actual.x, 1.2 + 0.013 * yaw.cos() + 0.027 * yaw.sin());
        near(actual.y, -0.7 + 0.013 * yaw.sin() - 0.027 * yaw.cos());
        let corners = wheel_corners(w, pose);
        let center = corners.into_iter().fold(Vec2::default(), |a, b| a + b) * 0.25;
        let expected = wheel_world_position(w, pose);
        near(center.x, expected.x);
        near(center.y, expected.y);
    }
}
#[test]
fn wheel_geometry_and_mass_are_used_by_runtime() {
    let mut c = config();
    let a = c.robot.assembly.as_mut().unwrap();
    for w in &mut a.wheels {
        w.position_m.x *= 1.2;
        w.position_m.y *= 1.5;
        w.radius_m *= 1.1;
    }
    a.mass_mode = MassMode::Components;
    a.masses = vec![mass("body", 0.3, 0.005, 0.01, 0.02, 0.0002)];
    let core = SimulationCore::new(c.clone(), Some(1000)).unwrap();
    let r = &core.resolved_experiment().config().robot;
    near(
        r.drivetrain.track_width_m,
        c.robot.drivetrain.track_width_m * 1.5,
    );
    near(
        r.drivetrain.wheel_radius_m,
        c.robot.drivetrain.wheel_radius_m * 1.1,
    );
    near(r.chassis.mass_kg, 0.3);
    near(r.chassis.center_of_mass_m.y, 0.01);
    let s = core.sample();
    near(
        s.normal_left_n + s.normal_right_n,
        0.3 * 9.80665 + s.downforce_extra_n,
    );
    assert!(s.normal_left_n > s.normal_right_n);
}
#[test]
fn unsupported_geometry_is_editable_but_never_silently_approximated() {
    for case in 0..5 {
        let mut c = config();
        let a = c.robot.assembly.as_mut().unwrap();
        match case {
            0 => a.wheels[0].radius_m *= 1.2,
            1 => a.wheels[0].angle_deg = 15.,
            2 => {
                a.wheels[0].kind = WheelKind::Caster;
                a.wheels[0].motor = None;
            }
            3 => a.wheels[0].height_m = 0.002,
            _ => a.wheels[0].tire.mu_lateral *= 0.5,
        }
        assert!(a.clone().validate(&c.robot).is_ok());
        assert!(ResolvedExperiment::new(&c)
            .unwrap_err()
            .contains("incompatible"));
    }
}
#[test]
fn invalid_association_ids_degenerate_support_and_nonfinite_values_fail() {
    for case in 0..6 {
        let mut c = config();
        let a = c.robot.assembly.as_mut().unwrap();
        match case {
            0 => a.wheels[0].motor = Some("missing".into()),
            1 => a.wheels[0].id = a.wheels[1].id.clone(),
            2 => a.wheels.iter_mut().for_each(|w| w.position_m.y = 0.),
            3 => a.wheels[0].radius_m = f64::NAN,
            4 => a.masses[0].component = Some("missing".into()),
            _ => a.masses[0].mass_kg = -1.,
        }
        assert!(a.clone().validate(&c.robot).is_err());
    }
}
#[test]
fn com_outside_support_is_warning_in_editor_and_error_for_run() {
    let mut c = config();
    c.robot.chassis.center_of_mass_m = Vec2::new(0.5, 0.);
    let warnings = c
        .robot
        .assembly
        .as_ref()
        .unwrap()
        .validate(&c.robot)
        .unwrap();
    assert!(warnings.iter().any(|w| w.contains("outside")));
    assert!(ResolvedExperiment::new(&c).unwrap_err().contains("outside"));
}
#[test]
fn duplicate_move_delete_and_history_restore_entire_assets() {
    let mut c = config();
    let initial = robot_json(&c.robot);
    let id = c.robot.sensors[0].id.clone();
    let mut history = RobotHistory::default();
    let before = c.robot.clone();
    let new = duplicate_component(&mut c.robot, &id).unwrap();
    assert_ne!(id, new);
    set_component_pose(&mut c.robot, &new, Pose2::new(0.021, 0.043, 0.9)).unwrap();
    history.record(before, &c.robot);
    let edited = robot_json(&c.robot);
    assert!(history.undo(&mut c.robot));
    assert_eq!(robot_json(&c.robot), initial);
    assert!(history.redo(&mut c.robot));
    assert_eq!(robot_json(&c.robot), edited);
    let before = c.robot.clone();
    remove_component(&mut c.robot, &new).unwrap();
    history.record(before, &c.robot);
    assert!(history.undo(&mut c.robot));
    assert_eq!(robot_json(&c.robot), edited);
    assert!(history.undo(&mut c.robot));
    let before = c.robot.clone();
    c.robot.name = "new branch".into();
    history.record(before, &c.robot);
    assert!(!history.redo(&mut c.robot));
}
#[test]
fn assembly_roundtrip_preserves_ids_heights_tires_masses_and_independent_sensors() {
    let mut c = config();
    c.robot.sensors[0].height_m = 0.012;
    c.robot.sensors[0].angle_deg = 43.;
    let a = c.robot.assembly.as_mut().unwrap();
    a.measured_com_height_m = 0.021;
    a.wheels[0].radius_m = 0.017;
    a.wheels[0].material = "borracha macia".into();
    a.masses[1].mass_kg = 0.04;
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(format!(
        "target/stage4-roundtrip-{}.json",
        std::process::id()
    ));
    save_robot_to_file(&c.robot, &path).unwrap();
    let actual = load_robot_from_file(&path).unwrap();
    assert_eq!(
        robotrace_sim::json::parse_json(&robot_json(&c.robot)).unwrap(),
        robotrace_sim::json::parse_json(&robot_json(&actual)).unwrap()
    );
    std::fs::remove_file(path).unwrap();
}
