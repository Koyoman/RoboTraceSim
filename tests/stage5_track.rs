use robotrace_sim::{
    config::{load_project, load_track_from_file, TrackConfig},
    io::persistence::{save_track_to_file, track_json},
    math::{Pose2, Vec2},
    rtsim_track::*,
    sim::SimulationCore,
    track::{definition::*, events::*, runtime::TrackRuntimeCache, TrackModel, TrackRuntime},
};
use std::path::Path;
fn track() -> TrackConfig {
    TrackConfig::from_parametric(TrackV2::default_closed_rectangle())
}
fn config() -> robotrace_sim::config::LoadedConfig {
    load_project(Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/basic/projeto_suction.rtsim"))
        .unwrap()
}
fn area(x: f64, y: f64, w: f64, h: f64) -> RectArea {
    RectArea {
        center_m: Vec2::new(x, y),
        size_m: Vec2::new(w, h),
        angle_deg: 0.,
    }
}
fn region(id: &str, x: f64, y: f64, mu: f64) -> SurfaceRegion {
    SurfaceRegion {
        id: id.into(),
        area: area(x, y, 0.4, 0.4),
        material: id.into(),
        mu,
        reflectance: 0.3,
        color: [30, 60, 90],
        height_m: 0.,
        roughness_m: 0.,
    }
}
fn gate(id: &str, kind: GateKind, x: f64) -> RaceGate {
    RaceGate {
        id: id.into(),
        kind,
        center_m: Vec2::new(x, 0.5),
        heading_deg: 0.,
        half_width_m: 0.2,
    }
}
fn race_track() -> TrackRuntime {
    let mut t = track();
    t.environment.race_enabled = true;
    t.environment.laps = 2;
    t.environment.gates = vec![
        gate("start", GateKind::Start, 0.5),
        gate("cp1", GateKind::Checkpoint, 1.),
        gate("cp2", GateKind::Checkpoint, 1.5),
        gate("finish", GateKind::Finish, 2.),
    ];
    TrackRuntime::try_new(t).unwrap()
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-10, "{a} != {b}");
}
#[test]
fn automatic_marks_and_gaps_use_the_same_oriented_rectangles_as_canvas() {
    let mut t = track();
    let runtime = TrackRuntime::try_new(t.clone()).unwrap();
    assert!(runtime.marks().len() >= 10);
    for m in runtime.marks() {
        near(runtime.reflectance_at(m.area.center_m), m.reflectance);
        let point = m.area.center_m
            + Vec2::new(
                -m.area.angle_deg.to_radians().sin(),
                m.area.angle_deg.to_radians().cos(),
            ) * 0.019;
        assert!(m.area.contains(point));
        near(runtime.reflectance_at(point), m.reflectance);
    }
    t.environment.marks.push(OpticalMark {
        id: "gap".into(),
        area: area(0.8, 0.5, 0.05, 0.05),
        kind: MarkKind::Gap,
        reflectance: 0.,
        color: [0, 0, 0],
    });
    let r = TrackRuntime::try_new(t).unwrap();
    near(r.reflectance_at(Vec2::new(0.8, 0.5)), r.base_reflectance());
    near(r.reflectance_at(Vec2::new(0.84, 0.5)), r.line_reflectance());
}
#[test]
fn overlapping_surface_last_wins_and_outside_is_explicit() {
    let mut t = track();
    t.environment.regions = vec![
        region("first", 0.8, 0.8, 0.7),
        region("last", 0.8, 0.8, 0.2),
    ];
    t.environment.outside_mu = 0.01;
    let runtime = TrackRuntime::try_new(t.clone()).unwrap();
    assert_eq!(runtime.surface_at(Vec2::new(0.8, 0.8)).material, "last");
    near(runtime.surface_mu_at(Vec2::new(0.8, 0.8)), 0.2);
    near(runtime.reflectance_at(Vec2::new(0.8, 0.8)), 0.3);
    assert!(!runtime.surface_at(Vec2::new(-0.01, 0.5)).inside);
    near(runtime.surface_mu_at(Vec2::new(-0.01, 0.5)), 0.01);
    move_item(&mut t.environment.regions, 0, 1).unwrap();
    assert_eq!(
        TrackRuntime::try_new(t)
            .unwrap()
            .surface_at(Vec2::new(0.8, 0.8))
            .material,
        "first"
    );
}
#[test]
fn analytic_arc_detects_narrow_line_between_visual_samples() {
    let mut t = TrackV2::default_closed_rectangle();
    t.origin = TrackPose {
        x_mm: 500.,
        y_mm: 500.,
        heading_deg: 0.,
    };
    t.segments = vec![TrackSegment::Arc(ArcSegment {
        id: "tight".into(),
        radius_mm: 10.,
        sweep_deg: 90.,
    })];
    t.markings.start_finish.enabled = false;
    t.markings.corner_markers.auto_generate = false;
    t.rules.overrides.line_width_mm = Some(0.02);
    t.closure.required = false;
    let mut cfg = TrackConfig::from_parametric(t.clone());
    cfg.environment.start_source = StartSource::Project;
    let r = TrackRuntime::try_new(cfg.clone()).unwrap();
    let angle = 0.123f64;
    let point = Vec2::new(0.5 + 0.01 * angle.sin(), 0.51 - 0.01 * angle.cos());
    near(r.distance_to_line_m(point), 0.);
    near(r.reflectance_at(point), r.line_reflectance());
    cfg.centerline = build_geometry_with_step(&t, 50.).centerline_m;
    let coarse = TrackRuntime::try_new(cfg).unwrap();
    near(coarse.reflectance_at(point), r.reflectance_at(point));
}
#[test]
fn crossing_lines_union_and_closed_rectangle_are_consistent() {
    let mut t = track();
    assert!(
        build_geometry(t.parametric.as_ref().unwrap())
            .closure_error
            .distance_mm
            < 1e-8
    );
    t.parametric = None;
    t.centerline = vec![
        Vec2::new(0., 0.),
        Vec2::new(1., 1.),
        Vec2::new(0., 1.),
        Vec2::new(1., 0.),
    ];
    let r = TrackRuntime::try_new(t).unwrap();
    near(r.distance_to_line_m(Vec2::new(0.5, 0.5)), 0.);
    near(r.reflectance_at(Vec2::new(0.5, 0.5)), r.line_reflectance());
}
#[test]
fn cache_reuses_immutable_geometry_and_rebuilds_on_definition_change_only() {
    let mut cfg = track();
    let mut cache = TrackRuntimeCache::default();
    let a = cache.get(&cfg).unwrap();
    for _ in 0..30 {
        let b = cache.get(&cfg).unwrap();
        assert!(a.shares_geometry_with(&b));
        let _ = b.reflectance_at(Vec2::new(0.8, 0.5));
    }
    assert_eq!(cache.build_count(), 1);
    cfg.centerline.clear();
    assert!(a.shares_geometry_with(&cache.get(&cfg).unwrap()));
    cfg.environment.regions.push(region("new", 0.8, 0.8, 0.1));
    let changed = cache.get(&cfg).unwrap();
    assert!(!a.shares_geometry_with(&changed));
    assert_eq!(cache.build_count(), 2);
    near(a.surface_mu_at(Vec2::new(0.8, 0.8)), 1.2);
    near(changed.surface_mu_at(Vec2::new(0.8, 0.8)), 0.1);
}
#[test]
fn each_wheel_queries_its_material_and_asymmetric_friction_affects_motion() {
    let mut cfg = config();
    cfg.track = track();
    cfg.track.environment.start_source = StartSource::Project;
    cfg.project.start_pose = Pose2::new(0.8, 0.8, 0.);
    cfg.robot.normal_force.model = robotrace_sim::io::models::NormalForceKind::None;
    cfg.robot.controller.kp = 0.;
    cfg.robot.controller.ki = 0.;
    cfg.robot.controller.kd = 0.;
    cfg.robot.controller.base_pwm = 0.8;
    let mut low = region("ice", 0.8, 0.7, 0.01);
    low.area.size_m = Vec2::new(1., 0.2);
    cfg.track.environment.regions.push(low);
    let mut core = SimulationCore::new(cfg, Some(10000)).unwrap();
    let contacts = core.contact_surfaces();
    assert_eq!(contacts.len(), 4);
    let values: Vec<_> = contacts.iter().map(|(_, s)| s.mu).collect();
    assert!(values.contains(&0.01) && values.contains(&1.2));
    core.advance_until(10000).unwrap();
    assert!((core.sample().wheel_force_left_n - core.sample().wheel_force_right_n).abs() > 1e-4);
    assert!(core.sample().wheel_force_right_n.abs() <= core.sample().normal_right_n * 0.01 + 1e-9);
}
#[test]
fn race_orders_multiple_crossings_rejects_reverse_and_requires_new_start_per_lap() {
    let r = race_track();
    let mut state = RaceState::new(&r, true);
    state.update(Vec2::new(0.4, 0.5), Vec2::new(2.1, 0.5), true, 100);
    assert_eq!(state.laps(), 1);
    assert_eq!(state.termination(), None);
    state.update(Vec2::new(2.1, 0.5), Vec2::new(1.9, 0.5), true, 200);
    state.update(Vec2::new(1.9, 0.5), Vec2::new(2.1, 0.5), true, 300);
    assert_eq!(state.laps(), 1);
    state.update(Vec2::new(2.1, 0.5), Vec2::new(0.4, 0.5), true, 400);
    state.update(Vec2::new(0.4, 0.5), Vec2::new(2.1, 0.5), true, 500);
    assert_eq!(state.laps(), 2);
    assert_eq!(state.termination(), Some("race_finished"));
    assert!(state
        .events()
        .iter()
        .any(|e| e.kind == RaceEventKind::ReverseCrossing));
}
#[test]
fn invalid_checkpoint_sequence_cannot_finish_lap() {
    let r = race_track();
    let mut state = RaceState::new(&r, true);
    state.update(Vec2::new(0.4, 0.5), Vec2::new(0.6, 0.5), true, 1);
    state.update(Vec2::new(1.4, 0.5), Vec2::new(1.6, 0.5), true, 2);
    state.update(Vec2::new(1.9, 0.5), Vec2::new(2.1, 0.5), true, 3);
    assert_eq!(state.laps(), 0);
    assert!(state
        .events()
        .iter()
        .any(|e| e.kind == RaceEventKind::WrongSequence));
}
#[test]
fn touching_plane_and_repeated_observations_do_not_duplicate_events() {
    let r = race_track();
    let mut state = RaceState::new(&r, true);
    state.update(Vec2::new(0.4, 0.5), Vec2::new(0.5, 0.5), true, 1);
    state.update(Vec2::new(0.5, 0.5), Vec2::new(0.6, 0.5), true, 2);
    for _ in 0..10 {
        state.update(Vec2::new(0.6, 0.5), Vec2::new(0.6, 0.5), true, 2);
    }
    assert_eq!(
        state
            .events()
            .iter()
            .filter(|e| e.kind == RaceEventKind::Started)
            .count(),
        1
    );
    state.update(Vec2::new(0.9, 1.), Vec2::new(1.1, 1.), true, 3);
    assert!(!state
        .events()
        .iter()
        .any(|e| e.kind == RaceEventKind::Checkpoint));
}
#[test]
fn area_exit_is_separate_from_line_loss_and_stops_when_requested() {
    let mut cfg = config();
    cfg.track = track();
    cfg.track.environment.start_source = StartSource::Project;
    cfg.track.environment.stop_on_exit = true;
    cfg.project.start_pose = Pose2::new(-0.2, 0.5, 0.);
    let core = SimulationCore::new(cfg.clone(), Some(1000)).unwrap();
    assert!(core.is_finished());
    assert_eq!(core.termination_reason(), "area_exit");
    assert_eq!(
        core.race_state().events()[0].kind,
        RaceEventKind::ExitedArea
    );
    cfg.project.start_pose = Pose2::new(1., 1., 0.);
    for sensor in &mut cfg.robot.sensors {
        sensor.acquisition.reflectance_noise_std = 0.;
        sensor.acquisition.adc_noise_lsb = 0.;
        sensor.asset.response_model = robotrace_sim::config::SensorResponseModel::Ideal;
    }
    let core = SimulationCore::new(cfg, Some(1000)).unwrap();
    assert!(!core.is_finished());
    assert!(!core.robot_over_line());
    assert!(!core.sample().line_visible);
}
#[test]
fn invalid_numbers_fail_in_every_rules_mode_and_relief_is_explicitly_unsupported() {
    for mode in [
        TrackRulesMode::Strict,
        TrackRulesMode::Warning,
        TrackRulesMode::Free,
    ] {
        let mut t = track();
        t.parametric.as_mut().unwrap().rules.mode = mode;
        if let TrackSegment::Arc(a) = &mut t.parametric.as_mut().unwrap().segments[1] {
            a.radius_mm = f64::NAN;
        }
        assert!(TrackRuntime::try_new(t).is_err());
    }
    let mut t = track();
    let mut r = region("relief", 0.8, 0.8, 1.);
    r.height_m = 0.003;
    r.roughness_m = 0.0001;
    t.environment.regions.push(r);
    let warnings = validate_definition(&t).unwrap();
    assert!(warnings.iter().any(|w| w.contains("metadata")));
    let rt = TrackRuntime::try_new(t.clone()).unwrap();
    near(rt.surface_at(Vec2::new(0.8, 0.8)).height_m, 0.003);
    t.environment.relief_enabled = true;
    assert!(TrackRuntime::try_new(t)
        .unwrap_err()
        .contains("vertical relief"));
}
#[test]
fn save_load_history_preserves_layers_ids_metadata_and_start_source() {
    let mut cfg = config();
    cfg.track = track();
    cfg.track
        .environment
        .regions
        .push(region("borracha", 0.8, 0.8, 0.7));
    cfg.track.environment.start_source = StartSource::Project;
    cfg.track.parametric.as_mut().unwrap().rules.source = "fixture de teste".into();
    cfg.track.parametric.as_mut().unwrap().rules.edition = "2026".into();
    let mut history = TrackEditHistory::default();
    let before = (cfg.track.clone(), cfg.project.start_pose);
    cfg.project.start_pose = Pose2::new(0.3, 0.4, 0.5);
    cfg.track
        .environment
        .regions
        .push(region("vidro", 1., 1., 0.1));
    history.record(before, &cfg);
    let edited = track_json(&cfg.track);
    let pose = cfg.project.start_pose;
    assert!(history.undo(&mut cfg));
    assert_eq!(cfg.track.environment.regions.len(), 1);
    assert!(history.redo(&mut cfg));
    assert_eq!(cfg.project.start_pose, pose);
    assert_eq!(track_json(&cfg.track), edited);
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("target/stage5-track-{}.json", std::process::id()));
    save_track_to_file(&cfg.track, &path).unwrap();
    let loaded = load_track_from_file(&path).unwrap();
    assert_eq!(
        robotrace_sim::json::parse_json(&track_json(&loaded)).unwrap(),
        robotrace_sim::json::parse_json(&edited).unwrap()
    );
    std::fs::remove_file(path).unwrap();
}
#[test]
fn explicit_start_source_has_identical_cli_and_runtime_resolution() {
    let mut cfg = config();
    cfg.project.start_pose = Pose2::new(0.9, 0.8, 0.7);
    cfg.track.environment.start_source = StartSource::Project;
    let core = SimulationCore::new(cfg.clone(), Some(1000)).unwrap();
    assert_eq!(core.state().pose, cfg.project.start_pose);
    cfg.track.environment.start_source = StartSource::Track;
    let expected = effective_start_pose(&cfg.track, cfg.project.start_pose).unwrap();
    let core = SimulationCore::new(cfg, Some(1000)).unwrap();
    assert_eq!(core.state().pose, expected);
}
#[test]
fn clipping_gap_to_overlapping_region_preserves_world_bounds() {
    let subject = area(0., 0., 2., 2.).corners();
    let clip = area(1., 0., 2., 2.).corners();
    let result = clip_polygon(&subject, &clip);
    assert_eq!(result.len(), 4);
    for p in result {
        assert!(p.x >= 0. && p.x <= 1. && p.y.abs() <= 1.);
    }
}
#[test]
fn core_finishes_on_race_and_records_actual_time_and_events() {
    let mut cfg = config();
    cfg.track = track();
    cfg.track.environment.start_source = StartSource::Project;
    cfg.project.start_pose = Pose2::new(0.8, 0.8, 0.);
    cfg.project.time.physics_dt_us = 50;
    cfg.track.environment.race_enabled = true;
    cfg.track.environment.laps = 1;
    cfg.track.environment.gates = vec![
        gate("start", GateKind::Start, 0.80001),
        gate("checkpoint", GateKind::Checkpoint, 0.80005),
        gate("finish", GateKind::Finish, 0.80010),
    ];
    for g in &mut cfg.track.environment.gates {
        g.center_m.y = 0.8;
    }
    cfg.robot.controller.kp = 0.;
    cfg.robot.controller.ki = 0.;
    cfg.robot.controller.kd = 0.;
    cfg.robot.controller.base_pwm = 0.8;
    cfg.project.replay_output = None;
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("target/stage5-race-{}.csv", std::process::id()));
    let summary = robotrace_sim::sim::run_simulation(
        cfg,
        robotrace_sim::sim::RunOptions {
            duration_us: Some(200000),
            output_csv: Some(path.clone()),
            output_replay: None,
            headless: true,
            benchmark: false,
            physics_dt_override_us: None,
        },
    )
    .unwrap();
    assert_eq!(summary.termination_reason, "race_finished");
    assert!(summary.duration_us > 0 && summary.duration_us < 200000);
    assert_eq!(summary.steps * 50, summary.duration_us);
    assert_eq!(
        summary.race_events.last().unwrap().kind,
        RaceEventKind::Finished
    );
    let events = std::fs::read_to_string(format!("{}.events.json", path.display())).unwrap();
    let json = robotrace_sim::json::parse_json(&events).unwrap();
    assert_eq!(
        json.get("termination").unwrap().as_str(),
        Some("race_finished")
    );
}
#[test]
fn clockwise_arcs_are_analytic_and_color_does_not_change_physics() {
    let mut cfg = track();
    let t = cfg.parametric.as_mut().unwrap();
    t.markings.start_finish.enabled = false;
    t.markings.corner_markers.auto_generate = false;
    t.origin = TrackPose {
        x_mm: 500.,
        y_mm: 500.,
        heading_deg: 0.,
    };
    t.segments = vec![TrackSegment::Arc(ArcSegment {
        id: "right".into(),
        radius_mm: 10.,
        sweep_deg: -90.,
    })];
    let runtime = TrackRuntime::try_new(cfg.clone()).unwrap();
    let angle = 0.31f64;
    let p = Vec2::new(0.5 + 0.01 * angle.sin(), 0.49 + 0.01 * angle.cos());
    near(runtime.distance_to_line_m(p), 0.);
    cfg.parametric.as_mut().unwrap().surface.line_color = "red".into();
    let colored = TrackRuntime::try_new(cfg).unwrap();
    near(runtime.reflectance_at(p), colored.reflectance_at(p));
    near(runtime.surface_mu_at(p), colored.surface_mu_at(p));
}
