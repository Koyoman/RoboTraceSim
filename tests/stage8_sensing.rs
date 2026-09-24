use robotrace_sim::{
    config::*,
    control::{
        estimator::Estimator,
        native::{Firmware, NativeAdapter},
        replay_controller::ReplayController,
        *,
    },
    controller::BuiltInPid,
    encoder::QuantizedEncoder,
    gyro::NoisyGyro,
    math::{Pose2, Vec2},
    models::{power::ActuatorMode, sensing::*},
    sensor::{SensorModel, SimpleLineSensor},
    sim::SimulationCore,
    track::TrackModel,
};
use std::path::Path;
fn cfg() -> LoadedConfig {
    load_project(Path::new("examples/basic/projeto.rtsim")).unwrap()
}
fn robot() -> RobotConfig {
    let mut r = cfg().robot;
    r.sensors.truncate(2);
    for s in &mut r.sensors {
        s.acquisition.reflectance_noise_std = 0.;
        s.acquisition.adc_noise_lsb = 0.;
        s.position_m = Vec2::default();
        s.angle_deg = 0.;
        s.asset.response_model = SensorResponseModel::Ideal;
        s.asset.detection_area = SensorDetectionArea::Point { radius_m: 0. };
    }
    r
}
struct Field(f64);
impl TrackModel for Field {
    fn reflectance_at(&self, p: Vec2) -> f64 {
        if self.0 < 0. {
            if p.x.abs() < 0.01 {
                1.
            } else {
                0.
            }
        } else {
            self.0
        }
    }
    fn surface_mu_at(&self, _: Vec2) -> f64 {
        1.
    }
    fn distance_to_line_m(&self, _: Vec2) -> f64 {
        0.
    }
    fn base_reflectance(&self) -> f64 {
        0.
    }
    fn line_reflectance(&self) -> f64 {
        1.
    }
}
#[test]
fn area_rotation_and_world_pose_change_integrated_signal() {
    let mut r = robot();
    r.sensors.truncate(1);
    r.sensors[0].asset.detection_area = SensorDetectionArea::Rectangle {
        width_m: 0.04,
        height_m: 0.002,
    };
    r.sensing.optical.insert(
        r.sensors[0].id.clone(),
        Acquisition {
            area_samples: 40,
            ..Default::default()
        },
    );
    let mut s = SimpleLineSensor::from_robot(&r);
    let out = s.sample(&Field(-1.), Pose2::default(), 0);
    assert!((out.channels[0].raw_reflectance - 0.5).abs() < 1e-12);
    r.sensors[0].angle_deg = 90.;
    let mut s = SimpleLineSensor::from_robot(&r);
    assert_eq!(
        s.sample(&Field(-1.), Pose2::default(), 0).channels[0].raw_reflectance,
        1.
    );
    assert_eq!(
        s.sample(&Field(-1.), Pose2::new(0.1, 0., 0.), 1000)
            .channels[0]
            .raw_reflectance,
        0.
    );
}
#[test]
fn phased_acquisition_conversion_delivery_and_hold_are_distinct() {
    let mut r = robot();
    let ids: Vec<String> = r.sensors.iter().map(|s| s.id.clone()).collect();
    for (i, id) in ids.iter().enumerate() {
        r.sensing.optical.insert(
            id.clone(),
            Acquisition {
                period_us: 1000,
                phase_us: i as u64 * 100,
                conversion_us: 100,
                latency_us: 100,
                ..Default::default()
            },
        );
    }
    let mut s = SimpleLineSensor::from_robot(&r);
    let a = s.advance(&Field(0.2), Pose2::default(), 0, 1000);
    assert!(a.channels.iter().all(|r| !r.valid));
    s.advance(&Field(0.8), Pose2::default(), 100, 1000);
    let a = s.advance(&Field(1.), Pose2::default(), 200, 1000);
    assert!(a.channels[0].valid && !a.channels[1].valid);
    assert!((a.channels[0].raw_reflectance - 0.2).abs() < 1e-12);
    let a = s.advance(&Field(0.), Pose2::default(), 300, 1000);
    assert_eq!(a.channels[1].acquired_us, 100);
    assert_eq!(a.channels[1].age_us, 200);
    assert_eq!(a.channels[1].raw_reflectance, 0.8);
    let held = s.advance(&Field(0.), Pose2::default(), 500, 1000);
    assert_eq!(a.adc, held.adc);
    assert_eq!(held.channels[0].age_us, 500);
    assert_eq!(s.ids(), ids.iter().map(String::as_str).collect::<Vec<_>>());
}
#[test]
fn digital_hysteresis_is_stateful_after_filter_and_adc() {
    let mut r = robot();
    r.sensors.truncate(1);
    r.sensors[0].asset.sensor_type = SensorType::LineDigital;
    r.sensors[0].asset.response_model = SensorResponseModel::Threshold { threshold: 0.5 };
    r.sensing.optical.insert(
        r.sensors[0].id.clone(),
        Acquisition {
            hysteresis: 0.2,
            ..Default::default()
        },
    );
    let mut s = SimpleLineSensor::from_robot(&r);
    for (t, x, expected) in [
        (0, 0.3, false),
        (1000, 0.7, true),
        (2000, 0.5, true),
        (3000, 0.3, false),
    ] {
        assert_eq!(
            s.sample(&Field(x), Pose2::default(), t).channels[0].digital,
            Some(expected)
        );
    }
}
#[test]
fn optical_filter_and_response_models_have_known_values() {
    let mut r = robot();
    r.sensors.truncate(1);
    r.sensing.optical.insert(
        r.sensors[0].id.clone(),
        Acquisition {
            filter_tau_s: 0.001,
            ..Default::default()
        },
    );
    let mut s = SimpleLineSensor::from_robot(&r);
    s.sample(&Field(0.), Pose2::default(), 0);
    let out = s.sample(&Field(1.), Pose2::default(), 1000);
    assert!((out.channels[0].filtered - (1. - (-1_f64).exp())).abs() < 1e-12);
    r.sensing.optical.clear();
    for (response, expected) in [
        (
            SensorResponseModel::Linear {
                gain: 2.,
                offset: 0.1,
            },
            0.6,
        ),
        (
            SensorResponseModel::Polynomial {
                coefficients: vec![0., 0., 1.],
            },
            0.0625,
        ),
        (
            SensorResponseModel::LookupTable {
                points: vec![
                    SensorResponsePoint {
                        input: 0.,
                        output: 1.,
                    },
                    SensorResponsePoint {
                        input: 1.,
                        output: 0.,
                    },
                ],
            },
            0.75,
        ),
    ] {
        r.sensors[0].asset.response_model = response;
        let out = SimpleLineSensor::from_robot(&r).sample(&Field(0.25), Pose2::default(), 0);
        assert!((out.channels[0].filtered - expected).abs() < 1e-12);
    }
    r.sensors[0].asset.response_model = SensorResponseModel::Linear {
        gain: 100.,
        offset: 0.,
    };
    assert_eq!(
        SimpleLineSensor::from_robot(&r)
            .sample(&Field(1.), Pose2::default(), 0)
            .adc[0],
        4095
    );
}
#[test]
fn disabling_reordering_and_other_rng_channels_do_not_change_sensor_noise() {
    let mut r = robot();
    for s in &mut r.sensors {
        s.acquisition.reflectance_noise_std = 0.1;
        s.acquisition.adc_noise_lsb = 3.;
    }
    let id = r.sensors[1].id.clone();
    let mut all = SimpleLineSensor::from_robot(&r);
    r.sensors[0].enabled = false;
    let mut one = SimpleLineSensor::from_robot(&r);
    for t in (0..10000).step_by(1000) {
        let a = all.sample(&Field(0.5), Pose2::default(), t);
        let b = one.sample(&Field(0.5), Pose2::default(), t);
        assert_eq!(
            a.channels.iter().find(|c| c.id == id).unwrap().adc,
            b.adc[0]
        );
    }
}
#[test]
fn encoder_shaft_quadrature_latency_filter_and_lost_pulses() {
    let r = robot();
    let settings = EncoderSettings {
        shaft_ratio: 10.,
        quadrature: 4,
        latency_us: 100,
        loss_probability: 0.,
        ..Default::default()
    };
    let mut e = QuantizedEncoder::with_settings(r.encoder.clone(), settings.clone());
    assert!(!e.sample(0., 0., 0).valid);
    let out = e.sample(std::f64::consts::TAU, 0., 1000);
    assert_eq!(out.left.ticks, 0);
    let out = e.deliver(1100);
    assert_eq!(out.left.ticks.abs(), r.encoder.ticks_per_rev as i64 * 40);
    assert!((out.left.velocity_rad_s.abs() - std::f64::consts::TAU / 0.001).abs() < 1e-8);
    let mut e = QuantizedEncoder::with_settings(
        r.encoder,
        EncoderSettings {
            loss_probability: 1.,
            ..settings
        },
    );
    e.sample(0., 0., 0);
    e.sample(3., 3., 1000);
    assert_eq!(e.deliver(1100).left.ticks, 0);
}
#[test]
fn imu_drift_acceleration_and_independent_streams_are_repeatable() {
    let r = robot();
    let settings = ImuSettings {
        latency_us: 100,
        yaw_misalignment_deg: 90.,
        accel_noise_std_m_s2: 0.,
        drift_std_rad_s_sqrt_s: 0.1,
        ..Default::default()
    };
    let mut a = NoisyGyro::with_settings(r.gyro.clone(), settings.clone());
    let mut b = NoisyGyro::with_settings(
        r.gyro,
        ImuSettings {
            accel_noise_std_m_s2: 1.,
            ..settings
        },
    );
    for t in (0..10000).step_by(1000) {
        a.sample_imu(1., Vec2::new(2., 0.), t);
        b.sample_imu(1., Vec2::new(2., 0.), t);
        let x = a.deliver(t + 100);
        let y = b.deliver(t + 100);
        assert_eq!(x.yaw_rate_rad_s, y.yaw_rate_rad_s);
        assert!(x.acceleration_m_s2.x.abs() < 1e-12);
        assert!((x.acceleration_m_s2.y + 2.).abs() < 1e-12);
        assert_eq!(x.age_us, 100);
    }
}
fn input(t: u64, position: f64) -> ControllerInput {
    ControllerInput {
        frame: SensorFrame {
            t_us: t,
            line_position_m: position,
            line_visible: true,
            ..Default::default()
        },
        ..Default::default()
    }
}
#[test]
fn pid_anti_windup_loss_recovery_and_velocity_feedback() {
    let mut r = robot();
    r.controller.kp = 100.;
    r.controller.ki = 100.;
    r.controller.kd = 0.;
    r.controller.base_pwm = 0.;
    r.controller.max_pwm = 0.5;
    r.sensing.control.loss_timeout_us = 1000;
    let mut p = BuiltInPid::from_robot(&r, 0., 1.);
    for t in 0..1000 {
        p.step_input(&input(t * 1000, 1.), 0.001);
    }
    let out = p.step_input(&input(1000000, 0.), 0.001);
    assert_eq!(out.pwm, [0.; 2]);
    let mut lost = input(1001000, 0.);
    lost.frame.line_visible = false;
    assert_eq!(p.step_input(&lost, 0.001).pwm, [0.; 2]);
    lost.frame.t_us += 1000;
    let out = p.step_input(&lost, 0.001);
    assert!(out.pwm[0] < 0. && out.pwm[1] > 0.);
    assert_eq!(p.step_input(&input(1003000, 0.), 0.001).pwm, [0.; 2]);
    r.sensing.control.speed_mode = 1;
    r.controller.kp = 0.;
    r.controller.ki = 0.;
    let mut p = BuiltInPid::from_robot(&r, 0., 1.);
    let mut i = input(0, 0.);
    i.frame.encoder.valid = true;
    let slow = p.step_input(&i, 0.001).pwm[0];
    i.frame.t_us = 1000;
    i.frame.encoder.left.velocity_rad_s = 100.;
    i.frame.encoder.right.velocity_rad_s = 100.;
    let fast = p.step_input(&i, 0.001).pwm[0];
    assert!(fast < slow);
}
#[test]
fn odometry_uses_cumulative_encoder_readings_and_gyro_without_truth() {
    let mut r = robot();
    r.sensing.control.gyro_weight = 0.;
    r.encoder.invert_left = false;
    r.encoder.invert_right = false;
    let mut e = Estimator::new(&r, 0., 1.);
    let mut f = SensorFrame::default();
    f.encoder.valid = true;
    e.update(&f);
    f.t_us = 1000;
    f.encoder.left.ticks = r.encoder.ticks_per_rev as i64;
    f.encoder.right.ticks = r.encoder.ticks_per_rev as i64;
    e.update(&f);
    let expected = std::f64::consts::TAU * r.drivetrain.wheel_radius_m;
    assert!((e.state.pose.x - expected).abs() < 1e-9);
    f.t_us = 2000;
    e.update(&f);
    assert!((e.state.pose.x - expected).abs() < 1e-9);
    r.sensing.control.gyro_weight = 1.;
    let mut e = Estimator::new(&r, 0., 1.);
    f.t_us = 0;
    f.imu.valid = true;
    f.imu.yaw_rate_rad_s = 2.;
    e.update(&f);
    f.t_us = 1000000;
    e.update(&f);
    assert!((e.state.pose.yaw - 2.).abs() < 1e-12);
}
#[test]
fn marks_learn_first_lap_and_reuse_speed_profile() {
    let mut r = robot();
    r.encoder.invert_left = false;
    r.encoder.invert_right = false;
    r.sensing.control.gyro_weight = 1.;
    r.sensing.control.profile_enabled = 1;
    r.sensing.control.mark_refractory_us = 0;
    r.sensing.control.lap_min_distance_m = 0.05;
    let mut e = Estimator::new(&r, 0., 1.);
    let mut f = SensorFrame {
        encoder: robotrace_sim::encoder::EncoderOutput {
            valid: true,
            ..Default::default()
        },
        ..Default::default()
    };
    f.optical = r
        .sensors
        .iter()
        .map(|s| OpticalSample {
            id: s.id.clone(),
            acquired_us: 0,
            available_us: 0,
            age_us: 0,
            valid: true,
            adc: 4095,
            digital: None,
        })
        .collect();
    e.update(&f);
    f.t_us = 100000;
    for c in &mut f.optical {
        c.adc = 0;
    }
    f.encoder.left.ticks = r.encoder.ticks_per_rev as i64;
    f.encoder.right.ticks = f.encoder.left.ticks;
    f.imu.valid = true;
    f.imu.yaw_rate_rad_s = 20.;
    e.update(&f);
    assert!(!e.state.profile.is_empty());
    f.t_us = 200000;
    for c in &mut f.optical {
        c.adc = 4095;
    }
    e.update(&f);
    assert_eq!(e.state.lap, 1);
    assert_eq!(e.state.marks, 2);
    assert_eq!(e.state.speed_factor, r.sensing.control.curve_speed_factor);
}
#[test]
fn replay_holds_commands_and_rejects_future_invalid_or_off_grid_records() {
    let text="t_us,pwm_left,pwm_right,downforce_pwm,mode_left,mode_right\n0,0.1,0.2,0,drive,drive\n100,0.4,0.5,0,brake,coast\n";
    let r = ReplayController::from_csv(text, 50).unwrap();
    assert_eq!(r.at(50).pwm, [0.1, 0.2]);
    assert_eq!(r.at(100).pwm, [0.4, 0.5]);
    assert_eq!(r.at(200).modes, [ActuatorMode::Brake, ActuatorMode::Coast]);
    assert_eq!(r.at(0).pwm, [0.1, 0.2]);
    assert!(ReplayController::from_csv(&text.replace("100,", "101,"), 50).is_err());
    assert!(ReplayController::from_csv(&text.replace("0.1", "NaN"), 50).is_err());
}
#[test]
fn scheduler_never_delivers_future_readings_and_snapshot_roundtrips() {
    let mut c = cfg();
    c.project.time.physics_dt_us = 50;
    for s in &c.robot.sensors {
        c.robot.sensing.optical.insert(
            s.id.clone(),
            Acquisition {
                period_us: 1000,
                latency_us: 100,
                conversion_us: 50,
                ..Default::default()
            },
        );
    }
    c.robot.sensing.encoder.latency_us = 100;
    c.robot.sensing.imu.latency_us = 100;
    let path = Path::new("target/stage8-roundtrip.json");
    robotrace_sim::io::persistence::save_robot_to_file(&c.robot, path).unwrap();
    assert_eq!(load_robot_from_file(path).unwrap().sensing, c.robot.sensing);
    let mut core = SimulationCore::new(c, Some(2000)).unwrap();
    assert!(core
        .controller_input()
        .frame
        .optical
        .iter()
        .all(|c| !c.valid));
    while core.try_step().unwrap() {
        let f = core.controller_input().frame;
        assert!(f.optical.iter().all(|r| !r.valid
            || (r.available_us <= f.t_us
                && r.acquired_us <= r.available_us
                && r.age_us == f.t_us - r.acquired_us)));
        assert!(!f.encoder.valid || f.encoder.available_us <= f.t_us);
        assert!(!f.imu.valid || f.imu.available_us <= f.t_us);
    }
}
struct InvalidFirmware;
impl Firmware for InvalidFirmware {
    fn reset(&mut self) -> Result<(), String> {
        Ok(())
    }
    fn step(&mut self, i: &ControllerInput) -> Result<TimedCommand, String> {
        Ok(TimedCommand {
            t_us: i.frame.t_us + 1,
            pwm: [0.; 2],
            downforce_pwm: 0.,
            modes: [ActuatorMode::Drive; 2],
        })
    }
}
#[test]
fn native_contract_rejects_wrong_timestamp_and_unsupported_configuration() {
    let mut a = NativeAdapter::new(Box::new(InvalidFirmware)).unwrap();
    assert!(a.step(&input(0, 0.)).is_err());
    let mut c = cfg();
    let id = c.robot.sensors[0].id.clone();
    c.robot.sensing.optical.insert(
        id,
        Acquisition {
            latency_us: 1,
            ..Default::default()
        },
    );
    assert!(SimulationCore::new(c, Some(1000)).is_err());
    let mut c = cfg();
    c.robot.sensors[0].asset.sensor_type = SensorType::DistanceToF;
    assert!(SimulationCore::new(c, Some(1000)).is_err());
}

#[test]
fn circle_sector_polygon_and_inverting_response_preserve_area_and_polarity() {
    let mut r = robot();
    r.sensors.truncate(1);
    for area in [
        SensorDetectionArea::Circle { radius_m: 0.01 },
        SensorDetectionArea::Cone {
            range_m: 0.01,
            angle_deg: 1.,
        },
        SensorDetectionArea::CustomPolygon {
            points_m: vec![
                Vec2::new(-0.01, -0.01),
                Vec2::new(0.01, -0.01),
                Vec2::new(0., 0.01),
            ],
        },
    ] {
        r.sensors[0].asset.detection_area = area;
        let out = SimpleLineSensor::from_robot(&r).sample(&Field(0.7), Pose2::default(), 0);
        assert!((out.channels[0].raw_reflectance - 0.7).abs() < 1e-12);
    }
    r.sensors[0].asset.response_model = SensorResponseModel::Linear {
        gain: -1.,
        offset: 1.,
    };
    let mut s = SimpleLineSensor::from_robot(&r);
    assert!(s.sample(&Field(1.), Pose2::default(), 0).line_visible);
    assert!(!s.sample(&Field(0.), Pose2::default(), 1000).line_visible);
}
#[test]
fn recorded_commands_reproduce_runtime_and_sensor_diagnostics() {
    let mut c = load_project(Path::new("examples/power/projeto.rtsim")).unwrap();
    c.robot.sensing.imu.latency_us = 50;
    let out = Path::new("target/stage8-replay-source.csv");
    let options = robotrace_sim::sim::RunOptions {
        duration_us: Some(2000),
        output_csv: Some(out.into()),
        output_replay: None,
        headless: true,
        benchmark: false,
        physics_dt_override_us: None,
    };
    let a = robotrace_sim::sim::run_simulation(c.clone(), options).unwrap();
    let text = std::fs::read_to_string("target/stage8-replay-source.csv.commands.csv").unwrap();
    c.robot.sensing.replay_csv = Some(text);
    let mut b = SimulationCore::new(c, Some(2000)).unwrap();
    b.advance_until(2000).unwrap();
    assert!((a.final_pose.x - b.state().pose.x).abs() < 1e-12);
    assert!((a.final_pose.yaw - b.state().pose.yaw).abs() < 1e-12);
    let lines = std::fs::read_to_string("target/stage8-replay-source.csv.sensors.jsonl").unwrap();
    assert_eq!(lines.lines().count(), a.samples as usize);
    for line in lines.lines() {
        let v = robotrace_sim::json::parse_json(line).unwrap();
        assert!(v.get("channels").is_some());
        assert!(v.get("estimated_pose").is_some());
    }
}
#[test]
fn native_firmware_runs_only_on_controller_ticks_and_stops_cleanly() {
    use std::sync::{Arc, Mutex};
    struct F(Arc<Mutex<Vec<u64>>>);
    impl Firmware for F {
        fn reset(&mut self) -> Result<(), String> {
            self.0.lock().unwrap().clear();
            Ok(())
        }
        fn step(&mut self, i: &ControllerInput) -> Result<TimedCommand, String> {
            self.0.lock().unwrap().push(i.frame.t_us);
            assert!(i
                .frame
                .optical
                .iter()
                .all(|r| !r.valid || r.available_us <= i.frame.t_us));
            Ok(TimedCommand {
                t_us: i.frame.t_us,
                pwm: [0.1; 2],
                downforce_pwm: 0.,
                modes: [ActuatorMode::Drive; 2],
            })
        }
        fn stop(&mut self) {
            self.0.lock().unwrap().push(u64::MAX);
        }
    }
    let calls = Arc::new(Mutex::new(Vec::new()));
    let mut c = cfg();
    c.project.time.controller_period_us = 1000;
    let mut core = SimulationCore::new(c, Some(3000)).unwrap();
    core.install_firmware(Box::new(F(calls.clone()))).unwrap();
    core.advance_until(3000).unwrap();
    drop(core);
    assert_eq!(*calls.lock().unwrap(), vec![0, 1000, 2000, 3000, u64::MAX]);
}

#[test]
fn closed_loop_speed_example_converges_and_odometry_tracks_straight_motion() {
    let c = load_project(Path::new("examples/sensing/projeto.rtsim")).unwrap();
    let target = c.robot.sensing.control.target_speed_m_s;
    let mut core = SimulationCore::new(c, Some(2000000)).unwrap();
    core.advance_until(2000000).unwrap();
    assert_eq!(core.time_us(), 2000000);
    assert!((core.state().vx_body_m_s - target).abs() < 0.1 * target);
    assert!(core.sensor_readings().line_visible);
    let distance = core.state().pose.x - 0.65;
    assert!((core.estimated_state().pose.x - distance).abs() < 0.002);
}

#[test]
fn multiplexed_slots_reject_overlap_and_allow_serial_conversion() {
    let mut c = cfg();
    c.project.time.physics_dt_us = 50;
    let ids: Vec<_> = c
        .robot
        .sensors
        .iter()
        .take(2)
        .map(|s| s.id.clone())
        .collect();
    for id in &ids {
        c.robot.sensing.optical.insert(
            id.clone(),
            Acquisition {
                period_us: 1000,
                mux_group: 1,
                conversion_us: 100,
                ..Default::default()
            },
        );
    }
    assert!(SimulationCore::new(c.clone(), Some(1000)).is_err());
    c.robot.sensing.optical.get_mut(&ids[1]).unwrap().phase_us = 100;
    assert!(SimulationCore::new(c, Some(1000)).is_ok());
}
