#[cfg(feature = "gui")]
mod gui {
    mod robot_editor {
        include!("app/robot_editor.rs");
    }
    use crate::calibration::{
        compare_project_with_real, import_real_log, tune_project_against_real,
        write_comparison_csv, write_comparison_report, write_normalized_real_log,
        write_tuning_report, ComparisonMetrics,
    };
    use crate::config::load_project;
    use crate::config::{
        apply_surface_profile, load_battery_profile_from_file, load_driver_profile_from_file,
        load_encoder_profile_from_file, load_fan_profile_from_file, load_gyro_profile_from_file,
        load_motor_profile_from_file, load_robot_from_file, load_sensor_asset_from_file,
        load_surface_profile_from_file, load_tire_profile_from_file, load_track_from_file,
        refresh_track_cache, surface_profile_from_track, BatteryConfig, BatteryProfile,
        ChassisConfig, DriverConfig, DriverProfile, DrivetrainConfig, EncoderConfig,
        EncoderProfile, FanConfig, FanCurveModel, FanProfile, GyroConfig, GyroProfile,
        LoadedConfig, MotorConfig, MotorProfile, NormalForceConfig, PidConfig, ProjectConfig,
        RobotConfig, RobotLineValidityArea, RobotSensorInstance, SensorAsset, SensorDetectionArea,
        SensorResponseModel, SensorResponsePoint, SensorType, TimeConfig, TireConfig, TireProfile,
        TrackConfig,
    };
    use crate::experiments::jobs::{Preview, SimulationWorker};
    use crate::io::persistence::*;
    use crate::math::{Pose2, Vec2};
    use crate::replay::export_replay_to_csv;
    use crate::rtsim_track::{
        auto_close_with_straight, build_geometry, center_start_finish_on_segment,
        clamp_start_finish_to_segment, next_segment_id, resolve_robot_start_pose, resolve_rules,
        resolve_start_finish_markers, robot_start_allowed_area_corners,
        start_finish_required_length_mm, valid_start_finish_segments, validate_track, ArcSegment,
        Severity, StartExitDirection, StraightSegment, TrackRulesMode, TrackSegment, TrackV2,
        ROBOT_START_MARKER_CLEARANCE_MM,
    };
    use crate::sim::RunOptions;
    use crate::telemetry::TelemetrySample;
    use eframe::egui;
    use robot_editor::*;
    mod track_editor {
        include!("app/track_editor.rs");
    }
    use std::fs;
    use std::path::{Path, PathBuf};
    use track_editor::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum AppView {
        Home,
        TrackEditor,
        RobotEditor,
        VisualSimulator,
        ReplayViewer,
        CalibrationTools,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum TrackFileCommand {
        None,
        New,
        Load,
        Save,
        SaveAs,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum SurfaceProfileCommand {
        None,
        New,
        Load,
        Save,
        SaveAs,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum RobotFileCommand {
        None,
        New,
        Load,
        Save,
        SaveAs,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ComponentAssetKind {
        MotorLeft,
        Driver,
        Battery,
        Tire,
        Fan,
        Encoder,
        Gyro,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ComponentAssetCommandKind {
        New,
        Load,
        Save,
        SaveAs,
    }

    #[derive(Debug, Clone, Copy)]
    struct ComponentAssetCommand {
        kind: ComponentAssetKind,
        command: ComponentAssetCommandKind,
    }

    #[derive(Debug, Default, Clone, Copy)]
    struct TrackPanelChanges {
        track_changed: bool,
        surface_changed: bool,
    }

    impl TrackPanelChanges {
        fn any(self) -> bool {
            self.track_changed || self.surface_changed
        }
    }

    #[derive(Debug, Clone, Copy)]
    struct RobotPreviewCamera {
        zoom: f32,
        pan_m: Vec2,
        min_zoom: f32,
        max_zoom: f32,
    }

    impl Default for RobotPreviewCamera {
        fn default() -> Self {
            Self {
                zoom: 1.0,
                pan_m: Vec2::new(0.0, 0.0),
                min_zoom: 0.35,
                max_zoom: 30.0,
            }
        }
    }

    impl RobotPreviewCamera {
        fn reset(&mut self) {
            self.zoom = 1.0;
            self.pan_m = Vec2::new(0.0, 0.0);
        }

        fn center(&mut self) {
            self.pan_m = Vec2::new(0.0, 0.0);
        }

        fn fit_rect(&mut self) {
            self.zoom = 1.0;
            self.pan_m = Vec2::new(0.0, 0.0);
        }

        fn viewport_bounds(self, base: Bounds) -> Bounds {
            viewport_bounds(
                base,
                self.zoom.clamp(self.min_zoom, self.max_zoom),
                self.pan_m,
            )
        }

        fn world_to_screen(self, rect: egui::Rect, bounds: Bounds, p: Vec2) -> egui::Pos2 {
            world_to_screen(rect, bounds, p)
        }

        fn screen_to_world(self, rect: egui::Rect, bounds: Bounds, pos: egui::Pos2) -> Vec2 {
            screen_to_world(rect, bounds, pos)
        }
    }

    pub fn run_app() -> Result<(), String> {
        let options = eframe::NativeOptions::default();
        eframe::run_native(
            concat!("Robotrace Sim ", env!("CARGO_PKG_VERSION")),
            options,
            Box::new(|cc| Box::new(RTSimApp::new(cc))),
        )
        .map_err(|err| err.to_string())
    }

    fn json_file_name_from_name(name: &str, fallback: &str) -> String {
        let mut cleaned = name
            .trim()
            .chars()
            .map(|c| match c {
                '<' | '>' | ':' | '"' | '/' | '\\' | '|' | '?' | '*' => '_',
                c if c.is_control() => '_',
                c => c,
            })
            .collect::<String>();

        cleaned = cleaned.trim_matches([' ', '.']).to_string();

        if cleaned.is_empty() {
            cleaned = fallback.to_string();
        }

        if !cleaned.to_lowercase().ends_with(".json") {
            cleaned.push_str(".json");
        }

        cleaned
    }

    struct RTSimApp {
        view: AppView,
        cfg: Option<LoadedConfig>,
        project_path_text: String,
        replay_path_text: String,
        track_file_path_text: String,
        track_dirty: bool,
        track_history: crate::track::definition::TrackEditHistory,
        track_history_pending: Option<(TrackConfig, Pose2)>,
        robot_file_path_text: String,
        robot_dirty: bool,
        robot_history: crate::models::robot::RobotHistory,
        robot_history_pending: Option<RobotConfig>,
        robot_selection: Option<String>,
        motor_left_asset_path_text: String,
        driver_asset_path_text: String,
        battery_asset_path_text: String,
        tire_asset_path_text: String,
        fan_asset_path_text: String,
        encoder_asset_path_text: String,
        gyro_asset_path_text: String,
        selected_fan_asset_index: usize,
        surface_profile_path_text: String,
        surface_profile_dirty: bool,
        status: String,
        selected_track_point: Option<usize>,
        track_view_zoom: f32,
        track_view_pan_m: Vec2,
        robot_preview_camera: RobotPreviewCamera,
        sim_session: Option<SimulationWorker>,
        sim_running: bool,
        worker_preview: Option<Preview>,
        auto_open_replay: bool,
        replay_config: Option<LoadedConfig>,
        replay_time_s: f64,
        replay_speed: f64,
        replay_playing: bool,
        auxiliary_job:
            Option<crate::experiments::jobs::BackgroundJob<(String, Option<ComparisonMetrics>)>>,
        replay_job: Option<
            crate::experiments::jobs::BackgroundJob<(
                crate::replay::IndexedReplay,
                Option<LoadedConfig>,
            )>,
        >,
        sim_duration_s: f64,
        last_sim_sample: Option<TelemetrySample>,
        replay: Option<crate::replay::IndexedReplay>,

        replay_max_samples: usize,
        real_log_path_text: String,
        calibration_csv_path_text: String,
        calibration_report_path_text: String,
        tuning_output_path_text: String,
        last_calibration_metrics: Option<ComparisonMetrics>,
        study_path: String,
        study_report_path: String,
    }

    impl RTSimApp {
        fn path_text(path: &Path) -> String {
            path.to_string_lossy().replace('\\', "/")
        }

        fn new(_cc: &eframe::CreationContext<'_>) -> Self {
            Self::initial()
        }
        fn initial() -> Self {
            let mut app = Self {
                view: AppView::Home,
                cfg: None,
                project_path_text: "examples/basic/projeto.rtsim".to_string(),
                replay_path_text: "examples/basic/resultado.rtlog".to_string(),
                track_file_path_text: "examples/basic/track.json".to_string(),
                track_dirty: false,
                track_history: Default::default(),
                track_history_pending: None,
                robot_file_path_text: "examples/basic/robot.json".to_string(),
                robot_dirty: false,
                robot_history: Default::default(),
                robot_history_pending: None,
                robot_selection: None,
                motor_left_asset_path_text: "RobotAssets/Motors/n20_simple_left.json".to_string(),
                driver_asset_path_text: "RobotAssets/Drivers/pwm_hbridge.json".to_string(),
                battery_asset_path_text: "RobotAssets/Batteries/2s_lipo_7400mv.json".to_string(),
                tire_asset_path_text: "RobotAssets/Tires/default_tire.json".to_string(),
                fan_asset_path_text: "RobotAssets/Fans/downforce_fan.json".to_string(),
                encoder_asset_path_text: "RobotAssets/Encoders/quantized_encoder.json".to_string(),
                gyro_asset_path_text: "RobotAssets/Gyros/noisy_gyro.json".to_string(),
                selected_fan_asset_index: 0,
                surface_profile_path_text: "examples/profiles/rob_trace_official.json".to_string(),
                surface_profile_dirty: false,
                status: "Abra um projeto .rtsim ou use o exemplo básico.".to_string(),
                selected_track_point: None,
                track_view_zoom: 1.0,
                track_view_pan_m: Vec2::new(0.0, 0.0),
                robot_preview_camera: RobotPreviewCamera::default(),
                sim_session: None,
                sim_running: false,
                worker_preview: None,
                replay_config: None,
                auto_open_replay: false,
                replay_time_s: 0.,
                replay_speed: 1.,
                replay_playing: false,
                auxiliary_job: None,
                replay_job: None,
                sim_duration_s: 10.0,
                last_sim_sample: None,
                replay: None,

                replay_max_samples: 2048,
                real_log_path_text: "examples/basic/real_log_demo.csv".to_string(),
                calibration_csv_path_text: "examples/basic/comparacao_v05.csv".to_string(),
                calibration_report_path_text: "examples/basic/comparacao_v05.txt".to_string(),
                tuning_output_path_text: "examples/basic/ajuste_v05.json".to_string(),
                last_calibration_metrics: None,
                study_path: "target/stage10-fixture/study.json".into(),
                study_report_path: "target/stage10-qualification.json".into(),
            };
            if Path::new(&app.project_path_text).exists() {
                app.load_project_from_path(PathBuf::from(app.project_path_text.clone()));
            } else {
                app.cfg = Some(default_loaded_config(PathBuf::from("projeto_v05.rtsim")));
            }
            app
        }

        fn set_status(&mut self, status: impl Into<String>) {
            self.status = status.into();
        }

        fn load_project_from_path(&mut self, path: PathBuf) {
            match load_project(&path) {
                Ok(cfg) => {
                    self.project_path_text = path.display().to_string();
                    self.sim_duration_s = cfg.project.duration_s;
                    self.replay_path_text = default_replay_path(&cfg).display().to_string();
                    self.track_file_path_text = Self::path_text(&resolve_child_path(
                        &cfg.project_path,
                        &cfg.project.track_path,
                    ));
                    self.robot_file_path_text = Self::path_text(&resolve_child_path(
                        &cfg.project_path,
                        &cfg.project.robot_path,
                    ));
                    self.surface_profile_path_text = cfg
                        .track
                        .parametric
                        .as_ref()
                        .map(|track| default_surface_profile_path(&track.rules.profile))
                        .unwrap_or_else(|| {
                            PathBuf::from("examples/profiles/rob_trace_official.json")
                        })
                        .display()
                        .to_string();
                    self.track_dirty = false;
                    self.robot_dirty = false;
                    self.surface_profile_dirty = false;
                    self.selected_track_point = None;
                    self.track_view_zoom = 1.0;
                    self.track_view_pan_m = Vec2::new(0.0, 0.0);
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.cfg = Some(cfg);
                    self.track_history.clear();
                    self.track_history_pending = None;
                    self.robot_history.clear();
                    self.robot_history_pending = None;
                    self.robot_selection = None;
                    self.set_status("Projeto carregado com sucesso.");
                }
                Err(err) => self.set_status(format!("Falha ao carregar projeto: {err}")),
            }
        }

        fn save_current_project(&mut self) {
            let result = self
                .cfg
                .as_ref()
                .ok_or_else(|| "nenhum projeto carregado".to_string())
                .and_then(save_loaded_config);
            match result {
                Ok(()) => {
                    self.track_dirty = false;
                    self.robot_dirty = false;
                    self.set_status("Projeto, robô e pista salvos.");
                }
                Err(err) => self.set_status(format!("Falha ao salvar: {err}")),
            }
        }

        fn update_project_start_pose_from_track(cfg: &mut LoadedConfig) {
            if let Ok(pose) =
                crate::track::definition::effective_start_pose(&cfg.track, cfg.project.start_pose)
            {
                cfg.project.start_pose = pose;
            }
        }

        fn reset_track_editor_state(&mut self) {
            self.track_history.clear();
            self.track_history_pending = None;
            self.selected_track_point = None;
            self.track_view_zoom = 1.0;
            self.track_view_pan_m = Vec2::new(0.0, 0.0);
            self.sim_session = None;
            self.last_sim_sample = None;
        }

        fn create_new_track(&mut self) {
            if self.track_dirty {
                self.set_status(
                    "There are unsaved track changes. Save or discard before creating another track.",
                );
                return;
            }

            let Some(cfg) = self.cfg.as_mut() else {
                self.set_status("Nenhum projeto carregado para receber uma nova pista.");
                return;
            };

            let mut track = TrackV2::default_closed_rectangle();
            track.name = "New Track".to_string();
            track.segments.clear();
            track.markings.start_finish.segment_id = "R1".to_string();
            cfg.track = TrackConfig::from_parametric(track);
            Self::update_project_start_pose_from_track(cfg);
            self.track_file_path_text = "examples/basic/new_track.json".to_string();
            self.surface_profile_path_text =
                "examples/profiles/rob_trace_official.json".to_string();
            self.track_dirty = true;
            self.surface_profile_dirty = true;
            self.reset_track_editor_state();
            self.set_status("New empty track created.");
        }

        fn load_track_asset(&mut self) {
            if self.track_dirty {
                self.set_status(
                    "There are unsaved track changes. Save or discard before loading another track.",
                );
                return;
            }

            let raw_path = self.track_file_path_text.trim();
            if raw_path.is_empty() {
                self.set_status("Track file path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, raw_path);
            match load_track_from_file(&path) {
                Ok(mut track) => {
                    refresh_track_cache(&mut track);
                    if let Some(cfg) = self.cfg.as_mut() {
                        cfg.project.track_path = path_relative_to_project(&cfg.project_path, &path);
                        cfg.track = track;
                        Self::update_project_start_pose_from_track(cfg);
                    } else {
                        let mut cfg = default_loaded_config(PathBuf::from("projeto_v05.rtsim"));
                        cfg.project.track_path = path.clone();
                        cfg.track = track;
                        Self::update_project_start_pose_from_track(&mut cfg);
                        self.cfg = Some(cfg);
                    }
                    self.track_file_path_text = Self::path_text(&path);
                    if let Some(profile_name) = self
                        .cfg
                        .as_ref()
                        .and_then(|cfg| cfg.track.parametric.as_ref())
                        .map(|track| track.rules.profile.clone())
                    {
                        self.surface_profile_path_text =
                            default_surface_profile_path(&profile_name)
                                .display()
                                .to_string();
                    }
                    self.track_dirty = false;
                    self.surface_profile_dirty = false;
                    self.reset_track_editor_state();
                    self.set_status(format!("Track loaded from {}", path.display()));
                }
                Err(err) => self.set_status(format!("Failed to load track: {err}")),
            }
        }

        fn save_track_asset(&mut self, save_as: bool) {
            let raw_path = self.track_file_path_text.trim();
            if raw_path.is_empty() {
                self.set_status("Track file path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, raw_path);

            let result = self
                .cfg
                .as_ref()
                .ok_or_else(|| "nenhuma pista carregada".to_string())
                .and_then(|cfg| save_track_to_file(&cfg.track, &path));

            match result {
                Ok(()) => {
                    if let Some(cfg) = self.cfg.as_mut() {
                        cfg.project.track_path = path_relative_to_project(&cfg.project_path, &path);
                    }
                    self.track_file_path_text = path.display().to_string();
                    self.track_dirty = false;
                    if save_as {
                        self.set_status(format!("Track saved as {}", path.display()));
                    } else {
                        self.set_status(format!("Track saved to {}", path.display()));
                    }
                }
                Err(err) => self.set_status(format!("Failed to save track: {err}")),
            }
        }

        fn create_new_surface_profile(&mut self) {
            if self.surface_profile_dirty {
                self.set_status(
                    "There are unsaved surface profile changes. Save or discard before creating another profile.",
                );
                return;
            }

            let Some(cfg) = self.cfg.as_mut() else {
                self.set_status("Nenhuma pista carregada para receber um surface profile.");
                return;
            };
            let Some(track) = cfg.track.parametric.as_mut() else {
                self.set_status("Surface profiles are available only for parametric tracks.");
                return;
            };

            let mut profile = surface_profile_from_track(track);
            profile.name = "custom training".to_string();
            profile.marker_profile = profile.name.clone();
            profile.rules_mode = TrackRulesMode::Warning;
            profile.line_width_mm = Some(19.0);
            profile.background_reflectance = 0.08;
            profile.line_reflectance = 0.86;
            apply_surface_profile(track, &profile);
            refresh_track_cache(&mut cfg.track);
            self.surface_profile_path_text = "examples/profiles/custom_training.json".to_string();
            self.track_dirty = true;
            self.surface_profile_dirty = true;
            self.sim_session = None;
            self.last_sim_sample = None;
            self.set_status("New surface profile created and applied to the current track.");
        }

        fn load_surface_profile_asset(&mut self) {
            if self.surface_profile_dirty {
                self.set_status(
                    "There are unsaved surface profile changes. Save or discard before loading another profile.",
                );
                return;
            }

            let raw_path = self.surface_profile_path_text.trim();
            if raw_path.is_empty() {
                self.set_status("Surface profile file path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, raw_path);
            match load_surface_profile_from_file(&path) {
                Ok(profile) => {
                    let Some(cfg) = self.cfg.as_mut() else {
                        self.set_status("Nenhuma pista carregada para aplicar o profile.");
                        return;
                    };
                    let Some(track) = cfg.track.parametric.as_mut() else {
                        self.set_status(
                            "Surface profiles are available only for parametric tracks.",
                        );
                        return;
                    };
                    apply_surface_profile(track, &profile);
                    refresh_track_cache(&mut cfg.track);
                    self.surface_profile_path_text = Self::path_text(&path);
                    self.track_dirty = true;
                    self.surface_profile_dirty = false;
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(format!("Surface profile loaded from {}", path.display()));
                }
                Err(err) => self.set_status(format!("Failed to load surface profile: {err}")),
            }
        }

        fn save_surface_profile_asset(&mut self, save_as: bool) {
            let raw_path = self.surface_profile_path_text.trim();
            if raw_path.is_empty() {
                self.set_status("Surface profile file path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, raw_path);

            let result = self
                .cfg
                .as_ref()
                .and_then(|cfg| cfg.track.parametric.as_ref())
                .ok_or_else(|| "nenhuma pista paramétrica carregada".to_string())
                .map(surface_profile_from_track)
                .and_then(|profile| save_surface_profile_to_file(&profile, &path));

            match result {
                Ok(()) => {
                    self.surface_profile_path_text = path.display().to_string();
                    self.surface_profile_dirty = false;
                    if save_as {
                        self.set_status(format!("Surface profile saved as {}", path.display()));
                    } else {
                        self.set_status(format!("Surface profile saved to {}", path.display()));
                    }
                }
                Err(err) => self.set_status(format!("Failed to save surface profile: {err}")),
            }
        }

        fn reset_simulation(&mut self) {
            let Some(cfg) = self.cfg.clone() else {
                self.set_status("Carregue um projeto.");
                return;
            };
            let duration_us = match crate::core::clock::duration_seconds_to_us(self.sim_duration_s)
            {
                Ok(v) => v,
                Err(e) => {
                    self.set_status(e);
                    return;
                }
            };
            let dir = PathBuf::from("target/runs").join(crate::sim::new_run_id());
            if let Err(e) = std::fs::create_dir_all(&dir) {
                self.set_status(e.to_string());
                return;
            }
            self.sim_session = None;
            self.worker_preview = None;
            self.last_sim_sample = None;
            self.sim_running = false;
            self.auto_open_replay = false;
            self.sim_session = Some(SimulationWorker::spawn(
                cfg,
                RunOptions {
                    duration_us: Some(duration_us),
                    output_csv: Some(dir.join("result.csv")),
                    output_replay: Some(dir.join("result.rtlog")),
                    headless: true,
                    benchmark: false,
                    physics_dt_override_us: None,
                },
                1,
                true,
            ));
            self.set_status("Execução preparada; configuração congelada em uma nova execução.");
        }
        fn run_headless_replay(&mut self) {
            self.reset_simulation();
            if let Some(w) = &self.sim_session {
                w.control.resume();
                self.sim_running = true;
                self.auto_open_replay = true;
                self.set_status("Calculando em segundo plano. O replay será aberto ao concluir.");
            }
        }
        fn poll_simulation(&mut self, ctx: &egui::Context) {
            if self.auxiliary_job.is_some() || self.replay_job.is_some() {
                ctx.request_repaint_after(std::time::Duration::from_millis(16));
            }
            if let Some(result) = self.auxiliary_job.as_ref().and_then(|j| j.take_result()) {
                self.auxiliary_job = None;
                match result {
                    Ok((text, metrics)) => {
                        if metrics.is_some() {
                            self.last_calibration_metrics = metrics;
                        }
                        self.set_status(text);
                    }
                    Err(e) => self.set_status(e),
                }
            }
            if let Some(result) = self.replay_job.as_ref().and_then(|j| j.take_result()) {
                self.replay_job = None;
                match result {
                    Ok((r, config)) => {
                        let count = r.samples;
                        self.replay_config = config;
                        self.replay = Some(r);
                        self.replay_time_s = 0.;
                        self.replay_playing = false;
                        self.view = AppView::ReplayViewer;
                        self.set_status(format!(
                            "Replay indexado: {count} amostras, leitura sob demanda."
                        ));
                    }
                    Err(e) => self.set_status(format!("Falha ao abrir replay: {e}")),
                }
            }

            if let Some(w) = &self.sim_session {
                if let Some(p) = w.control.latest() {
                    self.last_sim_sample = Some(p.sample.clone());
                    self.worker_preview = Some(p);
                }
                if !w.is_finished() {
                    ctx.request_repaint_after(std::time::Duration::from_millis(16));
                }
            }
            let result = self.sim_session.as_ref().and_then(|w| w.take_result());
            if let Some(result) = result {
                self.sim_running = false;
                match result {
                    Ok(summary) => {
                        self.set_status(format!(
                            "Execução {}: {} | {} passos",
                            summary.run_id, summary.termination_reason, summary.steps
                        ));
                        if let Some(path) = summary.replay_path {
                            self.replay_path_text = path.display().to_string();
                            if self.auto_open_replay {
                                self.load_replay();
                            }
                        }
                    }
                    Err(e) => self.set_status(format!("Falha no cálculo: {e}")),
                }
            }
        }
        fn load_replay(&mut self) {
            if self.replay_job.is_some() {
                self.set_status("Aguarde a abertura do replay em andamento.");
                return;
            }
            let path = PathBuf::from(self.replay_path_text.trim());
            let budget = self.replay_max_samples.max(128) * 1024;
            self.replay_job = Some(crate::experiments::jobs::BackgroundJob::spawn(move || {
                let replay =
                    crate::replay::IndexedReplay::open(&path, budget).map_err(|e| e.to_string())?;
                let config = if replay.legacy {
                    None
                } else {
                    let metadata =
                        crate::json::parse_json(&replay.metadata).map_err(|e| e.to_string())?;
                    metadata
                        .get("experiment")
                        .map(crate::config::config_from_snapshot)
                        .transpose()?
                };
                crate::experiments::jobs::check_cancelled()?;
                Ok((replay, config))
            }));
        }
        fn start_auxiliary(
            &mut self,
            f: impl FnOnce() -> Result<(String, Option<ComparisonMetrics>), String> + Send + 'static,
        ) {
            if self.auxiliary_job.is_some() {
                self.set_status("Uma operação já está em andamento; aguarde ou cancele.");
                return;
            }
            self.auxiliary_job = Some(crate::experiments::jobs::BackgroundJob::spawn(f));
            self.set_status("Executando em segundo plano...");
        }
        fn export_current_replay_to_csv(&mut self) {
            let input = PathBuf::from(self.replay_path_text.trim());
            let output = input.with_extension("csv");
            self.start_auxiliary(move || {
                let rows = export_replay_to_csv(&input, &output).map_err(|e| e.to_string())?;
                Ok((
                    format!("Exportadas {rows} amostras para {}", output.display()),
                    None,
                ))
            });
        }
        fn import_real_log_ui(&mut self) {
            let input = PathBuf::from(self.real_log_path_text.trim());
            let output = PathBuf::from(self.calibration_csv_path_text.trim())
                .with_file_name("real_normalized_v05.csv");
            self.start_auxiliary(move || {
                let log = import_real_log(&input)?;
                crate::experiments::jobs::write_output_atomic(&output, |p| {
                    write_normalized_real_log(&log, p).map_err(|e| e.to_string())
                })?;
                Ok((
                    format!(
                        "Importadas {} amostras em {}",
                        log.samples.len(),
                        output.display()
                    ),
                    None,
                ))
            });
        }
        fn compare_real_log_ui(&mut self) {
            let Some(cfg) = self.cfg.clone() else {
                self.set_status("Carregue um projeto.");
                return;
            };
            let input = PathBuf::from(self.real_log_path_text.trim());
            let csv = PathBuf::from(self.calibration_csv_path_text.trim());
            let output = PathBuf::from(self.calibration_report_path_text.trim());
            self.start_auxiliary(move || {
                let log = import_real_log(&input)?;
                let report = compare_project_with_real(cfg, &log, None)?;
                crate::experiments::jobs::write_output_atomic(&csv, |p| {
                    write_comparison_csv(&report, p).map_err(|e| e.to_string())
                })?;
                crate::experiments::jobs::write_output_atomic(&output, |p| {
                    write_comparison_report(&report, p).map_err(|e| e.to_string())
                })?;
                Ok((
                    format!(
                        "Comparação concluída; erro RMS de trajetória {:.4} m",
                        report.metrics.trajectory_error_m.rms
                    ),
                    Some(report.metrics),
                ))
            });
        }
        fn tune_real_log_ui(&mut self) {
            let Some(cfg) = self.cfg.clone() else {
                self.set_status("Carregue um projeto.");
                return;
            };
            let input = PathBuf::from(self.real_log_path_text.trim());
            let output = PathBuf::from(self.tuning_output_path_text.trim());
            self.start_auxiliary(move || {
                let log = import_real_log(&input)?;
                let report = tune_project_against_real(cfg, &log, None)?;
                crate::experiments::jobs::write_output_atomic(&output, |p| {
                    write_tuning_report(&report, p).map_err(|e| e.to_string())
                })?;
                Ok((
                    format!(
                        "Ajuste concluído; score {:.5} -> {:.5}",
                        report.baseline.score, report.best.metrics.score
                    ),
                    Some(report.best.metrics),
                ))
            });
        }

        fn sidebar(&mut self, ctx: &egui::Context) {
            egui::SidePanel::left("main_navigation")
                .resizable(false)
                .default_width(190.0)
                .show(ctx, |ui| {
                    ui.heading("RTSim v0.09");
                    if (self.auxiliary_job.is_some() || self.replay_job.is_some())
                        && ui.button("Cancelar operação").clicked()
                    {
                        if let Some(j) = &self.auxiliary_job {
                            j.cancel();
                        }
                        if let Some(j) = &self.replay_job {
                            j.cancel();
                        }
                    }
                    ui.separator();
                    nav_button(ui, &mut self.view, AppView::Home, "Home");
                    nav_button(ui, &mut self.view, AppView::TrackEditor, "Track Editor");
                    nav_button(ui, &mut self.view, AppView::RobotEditor, "Robot Editor");
                    nav_button(
                        ui,
                        &mut self.view,
                        AppView::VisualSimulator,
                        "Simulator visual",
                    );
                    nav_button(ui, &mut self.view, AppView::ReplayViewer, "Replay viewer");
                    nav_button(
                        ui,
                        &mut self.view,
                        AppView::CalibrationTools,
                        "Calibração atual",
                    );
                    ui.separator();
                    if ui.button("Save").clicked() {
                        self.save_current_project();
                    }
                    if ui.button("Reload").clicked() {
                        self.load_project_from_path(PathBuf::from(self.project_path_text.clone()));
                    }
                    ui.separator();
                    ui.label("Status");
                    ui.small(self.status.as_str());
                });
        }

        fn show_home(&mut self, ui: &mut egui::Ui) {
            ui.heading("Home");
            ui.label(
                "Ponto central para abrir projetos, salvar configurações e acessar os editores.",
            );
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Projeto .rtsim");
                ui.add(
                    egui::TextEdit::singleline(&mut self.project_path_text)
                        .desired_width(f32::INFINITY),
                );
            });
            ui.horizontal(|ui| {
                if ui.button("Abrir projeto").clicked() {
                    self.load_project_from_path(PathBuf::from(self.project_path_text.clone()));
                }
                if ui.button("Usar exemplo básico").clicked() {
                    self.project_path_text = "examples/basic/projeto.rtsim".to_string();
                    self.load_project_from_path(PathBuf::from(self.project_path_text.clone()));
                }
                if ui.button("Novo projeto em memória").clicked() {
                    self.cfg = Some(default_loaded_config(PathBuf::from("projeto_v05.rtsim")));
                    self.track_history.clear();
                    self.track_history_pending = None;
                    self.robot_history.clear();
                    self.robot_history_pending = None;
                    self.robot_selection = None;
                    self.project_path_text = "projeto_v05.rtsim".to_string();
                    self.track_file_path_text = "track.json".to_string();
                    self.surface_profile_path_text =
                        "examples/profiles/rob_trace_official.json".to_string();
                    self.track_dirty = false;
                    self.surface_profile_dirty = false;
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(
                        "Novo projeto atual criado em memória. Ajuste e salve quando quiser.",
                    );
                }
                if ui.button("Salvar").clicked() {
                    self.save_current_project();
                }
            });

            ui.separator();
            if let Some(cfg) = &self.cfg {
                egui::Grid::new("home_summary_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Projeto");
                        ui.label(cfg.project.name.as_str());
                        ui.end_row();
                        ui.label("Arquivo");
                        ui.label(cfg.project_path.display().to_string());
                        ui.end_row();
                        ui.label("Robô");
                        ui.label(cfg.robot.name.as_str());
                        ui.end_row();
                        ui.label("Pista");
                        ui.label(cfg.track.name.as_str());
                        ui.end_row();
                        ui.label("physics_dt_us");
                        ui.label(cfg.project.time.physics_dt_us.to_string());
                        ui.end_row();
                        ui.label("Duração padrão");
                        ui.label(format!("{:.3} s", cfg.project.duration_s));
                        ui.end_row();
                        ui.label("Sensores");
                        ui.label(format!("{} ADC {} bits", cfg.robot.sensors.len(), 12));
                        ui.end_row();
                        ui.label("Normal/downforce");
                        ui.label(cfg.robot.normal_force.model.as_str());
                        ui.end_row();
                    });

                ui.add_space(12.0);
                ui.horizontal(|ui| {
                    if ui.button("Editar pista").clicked() {
                        self.view = AppView::TrackEditor;
                    }
                    if ui.button("Editar robô").clicked() {
                        self.view = AppView::RobotEditor;
                    }
                    if ui.button("Abrir simulador visual").clicked() {
                        self.view = AppView::VisualSimulator;
                    }
                    if ui.button("Abrir replay viewer").clicked() {
                        self.view = AppView::ReplayViewer;
                    }
                    if ui.button("Abrir calibração atual").clicked() {
                        self.view = AppView::CalibrationTools;
                    }
                });
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Nenhum projeto carregado.",
                );
            }
        }

        fn show_track_editor(&mut self, ui: &mut egui::Ui) {
            let before = self
                .cfg
                .as_ref()
                .map(|c| (c.track.clone(), c.project.start_pose));
            let mut history_action = false;
            ui.horizontal(|ui| {
                if let Some(cfg) = self.cfg.as_mut() {
                    if ui.button("Desfazer pista").clicked() {
                        if let Some(old) = self.track_history_pending.take() {
                            self.track_history.record(old, cfg);
                        }
                        history_action = self.track_history.undo(cfg);
                    }
                    if ui.button("Refazer pista").clicked() {
                        if let Some(old) = self.track_history_pending.take() {
                            self.track_history.record(old, cfg);
                        }
                        history_action = self.track_history.redo(cfg);
                    }
                }
            });
            let mut invalidate_sim = history_action;
            let mut status_to_set: Option<String> = None;
            let mut selected_track_point = self.selected_track_point;
            let mut track_view_zoom = self.track_view_zoom;
            let mut track_view_pan_m = self.track_view_pan_m;
            let mut track_file_path_text = self.track_file_path_text.clone();
            let mut surface_profile_path_text = self.surface_profile_path_text.clone();
            let mut track_file_command = TrackFileCommand::None;
            let mut surface_profile_command = SurfaceProfileCommand::None;
            let mut local_track_dirty = self.track_dirty || history_action;
            let mut local_surface_profile_dirty = self.surface_profile_dirty;

            if let Some(cfg) = self.cfg.as_mut() {
                let preview_track = cfg.track.clone();
                let preview_runtime = cached_track(ui.ctx(), &cfg.track).ok();
                let preview_geometry = preview_runtime.as_ref().and_then(|r| r.geometry());
                let full_size = ui.available_size_before_wrap();
                let total_width = full_size.x;
                let total_height = full_size.y.max(360.0);
                let right_width = 380.0;
                let track_to_panel_gap = 2.0;
                let right_window_margin = 28.0;
                let left_width =
                    (total_width - right_width - track_to_panel_gap - right_window_margin)
                        .max(300.0);

                let mut panel_changes = TrackPanelChanges::default();
                ui.allocate_ui_with_layout(
                    egui::vec2(total_width, total_height),
                    egui::Layout::left_to_right(egui::Align::Min),
                    |ui| {
                        ui.allocate_ui_with_layout(
                            egui::vec2(left_width, total_height),
                            egui::Layout::top_down(egui::Align::Min),
                            |ui| {
                                ui.set_min_size(egui::vec2(left_width, total_height));
                                ui.set_max_width(left_width);
                                ui.horizontal(|ui| {
                                    ui.heading("Editor de pista");
                                    ui.add_space(8.0);
                                    ui.label("canvas + grid + marcações");
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            if ui.small_button("Fit").clicked() {
                                                track_view_zoom = 1.0;
                                                track_view_pan_m = Vec2::new(0.0, 0.0);
                                            }
                                            ui.label(format!(
                                                "Zoom {:.0}%",
                                                track_view_zoom * 100.0
                                            ));
                                            if ui.small_button("+").clicked() {
                                                track_view_zoom =
                                                    (track_view_zoom * 1.20).clamp(0.25, 12.0);
                                            }
                                            if ui.small_button("−").clicked() {
                                                track_view_zoom =
                                                    (track_view_zoom / 1.20).clamp(0.25, 12.0);
                                            }
                                        },
                                    );
                                });
                                if let Some(geometry) = &preview_geometry {
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(format!(
                                            "Comprimento total: {:.3} m",
                                            geometry.total_length_mm / 1000.0
                                        ));
                                        let err = geometry.closure_error;
                                        let closed = if let Some(track) = &preview_track.parametric
                                        {
                                            err.distance_mm <= track.closure.position_tolerance_mm
                                                && err.heading_error_deg.abs()
                                                    <= track.closure.heading_tolerance_deg
                                        } else {
                                            false
                                        };
                                        let color = if closed {
                                            egui::Color32::from_rgb(30, 130, 60)
                                        } else {
                                            egui::Color32::from_rgb(190, 55, 45)
                                        };
                                        ui.colored_label(
                                            color,
                                            format!(
                                                "Fechamento: dx={:.2} mm, dy={:.2} mm, dθ={:.3}°",
                                                err.dx_mm, err.dy_mm, err.heading_error_deg
                                            ),
                                        );
                                    });
                                }

                                let canvas_height = ui.available_height().max(260.0);
                                draw_track_view_with_height_zoomable(
                                    ui,
                                    &preview_track,
                                    None,
                                    &[],
                                    canvas_height,
                                    &mut track_view_zoom,
                                    &mut track_view_pan_m,
                                    None,
                                );
                            },
                        );

                        ui.add_space(track_to_panel_gap);

                        ui.allocate_ui_with_layout(
                            egui::vec2(right_width, total_height),
                            egui::Layout::top_down(egui::Align::Min),
                            |ui| {
                                ui.set_min_size(egui::vec2(right_width, total_height));
                                ui.set_max_width(right_width);
                                egui::ScrollArea::vertical()
                                    .id_source("track_editor_right_panel_scroll")
                                    .max_height(total_height)
                                    .show(ui, |ui| {
                                        let track =
                                            cfg.track.parametric.as_mut().expect("checked above");
                                        panel_changes = edit_track_properties_panel(
                                            ui,
                                            track,
                                            &mut selected_track_point,
                                            &mut status_to_set,
                                            &mut track_file_path_text,
                                            local_track_dirty,
                                            &mut track_file_command,
                                            &mut surface_profile_path_text,
                                            local_surface_profile_dirty,
                                            &mut surface_profile_command,
                                        );
                                        panel_changes.track_changed |= edit_track_environment(
                                            ui,
                                            &mut cfg.track,
                                            &mut cfg.project.start_pose,
                                        );
                                    });
                            },
                        );

                        ui.add_space(right_window_margin);
                    },
                );

                if panel_changes.any() {
                    refresh_track_cache(&mut cfg.track);
                    Self::update_project_start_pose_from_track(cfg);
                    local_track_dirty = true;
                    if panel_changes.surface_changed {
                        local_surface_profile_dirty = true;
                    }
                    invalidate_sim = true;
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Carregue um projeto para editar a pista.",
                );
            }

            if let (Some(before), Some(cfg)) = (before, self.cfg.as_ref()) {
                if !history_action
                    && (track_json(&before.0) != track_json(&cfg.track)
                        || before.1 != cfg.project.start_pose)
                    && self.track_history_pending.is_none()
                {
                    self.track_history_pending = Some(before);
                }
                if !ui.input(|i| i.pointer.any_down()) && !ui.ctx().wants_keyboard_input() {
                    if let Some(old) = self.track_history_pending.take() {
                        self.track_history.record(old, cfg);
                    }
                }
            }
            self.selected_track_point = selected_track_point;
            self.track_view_zoom = track_view_zoom;
            self.track_view_pan_m = track_view_pan_m;
            self.track_file_path_text = track_file_path_text;
            self.surface_profile_path_text = surface_profile_path_text;
            self.track_dirty = local_track_dirty;
            self.surface_profile_dirty = local_surface_profile_dirty;

            if invalidate_sim {
                self.sim_session = None;
                self.last_sim_sample = None;
            }
            if let Some(status) = status_to_set {
                self.set_status(status);
            }

            match track_file_command {
                TrackFileCommand::None => {}
                TrackFileCommand::New => self.create_new_track(),
                TrackFileCommand::Load => self.load_track_asset(),
                TrackFileCommand::Save => self.save_track_asset(false),
                TrackFileCommand::SaveAs => self.save_track_asset(true),
            }

            let profile_before = if surface_profile_command != SurfaceProfileCommand::None {
                self.cfg
                    .as_ref()
                    .map(|c| (c.track.clone(), c.project.start_pose))
            } else {
                None
            };
            match surface_profile_command {
                SurfaceProfileCommand::None => {}
                SurfaceProfileCommand::New => self.create_new_surface_profile(),
                SurfaceProfileCommand::Load => self.load_surface_profile_asset(),
                SurfaceProfileCommand::Save => self.save_surface_profile_asset(false),
                SurfaceProfileCommand::SaveAs => self.save_surface_profile_asset(true),
            }
            if let (Some(before), Some(cfg)) = (profile_before, self.cfg.as_ref()) {
                if let Some(pending) = self.track_history_pending.take() {
                    self.track_history.record(pending, cfg);
                }
                self.track_history.record(before, cfg);
            }
        }

        fn show_robot_editor(&mut self, ui: &mut egui::Ui) {
            let robot_before = self.cfg.as_ref().map(|c| c.robot.clone());
            let mut history_action = false;
            ui.horizontal(|ui| {
                if let Some(cfg) = self.cfg.as_mut() {
                    if ui.button("Desfazer").clicked() {
                        if let Some(before) = self.robot_history_pending.take() {
                            self.robot_history.record(before, &cfg.robot);
                        }
                        history_action = self.robot_history.undo(&mut cfg.robot);
                    }
                    if ui.button("Refazer").clicked() {
                        if let Some(before) = self.robot_history_pending.take() {
                            self.robot_history.record(before, &cfg.robot);
                        }
                        history_action = self.robot_history.redo(&mut cfg.robot);
                    }
                }
            });
            let mut robot_file_command = RobotFileCommand::None;
            let mut component_asset_command: Option<ComponentAssetCommand> = None;
            let mut invalidate_sim = history_action;
            let mut status_to_set: Option<String> = None;
            let mut robot_dirty = self.robot_dirty;
            let mut robot_file_path_text = self.robot_file_path_text.clone();
            let mut motor_left_asset_path_text = self.motor_left_asset_path_text.clone();
            let mut driver_asset_path_text = self.driver_asset_path_text.clone();
            let mut battery_asset_path_text = self.battery_asset_path_text.clone();
            let mut tire_asset_path_text = self.tire_asset_path_text.clone();
            let mut fan_asset_path_text = self.fan_asset_path_text.clone();
            let mut encoder_asset_path_text = self.encoder_asset_path_text.clone();
            let mut gyro_asset_path_text = self.gyro_asset_path_text.clone();
            let mut selected_fan_asset_index = self.selected_fan_asset_index;
            let mut robot_preview_camera = self.robot_preview_camera;

            if let Some(cfg) = self.cfg.as_mut() {
                let full_size = ui.available_size_before_wrap();
                let total_width = full_size.x;
                let total_height = full_size.y.max(360.0);
                let right_width = 380.0;
                let robot_to_panel_gap = 2.0;
                let right_window_margin = 28.0;
                let left_width =
                    (total_width - right_width - robot_to_panel_gap - right_window_margin)
                        .max(300.0);

                let project_path_for_assets = cfg.project_path.clone();

                ui.allocate_ui_with_layout(
                    egui::vec2(total_width, total_height),
                    egui::Layout::left_to_right(egui::Align::Min),
                    |ui| {
                        ui.allocate_ui_with_layout(
                            egui::vec2(left_width, total_height),
                            egui::Layout::top_down(egui::Align::Min),
                            |ui| {
                                ui.set_min_size(egui::vec2(left_width, total_height));
                                ui.set_max_width(left_width);
                                ui.horizontal(|ui| {
                                    ui.heading("Robot Preview");
                                    ui.add_space(8.0);
                                    ui.label("top view, X forward, Y lateral");
                                });
                                let preview_height = ui.available_height().max(320.0);
                                draw_robot_preview(
                                    ui,
                                    &mut cfg.robot,
                                    preview_height,
                                    &mut robot_preview_camera,
                                    &mut self.robot_selection,
                                );
                            },
                        );

                        ui.add_space(robot_to_panel_gap);

                        ui.allocate_ui_with_layout(
                            egui::vec2(right_width, total_height),
                            egui::Layout::top_down(egui::Align::Min),
                            |ui| {
                                ui.set_min_size(egui::vec2(right_width, total_height));
                                ui.set_max_width(right_width);
                                egui::ScrollArea::vertical()
                                    .id_source("robot_editor_right_panel_scroll")
                                    .max_height(total_height)
                                    .show(ui, |ui| {
                                        let assembly_changed = edit_assembly_panel(
                                            ui,
                                            &mut cfg.robot,
                                            &mut self.robot_selection,
                                        );
                                        let changed = edit_robot_properties_panel(
                                            ui,
                                            &mut cfg.robot,
                                            &mut robot_file_path_text,
                                            robot_dirty,
                                            &mut robot_file_command,
                                            &mut motor_left_asset_path_text,
                                            &mut driver_asset_path_text,
                                            &mut battery_asset_path_text,
                                            &mut tire_asset_path_text,
                                            &mut fan_asset_path_text,
                                            &mut encoder_asset_path_text,
                                            &mut gyro_asset_path_text,
                                            &mut selected_fan_asset_index,
                                            Some(project_path_for_assets.as_path()),
                                            &mut component_asset_command,
                                            &mut status_to_set,
                                        );
                                        if changed || assembly_changed {
                                            robot_dirty = true;
                                            invalidate_sim = true;
                                        }
                                    });
                            },
                        );

                        ui.add_space(right_window_margin);
                    },
                );
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Carregue um projeto para editar o robô.",
                );
            }

            if let (Some(before), Some(cfg)) = (robot_before, self.cfg.as_ref()) {
                if !history_action && robot_json(&before) != robot_json(&cfg.robot) {
                    robot_dirty = true;
                    invalidate_sim = true;
                    if self.robot_history_pending.is_none() {
                        self.robot_history_pending = Some(before);
                    }
                }
                if !ui.input(|i| i.pointer.any_down()) && !ui.ctx().wants_keyboard_input() {
                    if let Some(before) = self.robot_history_pending.take() {
                        self.robot_history.record(before, &cfg.robot);
                    }
                }
            }
            if history_action {
                robot_dirty = true;
            }
            self.robot_file_path_text = robot_file_path_text;
            self.motor_left_asset_path_text = motor_left_asset_path_text;
            self.driver_asset_path_text = driver_asset_path_text;
            self.battery_asset_path_text = battery_asset_path_text;
            self.tire_asset_path_text = tire_asset_path_text;
            self.fan_asset_path_text = fan_asset_path_text;
            self.encoder_asset_path_text = encoder_asset_path_text;
            self.gyro_asset_path_text = gyro_asset_path_text;
            self.selected_fan_asset_index = selected_fan_asset_index;
            self.robot_preview_camera = robot_preview_camera;
            self.robot_dirty = robot_dirty;

            if invalidate_sim {
                self.sim_session = None;
                self.last_sim_sample = None;
            }
            if let Some(status) = status_to_set {
                self.set_status(status);
            }

            match robot_file_command {
                RobotFileCommand::None => {}
                RobotFileCommand::New => self.create_new_robot(),
                RobotFileCommand::Load => self.load_robot_asset(),
                RobotFileCommand::Save => self.save_robot_asset(false),
                RobotFileCommand::SaveAs => self.save_robot_asset(true),
            }

            if let Some(command) = component_asset_command {
                self.handle_component_asset_command(command);
            }
        }

        fn create_new_robot(&mut self) {
            if self.robot_dirty {
                self.set_status(
                    "There are unsaved robot changes. Save or discard before creating another robot.",
                );
                return;
            }

            let Some(cfg) = self.cfg.as_mut() else {
                self.set_status("Nenhum projeto carregado para receber um novo robô.");
                return;
            };

            let mut robot = default_robot_config();
            robot.name = "New Robot".to_string();
            cfg.robot = robot;
            cfg.project.robot_path = PathBuf::from("Robots/New Robot.json");
            self.robot_file_path_text = "Robots/New Robot.json".to_string();
            self.selected_fan_asset_index = 0;
            self.robot_dirty = true;
            self.sim_session = None;
            self.last_sim_sample = None;
            self.robot_history.clear();
            self.robot_history_pending = None;
            self.robot_selection = None;
            self.set_status("New robot created.");
        }

        fn load_robot_asset(&mut self) {
            if self.robot_dirty {
                self.set_status(
                    "There are unsaved robot changes. Save or discard before loading another robot.",
                );
                return;
            }

            let raw_path = self.robot_file_path_text.trim();
            if raw_path.is_empty() {
                self.set_status("Robot file path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, raw_path);
            match load_robot_from_file(&path) {
                Ok(robot) => {
                    if let Some(cfg) = self.cfg.as_mut() {
                        cfg.project.robot_path = path_relative_to_project(&cfg.project_path, &path);
                        cfg.robot = robot;
                    } else {
                        let mut cfg = default_loaded_config(PathBuf::from("projeto_v05.rtsim"));
                        cfg.project.robot_path = path.clone();
                        cfg.robot = robot;
                        self.cfg = Some(cfg);
                    }
                    self.robot_file_path_text = Self::path_text(&path);
                    self.robot_dirty = false;
                    self.track_history.clear();
                    self.track_history_pending = None;
                    self.robot_history.clear();
                    self.robot_history_pending = None;
                    self.robot_selection = None;
                    self.selected_fan_asset_index = 0;
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(format!("Robot loaded from {}", Self::path_text(&path)));
                }
                Err(err) => self.set_status(format!("Failed to load robot: {err}")),
            }
        }

        fn save_robot_asset(&mut self, save_as: bool) {
            let raw_path = self.robot_file_path_text.trim();
            if raw_path.is_empty() {
                self.set_status("Robot file path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, raw_path);
            let result = self
                .cfg
                .as_ref()
                .ok_or_else(|| "nenhum robô carregado".to_string())
                .and_then(|cfg| save_robot_to_file(&cfg.robot, &path));

            match result {
                Ok(()) => {
                    if let Some(cfg) = self.cfg.as_mut() {
                        cfg.project.robot_path = path_relative_to_project(&cfg.project_path, &path);
                    }
                    self.robot_file_path_text = Self::path_text(&path);
                    self.robot_dirty = false;
                    if save_as {
                        self.set_status(format!("Robot saved as {}", Self::path_text(&path)));
                    } else {
                        self.set_status(format!("Robot saved to {}", Self::path_text(&path)));
                    }
                }
                Err(err) => self.set_status(format!("Failed to save robot: {err}")),
            }
        }

        fn handle_component_asset_command(&mut self, command: ComponentAssetCommand) {
            if let (Some(before), Some(cfg)) =
                (self.robot_history_pending.take(), self.cfg.as_ref())
            {
                self.robot_history.record(before, &cfg.robot);
            }
            let before = self.cfg.as_ref().map(|c| c.robot.clone());
            match command.command {
                ComponentAssetCommandKind::New => self.create_new_component_asset(command.kind),
                ComponentAssetCommandKind::Load => self.load_component_asset(command.kind),
                ComponentAssetCommandKind::Save => self.save_component_asset(command.kind, false),
                ComponentAssetCommandKind::SaveAs => self.save_component_asset(command.kind, true),
            }
            if let (Some(before), Some(cfg)) = (before, self.cfg.as_ref()) {
                self.robot_history.record(before, &cfg.robot);
            }
        }

        fn component_asset_path_text_mut(&mut self, kind: ComponentAssetKind) -> &mut String {
            match kind {
                ComponentAssetKind::MotorLeft => &mut self.motor_left_asset_path_text,
                ComponentAssetKind::Driver => &mut self.driver_asset_path_text,
                ComponentAssetKind::Battery => &mut self.battery_asset_path_text,
                ComponentAssetKind::Tire => &mut self.tire_asset_path_text,
                ComponentAssetKind::Fan => &mut self.fan_asset_path_text,
                ComponentAssetKind::Encoder => &mut self.encoder_asset_path_text,
                ComponentAssetKind::Gyro => &mut self.gyro_asset_path_text,
            }
        }

        fn create_new_component_asset(&mut self, kind: ComponentAssetKind) {
            let result: Result<String, String> = (|| {
                let Some(cfg) = self.cfg.as_mut() else {
                    return Err("nenhum robô carregado para aplicar componente".to_string());
                };
                match kind {
                    ComponentAssetKind::MotorLeft => {
                        cfg.robot.motor_left = default_motor();
                    }
                    ComponentAssetKind::Driver => {
                        cfg.robot.driver = DriverConfig {
                            model: "PwmHBridge".to_string(),
                            pwm_frequency_hz: 20_000.0,
                            mode: "brake".to_string(),
                            voltage_drop_v: 0.2,
                            pwm_resolution_bits: 10,
                            command_deadband: 0.001,
                            current_limit_a: 3.0,
                        };
                    }
                    ComponentAssetKind::Battery => {
                        cfg.robot.battery = BatteryConfig {
                            model: "VoltageSagBattery".to_string(),
                            cells: 2,
                            nominal_voltage_v: 7.4,
                            full_voltage_v: 7.4,
                            empty_voltage_v: 6.4,
                            capacity_mah: 300.0,
                            internal_resistance_ohm: 0.08,
                            initial_soc: 1.0,
                            current_limit_a: 60.0,
                        };
                    }
                    ComponentAssetKind::Tire => {
                        cfg.robot.tire = TireConfig {
                            model: "SlipRatioWheel".to_string(),
                            mu_longitudinal: 1.2,
                            mu_lateral: 1.0,
                            rolling_resistance: 0.015,
                            slip_velocity_epsilon_m_s: 0.05,
                        };
                    }
                    ComponentAssetKind::Encoder => {
                        cfg.robot.encoder = EncoderConfig {
                            model: "QuantizedEncoder".to_string(),
                            ticks_per_rev: 360,
                            invert_left: false,
                            invert_right: false,
                        };
                    }
                    ComponentAssetKind::Gyro => {
                        cfg.robot.gyro = GyroConfig {
                            model: "NoisyGyro".to_string(),
                            noise_std_rad_s: 0.01,
                            bias_rad_s: 0.0,
                            saturation_rad_s: 34.906585,
                            seed: 0x9A17_0002,
                        };
                    }
                    ComponentAssetKind::Fan => {
                        let fan = default_fan_config(cfg.robot.battery.nominal_voltage_v);
                        if cfg.robot.normal_force.fans.is_empty() {
                            cfg.robot.normal_force.fans.push(fan);
                        } else {
                            for instance in &mut cfg.robot.normal_force.fans {
                                let position = instance.position_m;
                                let id = instance.id.clone();
                                *instance = fan.clone();
                                instance.position_m = position;
                                instance.id = id;
                            }
                        }
                        cfg.robot.normal_force.model = crate::io::models::NormalForceKind::Fan;
                    }
                }
                Ok("New default component asset created in the editor. Use Save or Save As to persist it.".to_string())
            })();
            match result {
                Ok(status) => {
                    self.robot_dirty = true;
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(status);
                }
                Err(err) => self.set_status(format!("Failed to create component: {err}")),
            }
        }

        fn load_component_asset(&mut self, kind: ComponentAssetKind) {
            let raw_path = self.component_asset_path_text_mut(kind).trim().to_string();
            if raw_path.is_empty() {
                self.set_status("Component asset path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, &raw_path);
            let selected_fan_index = self.selected_fan_asset_index;
            let mut new_selected_fan_index = selected_fan_index;
            let result: Result<String, String> = (|| {
                let Some(cfg) = self.cfg.as_mut() else {
                    return Err("nenhum robô carregado para aplicar componente".to_string());
                };
                match kind {
                    ComponentAssetKind::MotorLeft => {
                        cfg.robot.motor_left = load_motor_profile_from_file(&path)?.motor;
                    }
                    ComponentAssetKind::Driver => {
                        cfg.robot.driver = load_driver_profile_from_file(&path)?.driver;
                    }
                    ComponentAssetKind::Battery => {
                        cfg.robot.battery = load_battery_profile_from_file(&path)?.battery;
                    }
                    ComponentAssetKind::Tire => {
                        cfg.robot.tire = load_tire_profile_from_file(&path)?.tire;
                    }
                    ComponentAssetKind::Encoder => {
                        cfg.robot.encoder = load_encoder_profile_from_file(&path)?.encoder;
                    }
                    ComponentAssetKind::Gyro => {
                        cfg.robot.gyro = load_gyro_profile_from_file(&path)?.gyro;
                    }
                    ComponentAssetKind::Fan => {
                        let fan = load_fan_profile_from_file(&path)?.fan;
                        if cfg.robot.normal_force.fans.is_empty() {
                            cfg.robot.normal_force.fans.push(fan.clone());
                            new_selected_fan_index = 0;
                        } else {
                            for instance in &mut cfg.robot.normal_force.fans {
                                let position = instance.position_m;
                                let id = instance.id.clone();
                                *instance = fan.clone();
                                instance.position_m = position;
                                instance.id = id;
                            }
                        }
                        cfg.robot.normal_force.model = crate::io::models::NormalForceKind::Fan;
                    }
                }
                Ok(format!("Component loaded from {}", Self::path_text(&path)))
            })();

            match result {
                Ok(status) => {
                    *self.component_asset_path_text_mut(kind) = Self::path_text(&path);
                    self.selected_fan_asset_index = new_selected_fan_index;
                    self.robot_dirty = true;
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(status);
                }
                Err(err) => self.set_status(format!("Failed to load component: {err}")),
            }
        }

        fn save_component_asset(&mut self, kind: ComponentAssetKind, save_as: bool) {
            let raw_path = self.component_asset_path_text_mut(kind).trim().to_string();
            if raw_path.is_empty() {
                self.set_status("Component asset path is empty.");
                return;
            }
            let project_path = self.cfg.as_ref().map(|cfg| cfg.project_path.as_path());
            let path = resolve_asset_path_text(project_path, &raw_path);
            let selected_fan_index = self.selected_fan_asset_index;
            let result: Result<(), String> = (|| {
                let cfg = self
                    .cfg
                    .as_ref()
                    .ok_or_else(|| "nenhum robô carregado".to_string())?;
                match kind {
                    ComponentAssetKind::MotorLeft => save_motor_profile_to_file(
                        &MotorProfile {
                            schema: "rtsim-motor-profile-v1".to_string(),
                            name: cfg.robot.motor_left.model.clone(),
                            motor: cfg.robot.motor_left.clone(),
                        },
                        &path,
                    ),
                    ComponentAssetKind::Driver => save_driver_profile_to_file(
                        &DriverProfile {
                            schema: "rtsim-driver-profile-v1".to_string(),
                            name: cfg.robot.driver.model.clone(),
                            driver: cfg.robot.driver.clone(),
                        },
                        &path,
                    ),
                    ComponentAssetKind::Battery => save_battery_profile_to_file(
                        &BatteryProfile {
                            schema: "rtsim-battery-profile-v1".to_string(),
                            name: cfg.robot.battery.model.clone(),
                            battery: cfg.robot.battery.clone(),
                        },
                        &path,
                    ),
                    ComponentAssetKind::Tire => save_tire_profile_to_file(
                        &TireProfile {
                            schema: "rtsim-tire-profile-v1".to_string(),
                            name: cfg.robot.tire.model.clone(),
                            tire: cfg.robot.tire.clone(),
                        },
                        &path,
                    ),
                    ComponentAssetKind::Encoder => save_encoder_profile_to_file(
                        &EncoderProfile {
                            schema: "rtsim-encoder-profile-v1".to_string(),
                            name: cfg.robot.encoder.model.clone(),
                            encoder: cfg.robot.encoder.clone(),
                        },
                        &path,
                    ),
                    ComponentAssetKind::Gyro => save_gyro_profile_to_file(
                        &GyroProfile {
                            schema: "rtsim-gyro-profile-v1".to_string(),
                            name: cfg.robot.gyro.model.clone(),
                            gyro: cfg.robot.gyro.clone(),
                        },
                        &path,
                    ),
                    ComponentAssetKind::Fan => {
                        let idx = selected_fan_index;
                        let fan = cfg
                            .robot
                            .normal_force
                            .fans
                            .get(idx)
                            .or_else(|| cfg.robot.normal_force.fans.last())
                            .ok_or_else(|| "nenhum fan disponível para salvar".to_string())?;
                        save_fan_profile_to_file(
                            &FanProfile {
                                schema: "rtsim-fan-profile-v1".to_string(),
                                name: format!("Fan {}", idx),
                                fan: fan.clone(),
                            },
                            &path,
                        )
                    }
                }
            })();

            match result {
                Ok(()) => {
                    *self.component_asset_path_text_mut(kind) = Self::path_text(&path);
                    if save_as {
                        self.set_status(format!("Component saved as {}", Self::path_text(&path)));
                    } else {
                        self.set_status(format!("Component saved to {}", Self::path_text(&path)));
                    }
                }
                Err(err) => self.set_status(format!("Failed to save component: {err}")),
            }
        }

        fn show_visual_simulator(&mut self, ui: &mut egui::Ui, _ctx: &egui::Context) {
            ui.heading("Simulador");
            ui.horizontal(|ui| {
                ui.label("Duração (s)");
                ui.add(egui::DragValue::new(&mut self.sim_duration_s).clamp_range(0.0..=3600.));
                if ui.button("Nova execução").clicked() {
                    self.reset_simulation();
                }
                if ui
                    .button(if self.sim_running {
                        "Pausar cálculo"
                    } else {
                        "Iniciar / retomar"
                    })
                    .clicked()
                {
                    if self.sim_session.as_ref().is_none_or(|w| w.is_finished()) {
                        self.reset_simulation();
                    }
                    if let Some(w) = &self.sim_session {
                        if self.sim_running {
                            w.control.pause();
                        } else {
                            w.control.resume();
                        }
                        self.sim_running = !self.sim_running;
                    }
                }
                if ui.button("Um passo").clicked() {
                    if self.sim_session.is_none() {
                        self.reset_simulation();
                    }
                    if let Some(w) = &self.sim_session {
                        w.control.step(1);
                        self.sim_running = false;
                    }
                }
                if ui.button("Cancelar").clicked() {
                    if let Some(w) = &self.sim_session {
                        w.control.cancel();
                    }
                }
                if ui.button("Calcular e reproduzir").clicked() {
                    self.run_headless_replay();
                }
            });
            if let Some(w) = &self.sim_session {
                ui.label(format!(
                    "Execução congelada: {} | frames visuais descartados: {}",
                    w.run_id,
                    w.control.dropped_frames()
                ));
            }
            if let Some(p) = &self.worker_preview {
                ui.add(egui::ProgressBar::new(p.progress as f32).show_percentage());
                ui.label(format!("{} passos | {}", p.steps, p.reason));
                egui::CollapsingHeader::new("Sensores e estimativa").show(ui, |ui| {
                    for r in &p.sensors.channels {
                        ui.label(format!(
                            "{}: ADC {} | válida {} | idade {} µs",
                            r.id, r.adc, r.valid, r.age_us
                        ));
                    }
                    ui.label(format!(
                        "Odometria: {:.3}, {:.3} m | yaw {:.3} rad",
                        p.estimate.pose.x, p.estimate.pose.y, p.estimate.pose.yaw
                    ));
                });
            }
            if let Some(p) = &self.worker_preview {
                egui::CollapsingHeader::new("Contatos e alimentacao").show(ui,|ui|{for (i,w) in p.contacts.wheels.iter().enumerate(){ui.label(format!("Roda {}: N {:.3} N | F longitudinal {:.3} N | lateral {:.3} N | slip {:.3} | {}",i,w.normal_n,w.force_long_n,w.force_lat_n,w.slip,w.regime));}if let Some(power)=&p.power{ui.label(format!("Barramento {:.3} V | {:.3} A | carga {:.1}% | iteracoes {} | residuo {:.6} V",power.voltage,power.current,power.soc*100.,power.iterations,power.residual));}});
            }
            if let Some(cfg) = self
                .sim_session
                .as_ref()
                .map(|w| &w.config)
                .or(self.cfg.as_ref())
            {
                let sample = self.last_sim_sample.as_ref();
                draw_track_view_with_height_zoomable(
                    ui,
                    &cfg.track,
                    sample.map(|s| Pose2::new(s.x_m, s.y_m, s.yaw_rad)),
                    &[],
                    440.,
                    &mut self.track_view_zoom,
                    &mut self.track_view_pan_m,
                    Some(&cfg.robot),
                );
                if let Some(s) = sample {
                    telemetry_panel(ui, s);
                }
            }
        }
        fn show_replay_viewer(&mut self, ui: &mut egui::Ui) {
            ui.heading("Reprodução do cálculo");
            ui.text_edit_singleline(&mut self.replay_path_text);
            ui.horizontal(|ui| {
                ui.label("Cache (KiB)");
                ui.add(egui::DragValue::new(&mut self.replay_max_samples).clamp_range(128..=16384));
                if ui.button("Abrir").clicked() {
                    self.load_replay();
                }
                if ui.button("Exportar CSV").clicked() {
                    self.export_current_replay_to_csv();
                }
                if ui.button("Calcular e abrir").clicked() {
                    self.run_headless_replay();
                }
            });
            if let Some(r) = &mut self.replay {
                ui.horizontal(|ui| {
                    if ui
                        .button(if self.replay_playing {
                            "Pausar reprodução"
                        } else {
                            "Reproduzir"
                        })
                        .clicked()
                    {
                        self.replay_playing = !self.replay_playing;
                    }
                    ui.label("Velocidade");
                    ui.add(egui::DragValue::new(&mut self.replay_speed).clamp_range(0.01..=100.));
                });
                let duration = r.duration_us as f64 * 1e-6;
                if self.replay_playing {
                    self.replay_time_s = (self.replay_time_s
                        + ui.input(|i| i.stable_dt) as f64 * self.replay_speed)
                        .min(duration);
                    ui.ctx().request_repaint();
                    if self.replay_time_s >= duration {
                        self.replay_playing = false;
                    }
                }
                ui.add(
                    egui::Slider::new(&mut self.replay_time_s, 0.0..=duration).text("tempo (s)"),
                );
                ui.label(format!(
                    "{} amostras | {} | cache {} bytes",
                    r.samples,
                    r.termination,
                    r.cached_bytes()
                ));
                match r.at_time((self.replay_time_s * 1e6).round() as u64, true) {
                    Ok(sample) => {
                        if let Some(cfg) = &self.replay_config {
                            draw_track_view(
                                ui,
                                &cfg.track,
                                Some(Pose2::new(sample.x_m, sample.y_m, sample.yaw_rad)),
                                &[],
                            );
                        } else {
                            draw_replay_path_only(ui, std::slice::from_ref(&sample), Some(&sample));
                        }
                        telemetry_panel(ui, &sample);
                    }
                    Err(e) => {
                        self.replay_playing = false;
                        ui.colored_label(egui::Color32::RED, e.to_string());
                    }
                }
            }
        }

        fn show_qualification_tools(&mut self, ui: &mut egui::Ui) {
            ui.group(|ui|{
                ui.heading("Estudo versionado e validação independente");
                ui.label("Manifesto JSON");ui.text_edit_singleline(&mut self.study_path);
                ui.label("Relatório JSON");ui.text_edit_singleline(&mut self.study_report_path);
                if ui.button("Executar estudo").clicked(){
                    let source=PathBuf::from(self.study_path.trim());let output=PathBuf::from(self.study_report_path.trim());
                    self.start_auxiliary(move||{
                        let report=crate::experiments::calibration::qualify(&source,&output)?;
                        let status=report.get("status").and_then(crate::json::JsonValue::as_str).unwrap_or("unknown");
                        let label=match status {"synthetic_or_unknown_not_physically_qualified"=>"Verificação concluída; dados sintéticos ou sem proveniência não qualificam a física", "measured_criteria_passed_review_required"=>"Critérios atendidos; revisão experimental necessária", "identifiability_screen_failed"=>"Dados insuficientes para distinguir os parâmetros", _=>"Critérios de validação não atendidos"};
                        Ok((format!("{label}. Relatório: {}",output.display()),None))
                    });
                }
                if ui.button("Avaliar passos, seeds e perturbações").clicked(){
                    if let Some(config)=self.cfg.clone(){
                        let output=PathBuf::from(self.study_report_path.trim());
                        self.start_auxiliary(move||{
                            crate::experiments::robustness::run_config(config,&output,100000)?;
                            Ok((format!("Robustez numérica salva em {}",output.display()),None))
                        });
                    }
                }
                ui.label("Dados sintéticos verificam o software. A qualificação física exige medições com proveniência.");
                ui.label("As ferramentas legadas abaixo não substituem o estudo versionado.");
            });
        }

        fn show_calibration_tools(&mut self, ui: &mut egui::Ui) {
            ui.heading("Calibração atual — comparação com dados reais");
            self.show_qualification_tools(ui);
            ui.label("Importe um CSV de log real, alinhe pelo tempo e compare trajetória, sensores e velocidade contra a simulação determinística do projeto carregado.");
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Log real CSV");
                ui.add(
                    egui::TextEdit::singleline(&mut self.real_log_path_text)
                        .desired_width(f32::INFINITY),
                );
            });
            ui.horizontal(|ui| {
                ui.label("CSV comparação");
                ui.add(
                    egui::TextEdit::singleline(&mut self.calibration_csv_path_text)
                        .desired_width(f32::INFINITY),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Relatório TXT");
                ui.add(
                    egui::TextEdit::singleline(&mut self.calibration_report_path_text)
                        .desired_width(f32::INFINITY),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Ajuste JSON");
                ui.add(
                    egui::TextEdit::singleline(&mut self.tuning_output_path_text)
                        .desired_width(f32::INFINITY),
                );
            });

            ui.horizontal(|ui| {
                if ui.button("Importar/normalizar log real").clicked() {
                    self.import_real_log_ui();
                }
                if ui.button("Comparar simulação vs real").clicked() {
                    self.compare_real_log_ui();
                }
                if ui.button("Ajustar parâmetros").clicked() {
                    self.tune_real_log_ui();
                }
            });

            ui.separator();
            if let Some(metrics) = &self.last_calibration_metrics {
                ui.strong("Últimas métricas");
                egui::Grid::new("calibration_metrics_grid")
                    .num_columns(2)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Amostras alinhadas");
                        ui.label(metrics.aligned_samples.to_string());
                        ui.end_row();
                        ui.label("Erro trajetória RMS");
                        ui.label(format!("{:.6} m", metrics.trajectory_error_m.rms));
                        ui.end_row();
                        ui.label("Erro yaw RMS");
                        ui.label(format!("{:.6} rad", metrics.yaw_error_rad.rms));
                        ui.end_row();
                        ui.label("Erro velocidade RMS");
                        ui.label(format!("{:.6} m/s", metrics.speed_error_m_s.rms));
                        ui.end_row();
                        ui.label("Erro sensores RMS");
                        ui.label(format!("{:.3} ADC", metrics.sensor_error_adc.rms));
                        ui.end_row();
                        ui.label("Erro linha RMS");
                        ui.label(format!("{:.6} m", metrics.line_error_m.rms));
                        ui.end_row();
                        ui.label("Score");
                        ui.label(format!("{:.9}", metrics.score));
                        ui.end_row();
                    });
            } else {
                ui.label("Nenhuma comparação executada ainda.");
            }
        }
    }

    fn edit_track_properties_panel(
        ui: &mut egui::Ui,
        track: &mut TrackV2,
        selected_segment: &mut Option<usize>,
        status_to_set: &mut Option<String>,
        track_file_path_text: &mut String,
        track_dirty: bool,
        track_file_command: &mut TrackFileCommand,
        surface_profile_path_text: &mut String,
        surface_profile_dirty: bool,
        surface_profile_command: &mut SurfaceProfileCommand,
    ) -> TrackPanelChanges {
        let mut changes = TrackPanelChanges::default();
        let panel_width = ui.available_width();

        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                ui.set_width(panel_width);
                ui.horizontal(|ui| {
                    if track_dirty {
                        ui.strong("Track File *modified");
                    } else {
                        ui.strong("Track File");
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add_sized([70.0, 22.0], egui::Button::new("Save As"))
                            .clicked()
                        {
                            let tracks_dir = std::env::current_dir()
                                .unwrap_or_else(|_| std::path::PathBuf::from("."))
                                .join("Tracks");
                            if let Some(path) = rfd::FileDialog::new()
                                .set_title("Save Track JSON")
                                .set_file_name(json_file_name_from_name(&track.name, "track"))
                                .add_filter("JSON", &["json"])
                                .set_directory(&tracks_dir)
                                .save_file()
                            {
                                *track_file_path_text = path.to_string_lossy().replace('\\', "/");
                                *track_file_command = TrackFileCommand::SaveAs;
                            }
                        }
                        if ui
                            .add_sized([52.0, 22.0], egui::Button::new("Save"))
                            .clicked()
                        {
                            *track_file_command = TrackFileCommand::Save;
                        }
                        if ui
                            .add_sized([52.0, 22.0], egui::Button::new("Load"))
                            .clicked()
                        {
                            if let Some(path) = rfd::FileDialog::new()
                                .set_title("Load Track JSON")
                                .set_directory(ensure_asset_dir("Tracks"))
                                .add_filter("JSON", &["json"])
                                .pick_file()
                            {
                                *track_file_path_text = path.to_string_lossy().replace('\\', "/");
                                *track_file_command = TrackFileCommand::Load;
                            }
                        }
                        if ui
                            .add_sized([48.0, 22.0], egui::Button::new("New"))
                            .clicked()
                        {
                            *track_file_command = TrackFileCommand::New;
                        }
                    });
                });
                if track_dirty {
                    ui.small(
                        egui::RichText::new("Unsaved track changes")
                            .color(egui::Color32::from_rgb(190, 130, 30)),
                    );
                }
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.add_sized([42.0, 22.0], egui::Label::new("Path"));
                    ui.add_sized(
                        [ui.available_width(), 22.0],
                        egui::TextEdit::singleline(track_file_path_text),
                    );
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.add_sized([42.0, 22.0], egui::Label::new("Name"));

                    if ui
                        .add_sized(
                            [ui.available_width(), 22.0],
                            egui::TextEdit::singleline(&mut track.name),
                        )
                        .changed()
                    {
                        changes.track_changed = true;
                    }
                });
            });

        ui.add_space(8.0);
        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                ui.set_width(panel_width);
                ui.strong("Geometry");
                ui.add_space(6.0);
                ui.columns(2, |columns| {
                    let (left, right) = columns.split_at_mut(1);
                    let area_ui = &mut left[0];
                    let origin_ui = &mut right[0];

                    area_ui.vertical(|ui| {
                        ui.label(egui::RichText::new("Area").small().strong());
                        ui.horizontal(|ui| {
                            if compact_drag_value_labeled(
                                ui,
                                "Width [mm]",
                                &mut track.area.width_mm,
                                10.0,
                                100.0..=100_000.0,
                                58.0,
                            ) {
                                changes.track_changed = true;
                            }
                            if compact_drag_value_labeled(
                                ui,
                                "Height [mm]",
                                &mut track.area.height_mm,
                                10.0,
                                100.0..=100_000.0,
                                58.0,
                            ) {
                                changes.track_changed = true;
                            }
                            if compact_drag_value_labeled(
                                ui,
                                "Grid [mm]",
                                &mut track.area.grid_mm,
                                10.0,
                                1.0..=1000.0,
                                52.0,
                            ) {
                                changes.track_changed = true;
                            }
                        });
                    });

                    origin_ui.vertical(|ui| {
                        ui.label(egui::RichText::new("Origin").small().strong());
                        ui.horizontal(|ui| {
                            if compact_drag_value_labeled(
                                ui,
                                "X [mm]",
                                &mut track.origin.x_mm,
                                1.0,
                                -100_000.0..=100_000.0,
                                58.0,
                            ) {
                                changes.track_changed = true;
                            }
                            if compact_drag_value_labeled(
                                ui,
                                "Y [mm]",
                                &mut track.origin.y_mm,
                                1.0,
                                -100_000.0..=100_000.0,
                                58.0,
                            ) {
                                changes.track_changed = true;
                            }
                            if compact_drag_value_labeled(
                                ui,
                                "Angle [deg]",
                                &mut track.origin.heading_deg,
                                0.5,
                                -360.0..=360.0,
                                52.0,
                            ) {
                                changes.track_changed = true;
                            }
                        });
                    });
                });
            });

        ui.add_space(8.0);
        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                ui.set_width(panel_width);
                ui.horizontal(|ui| {
                    ui.strong("Surface Rules");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add_sized([70.0, 22.0], egui::Button::new("Save As"))
                            .clicked()
                        {
                            let surface_dir = std::env::current_dir()
                                .unwrap_or_else(|_| std::path::PathBuf::from("."))
                                .join("SurfaceProfiles");
                            if let Some(path) = rfd::FileDialog::new()
                                .set_title("Save  JSON")
                                .set_file_name(json_file_name_from_name(
                                    &track.rules.profile,
                                    "surface_profile",
                                ))
                                .add_filter("JSON", &["json"])
                                .set_directory(&surface_dir)
                                .save_file()
                            {
                                *surface_profile_path_text =
                                    path.to_string_lossy().replace('\\', "/");
                                *surface_profile_command = SurfaceProfileCommand::SaveAs;
                            }
                        }
                        if ui
                            .add_sized([52.0, 22.0], egui::Button::new("Save"))
                            .clicked()
                        {
                            *surface_profile_command = SurfaceProfileCommand::Save;
                        }
                        if ui
                            .add_sized([52.0, 22.0], egui::Button::new("Load"))
                            .clicked()
                        {
                            if let Some(path) = rfd::FileDialog::new()
                                .set_title("Load Surface Profile JSON")
                                .set_directory(ensure_asset_dir("SurfaceProfiles"))
                                .add_filter("JSON", &["json"])
                                .pick_file()
                            {
                                *surface_profile_path_text =
                                    path.to_string_lossy().replace('\\', "/");
                                *surface_profile_command = SurfaceProfileCommand::Load;
                            }
                        }
                        if ui
                            .add_sized([48.0, 22.0], egui::Button::new("New"))
                            .clicked()
                        {
                            *surface_profile_command = SurfaceProfileCommand::New;
                        }
                    });
                });
                if surface_profile_dirty {
                    ui.small(
                        egui::RichText::new("Unsaved surface profile changes")
                            .color(egui::Color32::from_rgb(190, 130, 30)),
                    );
                }
                ui.add_space(6.0);

                ui.horizontal(|ui| {
                    ui.add_sized([42.0, 22.0], egui::Label::new("Path"));

                    ui.add_sized(
                        [ui.available_width(), 22.0],
                        egui::TextEdit::singleline(surface_profile_path_text),
                    );
                });

                ui.add_space(4.0);

                ui.horizontal(|ui| {
                    ui.add_sized([42.0, 22.0], egui::Label::new("Name"));

                    if ui
                        .add_sized(
                            [ui.available_width(), 22.0],
                            egui::TextEdit::singleline(&mut track.rules.profile),
                        )
                        .changed()
                    {
                        changes.surface_changed = true;
                    }
                });

                ui.add_space(8.0);
                let official = 19.0;
                let mut line_width = track.rules.overrides.line_width_mm.unwrap_or(official);
                let before_line_width = line_width;
                let mut default_values_clicked = false;
                const SURFACE_FIELD_GAP: f32 = 4.0;

                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.small("Marker rules");
                        let mut mode = track.rules.mode;
                        egui::ComboBox::from_id_source("track_rules_mode_right_panel")
                            .width(70.0)
                            .selected_text(mode.as_str())
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut mode, TrackRulesMode::Strict, "strict");
                                ui.selectable_value(&mut mode, TrackRulesMode::Warning, "warning");
                                ui.selectable_value(&mut mode, TrackRulesMode::Free, "free");
                            });
                        if mode != track.rules.mode {
                            track.rules.mode = mode;
                            changes.surface_changed = true;
                        }
                    });

                    if compact_drag_value_labeled(
                        ui,
                        "Line Width",
                        &mut line_width,
                        0.1,
                        1.0..=100.0,
                        58.0,
                    ) {
                        changes.surface_changed = true;
                    }

                    ui.add_space(SURFACE_FIELD_GAP);

                    if compact_drag_value_labeled(
                        ui,
                        "Background Reflec.",
                        &mut track.surface.base_reflectance,
                        0.01,
                        0.0..=1.0,
                        58.0,
                    ) {
                        changes.surface_changed = true;
                    }

                    ui.add_space(SURFACE_FIELD_GAP);

                    if compact_drag_value_labeled(
                        ui,
                        "Line Reflec.",
                        &mut track.surface.line_reflectance,
                        0.01,
                        0.0..=1.0,
                        58.0,
                    ) {
                        changes.surface_changed = true;
                    }

                    ui.add_space(SURFACE_FIELD_GAP);
                    ui.vertical(|ui| {
                        ui.small("");
                        if ui
                            .add_sized([72.0, 22.0], egui::Button::new("Default"))
                            .clicked()
                        {
                            default_values_clicked = true;
                        }
                    });
                });

                if default_values_clicked {
                    track.rules.profile = "robotrace official".to_string();
                    track.rules.mode = TrackRulesMode::Warning;
                    track.rules.overrides = Default::default();
                    track.surface.base_color = "black".to_string();
                    track.surface.line_color = "white".to_string();
                    track.surface.base_reflectance = 0.08;
                    track.surface.line_reflectance = 0.86;
                    track.surface.surface_mu = 1.20;
                    track.markings.start_finish.distance_mm = 1000.0;
                    track.markings.start_finish.margin_mm = 100.0;
                    changes.track_changed = true;
                    changes.surface_changed = true;
                } else if (line_width - before_line_width).abs() > f64::EPSILON {
                    track.rules.overrides.line_width_mm = Some(line_width);
                    changes.surface_changed = true;
                }
            });

        ui.add_space(8.0);
        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                ui.set_width(panel_width);
                ui.strong("Segments");
                ui.add_space(6.0);

                if track.segments.is_empty() {
                    *selected_segment = None;
                } else if selected_segment
                    .map(|idx| idx >= track.segments.len())
                    .unwrap_or(true)
                {
                    *selected_segment = Some(0);
                }

                let column_gap = 8.0;
                let available_width = ui.available_width();
                let list_width = 216.0;
                let command_width = (available_width - list_width - column_gap).max(160.0);

                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.set_width(command_width);

                        if track.segments.is_empty() {
                            ui.colored_label(
                                egui::Color32::from_rgb(190, 130, 30),
                                "No segment selected.",
                            );
                        } else if edit_selected_segment(ui, track, selected_segment, status_to_set)
                        {
                            changes.track_changed = true;
                        }

                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);
                        ui.label(egui::RichText::new("Add").small().strong());

                        let add_button_w = ((ui.available_width() - 8.0) / 3.0).max(54.0);
                        ui.horizontal(|ui| {
                            if ui
                                .add_sized([add_button_w, 22.0], egui::Button::new("Straight"))
                                .clicked()
                            {
                                let id = next_segment_id(&track.segments, "R");
                                track.segments.push(TrackSegment::Straight(StraightSegment {
                                    id,
                                    length_mm: 300.0,
                                }));
                                *selected_segment = Some(track.segments.len() - 1);
                                changes.track_changed = true;
                            }
                            if ui
                                .add_sized([add_button_w, 22.0], egui::Button::new("Left Arc"))
                                .clicked()
                            {
                                let id = next_segment_id(&track.segments, "C");
                                track.segments.push(TrackSegment::Arc(ArcSegment {
                                    id,
                                    radius_mm: 300.0,
                                    sweep_deg: 90.0,
                                }));
                                *selected_segment = Some(track.segments.len() - 1);
                                changes.track_changed = true;
                            }
                            if ui
                                .add_sized([add_button_w, 22.0], egui::Button::new("Right Arc"))
                                .clicked()
                            {
                                let id = next_segment_id(&track.segments, "C");
                                track.segments.push(TrackSegment::Arc(ArcSegment {
                                    id,
                                    radius_mm: 300.0,
                                    sweep_deg: -90.0,
                                }));
                                *selected_segment = Some(track.segments.len() - 1);
                                changes.track_changed = true;
                            }
                        });

                        if ui
                            .add_sized(
                                [ui.available_width(), 22.0],
                                egui::Button::new("Complete Track"),
                            )
                            .clicked()
                        {
                            match auto_complete_track_with_up_to_two_segments(track) {
                                Ok(message) => {
                                    *selected_segment = track.segments.len().checked_sub(1);
                                    *status_to_set = Some(message);
                                    changes.track_changed = true;
                                }
                                Err(err) => {
                                    *status_to_set =
                                        Some(format!("Complete Track not applied: {err}"));
                                }
                            }
                        }
                    });

                    ui.add_space(column_gap);

                    ui.vertical(|ui| {
                        ui.set_width(list_width);
                        ui.label(egui::RichText::new("List").small().strong());
                        ui.add_space(4.0);

                        if track.segments.is_empty() {
                            ui.colored_label(
                                egui::Color32::from_rgb(190, 130, 30),
                                "No segments yet.",
                            );
                        } else {
                            egui::ScrollArea::vertical()
                                .id_source("track_editor_segments_scroll")
                                .max_height(230.0)
                                .show(ui, |ui| {
                                    for (idx, segment) in track.segments.iter().enumerate() {
                                        let selected = *selected_segment == Some(idx);
                                        if ui
                                            .selectable_label(selected, segment_summary(segment))
                                            .clicked()
                                        {
                                            *selected_segment = Some(idx);
                                        }
                                    }
                                });
                        }
                    });
                });
            });

        ui.add_space(8.0);
        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                ui.set_width(panel_width);
                ui.strong("Start/Finish Marker");
                ui.add_space(6.0);
                if edit_start_finish(ui, track, status_to_set) {
                    changes.track_changed = true;
                }
            });

        ui.add_space(8.0);
        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                ui.set_width(panel_width);
                ui.strong("Validation");
                ui.add_space(6.0);
                let issues = validate_track(track);
                let errors = issues
                    .iter()
                    .filter(|issue| issue.severity == Severity::Error)
                    .count();
                let warnings = issues
                    .iter()
                    .filter(|issue| issue.severity == Severity::Warning)
                    .count();
                ui.label(format!(
                    "{errors} errors, {warnings} warnings, {} information",
                    issues.len().saturating_sub(errors + warnings)
                ));
                egui::ScrollArea::vertical()
                    .id_source("track_editor_validation_scroll")
                    .max_height(150.0)
                    .show(ui, |ui| {
                        if issues.is_empty() {
                            ui.colored_label(
                                egui::Color32::from_rgb(30, 130, 60),
                                "OK — no inconsistencies found.",
                            );
                        } else {
                            for issue in issues {
                                let color = match issue.severity {
                                    Severity::Error => egui::Color32::from_rgb(190, 55, 45),
                                    Severity::Warning => egui::Color32::from_rgb(190, 130, 30),
                                    Severity::Info => egui::Color32::from_rgb(80, 90, 110),
                                };
                                let sev = match issue.severity {
                                    Severity::Error => "Error",
                                    Severity::Warning => "Warning",
                                    Severity::Info => "Info",
                                };
                                let seg = issue
                                    .segment_id
                                    .map(|segment_id| format!(" [{segment_id}]"))
                                    .unwrap_or_default();
                                ui.colored_label(
                                    color,
                                    format!("{sev}{seg} — {}: {}", issue.rule_id, issue.message),
                                );
                            }
                        }
                    });
            });

        changes
    }

    #[derive(Debug, Clone)]
    enum AutoCloseSpec {
        Straight(f64),
        Arc { radius_mm: f64, sweep_deg: f64 },
    }

    #[derive(Debug, Clone)]
    struct AutoClosePlan {
        segments: Vec<TrackSegment>,
        priority: i32,
        straight_mm: f64,
        total_arc_mm: f64,
    }

    fn auto_complete_track_with_up_to_two_segments(track: &mut TrackV2) -> Result<String, String> {
        if track.segments.is_empty() {
            return Err("add at least one segment before completing the track".to_string());
        }

        if track_is_closed(track) {
            return Ok("Track is already closed.".to_string());
        }

        let base_len = track.segments.len();
        let mut best: Option<AutoClosePlan> = None;

        let mut straight_trial = track.clone();
        if auto_close_with_straight(&mut straight_trial).is_ok() && track_is_closed(&straight_trial)
        {
            let added = straight_trial.segments[base_len..].to_vec();
            consider_auto_close_plan(
                &mut best,
                AutoClosePlan {
                    straight_mm: total_straight_mm(&added),
                    total_arc_mm: total_arc_mm(&added),
                    priority: 0,
                    segments: added,
                },
            );
        }

        let geometry = build_geometry(track);
        let final_pose = geometry.final_pose;
        let target = Vec2::new(track.origin.x_mm, track.origin.y_mm);
        let current = Vec2::new(final_pose.x_mm, final_pose.y_mm);
        let delta = target - current;
        let theta = final_pose.heading_deg.to_radians();
        let forward = Vec2::new(theta.cos(), theta.sin());
        let left = Vec2::new(-theta.sin(), theta.cos());
        let delta_forward = delta.dot(forward);
        let delta_left = delta.dot(left);
        let heading_delta = normalize_degrees(track.origin.heading_deg - final_pose.heading_deg);
        let min_arc_radius = resolve_rules(&track.rules).min_arc_radius_mm.max(1.0);
        let min_straight_len = 0.001;

        add_single_arc_candidate(
            track,
            &mut best,
            heading_delta,
            delta_forward,
            delta_left,
            min_arc_radius,
        );
        add_straight_arc_candidate(
            track,
            &mut best,
            heading_delta,
            delta_forward,
            delta_left,
            min_straight_len,
            min_arc_radius,
        );
        add_arc_straight_candidate(
            track,
            &mut best,
            heading_delta,
            delta_forward,
            delta_left,
            min_straight_len,
            min_arc_radius,
        );
        add_two_arc_candidates(
            track,
            &mut best,
            delta,
            final_pose.heading_deg,
            heading_delta,
            min_arc_radius,
        );

        let Some(plan) = best else {
            return Err(
                "could not close with up to two generated segments. Try adding an intermediate straight/arc manually."
                    .to_string(),
            );
        };

        let summary = plan
            .segments
            .iter()
            .map(segment_summary)
            .collect::<Vec<_>>()
            .join(" + ");
        let added_count = plan.segments.len();
        track.segments.extend(plan.segments);
        Ok(format!(
            "Complete Track added {added_count} segment(s): {summary}."
        ))
    }

    fn add_single_arc_candidate(
        track: &TrackV2,
        best: &mut Option<AutoClosePlan>,
        heading_delta: f64,
        delta_forward: f64,
        delta_left: f64,
        min_arc_radius: f64,
    ) {
        if heading_delta.abs() < 1e-6 {
            return;
        }
        let sign = if heading_delta >= 0.0 { 1.0 } else { -1.0 };
        let angle = heading_delta.abs().to_radians();
        let lateral_coeff = sign * (1.0 - angle.cos());
        let forward_coeff = angle.sin();

        let radius = if forward_coeff.abs() > 1e-9 {
            delta_forward / forward_coeff
        } else if lateral_coeff.abs() > 1e-9 && delta_forward.abs() <= 0.5 {
            delta_left / lateral_coeff
        } else {
            return;
        };

        if !valid_arc_radius(radius, min_arc_radius) {
            return;
        }
        if (radius * lateral_coeff - delta_left).abs() > 0.5 {
            return;
        }

        consider_specs(
            track,
            best,
            2,
            vec![AutoCloseSpec::Arc {
                radius_mm: radius,
                sweep_deg: heading_delta,
            }],
        );
    }

    fn add_straight_arc_candidate(
        track: &TrackV2,
        best: &mut Option<AutoClosePlan>,
        heading_delta: f64,
        delta_forward: f64,
        delta_left: f64,
        min_straight_len: f64,
        min_arc_radius: f64,
    ) {
        if heading_delta.abs() < 1e-6 {
            return;
        }
        let sign = if heading_delta >= 0.0 { 1.0 } else { -1.0 };
        let angle = heading_delta.abs().to_radians();
        let lateral_coeff = sign * (1.0 - angle.cos());
        if lateral_coeff.abs() < 1e-9 {
            return;
        }
        let radius = delta_left / lateral_coeff;
        let length = delta_forward - radius * angle.sin();
        if !valid_arc_radius(radius, min_arc_radius) || length < min_straight_len {
            return;
        }

        consider_specs(
            track,
            best,
            1,
            vec![
                AutoCloseSpec::Straight(length),
                AutoCloseSpec::Arc {
                    radius_mm: radius,
                    sweep_deg: heading_delta,
                },
            ],
        );
    }

    fn add_arc_straight_candidate(
        track: &TrackV2,
        best: &mut Option<AutoClosePlan>,
        heading_delta: f64,
        delta_forward: f64,
        delta_left: f64,
        min_straight_len: f64,
        min_arc_radius: f64,
    ) {
        if heading_delta.abs() < 1e-6 {
            return;
        }
        let sign = if heading_delta >= 0.0 { 1.0 } else { -1.0 };
        let phi = heading_delta.to_radians();
        let angle = phi.abs();
        let det = sign * (1.0 - angle.cos());
        if det.abs() < 1e-9 {
            return;
        }

        let radius = (delta_forward * phi.sin() - delta_left * phi.cos()) / det;
        let length = (angle.sin() * delta_left - sign * (1.0 - angle.cos()) * delta_forward) / det;
        if !valid_arc_radius(radius, min_arc_radius) || length < min_straight_len {
            return;
        }

        consider_specs(
            track,
            best,
            1,
            vec![
                AutoCloseSpec::Arc {
                    radius_mm: radius,
                    sweep_deg: heading_delta,
                },
                AutoCloseSpec::Straight(length),
            ],
        );
    }

    fn add_two_arc_candidates(
        track: &TrackV2,
        best: &mut Option<AutoClosePlan>,
        delta: Vec2,
        start_heading_deg: f64,
        heading_delta: f64,
        min_arc_radius: f64,
    ) {
        let step_deg: f64 = 5.0;
        let max_abs_sweep: f64 = 330.0;

        for full_turn in -1..=1 {
            let total_heading_delta = heading_delta + 360.0 * full_turn as f64;
            let mut sweep1: f64 = -max_abs_sweep;

            while sweep1 <= max_abs_sweep {
                if sweep1.abs() < 1.0 {
                    sweep1 += step_deg;
                    continue;
                }

                let sweep2 = total_heading_delta - sweep1;

                if sweep2.abs() < 1.0 || sweep2.abs() > max_abs_sweep {
                    sweep1 += step_deg;
                    continue;
                }

                let v1 = arc_delta_for_radius(start_heading_deg, sweep1, 1.0);
                let v2 = arc_delta_for_radius(start_heading_deg + sweep1, sweep2, 1.0);
                let det = v1.x * v2.y - v1.y * v2.x;
                if det.abs() < 1e-9 {
                    sweep1 += step_deg;
                    continue;
                }

                let radius1 = (delta.x * v2.y - delta.y * v2.x) / det;
                let radius2 = (v1.x * delta.y - v1.y * delta.x) / det;
                if valid_arc_radius(radius1, min_arc_radius)
                    && valid_arc_radius(radius2, min_arc_radius)
                {
                    consider_specs(
                        track,
                        best,
                        3,
                        vec![
                            AutoCloseSpec::Arc {
                                radius_mm: radius1,
                                sweep_deg: sweep1,
                            },
                            AutoCloseSpec::Arc {
                                radius_mm: radius2,
                                sweep_deg: sweep2,
                            },
                        ],
                    );
                }

                sweep1 += step_deg;
            }
        }
    }

    fn consider_specs(
        track: &TrackV2,
        best: &mut Option<AutoClosePlan>,
        priority: i32,
        specs: Vec<AutoCloseSpec>,
    ) {
        let segments = auto_close_segments_with_ids(track, &specs);
        let mut trial = track.clone();
        trial.segments.extend(segments.iter().cloned());
        if !track_is_closed(&trial) {
            return;
        }

        consider_auto_close_plan(
            best,
            AutoClosePlan {
                straight_mm: total_straight_mm(&segments),
                total_arc_mm: total_arc_mm(&segments),
                priority,
                segments,
            },
        );
    }

    fn consider_auto_close_plan(best: &mut Option<AutoClosePlan>, candidate: AutoClosePlan) {
        let replace = match best {
            None => true,
            Some(current) => {
                candidate.priority < current.priority
                    || (candidate.priority == current.priority
                        && candidate.straight_mm > current.straight_mm + 1e-6)
                    || (candidate.priority == current.priority
                        && (candidate.straight_mm - current.straight_mm).abs() <= 1e-6
                        && candidate.segments.len() < current.segments.len())
                    || (candidate.priority == current.priority
                        && (candidate.straight_mm - current.straight_mm).abs() <= 1e-6
                        && candidate.segments.len() == current.segments.len()
                        && candidate.total_arc_mm < current.total_arc_mm)
            }
        };
        if replace {
            *best = Some(candidate);
        }
    }

    fn auto_close_segments_with_ids(track: &TrackV2, specs: &[AutoCloseSpec]) -> Vec<TrackSegment> {
        let mut ids = track.segments.clone();
        let mut segments = Vec::with_capacity(specs.len());
        for spec in specs {
            let segment = match *spec {
                AutoCloseSpec::Straight(length_mm) => TrackSegment::Straight(StraightSegment {
                    id: next_segment_id(&ids, "R"),
                    length_mm: length_mm.max(0.001),
                }),
                AutoCloseSpec::Arc {
                    radius_mm,
                    sweep_deg,
                } => TrackSegment::Arc(ArcSegment {
                    id: next_segment_id(&ids, "C"),
                    radius_mm: radius_mm.abs().max(0.001),
                    sweep_deg,
                }),
            };
            ids.push(segment.clone());
            segments.push(segment);
        }
        segments
    }

    fn track_is_closed(track: &TrackV2) -> bool {
        let closure = build_geometry(track).closure_error;
        closure.distance_mm <= track.closure.position_tolerance_mm.max(0.5)
            && closure.heading_error_deg.abs() <= track.closure.heading_tolerance_deg.max(0.1)
    }

    fn valid_arc_radius(radius_mm: f64, min_arc_radius: f64) -> bool {
        radius_mm.is_finite() && radius_mm >= min_arc_radius
    }

    fn total_straight_mm(segments: &[TrackSegment]) -> f64 {
        segments
            .iter()
            .map(|segment| match segment {
                TrackSegment::Straight(straight) => straight.length_mm.max(0.0),
                TrackSegment::Arc(_) => 0.0,
            })
            .sum()
    }

    fn total_arc_mm(segments: &[TrackSegment]) -> f64 {
        segments
            .iter()
            .map(|segment| match segment {
                TrackSegment::Straight(_) => 0.0,
                TrackSegment::Arc(arc) => arc.radius_mm.max(0.0) * arc.sweep_deg.to_radians().abs(),
            })
            .sum()
    }

    fn arc_delta_for_radius(heading_deg: f64, sweep_deg: f64, radius_mm: f64) -> Vec2 {
        let theta = heading_deg.to_radians();
        let forward = Vec2::new(theta.cos(), theta.sin());
        let left = Vec2::new(-theta.sin(), theta.cos());
        let sign = if sweep_deg >= 0.0 { 1.0 } else { -1.0 };
        let angle = sweep_deg.abs().to_radians();
        forward * (radius_mm * angle.sin()) + left * (sign * radius_mm * (1.0 - angle.cos()))
    }

    fn normalize_degrees(mut deg: f64) -> f64 {
        while deg > 180.0 {
            deg -= 360.0;
        }
        while deg <= -180.0 {
            deg += 360.0;
        }
        deg
    }

    fn edit_selected_segment(
        ui: &mut egui::Ui,
        track: &mut TrackV2,
        selected_segment: &mut Option<usize>,
        status_to_set: &mut Option<String>,
    ) -> bool {
        let Some(idx) = *selected_segment else {
            return false;
        };
        if idx >= track.segments.len() {
            *selected_segment = track.segments.len().checked_sub(1);
            return false;
        }

        let mut changed = false;
        let mut kind = match &track.segments[idx] {
            TrackSegment::Straight(_) => 0,
            TrackSegment::Arc(arc) if arc.sweep_deg >= 0.0 => 1,
            TrackSegment::Arc(_) => 2,
        };
        let current_kind = kind;

        let two_col_w = ((ui.available_width() - 6.0) * 0.5).max(72.0);
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.small("ID");
                if ui
                    .add_sized(
                        [two_col_w, 20.0],
                        egui::TextEdit::singleline(track.segments[idx].id_mut()),
                    )
                    .changed()
                {
                    changed = true;
                }
            });

            ui.add_space(6.0);

            ui.vertical(|ui| {
                ui.small("Type");
                egui::ComboBox::from_id_source("selected_segment_kind")
                    .width(two_col_w)
                    .selected_text(match kind {
                        0 => "Straight",
                        1 => "Left Arc",
                        _ => "Right Arc",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut kind, 0, "Straight");
                        ui.selectable_value(&mut kind, 1, "Left Arc");
                        ui.selectable_value(&mut kind, 2, "Right Arc");
                    });
            });
        });

        if kind != current_kind {
            let id = track.segments[idx].id().to_string();
            let old_len = track.segments[idx].length_mm().max(1.0);
            track.segments[idx] = match kind {
                0 => TrackSegment::Straight(StraightSegment {
                    id,
                    length_mm: old_len,
                }),
                1 => TrackSegment::Arc(ArcSegment {
                    id,
                    radius_mm: 300.0,
                    sweep_deg: 90.0,
                }),
                _ => TrackSegment::Arc(ArcSegment {
                    id,
                    radius_mm: 300.0,
                    sweep_deg: -90.0,
                }),
            };
            changed = true;
        }

        ui.add_space(6.0);
        let field_w = ((ui.available_width() - 12.0) / 3.0).max(52.0);
        match &mut track.segments[idx] {
            TrackSegment::Straight(straight) => {
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.small("Length [mm]");
                        if ui
                            .add_sized(
                                [field_w, 20.0],
                                egui::DragValue::new(&mut straight.length_mm)
                                    .speed(1.0)
                                    .clamp_range(0.001..=100_000.0),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });
                    ui.add_space(6.0);
                    ui.vertical(|ui| {
                        ui.small("Angle [deg]");
                        ui.add_sized([field_w, 20.0], egui::Label::new("—"));
                    });
                    ui.add_space(6.0);
                    ui.vertical(|ui| {
                        ui.small("Arc dir.");
                        ui.add_sized([field_w, 20.0], egui::Label::new("—"));
                    });
                });
            }
            TrackSegment::Arc(arc) => {
                let mut angle_abs = arc.sweep_deg.abs();
                let mut left = arc.sweep_deg >= 0.0;
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.small("Radius [mm]");
                        if ui
                            .add_sized(
                                [field_w, 20.0],
                                egui::DragValue::new(&mut arc.radius_mm)
                                    .speed(1.0)
                                    .clamp_range(0.001..=100_000.0),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                    });
                    ui.add_space(6.0);
                    ui.vertical(|ui| {
                        ui.small("Angle [deg]");
                        if ui
                            .add_sized(
                                [field_w, 20.0],
                                egui::DragValue::new(&mut angle_abs)
                                    .speed(0.5)
                                    .clamp_range(0.001..=360.0),
                            )
                            .changed()
                        {
                            arc.sweep_deg = angle_abs.copysign(arc.sweep_deg);
                            changed = true;
                        }
                    });
                    ui.add_space(6.0);
                    ui.vertical(|ui| {
                        ui.small("Arc dir.");
                        egui::ComboBox::from_id_source("selected_arc_direction")
                            .width(field_w)
                            .selected_text(if left { "left" } else { "right" })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut left, true, "left");
                                ui.selectable_value(&mut left, false, "right");
                            });
                    });
                });
                if left != (arc.sweep_deg >= 0.0) {
                    arc.sweep_deg = if left {
                        arc.sweep_deg.abs()
                    } else {
                        -arc.sweep_deg.abs()
                    };
                    changed = true;
                }
            }
        }

        ui.add_space(6.0);
        let button_w = ((ui.available_width() - 6.0) * 0.5).max(72.0);
        ui.horizontal(|ui| {
            if ui
                .add_sized([button_w, 22.0], egui::Button::new("Rename"))
                .clicked()
            {
                let prefix = match &track.segments[idx] {
                    TrackSegment::Straight(_) => "R",
                    TrackSegment::Arc(_) => "C",
                };
                let mut others = track.segments.clone();
                others.remove(idx);
                *track.segments[idx].id_mut() = next_segment_id(&others, prefix);
                *status_to_set = Some("Segment renamed automatically.".to_string());
                changed = true;
            }
            if ui
                .add_sized([button_w, 22.0], egui::Button::new("Remove"))
                .clicked()
            {
                track.segments.remove(idx);
                *selected_segment = if track.segments.is_empty() {
                    None
                } else {
                    Some(idx.min(track.segments.len() - 1))
                };
                changed = true;
            }
        });

        changed
    }

    fn edit_start_finish(
        ui: &mut egui::Ui,
        track: &mut TrackV2,
        status_to_set: &mut Option<String>,
    ) -> bool {
        let mut changed = false;
        let panel_width = ui.available_width();
        let row_gap = 8.0;
        let straight_w = (panel_width * 0.43).clamp(130.0, 170.0);
        let number_w = 86.0;
        let button_w = (panel_width - straight_w - number_w - row_gap * 3.0).clamp(94.0, 140.0);

        let valid_segments = valid_start_finish_segments(track);
        let valid_current = valid_segments
            .iter()
            .any(|segment| segment.id == track.markings.start_finish.segment_id);

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.small("Straight segment");
                if valid_segments.is_empty() {
                    ui.add_sized([straight_w, 20.0], egui::Label::new("no valid straight"));
                } else {
                    let mut selected_id = if valid_current {
                        track.markings.start_finish.segment_id.clone()
                    } else {
                        valid_segments[0].id.clone()
                    };
                    egui::ComboBox::from_id_source("start_finish_segment")
                        .width(straight_w)
                        .selected_text(if valid_current {
                            track.markings.start_finish.segment_id.as_str()
                        } else {
                            "select straight"
                        })
                        .show_ui(ui, |ui| {
                            for segment in &valid_segments {
                                ui.selectable_value(
                                    &mut selected_id,
                                    segment.id.clone(),
                                    format!("{} — L={:.1} mm", segment.id, segment.length_mm),
                                );
                            }
                        })
                        .response
                        .on_hover_text(
                            "Straight segment that contains the START and FINISH markers. Only long straight segments are shown.",
                        );
                    if selected_id != track.markings.start_finish.segment_id {
                        track.markings.start_finish.segment_id = selected_id.clone();
                        if let Some(segment) = valid_segments.iter().find(|s| s.id == selected_id) {
                            center_start_finish_on_segment(track, segment.length_mm);
                        }
                        changed = true;
                    }
                }
            });

            ui.add_space(row_gap);

            let selected_length_for_start = valid_segments
                .iter()
                .find(|segment| segment.id == track.markings.start_finish.segment_id)
                .map(|segment| segment.length_mm);
            let start_range = if let Some(length_mm) = selected_length_for_start {
                let max_start = (length_mm - track.markings.start_finish.margin_mm - track.markings.start_finish.distance_mm).max(track.markings.start_finish.margin_mm);
                track.markings.start_finish.margin_mm..=max_start
            } else {
                0.0..=100_000.0
            };
            if compact_drag_value_labeled(
                ui,
                "Start offset [mm]",
                &mut track.markings.start_finish.start_s_mm,
                1.0,
                start_range,
                number_w,
            ) {
                if let Some(length_mm) = selected_length_for_start {
                    clamp_start_finish_to_segment(track, length_mm);
                }
                changed = true;
            }
            ui.add_space(row_gap);
            ui.vertical(|ui| {
                ui.small("");
                if ui
                    .add_sized([button_w, 20.0], egui::Button::new("Default Values"))
                    .on_hover_text(
                        "Aplica valores oficiais: distância START/FINISH = 1000 mm, margem = 100 mm. Também posiciona o robô dentro da área permitida e com Heading 0° apontando para START.",
                    )
                    .clicked()
                {
                    track.markings.start_finish.distance_mm = 1000.0;
                    track.markings.start_finish.margin_mm = 100.0;
                    track.markings.start_finish.robot_start.delta_x_mm = ROBOT_START_MARKER_CLEARANCE_MM;
                    track.markings.start_finish.robot_start.delta_y_mm = 0.0;
                    track.markings.start_finish.robot_start.heading_deg = 0.0;
                    if let Some(length_mm) = selected_length_for_start {
                        if length_mm + 1e-6 >= start_finish_required_length_mm(&track.markings.start_finish) {
                            center_start_finish_on_segment(track, length_mm);
                        } else {
                            *status_to_set = Some(
                                "Default START/FINISH distance does not fit in the selected straight."
                                    .to_string(),
                            );
                        }
                    }
                    changed = true;
                }
            });
        });

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if compact_drag_value_labeled(
                ui,
                "Marker distance [mm]",
                &mut track.markings.start_finish.distance_mm,
                1.0,
                1.0..=100_000.0,
                number_w,
            ) {
                changed = true;
            }
            ui.add_space(row_gap);
            if compact_drag_value_labeled(
                ui,
                "End margin [mm]",
                &mut track.markings.start_finish.margin_mm,
                1.0,
                0.0..=100_000.0,
                number_w,
            ) {
                changed = true;
            }
            ui.add_space(row_gap);
            ui.vertical(|ui| {
                ui.small("");
                if ui
                    .add_sized(
                        [
                            (panel_width - number_w * 2.0 - row_gap * 3.0).max(80.0),
                            20.0,
                        ],
                        egui::Button::new("Center on selected straight"),
                    )
                    .on_hover_text("Centraliza a área START/FINISH dentro da reta selecionada.")
                    .clicked()
                {
                    if let Some(length_mm) = valid_segments
                        .iter()
                        .find(|segment| segment.id == track.markings.start_finish.segment_id)
                        .map(|segment| segment.length_mm)
                    {
                        center_start_finish_on_segment(track, length_mm);
                        changed = true;
                    }
                }
            });
        });

        if valid_segments.is_empty() {
            ui.colored_label(
                egui::Color32::from_rgb(190, 55, 45),
                format!(
                    "No valid straight. Minimum required: {:.1} mm.",
                    start_finish_required_length_mm(&track.markings.start_finish)
                ),
            );
        } else if !valid_current {
            ui.colored_label(
                egui::Color32::from_rgb(190, 130, 30),
                "Current straight cannot contain START/FINISH and is not listed as valid.",
            );
        }

        ui.add_space(8.0);
        ui.label(egui::RichText::new("Robot start pose").small().strong());
        let robot_field_w = ((panel_width - row_gap * 2.0) / 3.0).clamp(78.0, 116.0);
        ui.horizontal(|ui| {
            if compact_drag_value_labeled(
                ui,
                "ΔX from START [mm]",
                &mut track.markings.start_finish.robot_start.delta_x_mm,
                1.0,
                -100_000.0..=100_000.0,
                robot_field_w,
            ) {
                changed = true;
            }
            ui.add_space(row_gap);
            if compact_drag_value_labeled(
                ui,
                "ΔY from line [mm]",
                &mut track.markings.start_finish.robot_start.delta_y_mm,
                1.0,
                -100_000.0..=100_000.0,
                robot_field_w,
            ) {
                changed = true;
            }
            ui.add_space(row_gap);
            if compact_drag_value_labeled(
                ui,
                "Heading [deg]",
                &mut track.markings.start_finish.robot_start.heading_deg,
                0.5,
                -180.0..=180.0,
                robot_field_w,
            ) {
                changed = true;
            }
        });

        ui.add_space(6.0);
        ui.label(egui::RichText::new("Robot exit").small().strong());
        let mut direction = track.markings.start_finish.exit_direction;
        egui::ComboBox::from_id_source("start_finish_exit_direction")
            .width(panel_width)
            .selected_text(match direction {
                StartExitDirection::ToIncreasingS => "Goes to the right (+s)",
                StartExitDirection::ToDecreasingS => "Goes to the left (-s)",
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut direction,
                    StartExitDirection::ToIncreasingS,
                    "Goes to the right (+s)",
                );
                ui.selectable_value(
                    &mut direction,
                    StartExitDirection::ToDecreasingS,
                    "Goes to the left (-s)",
                );
            })
            .response
            .on_hover_text("Swap which end of the area is START and which is FINISH.");
        if direction != track.markings.start_finish.exit_direction {
            track.markings.start_finish.exit_direction = direction;
            changed = true;
        }

        if let Some(resolved) = resolve_start_finish_markers(track) {
            let robot_pose = resolve_robot_start_pose(track);
            ui.small(format!(
                "START s={:.1} mm | FINISH s={:.1} mm | START→FINISH heading {:.1}°",
                resolved.start_local_s_mm, resolved.finish_local_s_mm, resolved.travel_heading_deg
            ));
            if let Some(robot_pose) = robot_pose {
                ui.small(format!(
                    "Robot starts at x={:.3} m, y={:.3} m, heading={:.1}°.",
                    robot_pose.x_mm / 1000.0,
                    robot_pose.y_mm / 1000.0,
                    robot_pose.heading_deg
                ));
            }
        }

        changed
    }

    fn compact_drag_value_labeled(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut f64,
        speed: f64,
        range: std::ops::RangeInclusive<f64>,
        width: f32,
    ) -> bool {
        let mut changed = false;
        ui.vertical(|ui| {
            ui.small(label);
            if ui
                .add_sized(
                    [width, 20.0],
                    egui::DragValue::new(value).speed(speed).clamp_range(range),
                )
                .changed()
            {
                changed = true;
            }
        });
        changed
    }

    fn drag_value_row(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut f64,
        speed: f64,
        range: std::ops::RangeInclusive<f64>,
        width: f32,
    ) -> bool {
        ui.label(label);
        let changed = ui
            .add_sized(
                [width, 20.0],
                egui::DragValue::new(value).speed(speed).clamp_range(range),
            )
            .changed();
        ui.end_row();
        changed
    }

    fn drag_value_labeled(
        ui: &mut egui::Ui,
        label: &str,
        value: &mut f64,
        speed: f64,
        range: std::ops::RangeInclusive<f64>,
    ) -> bool {
        let mut changed = false;
        ui.vertical(|ui| {
            ui.small(label);
            if ui
                .add(egui::DragValue::new(value).speed(speed).clamp_range(range))
                .changed()
            {
                changed = true;
            }
        });
        changed
    }

    fn segment_summary(segment: &TrackSegment) -> String {
        match segment {
            TrackSegment::Straight(straight) => {
                format!("Straight {} — L={:.1} mm", straight.id, straight.length_mm)
            }
            TrackSegment::Arc(arc) => {
                format!(
                    "Arc {} — R={:.1} mm, θ={:.1}°",
                    arc.id, arc.radius_mm, arc.sweep_deg
                )
            }
        }
    }

    impl eframe::App for RTSimApp {
        fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
            self.poll_simulation(ctx);
            self.sidebar(ctx);
            egui::TopBottomPanel::bottom("bottom_status").show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Status:");
                    ui.label(self.status.as_str());
                });
            });
            egui::CentralPanel::default().show(ctx, |ui| match self.view {
                AppView::Home => self.show_home(ui),
                AppView::TrackEditor => self.show_track_editor(ui),
                AppView::RobotEditor => self.show_robot_editor(ui),
                AppView::VisualSimulator => self.show_visual_simulator(ui, ctx),
                AppView::ReplayViewer => self.show_replay_viewer(ui),
                AppView::CalibrationTools => self.show_calibration_tools(ui),
            });
        }
    }

    fn nav_button(ui: &mut egui::Ui, current: &mut AppView, target: AppView, label: &str) {
        if ui.selectable_label(*current == target, label).clicked() {
            *current = target;
        }
    }

    fn motor_editor(
        ui: &mut egui::Ui,
        title: &str,
        motor: &mut MotorConfig,
        invalidate: &mut bool,
    ) {
        ui.group(|ui| {
            ui.strong(title);
            ui.horizontal(|ui| {
                ui.label("Model");
                if ui.text_edit_singleline(&mut motor.model).changed() {
                    *invalidate = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Reduction");
                if ui
                    .add(
                        egui::DragValue::new(&mut motor.gear_ratio)
                            .speed(0.1)
                            .clamp_range(0.1..=1000.0),
                    )
                    .changed()
                {
                    *invalidate = true;
                }
                ui.label("Efficiency");
                if ui
                    .add(
                        egui::DragValue::new(&mut motor.efficiency)
                            .speed(0.01)
                            .clamp_range(0.0..=1.0),
                    )
                    .changed()
                {
                    *invalidate = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Tensao nominal [V]");
                if ui
                    .add(
                        egui::DragValue::new(&mut motor.nominal_voltage_v)
                            .speed(0.1)
                            .clamp_range(0.1..=1000.0),
                    )
                    .changed()
                {
                    *invalidate = true;
                }
                ui.label("RPM no load");
                if ui
                    .add(
                        egui::DragValue::new(&mut motor.no_load_rpm)
                            .speed(10.0)
                            .clamp_range(0.0..=500_000.0),
                    )
                    .changed()
                {
                    *invalidate = true;
                }
                let mut stall_mnm = motor.stall_torque_nm * 1000.0;
                ui.label("Tstall [mN·m]");
                if ui
                    .add(
                        egui::DragValue::new(&mut stall_mnm)
                            .speed(0.1)
                            .clamp_range(0.0..=10_000.0),
                    )
                    .changed()
                {
                    motor.stall_torque_nm = stall_mnm / 1000.0;
                    *invalidate = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Istall [A]");
                if ui
                    .add(
                        egui::DragValue::new(&mut motor.stall_current_a)
                            .speed(0.01)
                            .clamp_range(0.0..=1000.0),
                    )
                    .changed()
                {
                    *invalidate = true;
                }
            });
        });
    }

    struct CanvasChange {
        changed: bool,
    }

    fn draw_editable_track_canvas(
        ui: &mut egui::Ui,
        track: &mut TrackConfig,
        selected: &mut Option<usize>,
    ) -> CanvasChange {
        let desired = egui::vec2(ui.available_width(), 420.0);
        let (response, painter) = ui.allocate_painter(desired, egui::Sense::click_and_drag());
        let rect = response.rect;
        painter.rect_filled(rect, 6.0, egui::Color32::from_rgb(8, 8, 8));
        let bounds = track_bounds(track, None);
        draw_track_geometry(&painter, rect, bounds, track, None, &[]);

        let mut changed = false;
        for (i, p) in track.centerline.iter().enumerate() {
            let pos = world_to_screen(rect, bounds, *p);
            let color = if *selected == Some(i) {
                egui::Color32::from_rgb(220, 70, 50)
            } else {
                egui::Color32::from_rgb(40, 120, 220)
            };
            painter.circle_filled(pos, 5.0, color);
            painter.text(
                pos + egui::vec2(7.0, -7.0),
                egui::Align2::LEFT_BOTTOM,
                i.to_string(),
                egui::FontId::proportional(11.0),
                egui::Color32::DARK_GRAY,
            );
        }

        if let Some(pointer) = response.interact_pointer_pos() {
            if response.clicked() {
                let nearest = nearest_point(track, rect, bounds, pointer, 12.0);
                if nearest.is_some() {
                    *selected = nearest;
                }
            }
            if response.dragged() {
                if selected.is_none() {
                    *selected = nearest_point(track, rect, bounds, pointer, 18.0);
                }
                if let Some(idx) = *selected {
                    if idx < track.centerline.len() {
                        track.centerline[idx] = screen_to_world(rect, bounds, pointer);
                        changed = true;
                    }
                }
            }
        }
        CanvasChange { changed }
    }

    fn draw_track_view(
        ui: &mut egui::Ui,
        track: &TrackConfig,
        robot_pose: Option<Pose2>,
        trail: &[TelemetrySample],
    ) {
        draw_track_view_with_height(ui, track, robot_pose, trail, 440.0);
    }

    fn draw_track_view_with_height(
        ui: &mut egui::Ui,
        track: &TrackConfig,
        robot_pose: Option<Pose2>,
        trail: &[TelemetrySample],
        height: f32,
    ) {
        let mut zoom = 1.0;
        let mut pan_m = Vec2::new(0.0, 0.0);
        draw_track_view_with_height_zoomable(
            ui, track, robot_pose, trail, height, &mut zoom, &mut pan_m, None,
        );
    }

    fn draw_track_view_with_height_zoomable(
        ui: &mut egui::Ui,
        track: &TrackConfig,
        robot_pose: Option<Pose2>,
        trail: &[TelemetrySample],
        height: f32,
        zoom: &mut f32,
        pan_m: &mut Vec2,
        robot: Option<&RobotConfig>,
    ) {
        let desired = egui::vec2(ui.available_width(), height.max(220.0));
        let (response, painter) = ui.allocate_painter(desired, egui::Sense::click_and_drag());
        let rect = response.rect;
        painter.rect_filled(rect, 6.0, egui::Color32::from_rgb(8, 8, 8));

        let base_bounds = track_bounds(track, robot_pose.map(|p| Vec2::new(p.x, p.y)));
        *zoom = (*zoom).clamp(0.25, 12.0);
        let mut bounds = viewport_bounds(base_bounds, *zoom, *pan_m);

        if response.hovered() {
            let scroll_y = ui.input(|i| i.raw_scroll_delta.y);
            if scroll_y.abs() > 0.0 {
                let pointer = ui.input(|i| i.pointer.hover_pos()).unwrap_or(rect.center());
                let before = screen_to_world(rect, bounds, pointer);
                let factor = (scroll_y * 0.0015).exp();
                *zoom = (*zoom * factor).clamp(0.25, 12.0);
                bounds = viewport_bounds(base_bounds, *zoom, *pan_m);
                let after = screen_to_world(rect, bounds, pointer);
                pan_m.x += before.x - after.x;
                pan_m.y += before.y - after.y;
                bounds = viewport_bounds(base_bounds, *zoom, *pan_m);
            }
        }

        if response.dragged() {
            let delta = ui.input(|i| i.pointer.delta());
            let scale = world_screen_scale(rect, bounds).max(1e-9);
            pan_m.x -= delta.x as f64 / scale;
            pan_m.y += delta.y as f64 / scale;
            bounds = viewport_bounds(base_bounds, *zoom, *pan_m);
            ui.ctx().request_repaint();
        }

        draw_track_geometry(
            &painter,
            rect,
            bounds,
            track,
            if robot.is_some() { None } else { robot_pose },
            trail,
        );
        if let (Some(robot), Some(pose)) = (robot, robot_pose) {
            draw_assembly_world(&painter, rect, bounds, robot, pose);
        }

        let help = "Scroll: zoom | arraste: mover | Fit: reset";
        painter.text(
            rect.left_top() + egui::vec2(10.0, 10.0),
            egui::Align2::LEFT_TOP,
            help,
            egui::FontId::proportional(11.0),
            egui::Color32::from_rgb(110, 110, 110),
        );
    }

    fn draw_replay_path_only(
        ui: &mut egui::Ui,
        trail: &[TelemetrySample],
        sample: Option<&TelemetrySample>,
    ) {
        let desired = egui::vec2(ui.available_width(), 440.0);
        let (_response, painter) = ui.allocate_painter(desired, egui::Sense::hover());
        let rect = _response.rect;
        painter.rect_filled(rect, 6.0, egui::Color32::from_rgb(8, 8, 8));
        let bounds = replay_bounds(trail);
        if trail.len() >= 2 {
            for w in trail.windows(2) {
                let a = world_to_screen(rect, bounds, Vec2::new(w[0].x_m, w[0].y_m));
                let b = world_to_screen(rect, bounds, Vec2::new(w[1].x_m, w[1].y_m));
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(1.5, egui::Color32::from_rgb(40, 120, 220)),
                );
            }
        }
        if let Some(s) = sample {
            draw_robot(
                &painter,
                rect,
                bounds,
                Pose2::new(s.x_m, s.y_m, s.yaw_rad),
                0.12,
                0.09,
            );
        }
    }

    fn draw_track_geometry(
        painter: &egui::Painter,
        rect: egui::Rect,
        bounds: Bounds,
        track: &TrackConfig,
        robot_pose: Option<Pose2>,
        trail: &[TelemetrySample],
    ) {
        draw_track_substrate(painter, rect, bounds, track);
        draw_grid(painter, rect, bounds, track);
        let runtime = cached_track(painter.ctx(), track).ok();
        if let Some(runtime) = &runtime {
            draw_surface_regions(painter, rect, bounds, runtime);
        }
        if track.centerline.len() >= 2 {
            let px_width = world_len_to_screen(rect, bounds, track.line_width_m).max(2.0);
            for w in track.centerline.windows(2) {
                let a = world_to_screen(rect, bounds, w[0]);
                let b = world_to_screen(rect, bounds, w[1]);
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(px_width, {
                        let c = track
                            .parametric
                            .as_ref()
                            .map(|p| crate::track::definition::display_color(&p.surface.line_color))
                            .unwrap_or([245, 245, 245]);
                        egui::Color32::from_rgb(c[0], c[1], c[2])
                    }),
                );
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(110, 110, 110)),
                );
            }
        }
        if let Some(parametric) = &track.parametric {
            let origin = Vec2::new(
                parametric.origin.x_mm / 1000.0,
                parametric.origin.y_mm / 1000.0,
            );
            let origin_screen = world_to_screen(rect, bounds, origin);
            painter.circle_filled(origin_screen, 5.0, egui::Color32::from_rgb(80, 160, 255));
            let heading = parametric.origin.heading_deg.to_radians();
            let arrow_end = Vec2::new(
                origin.x + heading.cos() * 0.12,
                origin.y + heading.sin() * 0.12,
            );
            painter.line_segment(
                [origin_screen, world_to_screen(rect, bounds, arrow_end)],
                egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 160, 255)),
            );

            if let Some(corners) = robot_start_allowed_area_corners(parametric) {
                draw_robot_start_allowed_area(painter, rect, bounds, corners);
            }
            if let Some(pose) = resolve_robot_start_pose(parametric) {
                draw_robot_start_pose(painter, rect, bounds, pose);
            }
        }
        if let Some(runtime) = &runtime {
            draw_optical_marks(painter, rect, bounds, runtime);
        }
        if trail.len() >= 2 {
            let stride = (trail.len() / 2000).max(1);
            let compact: Vec<&TelemetrySample> = trail.iter().step_by(stride).collect();
            for w in compact.windows(2) {
                let a = world_to_screen(rect, bounds, Vec2::new(w[0].x_m, w[0].y_m));
                let b = world_to_screen(rect, bounds, Vec2::new(w[1].x_m, w[1].y_m));
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(1.5, egui::Color32::from_rgb(40, 140, 255)),
                );
            }
        }
        if let Some(pose) = robot_pose {
            draw_robot(painter, rect, bounds, pose, 0.12, 0.09);
        }
    }

    fn draw_grid(painter: &egui::Painter, rect: egui::Rect, bounds: Bounds, track: &TrackConfig) {
        let Some(parametric) = &track.parametric else {
            return;
        };
        let grid_m = (parametric.area.grid_mm / 1000.0).max(0.001);
        let min_x = (bounds.min_x / grid_m).floor() as i32;
        let max_x = (bounds.max_x / grid_m).ceil() as i32;
        let min_y = (bounds.min_y / grid_m).floor() as i32;
        let max_y = (bounds.max_y / grid_m).ceil() as i32;
        let stroke = egui::Stroke::new(0.5, egui::Color32::from_rgb(35, 35, 35));
        for ix in min_x..=max_x {
            let x = ix as f64 * grid_m;
            painter.line_segment(
                [
                    world_to_screen(rect, bounds, Vec2::new(x, bounds.min_y)),
                    world_to_screen(rect, bounds, Vec2::new(x, bounds.max_y)),
                ],
                stroke,
            );
        }
        for iy in min_y..=max_y {
            let y = iy as f64 * grid_m;
            painter.line_segment(
                [
                    world_to_screen(rect, bounds, Vec2::new(bounds.min_x, y)),
                    world_to_screen(rect, bounds, Vec2::new(bounds.max_x, y)),
                ],
                stroke,
            );
        }
        let area_rect = [
            world_to_screen(rect, bounds, Vec2::new(0.0, 0.0)),
            world_to_screen(
                rect,
                bounds,
                Vec2::new(parametric.area.width_mm / 1000.0, 0.0),
            ),
            world_to_screen(
                rect,
                bounds,
                Vec2::new(
                    parametric.area.width_mm / 1000.0,
                    parametric.area.height_mm / 1000.0,
                ),
            ),
            world_to_screen(
                rect,
                bounds,
                Vec2::new(0.0, parametric.area.height_mm / 1000.0),
            ),
        ];
        painter.add(egui::Shape::closed_line(
            area_rect.to_vec(),
            egui::Stroke::new(1.0, egui::Color32::from_rgb(80, 80, 80)),
        ));
    }

    fn draw_robot_start_allowed_area(
        painter: &egui::Painter,
        rect: egui::Rect,
        bounds: Bounds,
        corners_mm: [Vec2; 4],
    ) {
        let points: Vec<egui::Pos2> = corners_mm
            .iter()
            .map(|p| world_to_screen(rect, bounds, Vec2::new(p.x / 1000.0, p.y / 1000.0)))
            .collect();
        painter.add(egui::Shape::closed_line(
            points,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 135, 100)),
        ));
    }

    fn draw_robot_start_pose(
        painter: &egui::Painter,
        rect: egui::Rect,
        bounds: Bounds,
        pose: crate::rtsim_track::TrackPose,
    ) {
        let p = Vec2::new(pose.x_mm / 1000.0, pose.y_mm / 1000.0);
        let theta = pose.heading_deg.to_radians();
        let forward = Vec2::new(theta.cos(), theta.sin());
        let left = Vec2::new(-theta.sin(), theta.cos());
        let half_len = 0.055;
        let half_w = 0.035;
        let nose = p + forward * half_len;
        let rear_left = p - forward * half_len + left * half_w;
        let rear_right = p - forward * half_len - left * half_w;
        let shape = vec![
            world_to_screen(rect, bounds, nose),
            world_to_screen(rect, bounds, rear_left),
            world_to_screen(rect, bounds, rear_right),
        ];
        painter.add(egui::Shape::closed_line(
            shape,
            egui::Stroke::new(2.0, egui::Color32::from_rgb(230, 120, 80)),
        ));
        painter.text(
            world_to_screen(rect, bounds, p) + egui::vec2(6.0, -6.0),
            egui::Align2::LEFT_BOTTOM,
            "ROBOT START",
            egui::FontId::proportional(10.0),
            egui::Color32::from_rgb(230, 120, 80),
        );
    }

    fn draw_robot(
        painter: &egui::Painter,
        rect: egui::Rect,
        bounds: Bounds,
        pose: Pose2,
        length_m: f64,
        width_m: f64,
    ) {
        let corners = [
            Vec2::new(length_m * 0.5, width_m * 0.5),
            Vec2::new(length_m * 0.5, -width_m * 0.5),
            Vec2::new(-length_m * 0.5, -width_m * 0.5),
            Vec2::new(-length_m * 0.5, width_m * 0.5),
        ];
        let points: Vec<egui::Pos2> = corners
            .iter()
            .map(|p| world_to_screen(rect, bounds, pose.transform_point(*p)))
            .collect();
        painter.add(egui::Shape::closed_line(
            points,
            egui::Stroke::new(2.0, egui::Color32::from_rgb(200, 70, 50)),
        ));
        let nose = world_to_screen(
            rect,
            bounds,
            pose.transform_point(Vec2::new(length_m * 0.6, 0.0)),
        );
        let center = world_to_screen(rect, bounds, Vec2::new(pose.x, pose.y));
        painter.line_segment(
            [center, nose],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(200, 70, 50)),
        );
        painter.circle_filled(center, 3.5, egui::Color32::from_rgb(200, 70, 50));
    }

    fn telemetry_panel(ui: &mut egui::Ui, s: &TelemetrySample) {
        egui::CollapsingHeader::new("Telemetria")
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("telemetry_grid")
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("t");
                        ui.label(format!("{:.6} s", s.t_us as f64 / 1_000_000.0));
                        ui.end_row();
                        ui.label("pose");
                        ui.label(format!(
                            "x={:.4} m, y={:.4} m, yaw={:.3} rad",
                            s.x_m, s.y_m, s.yaw_rad
                        ));
                        ui.end_row();
                        ui.label("velocidade");
                        ui.label(format!(
                            "vx={:.3} m/s, vy={:.3} m/s, yaw_rate={:.3} rad/s",
                            s.vx_body_m_s, s.vy_body_m_s, s.yaw_rate_rad_s
                        ));
                        ui.end_row();
                        ui.label("linha");
                        ui.label(format!(
                            "pos={:.4} m, erro={:.4} m, visível={}, conf={:.2}",
                            s.line_position_m, s.line_error_m, s.line_visible, s.line_confidence
                        ));
                        ui.end_row();
                        ui.label("PWM");
                        ui.label(format!(
                            "L={:.3}, R={:.3}, downforce={:.3}",
                            s.pwm_left, s.pwm_right, s.pwm_downforce
                        ));
                        ui.end_row();
                        ui.label("bateria");
                        ui.label(format!(
                            "{:.3} V, {:.3} A",
                            s.battery_voltage_v, s.battery_current_a
                        ));
                        ui.end_row();
                        ui.label("normal rodas");
                        ui.label(format!(
                            "FL={:.3} FR={:.3} RL={:.3} RR={:.3} N",
                            s.normal_front_left_n,
                            s.normal_front_right_n,
                            s.normal_rear_left_n,
                            s.normal_rear_right_n
                        ));
                        ui.end_row();
                        ui.label("downforce");
                        ui.label(format!(
                            "extra={:.3} N, fan={:.3} N, sucção={:.3} N, I={:.3} A",
                            s.downforce_extra_n,
                            s.downforce_fan_n,
                            s.downforce_suction_n,
                            s.downforce_current_a
                        ));
                        ui.end_row();
                        ui.label("slip");
                        ui.label(format!("L={:.3}, R={:.3}", s.slip_left, s.slip_right));
                        ui.end_row();
                        ui.label("encoder");
                        ui.label(format!(
                            "L={}, R={}",
                            s.encoder_left_ticks, s.encoder_right_ticks
                        ));
                        ui.end_row();
                    });
            });
    }

    #[derive(Debug, Clone, Copy)]
    struct Bounds {
        min_x: f64,
        max_x: f64,
        min_y: f64,
        max_y: f64,
    }

    fn track_bounds(track: &TrackConfig, extra: Option<Vec2>) -> Bounds {
        let mut points = track.centerline.clone();
        if let Some(p) = extra {
            points.push(p);
        }
        bounds_from_points(&points)
    }

    fn replay_bounds(samples: &[TelemetrySample]) -> Bounds {
        let points: Vec<Vec2> = samples.iter().map(|s| Vec2::new(s.x_m, s.y_m)).collect();
        bounds_from_points(&points)
    }

    fn bounds_from_points(points: &[Vec2]) -> Bounds {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for p in points {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        if !min_x.is_finite() || !max_x.is_finite() || (max_x - min_x).abs() < 1e-9 {
            min_x = -0.5;
            max_x = 0.5;
        }
        if !min_y.is_finite() || !max_y.is_finite() || (max_y - min_y).abs() < 1e-9 {
            min_y = -0.5;
            max_y = 0.5;
        }
        let margin_x = ((max_x - min_x) * 0.08).max(0.10);
        let margin_y = ((max_y - min_y) * 0.20).max(0.10);
        Bounds {
            min_x: min_x - margin_x,
            max_x: max_x + margin_x,
            min_y: min_y - margin_y,
            max_y: max_y + margin_y,
        }
    }

    fn viewport_bounds(base: Bounds, zoom: f32, pan_m: Vec2) -> Bounds {
        let zoom = (zoom as f64).clamp(0.25, 30.0);
        let cx = (base.min_x + base.max_x) * 0.5 + pan_m.x;
        let cy = (base.min_y + base.max_y) * 0.5 + pan_m.y;
        let half_w = (base.max_x - base.min_x) * 0.5 / zoom;
        let half_h = (base.max_y - base.min_y) * 0.5 / zoom;
        Bounds {
            min_x: cx - half_w,
            max_x: cx + half_w,
            min_y: cy - half_h,
            max_y: cy + half_h,
        }
    }

    fn world_screen_scale(rect: egui::Rect, b: Bounds) -> f64 {
        let sx = rect.width() as f64 / (b.max_x - b.min_x).max(1e-9);
        let sy = rect.height() as f64 / (b.max_y - b.min_y).max(1e-9);
        sx.min(sy)
    }

    fn world_to_screen(rect: egui::Rect, b: Bounds, p: Vec2) -> egui::Pos2 {
        let sx = rect.width() as f64 / (b.max_x - b.min_x).max(1e-9);
        let sy = rect.height() as f64 / (b.max_y - b.min_y).max(1e-9);
        let scale = sx.min(sy);
        let world_w_px = (b.max_x - b.min_x) * scale;
        let world_h_px = (b.max_y - b.min_y) * scale;
        let ox = rect.left() as f64 + (rect.width() as f64 - world_w_px) * 0.5;
        let oy = rect.top() as f64 + (rect.height() as f64 - world_h_px) * 0.5;
        egui::pos2(
            (ox + (p.x - b.min_x) * scale) as f32,
            (oy + (b.max_y - p.y) * scale) as f32,
        )
    }

    fn screen_to_world(rect: egui::Rect, b: Bounds, pos: egui::Pos2) -> Vec2 {
        let sx = rect.width() as f64 / (b.max_x - b.min_x).max(1e-9);
        let sy = rect.height() as f64 / (b.max_y - b.min_y).max(1e-9);
        let scale = sx.min(sy);
        let world_w_px = (b.max_x - b.min_x) * scale;
        let world_h_px = (b.max_y - b.min_y) * scale;
        let ox = rect.left() as f64 + (rect.width() as f64 - world_w_px) * 0.5;
        let oy = rect.top() as f64 + (rect.height() as f64 - world_h_px) * 0.5;
        Vec2::new(
            b.min_x + (pos.x as f64 - ox) / scale,
            b.max_y - (pos.y as f64 - oy) / scale,
        )
    }

    fn world_len_to_screen(rect: egui::Rect, b: Bounds, len: f64) -> f32 {
        let sx = rect.width() as f64 / (b.max_x - b.min_x).max(1e-9);
        let sy = rect.height() as f64 / (b.max_y - b.min_y).max(1e-9);
        (len * sx.min(sy)) as f32
    }

    fn nearest_point(
        track: &TrackConfig,
        rect: egui::Rect,
        bounds: Bounds,
        pointer: egui::Pos2,
        max_dist_px: f32,
    ) -> Option<usize> {
        let mut best = None;
        let mut best_dist = max_dist_px;
        for (i, p) in track.centerline.iter().enumerate() {
            let pos = world_to_screen(rect, bounds, *p);
            let dist = pos.distance(pointer);
            if dist <= best_dist {
                best = Some(i);
                best_dist = dist;
            }
        }
        best
    }

    fn default_replay_path(cfg: &LoadedConfig) -> PathBuf {
        cfg.project
            .replay_output
            .as_ref()
            .map(|p| resolve_child_path(&cfg.project_path, p))
            .unwrap_or_else(|| cfg.project_path.with_extension("rtlog"))
    }

    fn resolve_child_path(project_path: &Path, child: &Path) -> PathBuf {
        if child.is_absolute() {
            child.to_path_buf()
        } else {
            project_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(child)
        }
    }

    fn path_relative_to_project(project_path: &Path, child: &Path) -> PathBuf {
        if child.is_absolute() {
            return child.to_path_buf();
        }
        let base_dir = project_path.parent().unwrap_or_else(|| Path::new("."));
        child
            .strip_prefix(base_dir)
            .map(Path::to_path_buf)
            .unwrap_or_else(|_| child.to_path_buf())
    }

    fn resolve_asset_path_text(project_path: Option<&Path>, text: &str) -> PathBuf {
        let path = PathBuf::from(text.trim());
        if path.is_absolute() || path.exists() || path.components().count() > 1 {
            path
        } else if let Some(project_path) = project_path {
            project_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(path)
        } else {
            path
        }
    }

    fn default_surface_profile_path(profile_name: &str) -> PathBuf {
        Path::new("examples/profiles")
            .join(format!("{}.json", sanitize_asset_filename(profile_name)))
    }

    fn sanitize_asset_filename(name: &str) -> String {
        let mut out = String::new();
        for ch in name.chars() {
            if ch.is_ascii_alphanumeric() {
                out.push(ch.to_ascii_lowercase());
            } else if ch.is_whitespace() || matches!(ch, '-' | '_' | '.') {
                if !out.ends_with('_') {
                    out.push('_');
                }
            }
        }
        let out = out.trim_matches('_').to_string();
        if out.is_empty() {
            "surface_profile".to_string()
        } else {
            out
        }
    }

    fn default_loaded_config(project_path: PathBuf) -> LoadedConfig {
        let project = ProjectConfig {
            schema: "rtsim-project-v1".to_string(),
            name: "novo-projeto-atual".to_string(),
            robot_path: PathBuf::from("robot.json"),
            track_path: PathBuf::from("track.json"),
            time: TimeConfig::default(),
            duration_s: 10.0,
            start_pose: Pose2::new(0.7, 0.5, 0.0),
            csv_output: Some(PathBuf::from("resultado.csv")),
            replay_output: Some(PathBuf::from("resultado.rtlog")),
        };
        let robot = RobotConfig {
            sensing: Default::default(),
            powertrain: None,
            physics: None,
            assembly: None,
            schema: "rtsim-robot-v8".to_string(),
            name: "Simple N20 Robot".to_string(),
            chassis: ChassisConfig {
                mass_kg: 0.180,
                inertia_kg_m2: 0.00045,
                center_of_mass_m: Vec2::new(0.0, 0.0),
                length_m: 0.120,
                width_m: 0.090,
            },
            drivetrain: DrivetrainConfig {
                wheel_radius_m: 0.010,
                wheel_width_m: 0.010,
                track_width_m: 0.082,
                wheelbase_m: 0.084,
                wheel_inertia_kg_m2: 1e-7,
            },
            normal_force: NormalForceConfig {
                model: crate::io::models::NormalForceKind::None,

                command_pwm_default: 0.0,
                position_m: Vec2::new(0.0, 0.0),
                max_force_n: 0.0,
                max_current_a: 0.0,
                response_time_s: 0.0,
                chamber_area_m2: 0.0,
                max_delta_pressure_pa: 0.0,
                leakage_factor: 0.0,
                speed_sensitivity: 0.0,
                force_curve: Vec::new(),
                fans: Vec::new(),
            },
            tire: TireConfig {
                model: "SlipRatioWheel".to_string(),
                mu_longitudinal: 1.2,
                mu_lateral: 1.0,
                rolling_resistance: 0.015,
                slip_velocity_epsilon_m_s: 0.05,
            },
            motor_left: default_motor(),
            motor_right: default_motor(),
            driver: crate::config::DriverConfig {
                model: "PwmHBridge".to_string(),
                pwm_frequency_hz: 20_000.0,
                mode: "brake".to_string(),
                voltage_drop_v: 0.2,
                pwm_resolution_bits: 10,
                command_deadband: 0.001,
                current_limit_a: 3.0,
            },
            battery: BatteryConfig {
                model: "VoltageSagBattery".to_string(),
                cells: 2,
                nominal_voltage_v: 7.4,
                full_voltage_v: 7.4,
                empty_voltage_v: 6.4,
                capacity_mah: 300.0,
                internal_resistance_ohm: 0.08,
                initial_soc: 1.0,
                current_limit_a: 60.0,
            },
            sensors: vec![default_sensor_instance()],
            line_validity_areas: vec![RobotLineValidityArea {
                name: "Main body".to_string(),
                position_m: Vec2::new(0.0, 0.0),
                length_m: 0.120,
                width_m: 0.090,
                angle_deg: 0.0,
                enabled: true,
            }],
            encoder: EncoderConfig {
                model: "QuantizedEncoder".to_string(),
                ticks_per_rev: 360,
                invert_left: false,
                invert_right: false,
            },
            gyro: GyroConfig {
                model: "NoisyGyro".to_string(),
                noise_std_rad_s: 0.01,
                bias_rad_s: 0.0,
                saturation_rad_s: 34.9,
                seed: 2467,
            },
            controller: PidConfig {
                kp: 13.0,
                ki: 0.0,
                kd: 0.035,
                base_pwm: 0.34,
                max_pwm: 0.90,
                target_position_m: 0.0,
                downforce_pwm: 0.0,
            },
        };
        let track = TrackConfig::from_parametric(TrackV2::default_closed_rectangle());
        LoadedConfig {
            project_path,
            project,
            robot,
            track,
        }
    }

    #[cfg(test)]
    mod stage9_gui_tests {
        use super::*;
        #[test]
        fn simulation_and_replay_panels_render_without_advancing_physics() {
            let mut app = RTSimApp::initial();
            app.sim_duration_s = 0.001;
            app.reset_simulation();
            let ctx = egui::Context::default();
            for _ in 0..3 {
                let _ = ctx.run(Default::default(), |ctx| {
                    app.poll_simulation(ctx);
                    egui::CentralPanel::default()
                        .show(ctx, |ui| app.show_visual_simulator(ui, ctx));
                });
            }
            assert!(app
                .sim_session
                .as_ref()
                .unwrap()
                .control
                .latest()
                .is_none_or(|p| p.steps == 0));
            app.sim_session = None;
            let config = app.cfg.clone().unwrap();
            let mut core = crate::sim::SimulationCore::new(config.clone(), Some(1000)).unwrap();
            let file = PathBuf::from("target")
                .join(format!("stage9-ui-{}.rtlog", crate::sim::new_run_id()));
            let mut writer =
                crate::replay::BinaryReplayLogger::create(&file, core.sample().sensor_adc.len())
                    .unwrap();
            writer.write_sample(&core.sample()).unwrap();
            core.advance_until(1000).unwrap();
            writer.write_sample(&core.sample()).unwrap();
            writer.finish("duration").unwrap();
            drop(writer);
            app.replay = Some(crate::replay::IndexedReplay::open(&file, 128 * 1024).unwrap());
            app.replay_config = Some(config);
            app.replay_time_s = 0.0005;
            let result = ctx.run(Default::default(), |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| app.show_replay_viewer(ui));
            });
            assert!(!result.shapes.is_empty());
            assert_eq!(core.time_us(), 1000);
        }
    }

    fn default_robot_config() -> RobotConfig {
        default_loaded_config(PathBuf::from("robot_default_tmp.rtsim")).robot
    }

    fn default_motor() -> MotorConfig {
        MotorConfig {
            nominal_voltage_v: 7.4,
            model: "DcMotorSimple".to_string(),
            gear_ratio: 30.0,
            efficiency: 0.75,
            no_load_rpm: 1800.0,
            stall_torque_nm: 0.005,
            stall_current_a: 1.6,
        }
    }

    fn default_sensor_asset() -> SensorAsset {
        SensorAsset {
            name: "Default Line Sensor".to_string(),
            model: "GenericAnalogLineSensor".to_string(),
            sensor_type: SensorType::LineAnalog,
            visual_width_m: 0.008,
            visual_height_m: 0.008,
            visual_radius_m: 0.004,
            detection_area: SensorDetectionArea::Rectangle {
                width_m: 0.005,
                height_m: 0.002,
            },
            response_model: SensorResponseModel::Ideal,
            notes: String::new(),
        }
    }

    fn default_sensor_instance() -> RobotSensorInstance {
        RobotSensorInstance {
            height_m: 0.0,
            acquisition: crate::config::SensorAcquisition::default(),
            id: crate::io::assets::new_instance_id(),
            name: "Front Left".to_string(),
            asset_path: PathBuf::from("RobotAssets/Sensors/default_line_sensor.json"),
            asset: default_sensor_asset(),
            position_m: Vec2::new(0.095, 0.025),
            angle_deg: 0.0,
            enabled: true,
            visible_in_preview: true,
        }
    }

    fn default_fan_config(nominal_voltage_v: f64) -> FanConfig {
        FanConfig {
            nominal_rpm: 30000.,
            id: crate::io::assets::new_instance_id(),
            position_m: Vec2::new(0.0, 0.0),
            visual_radius_m: 0.012,
            action_radius_m: 0.020,
            max_force_n: 0.50,
            max_current_a: 0.90,
            nominal_voltage_v,
            nominal_current_a: 0.70,
            power_w: nominal_voltage_v * 0.70,
            min_pwm: 0.0,
            max_pwm: 1.0,
            response_time_s: 0.03,
            pwm_scale: 1.0,
            enabled_pwm: 1.0,
            curve_model: FanCurveModel::LookupTable,
            force_curve: vec![(0.0, 0.0), (0.5, 0.18), (1.0, 0.50)],
        }
    }
}

#[cfg(feature = "gui")]
pub use gui::run_app;

#[cfg(not(feature = "gui"))]
pub fn run_app() -> Result<(), String> {
    Err("a interface gráfica atual foi adicionada atrás da feature 'gui'. Compile com: cargo run --features gui -- ui".to_string())
}
