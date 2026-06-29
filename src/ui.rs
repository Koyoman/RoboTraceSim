#[cfg(feature = "gui")]
mod gui {
    use crate::calibration::{
        compare_project_with_real, import_real_log, tune_project_against_real,
        write_comparison_csv, write_comparison_report, write_normalized_real_log,
        write_tuning_report, ComparisonMetrics,
    };
    use crate::config::load_project;
    use crate::config::{
        apply_surface_profile, load_surface_profile_from_file, load_track_from_file,
        refresh_track_cache, surface_profile_from_track, BatteryConfig, ChassisConfig,
        DrivetrainConfig, EncoderConfig, FanConfig, GyroConfig, LineSensorConfig, LoadedConfig,
        MotorConfig, NormalForceConfig, PidConfig, ProjectConfig, RobotConfig, SurfaceProfile,
        TimeConfig, TireConfig, TrackConfig,
    };
    use crate::math::{Pose2, Vec2};
    use crate::replay::{export_replay_to_csv, load_replay_samples, ReplayData};
    use crate::rtsim_track::{
        auto_close_with_straight, build_geometry, center_start_finish_on_segment,
        clamp_start_finish_to_segment, next_segment_id, resolve_robot_start_pose, resolve_rules,
        resolve_start_finish_markers, robot_start_allowed_area_corners,
        start_finish_required_length_mm, valid_start_finish_segments, validate_track, ArcSegment,
        MarkerSide, Severity, StartExitDirection, StraightSegment, TrackRulesMode, TrackSegment,
        TrackV2, ROBOT_START_MARKER_CLEARANCE_MM, ROBOT_START_MAX_LATERAL_OFFSET_MM,
    };
    use crate::sim::{run_simulation, RunOptions, SimulationSession};
    use crate::telemetry::TelemetrySample;
    use eframe::egui;
    use std::fs;
    use std::path::{Path, PathBuf};

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

    pub fn run_app() -> Result<(), String> {
        let options = eframe::NativeOptions::default();
        eframe::run_native(
            "Robotrace Sim v0.08",
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
        surface_profile_path_text: String,
        surface_profile_dirty: bool,
        status: String,
        selected_track_point: Option<usize>,
        track_view_zoom: f32,
        track_view_pan_m: Vec2,
        sim_session: Option<SimulationSession>,
        sim_running: bool,
        sim_steps_per_frame: u64,
        sim_duration_s: f64,
        last_sim_sample: Option<TelemetrySample>,
        replay: Option<ReplayData>,
        replay_index: usize,
        replay_max_samples: usize,
        real_log_path_text: String,
        calibration_csv_path_text: String,
        calibration_report_path_text: String,
        tuning_output_path_text: String,
        last_calibration_metrics: Option<ComparisonMetrics>,
    }

    impl RTSimApp {
        fn path_text(path: &Path) -> String {
            path.to_string_lossy().replace('\\', "/")
        }

        fn new(_cc: &eframe::CreationContext<'_>) -> Self {
            let mut app = Self {
                view: AppView::Home,
                cfg: None,
                project_path_text: "examples/basic/projeto.rtsim".to_string(),
                replay_path_text: "examples/basic/resultado.rtlog".to_string(),
                track_file_path_text: "examples/basic/track.json".to_string(),
                track_dirty: false,
                surface_profile_path_text: "examples/profiles/rob_trace_official.json".to_string(),
                surface_profile_dirty: false,
                status: "Abra um projeto .rtsim ou use o exemplo básico.".to_string(),
                selected_track_point: None,
                track_view_zoom: 1.0,
                track_view_pan_m: Vec2::new(0.0, 0.0),
                sim_session: None,
                sim_running: false,
                sim_steps_per_frame: 40,
                sim_duration_s: 10.0,
                last_sim_sample: None,
                replay: None,
                replay_index: 0,
                replay_max_samples: 200_000,
                real_log_path_text: "examples/basic/real_log_demo.csv".to_string(),
                calibration_csv_path_text: "examples/basic/comparacao_v05.csv".to_string(),
                calibration_report_path_text: "examples/basic/comparacao_v05.txt".to_string(),
                tuning_output_path_text: "examples/basic/ajuste_v05.json".to_string(),
                last_calibration_metrics: None,
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
                    self.track_file_path_text =
                        resolve_child_path(&cfg.project_path, &cfg.project.track_path)
                            .display()
                            .to_string();
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
                    self.surface_profile_dirty = false;
                    self.selected_track_point = None;
                    self.track_view_zoom = 1.0;
                    self.track_view_pan_m = Vec2::new(0.0, 0.0);
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.cfg = Some(cfg);
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
                    self.set_status("Projeto, robô e pista salvos.");
                }
                Err(err) => self.set_status(format!("Falha ao salvar: {err}")),
            }
        }

        fn update_project_start_pose_from_track(cfg: &mut LoadedConfig) {
            if let Some(parametric) = &cfg.track.parametric {
                if let Some(start_pose) = resolve_robot_start_pose(parametric) {
                    cfg.project.start_pose = Pose2::new(
                        start_pose.x_mm / 1000.0,
                        start_pose.y_mm / 1000.0,
                        start_pose.heading_deg.to_radians(),
                    );
                }
            }
        }

        fn reset_track_editor_state(&mut self) {
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
            let duration_us = Some((self.sim_duration_s.max(0.0) * 1_000_000.0).round() as u64);
            match self.cfg.clone() {
                Some(cfg) => match SimulationSession::new(cfg, duration_us) {
                    Ok(session) => {
                        self.last_sim_sample = Some(session.sample());
                        self.sim_session = Some(session);
                        self.sim_running = false;
                        self.set_status("Simulador visual reiniciado.");
                    }
                    Err(err) => {
                        self.set_status(format!("Falha ao iniciar simulação visual: {err}"))
                    }
                },
                None => self.set_status("Carregue ou crie um projeto antes de simular."),
            }
        }

        fn run_headless_replay(&mut self) {
            let Some(cfg) = self.cfg.clone() else {
                self.set_status("Carregue um projeto antes de gerar replay.");
                return;
            };
            let replay_path = default_replay_path(&cfg);
            let csv_path = default_csv_path(&cfg);
            let result = run_simulation(
                cfg,
                RunOptions {
                    duration_us: Some((self.sim_duration_s.max(0.0) * 1_000_000.0).round() as u64),
                    output_csv: Some(csv_path.clone()),
                    output_replay: Some(replay_path.clone()),
                    headless: true,
                    benchmark: false,
                    physics_dt_override_us: None,
                },
            );
            match result {
                Ok(summary) => {
                    self.replay_path_text = replay_path.display().to_string();
                    self.set_status(format!(
                        "Replay gerado: {} amostras em {:.3}s simulados.",
                        summary.steps, summary.simulated_time_s
                    ));
                }
                Err(err) => self.set_status(format!("Falha ao gerar replay: {err}")),
            }
        }

        fn load_replay(&mut self) {
            let path = PathBuf::from(self.replay_path_text.trim());
            match load_replay_samples(&path, self.replay_max_samples) {
                Ok(replay) => {
                    let count = replay.samples.len();
                    self.replay = Some(replay);
                    self.replay_index = 0;
                    self.view = AppView::ReplayViewer;
                    self.set_status(format!("Replay carregado com {count} amostras."));
                }
                Err(err) => self.set_status(format!("Falha ao carregar replay: {err}")),
            }
        }

        fn export_current_replay_to_csv(&mut self) {
            let replay_path = PathBuf::from(self.replay_path_text.trim());
            let csv_path = replay_path.with_extension("csv");
            match export_replay_to_csv(&replay_path, &csv_path) {
                Ok(rows) => self.set_status(format!(
                    "Replay exportado para {} com {rows} linhas.",
                    csv_path.display()
                )),
                Err(err) => self.set_status(format!("Falha ao exportar replay: {err}")),
            }
        }

        fn import_real_log_ui(&mut self) {
            let input = PathBuf::from(self.real_log_path_text.trim());
            let output = PathBuf::from(self.calibration_csv_path_text.trim())
                .with_file_name("real_normalized_v05.csv");
            match import_real_log(&input).and_then(|log| {
                write_normalized_real_log(&log, &output)
                    .map_err(|e| format!("Falha ao salvar log normalizado: {e}"))?;
                Ok((log.samples.len(), log.sensor_count))
            }) {
                Ok((samples, sensors)) => self.set_status(format!(
                    "Log real importado: {samples} amostras, {sensors} sensores. Normalizado em {}.",
                    output.display()
                )),
                Err(err) => self.set_status(format!("Falha ao importar log real: {err}")),
            }
        }

        fn compare_real_log_ui(&mut self) {
            let Some(cfg) = self.cfg.clone() else {
                self.set_status("Carregue um projeto antes de comparar com log real.");
                return;
            };
            let real_path = PathBuf::from(self.real_log_path_text.trim());
            let csv_path = PathBuf::from(self.calibration_csv_path_text.trim());
            let report_path = PathBuf::from(self.calibration_report_path_text.trim());
            let result = import_real_log(&real_path).and_then(|real| {
                let report = compare_project_with_real(cfg, &real, None)?;
                write_comparison_csv(&report, &csv_path)
                    .map_err(|e| format!("Falha ao salvar CSV de comparação: {e}"))?;
                write_comparison_report(&report, &report_path)
                    .map_err(|e| format!("Falha ao salvar relatório: {e}"))?;
                Ok(report.metrics)
            });
            match result {
                Ok(metrics) => {
                    self.last_calibration_metrics = Some(metrics.clone());
                    self.set_status(format!(
                        "Comparação concluída: erro RMS trajetória {:.4} m, velocidade {:.4} m/s, sensores {:.2} ADC.",
                        metrics.trajectory_error_m.rms,
                        metrics.speed_error_m_s.rms,
                        metrics.sensor_error_adc.rms
                    ));
                }
                Err(err) => self.set_status(format!("Falha na comparação: {err}")),
            }
        }

        fn tune_real_log_ui(&mut self) {
            let Some(cfg) = self.cfg.clone() else {
                self.set_status("Carregue um projeto antes de ajustar parâmetros.");
                return;
            };
            let real_path = PathBuf::from(self.real_log_path_text.trim());
            let output = PathBuf::from(self.tuning_output_path_text.trim());
            let result = import_real_log(&real_path).and_then(|real| {
                let report = tune_project_against_real(cfg, &real, None)?;
                write_tuning_report(&report, &output)
                    .map_err(|e| format!("Falha ao salvar ajuste: {e}"))?;
                Ok(report)
            });
            match result {
                Ok(report) => {
                    self.last_calibration_metrics = Some(report.best.metrics.clone());
                    self.set_status(format!(
                        "Ajuste concluído: score {:.5} -> {:.5}, μ={:.3}, escala torque={:.3}.",
                        report.baseline.score,
                        report.best.metrics.score,
                        report.best.mu_longitudinal,
                        report.best.stall_torque_scale
                    ));
                }
                Err(err) => self.set_status(format!("Falha no ajuste de parâmetros: {err}")),
            }
        }

        fn sidebar(&mut self, ctx: &egui::Context) {
            egui::SidePanel::left("main_navigation")
                .resizable(false)
                .default_width(190.0)
                .show(ctx, |ui| {
                    ui.heading("RTSim v0.09");
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
                        "Calibração v0.5",
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
                    self.project_path_text = "projeto_v05.rtsim".to_string();
                    self.track_file_path_text = "track.json".to_string();
                    self.surface_profile_path_text =
                        "examples/profiles/rob_trace_official.json".to_string();
                    self.track_dirty = false;
                    self.surface_profile_dirty = false;
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(
                        "Novo projeto v0.5 criado em memória. Ajuste e salve quando quiser.",
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
                        ui.label(format!(
                            "{} ADC {} bits",
                            cfg.robot.line_sensor.count, cfg.robot.line_sensor.adc_bits
                        ));
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
                    if ui.button("Abrir calibração v0.5").clicked() {
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
            let mut invalidate_sim = false;
            let mut status_to_set: Option<String> = None;
            let mut selected_track_point = self.selected_track_point;
            let mut track_view_zoom = self.track_view_zoom;
            let mut track_view_pan_m = self.track_view_pan_m;
            let mut track_file_path_text = self.track_file_path_text.clone();
            let mut surface_profile_path_text = self.surface_profile_path_text.clone();
            let mut track_file_command = TrackFileCommand::None;
            let mut surface_profile_command = SurfaceProfileCommand::None;
            let mut local_track_dirty = self.track_dirty;
            let mut local_surface_profile_dirty = self.surface_profile_dirty;

            if let Some(cfg) = self.cfg.as_mut() {
                let preview_track = cfg.track.clone();
                let preview_geometry = cfg.track.parametric.as_ref().map(build_geometry);
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

            match surface_profile_command {
                SurfaceProfileCommand::None => {}
                SurfaceProfileCommand::New => self.create_new_surface_profile(),
                SurfaceProfileCommand::Load => self.load_surface_profile_asset(),
                SurfaceProfileCommand::Save => self.save_surface_profile_asset(false),
                SurfaceProfileCommand::SaveAs => self.save_surface_profile_asset(true),
            }
        }

        fn show_robot_editor(&mut self, ui: &mut egui::Ui) {
            ui.heading("Editor de robô");
            let mut save_clicked = false;
            let mut invalidate_sim = false;
            if let Some(cfg) = self.cfg.as_mut() {
                ui.horizontal(|ui| {
                    ui.label("Nome");
                    if ui.text_edit_singleline(&mut cfg.robot.name).changed() {
                        invalidate_sim = true;
                    }
                    ui.label("Schema");
                    ui.text_edit_singleline(&mut cfg.robot.schema);
                });

                egui::CollapsingHeader::new("Chassi")
                    .default_open(true)
                    .show(ui, |ui| {
                        let c = &mut cfg.robot.chassis;
                        let mut mass_g = c.mass_kg * 1000.0;
                        let mut com_x_mm = c.center_of_mass_m.x * 1000.0;
                        let mut com_y_mm = c.center_of_mass_m.y * 1000.0;
                        let mut length_mm = c.length_m * 1000.0;
                        let mut width_mm = c.width_m * 1000.0;
                        ui.horizontal(|ui| {
                            ui.label("Massa [g]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut mass_g)
                                        .speed(1.0)
                                        .clamp_range(1.0..=5000.0),
                                )
                                .changed()
                            {
                                c.mass_kg = mass_g / 1000.0;
                                invalidate_sim = true;
                            }
                            ui.label("Inércia yaw [kg·m²]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut c.inertia_kg_m2)
                                        .speed(0.00001)
                                        .clamp_range(1e-8..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("COM x/y [mm]");
                            if ui
                                .add(egui::DragValue::new(&mut com_x_mm).speed(0.5))
                                .changed()
                            {
                                c.center_of_mass_m.x = com_x_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            if ui
                                .add(egui::DragValue::new(&mut com_y_mm).speed(0.5))
                                .changed()
                            {
                                c.center_of_mass_m.y = com_y_mm / 1000.0;
                                invalidate_sim = true;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Comprimento/largura [mm]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut length_mm)
                                        .speed(0.5)
                                        .clamp_range(1.0..=1000.0),
                                )
                                .changed()
                            {
                                c.length_m = length_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            if ui
                                .add(
                                    egui::DragValue::new(&mut width_mm)
                                        .speed(0.5)
                                        .clamp_range(1.0..=1000.0),
                                )
                                .changed()
                            {
                                c.width_m = width_mm / 1000.0;
                                invalidate_sim = true;
                            }
                        });
                    });

                egui::CollapsingHeader::new("Transmissão e rodas")
                    .default_open(true)
                    .show(ui, |ui| {
                        let d = &mut cfg.robot.drivetrain;
                        let mut radius_mm = d.wheel_radius_m * 1000.0;
                        let mut width_mm = d.wheel_width_m * 1000.0;
                        let mut track_width_mm = d.track_width_m * 1000.0;
                        let mut wheelbase_mm = d.wheelbase_m * 1000.0;
                        ui.horizontal(|ui| {
                            ui.label("Raio roda [mm]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut radius_mm)
                                        .speed(0.1)
                                        .clamp_range(1.0..=100.0),
                                )
                                .changed()
                            {
                                d.wheel_radius_m = radius_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            ui.label("Largura roda [mm]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut width_mm)
                                        .speed(0.1)
                                        .clamp_range(1.0..=100.0),
                                )
                                .changed()
                            {
                                d.wheel_width_m = width_mm / 1000.0;
                                invalidate_sim = true;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Bitola [mm]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut track_width_mm)
                                        .speed(0.5)
                                        .clamp_range(1.0..=500.0),
                                )
                                .changed()
                            {
                                d.track_width_m = track_width_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            ui.label("Entre-eixos [mm]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut wheelbase_mm)
                                        .speed(0.5)
                                        .clamp_range(1.0..=500.0),
                                )
                                .changed()
                            {
                                d.wheelbase_m = wheelbase_mm / 1000.0;
                                invalidate_sim = true;
                            }
                        });
                    });

                egui::CollapsingHeader::new("Pneu / atrito")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Modelo");
                            egui::ComboBox::from_id_source("tire_model")
                                .selected_text(cfg.robot.tire.model.as_str())
                                .show_ui(ui, |ui| {
                                    for model in [
                                        "IdealWheel",
                                        "CoulombFrictionWheel",
                                        "SlipRatioWheel",
                                        "LoadSensitiveWheel",
                                    ] {
                                        ui.selectable_value(
                                            &mut cfg.robot.tire.model,
                                            model.to_string(),
                                            model,
                                        );
                                    }
                                });
                        });
                        ui.horizontal(|ui| {
                            ui.label("μ longitudinal");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.tire.mu_longitudinal)
                                        .speed(0.01)
                                        .clamp_range(0.0..=5.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("μ lateral");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.tire.mu_lateral)
                                        .speed(0.01)
                                        .clamp_range(0.0..=5.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Rolamento");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.tire.rolling_resistance)
                                        .speed(0.001)
                                        .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                    });

                egui::CollapsingHeader::new("Motor, driver e bateria")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.columns(2, |cols| {
                            let (left_col, right_col) = cols.split_at_mut(1);
                            motor_editor(
                                &mut left_col[0],
                                "Motor esquerdo",
                                &mut cfg.robot.motor_left,
                                &mut invalidate_sim,
                            );
                            motor_editor(
                                &mut right_col[0],
                                "Motor direito",
                                &mut cfg.robot.motor_right,
                                &mut invalidate_sim,
                            );
                        });
                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.label("Driver");
                            ui.text_edit_singleline(&mut cfg.robot.driver.model);
                            ui.label("PWM [Hz]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.driver.pwm_frequency_hz)
                                        .speed(100.0)
                                        .clamp_range(10.0..=200_000.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Modo");
                            egui::ComboBox::from_id_source("driver_mode")
                                .selected_text(cfg.robot.driver.mode.as_str())
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut cfg.robot.driver.mode,
                                        "brake".to_string(),
                                        "brake",
                                    );
                                    ui.selectable_value(
                                        &mut cfg.robot.driver.mode,
                                        "coast".to_string(),
                                        "coast",
                                    );
                                });
                        });
                        ui.horizontal(|ui| {
                            ui.label("Queda driver [V]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.driver.voltage_drop_v)
                                        .speed(0.01)
                                        .clamp_range(0.0..=5.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Limite corrente [A]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.driver.current_limit_a)
                                        .speed(0.1)
                                        .clamp_range(0.0..=500.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.label("Bateria");
                            ui.text_edit_singleline(&mut cfg.robot.battery.model);
                            ui.label("Células");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.battery.cells)
                                        .clamp_range(1.0..=8.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("V nominal");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.battery.nominal_voltage_v)
                                        .speed(0.1),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("R interna [Ω]");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut cfg.robot.battery.internal_resistance_ohm,
                                    )
                                    .speed(0.001)
                                    .clamp_range(0.0..=10.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                    });

                egui::CollapsingHeader::new("Sensor, encoder, gyro e controle")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Sensores");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.line_sensor.count)
                                        .clamp_range(2.0..=64.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            let mut sensor_width_mm = cfg.robot.line_sensor.width_m * 1000.0;
                            let mut forward_mm = cfg.robot.line_sensor.forward_offset_m * 1000.0;
                            ui.label("Largura array [mm]");
                            if ui
                                .add(egui::DragValue::new(&mut sensor_width_mm).speed(0.5))
                                .changed()
                            {
                                cfg.robot.line_sensor.width_m = sensor_width_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            ui.label("Offset frontal [mm]");
                            if ui
                                .add(egui::DragValue::new(&mut forward_mm).speed(0.5))
                                .changed()
                            {
                                cfg.robot.line_sensor.forward_offset_m = forward_mm / 1000.0;
                                invalidate_sim = true;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("ADC bits");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.line_sensor.adc_bits)
                                        .clamp_range(1.0..=24.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Ruído reflectância");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut cfg.robot.line_sensor.reflectance_noise_std,
                                    )
                                    .speed(0.001)
                                    .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Ticks encoder/rev");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.encoder.ticks_per_rev)
                                        .clamp_range(1.0..=100_000.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Ruído gyro [rad/s]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.gyro.noise_std_rad_s)
                                        .speed(0.001)
                                        .clamp_range(0.0..=10.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.label("PID kp/ki/kd");
                            if ui
                                .add(egui::DragValue::new(&mut cfg.robot.controller.kp).speed(0.1))
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            if ui
                                .add(egui::DragValue::new(&mut cfg.robot.controller.ki).speed(0.01))
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.controller.kd).speed(0.001),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("PWM base");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.controller.base_pwm)
                                        .speed(0.01)
                                        .clamp_range(-1.0..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("PWM máx");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.controller.max_pwm)
                                        .speed(0.01)
                                        .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                    });

                egui::CollapsingHeader::new("Normal / downforce / sucção")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Modelo");
                            egui::ComboBox::from_id_source("normal_force_model")
                                .selected_text(cfg.robot.normal_force.model.as_str())
                                .show_ui(ui, |ui| {
                                    for model in [
                                        "NoDownforce",
                                        "ConstantDownforce",
                                        "FanDownforce",
                                        "SuctionDownforce",
                                        "MeasuredDownforceCurve",
                                    ] {
                                        ui.selectable_value(
                                            &mut cfg.robot.normal_force.model,
                                            model.to_string(),
                                            model,
                                        );
                                    }
                                });
                            ui.label("PWM padrão");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut cfg.robot.normal_force.command_pwm_default,
                                    )
                                    .speed(0.01)
                                    .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                cfg.robot.controller.downforce_pwm =
                                    cfg.robot.normal_force.command_pwm_default;
                                invalidate_sim = true;
                            }
                            ui.label("PWM controle");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.controller.downforce_pwm)
                                        .speed(0.01)
                                        .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Força máx [N]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.normal_force.max_force_n)
                                        .speed(0.01)
                                        .clamp_range(0.0..=100.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Corrente máx [A]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut cfg.robot.normal_force.max_current_a)
                                        .speed(0.01)
                                        .clamp_range(0.0..=100.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("Resposta [s]");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut cfg.robot.normal_force.response_time_s,
                                    )
                                    .speed(0.001)
                                    .clamp_range(0.0..=10.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                        ui.horizontal(|ui| {
                            let mut x_mm = cfg.robot.normal_force.position_m.x * 1000.0;
                            let mut y_mm = cfg.robot.normal_force.position_m.y * 1000.0;
                            ui.label("Posição aplicação x/y [mm]");
                            if ui.add(egui::DragValue::new(&mut x_mm).speed(0.5)).changed() {
                                cfg.robot.normal_force.position_m.x = x_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            if ui.add(egui::DragValue::new(&mut y_mm).speed(0.5)).changed() {
                                cfg.robot.normal_force.position_m.y = y_mm / 1000.0;
                                invalidate_sim = true;
                            }
                            ui.label("Área sucção [m²]");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut cfg.robot.normal_force.chamber_area_m2,
                                    )
                                    .speed(0.0001)
                                    .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                            ui.label("ΔP máx [Pa]");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut cfg.robot.normal_force.max_delta_pressure_pa,
                                    )
                                    .speed(10.0)
                                    .clamp_range(0.0..=100_000.0),
                                )
                                .changed()
                            {
                                invalidate_sim = true;
                            }
                        });
                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.strong("Fans");
                            if ui.button("Adicionar fan").clicked() {
                                cfg.robot.normal_force.fans.push(FanConfig {
                                    position_m: Vec2::new(0.03, 0.03),
                                    max_force_n: 0.5,
                                    max_current_a: 0.7,
                                    nominal_voltage_v: cfg.robot.battery.nominal_voltage_v,
                                    response_time_s: 0.03,
                                    pwm_scale: 1.0,
                                    enabled_pwm: 1.0,
                                    force_curve: vec![(0.0, 0.0), (1.0, 0.5)],
                                });
                                cfg.robot.normal_force.model = "FanDownforce".to_string();
                                invalidate_sim = true;
                            }
                        });
                        let mut remove_fan: Option<usize> = None;
                        for (idx, fan) in cfg.robot.normal_force.fans.iter_mut().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(format!("Fan {idx}"));
                                let mut x_mm = fan.position_m.x * 1000.0;
                                let mut y_mm = fan.position_m.y * 1000.0;
                                ui.label("x/y [mm]");
                                if ui.add(egui::DragValue::new(&mut x_mm).speed(0.5)).changed() {
                                    fan.position_m.x = x_mm / 1000.0;
                                    invalidate_sim = true;
                                }
                                if ui.add(egui::DragValue::new(&mut y_mm).speed(0.5)).changed() {
                                    fan.position_m.y = y_mm / 1000.0;
                                    invalidate_sim = true;
                                }
                                ui.label("Fmax [N]");
                                if ui
                                    .add(egui::DragValue::new(&mut fan.max_force_n).speed(0.01))
                                    .changed()
                                {
                                    invalidate_sim = true;
                                }
                                ui.label("Imax [A]");
                                if ui
                                    .add(egui::DragValue::new(&mut fan.max_current_a).speed(0.01))
                                    .changed()
                                {
                                    invalidate_sim = true;
                                }
                                if ui.button("remover").clicked() {
                                    remove_fan = Some(idx);
                                }
                            });
                        }
                        if let Some(idx) = remove_fan {
                            cfg.robot.normal_force.fans.remove(idx);
                            invalidate_sim = true;
                        }
                    });

                ui.separator();
                if ui.button("Salvar robô/projeto").clicked() {
                    save_clicked = true;
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Carregue um projeto para editar o robô.",
                );
            }
            if invalidate_sim {
                self.sim_session = None;
                self.last_sim_sample = None;
            }
            if save_clicked {
                self.save_current_project();
            }
        }

        fn show_visual_simulator(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
            ui.heading("Simulador visual");
            ui.horizontal(|ui| {
                ui.label("Duração [s]");
                if ui
                    .add(
                        egui::DragValue::new(&mut self.sim_duration_s)
                            .speed(0.1)
                            .clamp_range(0.0..=3600.0),
                    )
                    .changed()
                {
                    if let Some(cfg) = self.cfg.as_mut() {
                        cfg.project.duration_s = self.sim_duration_s;
                    }
                    self.sim_session = None;
                    self.last_sim_sample = None;
                }
                ui.label("Passos por frame");
                ui.add(
                    egui::DragValue::new(&mut self.sim_steps_per_frame)
                        .speed(1.0)
                        .clamp_range(1.0..=20_000.0),
                );
                if ui.button("Reset").clicked() {
                    self.reset_simulation();
                }
                if ui
                    .button(if self.sim_running { "Pausar" } else { "Play" })
                    .clicked()
                {
                    if self.sim_session.is_none() {
                        self.reset_simulation();
                    }
                    self.sim_running = !self.sim_running;
                }
                if ui.button("Step").clicked() {
                    if self.sim_session.is_none() {
                        self.reset_simulation();
                    }
                    if let Some(session) = self.sim_session.as_mut() {
                        self.last_sim_sample = Some(session.advance_steps(1));
                    }
                }
                if ui.button("Gerar replay headless").clicked() {
                    self.run_headless_replay();
                }
            });

            if self.sim_running {
                if self.sim_session.is_none() {
                    self.reset_simulation();
                }
                if let Some(session) = self.sim_session.as_mut() {
                    self.last_sim_sample = Some(session.advance_steps(self.sim_steps_per_frame));
                    if session.is_finished() {
                        self.sim_running = false;
                    }
                }
                ctx.request_repaint();
            }

            let progress = self
                .sim_session
                .as_ref()
                .map(|s| s.progress())
                .unwrap_or(0.0) as f32;
            ui.add(egui::ProgressBar::new(progress).show_percentage());

            ui.separator();
            if let Some(cfg) = &self.cfg {
                let sample = self
                    .sim_session
                    .as_ref()
                    .map(|s| s.sample())
                    .or_else(|| self.last_sim_sample.clone());
                let robot_pose = sample.as_ref().map(|s| Pose2::new(s.x_m, s.y_m, s.yaw_rad));
                draw_track_view(ui, &cfg.track, robot_pose, &[]);
                if let Some(sample) = sample {
                    telemetry_panel(ui, &sample);
                } else {
                    ui.label("Pressione Reset ou Play para iniciar a sessão visual.");
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Carregue um projeto antes de simular.",
                );
            }
        }

        fn show_replay_viewer(&mut self, ui: &mut egui::Ui) {
            ui.heading("Replay viewer");
            ui.horizontal(|ui| {
                ui.label("Replay .rtlog");
                ui.add(
                    egui::TextEdit::singleline(&mut self.replay_path_text)
                        .desired_width(f32::INFINITY),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Máx. amostras");
                ui.add(
                    egui::DragValue::new(&mut self.replay_max_samples)
                        .clamp_range(1.0..=2_000_000.0),
                );
                if ui.button("Carregar replay").clicked() {
                    self.load_replay();
                }
                if ui.button("Exportar CSV").clicked() {
                    self.export_current_replay_to_csv();
                }
                if ui.button("Gerar replay da simulação atual").clicked() {
                    self.run_headless_replay();
                }
            });
            ui.separator();

            if let Some(replay) = &self.replay {
                if !replay.samples.is_empty() {
                    let max_idx = replay.samples.len() - 1;
                    self.replay_index = self.replay_index.min(max_idx);
                    ui.add(egui::Slider::new(&mut self.replay_index, 0..=max_idx).text("amostra"));
                    let sample = replay.samples[self.replay_index].clone();
                    ui.label(format!(
                        "Amostra {}/{} | t = {:.6} s | sensores = {}",
                        self.replay_index + 1,
                        replay.samples.len(),
                        sample.t_us as f64 / 1_000_000.0,
                        replay.sensor_count
                    ));
                    if let Some(cfg) = &self.cfg {
                        draw_track_view(
                            ui,
                            &cfg.track,
                            Some(Pose2::new(sample.x_m, sample.y_m, sample.yaw_rad)),
                            &replay.samples[..=self.replay_index],
                        );
                    } else {
                        draw_replay_path_only(
                            ui,
                            &replay.samples[..=self.replay_index],
                            Some(&sample),
                        );
                    }
                    telemetry_panel(ui, &sample);
                } else {
                    ui.colored_label(
                        egui::Color32::from_rgb(170, 95, 0),
                        "Replay carregado sem amostras.",
                    );
                }
            } else {
                ui.label("Carregue um arquivo .rtlog para visualizar a trajetória e a telemetria.");
            }
        }

        fn show_calibration_tools(&mut self, ui: &mut egui::Ui) {
            ui.heading("Calibração v0.5 — comparação com dados reais");
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
                            *track_file_command = TrackFileCommand::Load;
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
                    let field_width = (ui.available_width() - 32.0).max(120.0);
                    ui.add_sized(
                        [field_width, 22.0],
                        egui::TextEdit::singleline(track_file_path_text),
                    );
                    if ui
                        .add_sized([26.0, 22.0], egui::Button::new("..."))
                        .clicked()
                    {
                        let tracks_dir = std::env::current_dir()
                            .unwrap_or_else(|_| std::path::PathBuf::from("."))
                            .join("Tracks");
                        if let Some(path) = rfd::FileDialog::new()
                            .set_title("Open Track JSON")
                            .set_directory(&tracks_dir)
                            .add_filter("JSON", &["json"])
                            .pick_file()
                        {
                            *track_file_path_text = path.to_string_lossy().replace('\\', "/");
                        }
                    }
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
                            *surface_profile_command = SurfaceProfileCommand::Load;
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

                    let field_width = (ui.available_width() - 32.0).max(120.0);
                    ui.add_sized(
                        [field_width, 22.0],
                        egui::TextEdit::singleline(surface_profile_path_text),
                    );

                    if ui
                        .add_sized([26.0, 22.0], egui::Button::new("..."))
                        .clicked()
                    {
                        let surface_dir = std::env::current_dir()
                            .unwrap_or_else(|_| std::path::PathBuf::from("."))
                            .join("SurfaceProfiles");
                        if let Some(path) = rfd::FileDialog::new()
                            .set_title("Open Surface Profile JSON")
                            .add_filter("JSON", &["json"])
                            .set_directory(&surface_dir)
                            .pick_file()
                        {
                            *surface_profile_path_text = path.to_string_lossy().replace('\\', "/");
                        }
                    }
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
                ui.label("Modelo");
                ui.text_edit_singleline(&mut motor.model);
            });
            ui.horizontal(|ui| {
                ui.label("Redução");
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
                ui.label("Eficiência");
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
                ui.label("RPM sem carga");
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
            ui, track, robot_pose, trail, height, &mut zoom, &mut pan_m,
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

        draw_track_geometry(&painter, rect, bounds, track, robot_pose, trail);

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
        draw_grid(painter, rect, bounds, track);
        if track.centerline.len() >= 2 {
            let px_width = world_len_to_screen(rect, bounds, track.line_width_m).max(2.0);
            for w in track.centerline.windows(2) {
                let a = world_to_screen(rect, bounds, w[0]);
                let b = world_to_screen(rect, bounds, w[1]);
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(px_width, egui::Color32::from_rgb(245, 245, 245)),
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

            let geometry = build_geometry(parametric);
            if let Some(resolved) = resolve_start_finish_markers(parametric) {
                if let Some(corners) = robot_start_allowed_area_corners(parametric) {
                    draw_robot_start_allowed_area(painter, rect, bounds, corners);
                }
                if let Some(robot_start_pose) = resolve_robot_start_pose(parametric) {
                    draw_robot_start_pose(painter, rect, bounds, robot_start_pose);
                }
                draw_track_marker(
                    painter,
                    rect,
                    bounds,
                    resolved.start_pose,
                    "START",
                    egui::Color32::from_rgb(80, 220, 120),
                    MarkerSide::Right,
                    track.line_width_m,
                );
                draw_track_marker(
                    painter,
                    rect,
                    bounds,
                    resolved.finish_pose,
                    "FINISH",
                    egui::Color32::from_rgb(255, 200, 70),
                    MarkerSide::Right,
                    track.line_width_m,
                );
            }
            if parametric.markings.corner_markers.auto_generate {
                let reverse = !parametric
                    .markings
                    .start_finish
                    .exit_direction
                    .is_increasing_s();
                for seg in &geometry.segment_poses {
                    if seg.kind == "arc" {
                        let mut start_pose = seg.start;
                        let mut end_pose = seg.end;
                        if reverse {
                            start_pose.heading_deg += 180.0;
                            end_pose.heading_deg += 180.0;
                        }
                        draw_track_marker(
                            painter,
                            rect,
                            bounds,
                            start_pose,
                            "CM",
                            egui::Color32::from_rgb(120, 200, 255),
                            MarkerSide::Left,
                            track.line_width_m,
                        );
                        draw_track_marker(
                            painter,
                            rect,
                            bounds,
                            end_pose,
                            "CM",
                            egui::Color32::from_rgb(120, 200, 255),
                            MarkerSide::Left,
                            track.line_width_m,
                        );
                    }
                }
            }
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

    fn draw_track_marker(
        painter: &egui::Painter,
        rect: egui::Rect,
        bounds: Bounds,
        pose: crate::rtsim_track::TrackPose,
        label: &str,
        color: egui::Color32,
        side: MarkerSide,
        line_width_m: f64,
    ) {
        let p = Vec2::new(pose.x_mm / 1000.0, pose.y_mm / 1000.0);
        let theta = pose.heading_deg.to_radians();
        let left_normal = Vec2::new(-theta.sin(), theta.cos());
        let side_sign = match side {
            MarkerSide::Left => 1.0,
            MarkerSide::Right => -1.0,
            MarkerSide::Center => 0.0,
        };
        let lateral_offset_m = side_sign * (line_width_m * 0.5 + 0.04);
        let center = Vec2::new(
            p.x + left_normal.x * lateral_offset_m,
            p.y + left_normal.y * lateral_offset_m,
        );
        let half_marker_len_m = 0.02;
        let a_world = Vec2::new(
            center.x - left_normal.x * half_marker_len_m,
            center.y - left_normal.y * half_marker_len_m,
        );
        let b_world = Vec2::new(
            center.x + left_normal.x * half_marker_len_m,
            center.y + left_normal.y * half_marker_len_m,
        );
        let a = world_to_screen(rect, bounds, a_world);
        let b = world_to_screen(rect, bounds, b_world);
        let pos = world_to_screen(rect, bounds, center);
        painter.line_segment([a, b], egui::Stroke::new(3.0, color));
        painter.text(
            pos + egui::vec2(6.0, 6.0),
            egui::Align2::LEFT_TOP,
            label,
            egui::FontId::proportional(11.0),
            color,
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
        let zoom = (zoom as f64).clamp(0.25, 12.0);
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

    fn default_csv_path(cfg: &LoadedConfig) -> PathBuf {
        cfg.project
            .csv_output
            .as_ref()
            .map(|p| resolve_child_path(&cfg.project_path, p))
            .unwrap_or_else(|| cfg.project_path.with_extension("csv"))
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

    fn save_track_to_file(track: &TrackConfig, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        fs::write(path, track_json(track)).map_err(|e| e.to_string())
    }

    fn save_surface_profile_to_file(profile: &SurfaceProfile, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        fs::write(path, surface_profile_json(profile)).map_err(|e| e.to_string())
    }

    fn save_loaded_config(cfg: &LoadedConfig) -> Result<(), String> {
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
        let robot_path = resolve_child_path(&cfg.project_path, &cfg.project.robot_path);
        let track_path = resolve_child_path(&cfg.project_path, &cfg.project.track_path);
        if let Some(parent) = robot_path.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        if let Some(parent) = track_path.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        fs::write(&cfg.project_path, project_json(&cfg.project)).map_err(|e| e.to_string())?;
        fs::write(&robot_path, robot_json(&cfg.robot)).map_err(|e| e.to_string())?;
        fs::write(&track_path, track_json(&cfg.track)).map_err(|e| e.to_string())?;
        Ok(())
    }

    fn project_json(project: &ProjectConfig) -> String {
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
            escape_json(&RTSimApp::path_text(&project.robot_path))
        ));
        out.push_str(&format!(
            "  \"track\": \"{}\",\n",
            escape_json(&RTSimApp::path_text(&project.track_path))
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
        out.push_str(&format!("    \"duration_s\": {:.9},\n", project.duration_s));
        out.push_str(&format!(
            "    \"start_pose_m\": [{:.9}, {:.9}, {:.9}]\n",
            project.start_pose.x, project.start_pose.y, project.start_pose.yaw
        ));
        out.push_str("  },\n");
        out.push_str("  \"log\": {\n");
        out.push_str(&format!(
            "    \"csv\": \"{}\",\n",
            escape_json(
                &project
                    .csv_output
                    .as_ref()
                    .map(|p| p.display().to_string())
                    .unwrap_or_else(|| "resultado.csv".to_string())
            )
        ));
        out.push_str(&format!(
            "    \"replay\": \"{}\"\n",
            escape_json(
                &project
                    .replay_output
                    .as_ref()
                    .map(|p| p.display().to_string())
                    .unwrap_or_else(|| "resultado.rtlog".to_string())
            )
        ));
        out.push_str("  }\n}\n");
        out
    }

    fn robot_json(robot: &RobotConfig) -> String {
        let mut out = String::new();
        out.push_str("{\n");
        out.push_str(&format!(
            "  \"robot_schema\": \"{}\",\n",
            escape_json(&robot.schema)
        ));
        out.push_str(&format!("  \"name\": \"{}\",\n", escape_json(&robot.name)));
        out.push_str("  \"chassis\": {\n");
        out.push_str(&format!("    \"model\": \"RigidBody2DChassis\",\n    \"mass_g\": {:.6},\n    \"inertia_kg_m2\": {:.12},\n    \"center_of_mass_mm\": [{:.6}, {:.6}],\n    \"length_mm\": {:.6},\n    \"width_mm\": {:.6}\n  }},\n",
            robot.chassis.mass_kg * 1000.0,
            robot.chassis.inertia_kg_m2,
            robot.chassis.center_of_mass_m.x * 1000.0,
            robot.chassis.center_of_mass_m.y * 1000.0,
            robot.chassis.length_m * 1000.0,
            robot.chassis.width_m * 1000.0));
        out.push_str(&normal_force_json(&robot.normal_force));
        out.push_str(",\n  \"drivetrain\": {\n");
        out.push_str(&format!("    \"type\": \"DifferentialDrive4Wheel\",\n    \"wheel_radius_mm\": {:.6},\n    \"wheel_width_mm\": {:.6},\n    \"track_width_mm\": {:.6},\n    \"wheelbase_mm\": {:.6},\n    \"wheel_inertia_g_cm2\": {:.9}\n  }},\n",
            robot.drivetrain.wheel_radius_m * 1000.0,
            robot.drivetrain.wheel_width_m * 1000.0,
            robot.drivetrain.track_width_m * 1000.0,
            robot.drivetrain.wheelbase_m * 1000.0,
            robot.drivetrain.wheel_inertia_kg_m2 / 1e-7));
        out.push_str(&format!("  \"tire\": {{\n    \"model\": \"{}\",\n    \"mu_longitudinal\": {:.9},\n    \"mu_lateral\": {:.9},\n    \"rolling_resistance\": {:.9},\n    \"slip_velocity_epsilon_m_s\": {:.9}\n  }},\n",
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
        out.push_str(&format!("  \"driver\": {{\n    \"model\": \"{}\",\n    \"pwm_frequency_hz\": {:.6},\n    \"mode\": \"{}\",\n    \"voltage_drop_v\": {:.9},\n    \"pwm_resolution_bits\": {},\n    \"command_deadband\": {:.9},\n    \"current_limit_a\": {:.9}\n  }},\n",
            escape_json(&robot.driver.model), robot.driver.pwm_frequency_hz, escape_json(&robot.driver.mode), robot.driver.voltage_drop_v, robot.driver.pwm_resolution_bits, robot.driver.command_deadband, robot.driver.current_limit_a));
        out.push_str(&format!("  \"battery\": {{\n    \"model\": \"{}\",\n    \"nominal_voltage_v\": {:.9},\n    \"full_voltage_v\": {:.9},\n    \"empty_voltage_v\": {:.9},\n    \"cells\": {},\n    \"capacity_mah\": {:.9},\n    \"internal_resistance_ohm\": {:.9},\n    \"initial_soc\": {:.9},\n    \"current_limit_a\": {:.9}\n  }},\n",
            escape_json(&robot.battery.model), robot.battery.nominal_voltage_v, robot.battery.full_voltage_v, robot.battery.empty_voltage_v, robot.battery.cells, robot.battery.capacity_mah, robot.battery.internal_resistance_ohm, robot.battery.initial_soc, robot.battery.current_limit_a));
        out.push_str(&format!("  \"line_sensor\": {{\n    \"model\": \"{}\",\n    \"count\": {},\n    \"width_mm\": {:.6},\n    \"forward_offset_mm\": {:.6},\n    \"adc_bits\": {},\n    \"gain\": {:.9},\n    \"offset\": {:.9},\n    \"reflectance_noise_std\": {:.9},\n    \"adc_noise_lsb\": {:.9},\n    \"seed\": {}\n  }},\n",
            escape_json(&robot.line_sensor.model), robot.line_sensor.count, robot.line_sensor.width_m * 1000.0, robot.line_sensor.forward_offset_m * 1000.0, robot.line_sensor.adc_bits, robot.line_sensor.gain, robot.line_sensor.offset, robot.line_sensor.reflectance_noise_std, robot.line_sensor.adc_noise_lsb, robot.line_sensor.seed));
        out.push_str(&format!("  \"encoder\": {{\n    \"model\": \"{}\",\n    \"ticks_per_rev\": {},\n    \"invert_left\": {},\n    \"invert_right\": {}\n  }},\n",
            escape_json(&robot.encoder.model), robot.encoder.ticks_per_rev, robot.encoder.invert_left, robot.encoder.invert_right));
        out.push_str(&format!("  \"gyro\": {{\n    \"model\": \"{}\",\n    \"noise_std_rad_s\": {:.9},\n    \"bias_rad_s\": {:.9},\n    \"saturation_rad_s\": {:.9},\n    \"seed\": {}\n  }},\n",
            escape_json(&robot.gyro.model), robot.gyro.noise_std_rad_s, robot.gyro.bias_rad_s, robot.gyro.saturation_rad_s, robot.gyro.seed));
        out.push_str(&format!("  \"controller\": {{\n    \"model\": \"BuiltInPid\",\n    \"kp\": {:.9},\n    \"ki\": {:.9},\n    \"kd\": {:.9},\n    \"base_pwm\": {:.9},\n    \"max_pwm\": {:.9},\n    \"target_position_mm\": {:.9},\n    \"downforce_pwm\": {:.9}\n  }}\n}}\n",
            robot.controller.kp, robot.controller.ki, robot.controller.kd, robot.controller.base_pwm, robot.controller.max_pwm, robot.controller.target_position_m * 1000.0, robot.controller.downforce_pwm));
        out
    }

    fn normal_force_json(normal: &NormalForceConfig) -> String {
        let mut out = String::new();
        out.push_str("  \"normal_force\": {\n");
        out.push_str(&format!(
            "    \"model\": \"{}\",\n",
            escape_json(&normal.model)
        ));
        out.push_str(&format!(
            "    \"default_pwm\": {:.9},\n",
            normal.command_pwm_default
        ));
        out.push_str(&format!(
            "    \"position_mm\": [{:.6}, {:.6}],\n",
            normal.position_m.x * 1000.0,
            normal.position_m.y * 1000.0
        ));
        out.push_str(&format!(
            "    \"max_force_n\": {:.9},\n",
            normal.max_force_n
        ));
        out.push_str(&format!(
            "    \"max_current_a\": {:.9},\n",
            normal.max_current_a
        ));
        out.push_str(&format!(
            "    \"response_time_s\": {:.9},\n",
            normal.response_time_s
        ));
        out.push_str(&format!(
            "    \"chamber_area_m2\": {:.9},\n",
            normal.chamber_area_m2
        ));
        out.push_str(&format!(
            "    \"max_delta_pressure_pa\": {:.9},\n",
            normal.max_delta_pressure_pa
        ));
        out.push_str(&format!(
            "    \"leakage_factor\": {:.9},\n",
            normal.leakage_factor
        ));
        out.push_str(&format!(
            "    \"speed_sensitivity\": {:.9},\n",
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
                out.push_str(&format!("      {{\n        \"position_mm\": [{:.6}, {:.6}],\n        \"max_force_n\": {:.9},\n        \"max_current_a\": {:.9},\n        \"nominal_voltage_v\": {:.9},\n        \"response_time_s\": {:.9},\n        \"pwm_scale\": {:.9},\n        \"pwm\": {:.9},\n        \"force_curve\": {}\n      }}",
                    fan.position_m.x * 1000.0, fan.position_m.y * 1000.0, fan.max_force_n, fan.max_current_a, fan.nominal_voltage_v, fan.response_time_s, fan.pwm_scale, fan.enabled_pwm, curve_json(&fan.force_curve)));
            }
            out.push('\n');
            out.push_str("    ");
        }
        out.push_str("]\n  }");
        out
    }

    fn motor_json(motor: &MotorConfig, indent: usize) -> String {
        let pad = " ".repeat(indent);
        format!(
            "{{\n{pad}  \"model\": \"{}\",\n{pad}  \"gear_ratio\": {:.9},\n{pad}  \"efficiency\": {:.9},\n{pad}  \"no_load_rpm\": {:.9},\n{pad}  \"stall_torque_mnm\": {:.9},\n{pad}  \"stall_current_a\": {:.9}\n{pad}}}",
            escape_json(&motor.model),
            motor.gear_ratio,
            motor.efficiency,
            motor.no_load_rpm,
            motor.stall_torque_nm * 1000.0,
            motor.stall_current_a,
        )
    }

    fn track_json(track: &TrackConfig) -> String {
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
            "  \"line_width_mm\": {:.6},\n",
            track.line_width_m * 1000.0
        ));
        out.push_str(&format!(
            "  \"base_reflectance\": {:.9},\n",
            track.base_reflectance
        ));
        out.push_str(&format!(
            "  \"line_reflectance\": {:.9},\n",
            track.line_reflectance
        ));
        out.push_str(&format!("  \"surface_mu\": {:.9},\n", track.surface_mu));
        out.push_str("  \"centerline_m\": [\n");
        for (i, p) in track.centerline.iter().enumerate() {
            let suffix = if i + 1 == track.centerline.len() {
                ""
            } else {
                ","
            };
            out.push_str(&format!("    [{:.9}, {:.9}]{}\n", p.x, p.y, suffix));
        }
        out.push_str("  ]\n}\n");
        out
    }

    fn track_v2_json(track: &TrackV2) -> String {
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
        out.push_str(&format!("    \"width_mm\": {:.6},\n", track.area.width_mm));
        out.push_str(&format!(
            "    \"height_mm\": {:.6},\n",
            track.area.height_mm
        ));
        out.push_str(&format!("    \"grid_mm\": {:.6}\n", track.area.grid_mm));
        out.push_str("  },\n");
        out.push_str("  \"origin\": {\n");
        out.push_str(&format!("    \"x_mm\": {:.6},\n", track.origin.x_mm));
        out.push_str(&format!("    \"y_mm\": {:.6},\n", track.origin.y_mm));
        out.push_str(&format!(
            "    \"heading_deg\": {:.9}\n",
            track.origin.heading_deg
        ));
        out.push_str("  },\n");
        out.push_str("  \"rules\": {\n");
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
            "    \"base_reflectance\": {:.9},\n",
            track.surface.base_reflectance
        ));
        out.push_str(&format!(
            "    \"line_reflectance\": {:.9},\n",
            track.surface.line_reflectance
        ));
        out.push_str(&format!(
            "    \"surface_mu\": {:.9}\n",
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
                        "    {{ \"id\": \"{}\", \"kind\": \"straight\", \"length_mm\": {:.6} }}{}\n",
                        escape_json(&straight.id), straight.length_mm, suffix
                    ));
                }
                TrackSegment::Arc(arc) => {
                    out.push_str(&format!(
                        "    {{ \"id\": \"{}\", \"kind\": \"arc\", \"radius_mm\": {:.6}, \"sweep_deg\": {:.9} }}{}\n",
                        escape_json(&arc.id), arc.radius_mm, arc.sweep_deg, suffix
                    ));
                }
            }
        }
        out.push_str("  ],\n");
        out.push_str("  \"closure\": {\n");
        out.push_str(&format!("    \"required\": {},\n", track.closure.required));
        out.push_str(&format!(
            "    \"position_tolerance_mm\": {:.6},\n",
            track.closure.position_tolerance_mm
        ));
        out.push_str(&format!(
            "    \"heading_tolerance_deg\": {:.9}\n",
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
            "      \"start_s_mm\": {:.6},\n",
            track.markings.start_finish.start_s_mm
        ));
        out.push_str(&format!(
            "      \"distance_mm\": {:.6},\n",
            track.markings.start_finish.distance_mm
        ));
        out.push_str(&format!(
            "      \"margin_mm\": {:.6},\n",
            track.markings.start_finish.margin_mm
        ));
        out.push_str(&format!(
            "      \"exit_direction\": \"{}\",\n",
            track.markings.start_finish.exit_direction.as_str()
        ));
        out.push_str("      \"robot_start\": {\n");
        out.push_str(&format!(
            "        \"delta_x_mm\": {:.6},\n",
            track.markings.start_finish.robot_start.delta_x_mm
        ));
        out.push_str(&format!(
            "        \"delta_y_mm\": {:.6},\n",
            track.markings.start_finish.robot_start.delta_y_mm
        ));
        out.push_str(&format!(
            "        \"heading_deg\": {:.9}\n",
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

    fn surface_profile_json(profile: &SurfaceProfile) -> String {
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
            out.push_str(&format!("  \"line_width_mm\": {:.9},\n", line_width_mm));
        }
        out.push_str(&format!(
            "  \"background_reflectance\": {:.9},\n",
            profile.background_reflectance
        ));
        out.push_str(&format!(
            "  \"line_reflectance\": {:.9},\n",
            profile.line_reflectance
        ));
        out.push_str(&format!(
            "  \"marker_profile\": \"{}\",\n",
            escape_json(&profile.marker_profile)
        ));
        out.push_str("  \"rules\": {\n");
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
            "    \"base_reflectance\": {:.9},\n",
            profile.background_reflectance
        ));
        out.push_str(&format!(
            "    \"line_reflectance\": {:.9},\n",
            profile.line_reflectance
        ));
        out.push_str(&format!("    \"surface_mu\": {:.9}\n", profile.surface_mu));
        out.push_str("  }\n");
        out.push_str("}\n");
        out
    }

    fn push_opt_num(fields: &mut Vec<String>, key: &str, value: Option<f64>) {
        if let Some(value) = value {
            fields.push(format!("\"{}\": {:.9}", key, value));
        }
    }

    fn push_opt_bool(fields: &mut Vec<String>, key: &str, value: Option<bool>) {
        if let Some(value) = value {
            fields.push(format!("\"{}\": {}", key, value));
        }
    }

    fn curve_json(curve: &[(f64, f64)]) -> String {
        let mut out = String::from("[");
        for (idx, (x, y)) in curve.iter().enumerate() {
            if idx > 0 {
                out.push_str(", ");
            }
            out.push_str(&format!("[{:.9}, {:.9}]", x, y));
        }
        out.push(']');
        out
    }

    fn escape_json(input: &str) -> String {
        input
            .replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "\\r")
            .replace('\t', "\\t")
    }

    fn default_loaded_config(project_path: PathBuf) -> LoadedConfig {
        let project = ProjectConfig {
            schema: "rtsim-project-v1".to_string(),
            name: "novo-projeto-v0.5".to_string(),
            robot_path: PathBuf::from("robot.json"),
            track_path: PathBuf::from("track.json"),
            time: TimeConfig::default(),
            duration_s: 10.0,
            start_pose: Pose2::new(0.7, 0.5, 0.0),
            csv_output: Some(PathBuf::from("resultado.csv")),
            replay_output: Some(PathBuf::from("resultado.rtlog")),
        };
        let robot = RobotConfig {
            schema: "rtsim-robot-v4".to_string(),
            name: "Simple N20 PID Robot v0.5".to_string(),
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
                model: "NoDownforce".to_string(),
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
            line_sensor: LineSensorConfig {
                model: "NoisyAdcSensor".to_string(),
                count: 16,
                width_m: 0.072,
                forward_offset_m: 0.055,
                adc_bits: 12,
                gain: 1.0,
                offset: 0.0,
                reflectance_noise_std: 0.01,
                adc_noise_lsb: 1.0,
                seed: 1371,
            },
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

    fn default_motor() -> MotorConfig {
        MotorConfig {
            model: "DcMotorSimple".to_string(),
            gear_ratio: 30.0,
            efficiency: 0.75,
            no_load_rpm: 1800.0,
            stall_torque_nm: 0.005,
            stall_current_a: 1.6,
        }
    }
}

#[cfg(feature = "gui")]
pub use gui::run_app;

#[cfg(not(feature = "gui"))]
pub fn run_app() -> Result<(), String> {
    Err("a interface gráfica v0.5 foi adicionada atrás da feature 'gui'. Compile com: cargo run --features gui -- ui".to_string())
}
