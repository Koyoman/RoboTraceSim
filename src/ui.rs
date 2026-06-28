#[cfg(feature = "gui")]
mod gui {
    use crate::config::load_project;
    use crate::config::{
        BatteryConfig, ChassisConfig, DrivetrainConfig, EncoderConfig, FanConfig, GyroConfig,
        LineSensorConfig, LoadedConfig, MotorConfig, NormalForceConfig, PidConfig, ProjectConfig,
        RobotConfig, TimeConfig, TireConfig, TrackConfig,
    };
    use crate::math::{Pose2, Vec2};
    use crate::replay::{export_replay_to_csv, load_replay_samples, ReplayData};
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
    }

    pub fn run_app() -> Result<(), String> {
        let options = eframe::NativeOptions::default();
        eframe::run_native(
            "Robotrace Sim v0.4",
            options,
            Box::new(|cc| Box::new(RTSimApp::new(cc))),
        )
        .map_err(|err| err.to_string())
    }

    struct RTSimApp {
        view: AppView,
        cfg: Option<LoadedConfig>,
        project_path_text: String,
        replay_path_text: String,
        status: String,
        selected_track_point: Option<usize>,
        sim_session: Option<SimulationSession>,
        sim_running: bool,
        sim_steps_per_frame: u64,
        sim_duration_s: f64,
        last_sim_sample: Option<TelemetrySample>,
        replay: Option<ReplayData>,
        replay_index: usize,
        replay_max_samples: usize,
    }

    impl RTSimApp {
        fn new(_cc: &eframe::CreationContext<'_>) -> Self {
            let mut app = Self {
                view: AppView::Home,
                cfg: None,
                project_path_text: "examples/basic/projeto.rtsim".to_string(),
                replay_path_text: "examples/basic/resultado.rtlog".to_string(),
                status: "Abra um projeto .rtsim ou use o exemplo básico.".to_string(),
                selected_track_point: None,
                sim_session: None,
                sim_running: false,
                sim_steps_per_frame: 40,
                sim_duration_s: 10.0,
                last_sim_sample: None,
                replay: None,
                replay_index: 0,
                replay_max_samples: 200_000,
            };
            if Path::new(&app.project_path_text).exists() {
                app.load_project_from_path(PathBuf::from(app.project_path_text.clone()));
            } else {
                app.cfg = Some(default_loaded_config(PathBuf::from("projeto_v04.rtsim")));
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
                    self.selected_track_point = None;
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
                Ok(()) => self.set_status("Projeto, robô e pista salvos."),
                Err(err) => self.set_status(format!("Falha ao salvar: {err}")),
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

        fn sidebar(&mut self, ctx: &egui::Context) {
            egui::SidePanel::left("main_navigation")
                .resizable(false)
                .default_width(190.0)
                .show(ctx, |ui| {
                    ui.heading("RTSim v0.4");
                    ui.label("Interface única");
                    ui.separator();
                    nav_button(ui, &mut self.view, AppView::Home, "Home");
                    nav_button(ui, &mut self.view, AppView::TrackEditor, "Editor de pista");
                    nav_button(ui, &mut self.view, AppView::RobotEditor, "Editor de robô");
                    nav_button(
                        ui,
                        &mut self.view,
                        AppView::VisualSimulator,
                        "Simulador visual",
                    );
                    nav_button(ui, &mut self.view, AppView::ReplayViewer, "Replay viewer");
                    ui.separator();
                    if ui.button("Salvar tudo").clicked() {
                        self.save_current_project();
                    }
                    if ui.button("Recarregar projeto").clicked() {
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
                    self.cfg = Some(default_loaded_config(PathBuf::from("projeto_v04.rtsim")));
                    self.project_path_text = "projeto_v04.rtsim".to_string();
                    self.sim_session = None;
                    self.last_sim_sample = None;
                    self.set_status(
                        "Novo projeto v0.4 criado em memória. Ajuste e salve quando quiser.",
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
                });
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Nenhum projeto carregado.",
                );
            }
        }

        fn show_track_editor(&mut self, ui: &mut egui::Ui) {
            ui.heading("Editor de pista");
            let mut save_clicked = false;
            let mut invalidate_sim = false;
            if let Some(cfg) = self.cfg.as_mut() {
                ui.horizontal(|ui| {
                    ui.label("Nome");
                    if ui.text_edit_singleline(&mut cfg.track.name).changed() {
                        invalidate_sim = true;
                    }
                    ui.label("Modelo");
                    ui.text_edit_singleline(&mut cfg.track.model);
                });
                ui.horizontal(|ui| {
                    let mut line_width_mm = cfg.track.line_width_m * 1000.0;
                    ui.label("Largura da linha [mm]");
                    if ui
                        .add(
                            egui::DragValue::new(&mut line_width_mm)
                                .speed(0.1)
                                .clamp_range(1.0..=100.0),
                        )
                        .changed()
                    {
                        cfg.track.line_width_m = line_width_mm / 1000.0;
                        invalidate_sim = true;
                    }
                    ui.label("Atrito superfície μ");
                    if ui
                        .add(
                            egui::DragValue::new(&mut cfg.track.surface_mu)
                                .speed(0.01)
                                .clamp_range(0.05..=5.0),
                        )
                        .changed()
                    {
                        invalidate_sim = true;
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Refletância base");
                    if ui
                        .add(
                            egui::DragValue::new(&mut cfg.track.base_reflectance)
                                .speed(0.01)
                                .clamp_range(0.0..=1.0),
                        )
                        .changed()
                    {
                        invalidate_sim = true;
                    }
                    ui.label("Refletância linha");
                    if ui
                        .add(
                            egui::DragValue::new(&mut cfg.track.line_reflectance)
                                .speed(0.01)
                                .clamp_range(0.0..=1.0),
                        )
                        .changed()
                    {
                        invalidate_sim = true;
                    }
                });
                ui.separator();

                let canvas_response =
                    draw_editable_track_canvas(ui, &mut cfg.track, &mut self.selected_track_point);
                if canvas_response.changed {
                    invalidate_sim = true;
                }
                ui.small("Arraste um ponto no canvas para mover a geometria da linha. Clique em um ponto para selecioná-lo.");

                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Adicionar ponto no final").clicked() {
                        let new_point = cfg
                            .track
                            .centerline
                            .last()
                            .copied()
                            .map(|p| Vec2::new(p.x + 0.25, p.y))
                            .unwrap_or(Vec2::new(0.0, 0.0));
                        cfg.track.centerline.push(new_point);
                        self.selected_track_point = Some(cfg.track.centerline.len() - 1);
                        invalidate_sim = true;
                    }
                    if ui.button("Remover selecionado").clicked() {
                        if let Some(idx) = self.selected_track_point {
                            if cfg.track.centerline.len() > 2 && idx < cfg.track.centerline.len() {
                                cfg.track.centerline.remove(idx);
                                self.selected_track_point = None;
                                invalidate_sim = true;
                            }
                        }
                    }
                    if ui.button("Salvar pista/projeto").clicked() {
                        save_clicked = true;
                    }
                });

                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .show(ui, |ui| {
                        egui::Grid::new("track_points_grid")
                            .striped(true)
                            .num_columns(4)
                            .show(ui, |ui| {
                                ui.strong("#");
                                ui.strong("x [m]");
                                ui.strong("y [m]");
                                ui.strong("Selecionar");
                                ui.end_row();
                                for (i, point) in cfg.track.centerline.iter_mut().enumerate() {
                                    ui.label(i.to_string());
                                    if ui
                                        .add(egui::DragValue::new(&mut point.x).speed(0.005))
                                        .changed()
                                    {
                                        invalidate_sim = true;
                                    }
                                    if ui
                                        .add(egui::DragValue::new(&mut point.y).speed(0.005))
                                        .changed()
                                    {
                                        invalidate_sim = true;
                                    }
                                    if ui
                                        .selectable_label(
                                            self.selected_track_point == Some(i),
                                            "selecionar",
                                        )
                                        .clicked()
                                    {
                                        self.selected_track_point = Some(i);
                                    }
                                    ui.end_row();
                                }
                            });
                    });
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(170, 95, 0),
                    "Carregue um projeto para editar a pista.",
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
        painter.rect_filled(rect, 6.0, egui::Color32::from_gray(245));
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
        let desired = egui::vec2(ui.available_width(), 440.0);
        let (_response, painter) = ui.allocate_painter(desired, egui::Sense::hover());
        let rect = _response.rect;
        painter.rect_filled(rect, 6.0, egui::Color32::from_gray(245));
        let bounds = track_bounds(track, robot_pose.map(|p| Vec2::new(p.x, p.y)));
        draw_track_geometry(&painter, rect, bounds, track, robot_pose, trail);
    }

    fn draw_replay_path_only(
        ui: &mut egui::Ui,
        trail: &[TelemetrySample],
        sample: Option<&TelemetrySample>,
    ) {
        let desired = egui::vec2(ui.available_width(), 440.0);
        let (_response, painter) = ui.allocate_painter(desired, egui::Sense::hover());
        let rect = _response.rect;
        painter.rect_filled(rect, 6.0, egui::Color32::from_gray(245));
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
        if track.centerline.len() >= 2 {
            for w in track.centerline.windows(2) {
                let a = world_to_screen(rect, bounds, w[0]);
                let b = world_to_screen(rect, bounds, w[1]);
                let px_width = world_len_to_screen(rect, bounds, track.line_width_m).max(2.0);
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(px_width, egui::Color32::from_rgb(25, 25, 25)),
                );
                painter.line_segment(
                    [a, b],
                    egui::Stroke::new(1.0, egui::Color32::from_gray(100)),
                );
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
                    egui::Stroke::new(1.5, egui::Color32::from_rgb(40, 120, 220)),
                );
            }
        }
        if let Some(pose) = robot_pose {
            draw_robot(painter, rect, bounds, pose, 0.12, 0.09);
        }
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

    fn save_loaded_config(cfg: &LoadedConfig) -> Result<(), String> {
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
            escape_json(&project.robot_path.display().to_string())
        ));
        out.push_str(&format!(
            "  \"track\": \"{}\",\n",
            escape_json(&project.track_path.display().to_string())
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
            name: "novo-projeto-v0.4".to_string(),
            robot_path: PathBuf::from("robot.json"),
            track_path: PathBuf::from("track.json"),
            time: TimeConfig::default(),
            duration_s: 10.0,
            start_pose: Pose2::new(0.0, 0.035, 0.0),
            csv_output: Some(PathBuf::from("resultado.csv")),
            replay_output: Some(PathBuf::from("resultado.rtlog")),
        };
        let robot = RobotConfig {
            schema: "rtsim-robot-v4".to_string(),
            name: "Simple N20 PID Robot v0.4".to_string(),
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
        let track = TrackConfig {
            schema: "rtsim-track-v1".to_string(),
            name: "Simple Vector S-Curve".to_string(),
            model: "VectorTrack".to_string(),
            line_width_m: 0.019,
            base_reflectance: 0.86,
            line_reflectance: 0.08,
            surface_mu: 1.20,
            centerline: vec![
                Vec2::new(0.0, 0.00),
                Vec2::new(0.5, 0.00),
                Vec2::new(1.0, 0.04),
                Vec2::new(1.5, 0.10),
                Vec2::new(2.0, 0.10),
                Vec2::new(2.5, 0.03),
                Vec2::new(3.0, -0.04),
                Vec2::new(3.5, -0.08),
                Vec2::new(4.0, -0.02),
                Vec2::new(4.5, 0.05),
                Vec2::new(5.0, 0.00),
            ],
        };
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
    Err("a interface gráfica v0.4 foi adicionada atrás da feature 'gui'. Compile com: cargo run --features gui -- ui".to_string())
}
