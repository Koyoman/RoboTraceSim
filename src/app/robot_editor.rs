use super::*;
use crate::models::robot::*;
#[allow(clippy::too_many_arguments)]
pub(super) fn edit_robot_properties_panel(
    ui: &mut egui::Ui,
    robot: &mut RobotConfig,
    robot_file_path_text: &mut String,
    robot_dirty: bool,
    robot_file_command: &mut RobotFileCommand,
    motor_left_asset_path_text: &mut String,
    driver_asset_path_text: &mut String,
    battery_asset_path_text: &mut String,
    tire_asset_path_text: &mut String,
    fan_asset_path_text: &mut String,
    encoder_asset_path_text: &mut String,
    gyro_asset_path_text: &mut String,
    selected_fan_asset_index: &mut usize,
    project_path: Option<&Path>,
    component_asset_command: &mut Option<ComponentAssetCommand>,
    status_to_set: &mut Option<String>,
) -> bool {
    let mut changed = false;
    let panel_width = ui.available_width();

    egui::Frame::group(ui.style())
        .inner_margin(egui::Margin::same(8.0))
        .show(ui, |ui| {
            ui.set_width(panel_width);
            ui.horizontal(|ui| {
                if robot_dirty {
                    ui.strong("Robot File *modified");
                } else {
                    ui.strong("Robot File");
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .add_sized([70.0, 22.0], egui::Button::new("Save As"))
                        .clicked()
                    {
                        let robots_dir = std::env::current_dir()
                            .unwrap_or_else(|_| PathBuf::from("."))
                            .join("Robots");
                        let _ = fs::create_dir_all(&robots_dir);
                        if let Some(path) = rfd::FileDialog::new()
                            .set_title("Save Robot JSON")
                            .set_file_name(json_file_name_from_name(&robot.name, "robot"))
                            .add_filter("JSON", &["json"])
                            .set_directory(&robots_dir)
                            .save_file()
                        {
                            *robot_file_path_text = path.to_string_lossy().replace('\\', "/");
                            *robot_file_command = RobotFileCommand::SaveAs;
                        }
                    }
                    if ui
                        .add_sized([52.0, 22.0], egui::Button::new("Save"))
                        .clicked()
                    {
                        *robot_file_command = RobotFileCommand::Save;
                    }
                    if ui
                        .add_sized([52.0, 22.0], egui::Button::new("Load"))
                        .clicked()
                    {
                        let robots_dir = ensure_asset_dir("Robots");
                        if let Some(path) = rfd::FileDialog::new()
                            .set_title("Load Robot JSON")
                            .set_directory(&robots_dir)
                            .add_filter("JSON", &["json"])
                            .pick_file()
                        {
                            *robot_file_path_text = path.to_string_lossy().replace('\\', "/");
                            *robot_file_command = RobotFileCommand::Load;
                        }
                    }
                    if ui
                        .add_sized([48.0, 22.0], egui::Button::new("New"))
                        .clicked()
                    {
                        *robot_file_command = RobotFileCommand::New;
                    }
                });
            });
            if robot_dirty {
                ui.small(
                    egui::RichText::new("Unsaved robot changes")
                        .color(egui::Color32::from_rgb(190, 130, 30)),
                );
            }
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.add_sized([42.0, 22.0], egui::Label::new("Path"));
                ui.add_sized(
                    [ui.available_width(), 22.0],
                    egui::TextEdit::singleline(robot_file_path_text),
                );
            });
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.add_sized([42.0, 22.0], egui::Label::new("Name"));
                if ui
                    .add_sized(
                        [ui.available_width(), 22.0],
                        egui::TextEdit::singleline(&mut robot.name),
                    )
                    .changed()
                {
                    changed = true;
                }
            });
        });

    ui.add_space(8.0);
    egui::CollapsingHeader::new("Chassis")
        .default_open(true)
        .show(ui, |ui| {
            let c = &mut robot.chassis;
            let mut mass_g = c.mass_kg * 1000.0;
            let mut com_x_mm = c.center_of_mass_m.x * 1000.0;
            let mut com_y_mm = c.center_of_mass_m.y * 1000.0;
            let mut length_mm = c.length_m * 1000.0;
            let mut width_mm = c.width_m * 1000.0;
            ui.horizontal_wrapped(|ui| {
                ui.label("Mass [g]");
                if ui
                    .add(
                        egui::DragValue::new(&mut mass_g)
                            .speed(1.0)
                            .clamp_range(1.0..=5000.0),
                    )
                    .changed()
                {
                    c.mass_kg = mass_g / 1000.0;
                    changed = true;
                }
                ui.label("Yaw inertia [kg·m²]");
                if ui
                    .add(
                        egui::DragValue::new(&mut c.inertia_kg_m2)
                            .speed(0.00001)
                            .clamp_range(1e-8..=1.0),
                    )
                    .changed()
                {
                    changed = true;
                }
            });
            ui.horizontal_wrapped(|ui| {
                ui.label("COM x/y [mm]");
                if ui
                    .add(egui::DragValue::new(&mut com_x_mm).speed(0.5))
                    .changed()
                {
                    c.center_of_mass_m.x = com_x_mm / 1000.0;
                    changed = true;
                }
                if ui
                    .add(egui::DragValue::new(&mut com_y_mm).speed(0.5))
                    .changed()
                {
                    c.center_of_mass_m.y = com_y_mm / 1000.0;
                    changed = true;
                }
            });
            ui.horizontal_wrapped(|ui| {
                ui.label("Length/width [mm]");
                if ui
                    .add(
                        egui::DragValue::new(&mut length_mm)
                            .speed(0.5)
                            .clamp_range(1.0..=1000.0),
                    )
                    .changed()
                {
                    c.length_m = length_mm / 1000.0;
                    changed = true;
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
                    changed = true;
                }
            });
        });

    egui::CollapsingHeader::new("Line validity areas")
        .default_open(true)
        .show(ui, |ui| {
            ui.small("The robot is valid while any enabled rectangle overlaps the course line.");
            if ui.button("Add rectangle").clicked() {
                robot.line_validity_areas.push(RobotLineValidityArea {
                    name: format!("Area {}", robot.line_validity_areas.len() + 1),
                    position_m: Vec2::new(0.0, 0.0),
                    length_m: 0.040,
                    width_m: 0.040,
                    angle_deg: 0.0,
                    enabled: true,
                });
                changed = true;
            }
            let mut remove = None;
            for (idx, area) in robot.line_validity_areas.iter_mut().enumerate() {
                ui.group(|ui| {
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut area.enabled, "").changed() {
                            changed = true;
                        }
                        if ui.text_edit_singleline(&mut area.name).changed() {
                            changed = true;
                        }
                        if ui.small_button("Remove").clicked() {
                            remove = Some(idx);
                        }
                    });
                    let mut x_mm = area.position_m.x * 1000.0;
                    let mut y_mm = area.position_m.y * 1000.0;
                    let mut length_mm = area.length_m * 1000.0;
                    let mut width_mm = area.width_m * 1000.0;
                    ui.horizontal_wrapped(|ui| {
                        ui.label("X [mm]");
                        if ui.add(egui::DragValue::new(&mut x_mm).speed(0.5)).changed() {
                            area.position_m.x = x_mm / 1000.0;
                            changed = true;
                        }
                        ui.label("Y [mm]");
                        if ui.add(egui::DragValue::new(&mut y_mm).speed(0.5)).changed() {
                            area.position_m.y = y_mm / 1000.0;
                            changed = true;
                        }
                        ui.label("Angle [deg]");
                        if ui
                            .add(egui::DragValue::new(&mut area.angle_deg).speed(1.0))
                            .changed()
                        {
                            changed = true;
                        }
                    });
                    ui.horizontal_wrapped(|ui| {
                        ui.label("Length [mm]");
                        if ui
                            .add(
                                egui::DragValue::new(&mut length_mm)
                                    .speed(0.5)
                                    .clamp_range(0.1..=250.0),
                            )
                            .changed()
                        {
                            area.length_m = length_mm / 1000.0;
                            changed = true;
                        }
                        ui.label("Width [mm]");
                        if ui
                            .add(
                                egui::DragValue::new(&mut width_mm)
                                    .speed(0.5)
                                    .clamp_range(0.1..=250.0),
                            )
                            .changed()
                        {
                            area.width_m = width_mm / 1000.0;
                            changed = true;
                        }
                    });
                });
            }
            if let Some(idx) = remove {
                robot.line_validity_areas.remove(idx);
                changed = true;
            }
            if robot.line_validity_areas.is_empty() {
                ui.colored_label(
                    egui::Color32::from_rgb(190, 130, 30),
                    "No validity area: the robot can never be considered over the line.",
                );
            }
        });

    egui::CollapsingHeader::new("Catálogo de pneus — aplicar à montagem")
        .default_open(false)
        .show(ui, |ui| {
            ui.label("Este painel edita o modelo do catálogo. Use Aplicar para mudar as rodas.");
            if ui
                .button("Aplicar pneu do catálogo a todas as rodas")
                .clicked()
            {
                if let Some(a) = &mut robot.assembly {
                    for w in &mut a.wheels {
                        w.tire = robot.tire.clone();
                    }
                }
                changed = true;
            }

            component_asset_row(
                ui,
                "Tire",
                tire_asset_path_text,
                ComponentAssetKind::Tire,
                "RobotAssets/Tires",
                json_file_name_from_name(&robot.tire.model, "tire"),
                component_asset_command,
            );
            egui::CollapsingHeader::new("Technical tire parameters")
                .default_open(false)
                .show(ui, |ui| {
                    egui::Grid::new("technical_tire_grid")
                        .num_columns(2)
                        .spacing([12.0, 6.0])
                        .show(ui, |ui| {
                            ui.label("Model");
                            if ui
                                .add(
                                    egui::TextEdit::singleline(&mut robot.tire.model)
                                        .desired_width(190.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("μ longitudinal");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.tire.mu_longitudinal)
                                        .speed(0.01)
                                        .clamp_range(0.0..=5.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("μ lateral");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.tire.mu_lateral)
                                        .speed(0.01)
                                        .clamp_range(0.0..=5.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Rolling");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.tire.rolling_resistance)
                                        .speed(0.001)
                                        .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Slip epsilon [m/s]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.tire.slip_velocity_epsilon_m_s)
                                        .speed(0.001)
                                        .clamp_range(0.0..=10.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                        });
                });
        });

    egui::CollapsingHeader::new("Motor, Driver and Battery")
        .default_open(false)
        .show(ui, |ui| {
            component_asset_row(
                ui,
                "Motor",
                motor_left_asset_path_text,
                ComponentAssetKind::MotorLeft,
                "RobotAssets/Motors",
                json_file_name_from_name(&robot.motor_left.model, "motor"),
                component_asset_command,
            );
            let mut motor_changed = false;
            egui::CollapsingHeader::new("Technical motor parameters")
                .default_open(false)
                .show(ui, |ui| {
                    motor_editor(ui, "Left motor", &mut robot.motor_left, &mut motor_changed);
                });
            motor_editor(
                ui,
                "Right motor",
                &mut robot.motor_right,
                &mut motor_changed,
            );
            changed |= motor_changed;
            ui.separator();
            component_asset_row(
                ui,
                "Driver",
                driver_asset_path_text,
                ComponentAssetKind::Driver,
                "RobotAssets/Drivers",
                json_file_name_from_name(&robot.driver.model, "driver"),
                component_asset_command,
            );
            egui::CollapsingHeader::new("Technical driver parameters")
                .default_open(false)
                .show(ui, |ui| {
                    egui::Grid::new("technical_driver_grid")
                        .num_columns(2)
                        .spacing([12.0, 6.0])
                        .show(ui, |ui| {
                            ui.label("Driver");
                            if ui
                                .add(
                                    egui::TextEdit::singleline(&mut robot.driver.model)
                                        .desired_width(190.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("PWM [Hz]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.driver.pwm_frequency_hz)
                                        .speed(100.0)
                                        .clamp_range(10.0..=200_000.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Mode");
                            let old_mode = robot.driver.mode.clone();
                            egui::ComboBox::from_id_source("robot_driver_mode")
                                .selected_text(robot.driver.mode.as_str())
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut robot.driver.mode,
                                        "brake".to_string(),
                                        "brake",
                                    );
                                    ui.selectable_value(
                                        &mut robot.driver.mode,
                                        "coast".to_string(),
                                        "coast",
                                    );
                                });
                            if robot.driver.mode != old_mode {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Driver drop [V]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.driver.voltage_drop_v)
                                        .speed(0.01)
                                        .clamp_range(0.0..=5.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Current limit [A]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.driver.current_limit_a)
                                        .speed(0.1)
                                        .clamp_range(0.0..=500.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("PWM resolution [bits]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.driver.pwm_resolution_bits)
                                        .clamp_range(1.0..=32.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Command deadband");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.driver.command_deadband)
                                        .speed(0.0001)
                                        .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                        });
                });
            ui.separator();
            component_asset_row(
                ui,
                "Battery",
                battery_asset_path_text,
                ComponentAssetKind::Battery,
                "RobotAssets/Batteries",
                json_file_name_from_name(&robot.battery.model, "battery"),
                component_asset_command,
            );
            egui::CollapsingHeader::new("Technical battery parameters")
                .default_open(false)
                .show(ui, |ui| {
                    egui::Grid::new("technical_battery_grid")
                        .num_columns(2)
                        .spacing([12.0, 6.0])
                        .show(ui, |ui| {
                            ui.label("Battery");
                            if ui
                                .add(
                                    egui::TextEdit::singleline(&mut robot.battery.model)
                                        .desired_width(190.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Cells");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.cells)
                                        .clamp_range(1.0..=8.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Nominal V");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.nominal_voltage_v)
                                        .speed(0.1),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("R int [Ω]");
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut robot.battery.internal_resistance_ohm,
                                    )
                                    .speed(0.001)
                                    .clamp_range(0.0..=10.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Full V");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.full_voltage_v)
                                        .speed(0.1),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Empty V");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.empty_voltage_v)
                                        .speed(0.1),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Capacity [mAh]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.capacity_mah)
                                        .speed(1.0)
                                        .clamp_range(1.0..=1_000_000.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Initial SOC");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.initial_soc)
                                        .speed(0.01)
                                        .clamp_range(0.0..=1.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                            ui.label("Current limit [A]");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut robot.battery.current_limit_a)
                                        .speed(0.1)
                                        .clamp_range(0.0..=10_000.0),
                                )
                                .changed()
                            {
                                changed = true;
                            }
                            ui.end_row();
                        });
                });
        });

    egui::CollapsingHeader::new("Sensors")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.strong("Shared sensor model");
                if ui.button("Add Sensor").clicked() {
                    let mut instance = default_sensor_instance();
                    if let Some(shared) = robot.sensors.first() {
                        instance.asset = shared.asset.clone();
                        instance.asset_path = shared.asset_path.clone();
                    }
                    instance.name = format!("Sensor {}", robot.sensors.len() + 1);
                    robot.sensors.push(instance);
                    changed = true;
                }
            });
            changed |=
                edit_sensor_instances_ui(ui, &mut robot.sensors, project_path, status_to_set);
        });

    egui::CollapsingHeader::new("Encoder and Gyro")
        .default_open(false)
        .show(ui, |ui| {
            component_asset_row(
                ui,
                "Encoder",
                encoder_asset_path_text,
                ComponentAssetKind::Encoder,
                "RobotAssets/Encoders",
                json_file_name_from_name(&robot.encoder.model, "encoder"),
                component_asset_command,
            );
            egui::CollapsingHeader::new("Technical encoder parameters")
                .default_open(false)
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.label("Ticks/rev");
                        if ui
                            .add(
                                egui::DragValue::new(&mut robot.encoder.ticks_per_rev)
                                    .clamp_range(1.0..=100_000.0),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                        if ui
                            .checkbox(&mut robot.encoder.invert_left, "Invert left")
                            .changed()
                        {
                            changed = true;
                        }
                        if ui
                            .checkbox(&mut robot.encoder.invert_right, "Invert right")
                            .changed()
                        {
                            changed = true;
                        }
                    });
                });
            ui.separator();
            component_asset_row(
                ui,
                "Gyro",
                gyro_asset_path_text,
                ComponentAssetKind::Gyro,
                "RobotAssets/Gyros",
                json_file_name_from_name(&robot.gyro.model, "gyro"),
                component_asset_command,
            );
            egui::CollapsingHeader::new("Technical gyro parameters")
                .default_open(false)
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.label("Noise [rad/s]");
                        if ui
                            .add(
                                egui::DragValue::new(&mut robot.gyro.noise_std_rad_s)
                                    .speed(0.001)
                                    .clamp_range(0.0..=10.0),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("Bias [rad/s]");
                        if ui
                            .add(egui::DragValue::new(&mut robot.gyro.bias_rad_s).speed(0.001))
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("Saturation [rad/s]");
                        if ui
                            .add(
                                egui::DragValue::new(&mut robot.gyro.saturation_rad_s)
                                    .speed(0.1)
                                    .clamp_range(0.0..=1_000.0),
                            )
                            .changed()
                        {
                            changed = true;
                        }
                        ui.label("Noise seed");
                        if ui.add(egui::DragValue::new(&mut robot.gyro.seed)).changed() {
                            changed = true;
                        }
                    });
                });
        });

    egui::CollapsingHeader::new("Normal / Downforce / Suction")
        .default_open(false)
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label("Model");
                let old_model = robot.normal_force.model.clone();
                egui::ComboBox::from_id_source("robot_normal_force_model")
                    .selected_text(robot.normal_force.model.as_str())
                    .show_ui(ui, |ui| {
                        for model in [
                            "NoDownforce",
                            "ConstantDownforce",
                            "FanDownforce",
                            "SuctionDownforce",
                            "MeasuredDownforceCurve",
                        ] {
                            ui.selectable_value(
                                &mut robot.normal_force.model,
                                crate::io::models::NormalForceKind::parse(model).unwrap(),
                                model,
                            );
                        }
                    });
                if robot.normal_force.model != old_model {
                    changed = true;
                }
            });
            ui.horizontal_wrapped(|ui| {
                ui.label("Max force [N]");
                if ui
                    .add(
                        egui::DragValue::new(&mut robot.normal_force.max_force_n)
                            .speed(0.01)
                            .clamp_range(0.0..=100.0),
                    )
                    .changed()
                {
                    changed = true;
                }
                ui.label("Max current [A]");
                if ui
                    .add(
                        egui::DragValue::new(&mut robot.normal_force.max_current_a)
                            .speed(0.01)
                            .clamp_range(0.0..=100.0),
                    )
                    .changed()
                {
                    changed = true;
                }
                ui.label("Response [s]");
                if ui
                    .add(
                        egui::DragValue::new(&mut robot.normal_force.response_time_s)
                            .speed(0.001)
                            .clamp_range(0.0..=10.0),
                    )
                    .changed()
                {
                    changed = true;
                }
            });
            ui.horizontal_wrapped(|ui| {
                let mut x_mm = robot.normal_force.position_m.x * 1000.0;
                let mut y_mm = robot.normal_force.position_m.y * 1000.0;
                ui.label("Apply pos x/y [mm]");
                if ui.add(egui::DragValue::new(&mut x_mm).speed(0.5)).changed() {
                    robot.normal_force.position_m.x = x_mm / 1000.0;
                    changed = true;
                }
                if ui.add(egui::DragValue::new(&mut y_mm).speed(0.5)).changed() {
                    robot.normal_force.position_m.y = y_mm / 1000.0;
                    changed = true;
                }
                ui.label("Suction area [m²]");
                if ui
                    .add(
                        egui::DragValue::new(&mut robot.normal_force.chamber_area_m2)
                            .speed(0.0001)
                            .clamp_range(0.0..=1.0),
                    )
                    .changed()
                {
                    changed = true;
                }
            });
            ui.separator();
            ui.horizontal(|ui| {
                ui.strong("Fans");
                if ui.button("Add fan").clicked() {
                    let mut fan = robot
                        .normal_force
                        .fans
                        .first()
                        .cloned()
                        .unwrap_or_else(|| default_fan_config(robot.battery.nominal_voltage_v));
                    fan.id = crate::io::assets::new_instance_id();
                    robot.normal_force.fans.push(fan);
                    *selected_fan_asset_index = robot.normal_force.fans.len() - 1;
                    robot.normal_force.model = crate::io::models::NormalForceKind::Fan;
                    changed = true;
                }
            });
            component_asset_row(
                ui,
                "Fan model",
                fan_asset_path_text,
                ComponentAssetKind::Fan,
                "RobotAssets/Fans",
                "fan_model.json".to_string(),
                component_asset_command,
            );
            if let Some(fan) = robot.normal_force.fans.first() {
                ui.label(format!(
                    "Shared model: {} V / {:.2} A / {:.2} N",
                    fan.nominal_voltage_v, fan.nominal_current_a, fan.max_force_n
                ));
            }
            let mut remove_fan: Option<usize> = None;
            for (idx, fan) in robot.normal_force.fans.iter_mut().enumerate() {
                ui.horizontal_wrapped(|ui| {
                    ui.label(format!("Fan {}", idx + 1));
                    let mut x_mm = fan.position_m.x * 1000.0;
                    let mut y_mm = fan.position_m.y * 1000.0;
                    ui.label("x/y [mm]");
                    if ui.add(egui::DragValue::new(&mut x_mm).speed(0.5)).changed() {
                        fan.position_m.x = x_mm / 1000.0;
                        changed = true;
                    }
                    if ui.add(egui::DragValue::new(&mut y_mm).speed(0.5)).changed() {
                        fan.position_m.y = y_mm / 1000.0;
                        changed = true;
                    }
                    if ui.button("remove").clicked() {
                        remove_fan = Some(idx);
                    }
                });
                if idx == 0 {
                    egui::CollapsingHeader::new("Technical fan model parameters")
                        .id_source(format!("fan_technical_params_{idx}"))
                        .default_open(false)
                        .show(ui, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                let mut visual_radius_mm = fan.visual_radius_m * 1000.0;
                                let mut action_radius_mm = fan.action_radius_m * 1000.0;
                                ui.label("visual/action radius [mm]");
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut visual_radius_mm)
                                            .speed(0.2)
                                            .clamp_range(0.0..=125.0),
                                    )
                                    .changed()
                                {
                                    fan.visual_radius_m = visual_radius_mm / 1000.0;
                                    changed = true;
                                }
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut action_radius_mm)
                                            .speed(0.2)
                                            .clamp_range(0.0..=250.0),
                                    )
                                    .changed()
                                {
                                    fan.action_radius_m = action_radius_mm / 1000.0;
                                    changed = true;
                                }
                                ui.label("nominal V/I");
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut fan.nominal_voltage_v).speed(0.1),
                                    )
                                    .changed()
                                {
                                    fan.power_w = fan.nominal_voltage_v * fan.nominal_current_a;
                                    changed = true;
                                }
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut fan.nominal_current_a)
                                            .speed(0.01),
                                    )
                                    .changed()
                                {
                                    fan.power_w = fan.nominal_voltage_v * fan.nominal_current_a;
                                    changed = true;
                                }
                                ui.label("RPM nominal");
                                changed |= ui
                                    .add(egui::DragValue::new(&mut fan.nominal_rpm).speed(100.))
                                    .changed();
                                ui.label("response [s]");
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut fan.response_time_s)
                                            .speed(0.001)
                                            .clamp_range(0.0..=10.0),
                                    )
                                    .changed()
                                {
                                    changed = true;
                                }
                                ui.label("Max force [N]");
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut fan.max_force_n)
                                            .speed(0.01)
                                            .clamp_range(0.0..=100.0),
                                    )
                                    .changed()
                                {
                                    changed = true;
                                }
                                ui.label("Max current [A]");
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut fan.max_current_a)
                                            .speed(0.01)
                                            .clamp_range(0.0..=100.0),
                                    )
                                    .changed()
                                {
                                    changed = true;
                                }
                            });
                            ui.horizontal_wrapped(|ui| {
                                ui.label("Force curve");
                                let old_curve = fan.curve_model;
                                egui::ComboBox::from_id_source(format!("fan_curve_model_{idx}"))
                                    .selected_text(fan.curve_model.as_str())
                                    .show_ui(ui, |ui| {
                                        for curve in [
                                            FanCurveModel::Linear,
                                            FanCurveModel::Exponential,
                                            FanCurveModel::Polynomial,
                                            FanCurveModel::LookupTable,
                                            FanCurveModel::Custom,
                                        ] {
                                            ui.selectable_value(
                                                &mut fan.curve_model,
                                                curve,
                                                curve.as_str(),
                                            );
                                        }
                                    });
                                if fan.curve_model != old_curve {
                                    changed = true;
                                }
                            });
                        });
                }
            }
            if let Some(idx) = remove_fan {
                robot.normal_force.fans.remove(idx);
                if robot.normal_force.fans.is_empty() {
                    *selected_fan_asset_index = 0;
                } else {
                    *selected_fan_asset_index =
                        (*selected_fan_asset_index).min(robot.normal_force.fans.len() - 1);
                }
                changed = true;
            }
        });

    changed
}

pub(super) fn component_asset_row(
    ui: &mut egui::Ui,
    label: &str,
    path_text: &mut String,
    kind: ComponentAssetKind,
    dir: &str,
    suggested_file_name: String,
    command: &mut Option<ComponentAssetCommand>,
) {
    ui.horizontal(|ui| {
        let spacing = ui.spacing().item_spacing.x;
        let buttons_width = 38.0 + 40.0 + 40.0 + 56.0 + spacing * 4.0;
        let field_width = (ui.available_width() - buttons_width).max(60.0);
        ui.add_sized([field_width, 22.0], egui::TextEdit::singleline(path_text));
        if ui
            .add_sized([38.0, 22.0], egui::Button::new("New"))
            .clicked()
        {
            *command = Some(ComponentAssetCommand {
                kind,
                command: ComponentAssetCommandKind::New,
            });
        }
        if ui
            .add_sized([40.0, 22.0], egui::Button::new("Load"))
            .clicked()
        {
            let dir = ensure_asset_dir(dir);
            if let Some(path) = rfd::FileDialog::new()
                .set_title(format!("Load {label} JSON"))
                .set_directory(&dir)
                .add_filter("JSON", &["json"])
                .pick_file()
            {
                *path_text = path.to_string_lossy().replace('\\', "/");
                *command = Some(ComponentAssetCommand {
                    kind,
                    command: ComponentAssetCommandKind::Load,
                });
            }
        }
        if ui
            .add_sized([40.0, 22.0], egui::Button::new("Save"))
            .clicked()
        {
            *command = Some(ComponentAssetCommand {
                kind,
                command: ComponentAssetCommandKind::Save,
            });
        }
        if ui
            .add_sized([56.0, 22.0], egui::Button::new("Save As"))
            .clicked()
        {
            let dir = ensure_asset_dir(dir);
            if let Some(path) = rfd::FileDialog::new()
                .set_title(format!("Save {label} JSON"))
                .set_file_name(suggested_file_name)
                .add_filter("JSON", &["json"])
                .set_directory(&dir)
                .save_file()
            {
                *path_text = path.to_string_lossy().replace('\\', "/");
                *command = Some(ComponentAssetCommand {
                    kind,
                    command: ComponentAssetCommandKind::SaveAs,
                });
            }
        }
    });
}

enum SensorInstanceAction {
    Duplicate(usize),
    Remove(usize),
}

pub(super) fn edit_sensor_instances_ui(
    ui: &mut egui::Ui,
    sensors: &mut Vec<RobotSensorInstance>,
    project_path: Option<&Path>,
    status_to_set: &mut Option<String>,
) -> bool {
    let mut changed = false;
    let mut action: Option<SensorInstanceAction> = None;
    if sensors.is_empty() {
        ui.label("No sensor instances. Use Add Sensor to place one.");
    }
    for (idx, sensor) in sensors.iter_mut().enumerate() {
        ui.group(|ui| {
            ui.set_width(ui.available_width());

            // Linha 1: título e flags visuais
            ui.horizontal_wrapped(|ui| {
                ui.strong(format!("{}: {}", sensor.id, sensor.asset.model));
                changed |= ui.checkbox(&mut sensor.enabled, "Enabled").changed();
                changed |= ui
                    .checkbox(&mut sensor.visible_in_preview, "Visible")
                    .changed();
            });

            // Linha 2: botões de ação da instância
            ui.horizontal_wrapped(|ui| {
                if ui
                    .add_sized([78.0, 22.0], egui::Button::new("Duplicate"))
                    .clicked()
                {
                    action = Some(SensorInstanceAction::Duplicate(idx));
                }

                if ui
                    .add_sized([108.0, 22.0], egui::Button::new("Remove Sensor"))
                    .clicked()
                {
                    action = Some(SensorInstanceAction::Remove(idx));
                }
            });

            ui.add_space(4.0);

            // Linha 3: nome da instância
            ui.horizontal(|ui| {
                ui.add_sized([88.0, 22.0], egui::Label::new("Instance name"));

                if ui
                    .add_sized(
                        [ui.available_width(), 22.0],
                        egui::TextEdit::singleline(&mut sensor.name),
                    )
                    .changed()
                {
                    changed = true;
                }
            });

            // Linha 4: posição e ângulo
            let mut x_mm = sensor.position_m.x * 1000.0;
            let mut y_mm = sensor.position_m.y * 1000.0;

            ui.horizontal_wrapped(|ui| {
                ui.label("X [mm]");
                if ui
                    .add_sized([56.0, 22.0], egui::DragValue::new(&mut x_mm).speed(0.5))
                    .changed()
                {
                    sensor.position_m.x = x_mm / 1000.0;
                    changed = true;
                }

                ui.label("Y [mm]");
                if ui
                    .add_sized([56.0, 22.0], egui::DragValue::new(&mut y_mm).speed(0.5))
                    .changed()
                {
                    sensor.position_m.y = y_mm / 1000.0;
                    changed = true;
                }

                ui.label("Angle [deg]");
                if ui
                    .add_sized(
                        [56.0, 22.0],
                        egui::DragValue::new(&mut sensor.angle_deg).speed(1.0),
                    )
                    .changed()
                {
                    changed = true;
                }
            });

            ui.horizontal_wrapped(|ui| {
                ui.label("ADC bits");
                changed |= ui
                    .add(egui::DragValue::new(&mut sensor.acquisition.adc_bits).clamp_range(1..=24))
                    .changed();
                ui.label("Noise");
                changed |= ui
                    .add(
                        egui::DragValue::new(&mut sensor.acquisition.reflectance_noise_std)
                            .speed(0.001)
                            .clamp_range(0.0..=1.0),
                    )
                    .changed();
                ui.label("ADC noise [LSB]");
                changed |= ui
                    .add(
                        egui::DragValue::new(&mut sensor.acquisition.adc_noise_lsb)
                            .speed(0.1)
                            .clamp_range(0.0..=1000.0),
                    )
                    .changed();
                ui.label("Seed");
                changed |= ui
                    .add(egui::DragValue::new(&mut sensor.acquisition.seed))
                    .changed();
            });

            // Linha 5: asset path + seletor de arquivo
            {
                let mut path_text = sensor.asset_path.to_string_lossy().replace('\\', "/");

                ui.horizontal(|ui| {
                    ui.add_sized([64.0, 22.0], egui::Label::new("Asset path"));

                    if ui
                        .add_sized(
                            [ui.available_width(), 22.0],
                            egui::TextEdit::singleline(&mut path_text),
                        )
                        .changed()
                    {
                        sensor.asset_path = PathBuf::from(path_text.clone());
                        changed = true;
                    }
                });

                // Linha 6: comandos do asset
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .add_sized([44.0, 22.0], egui::Button::new("Load"))
                        .clicked()
                    {
                        if let Some(path) = rfd::FileDialog::new()
                            .set_title("Load sensor asset JSON")
                            .set_directory(ensure_asset_dir("RobotAssets/Sensors"))
                            .add_filter("JSON", &["json"])
                            .pick_file()
                        {
                            match load_sensor_asset_from_file(&path) {
                                Ok(asset) => {
                                    sensor.asset = asset;
                                    sensor.asset_path = path.clone();
                                    *status_to_set = Some(format!(
                                        "Sensor asset loaded from {}",
                                        path.display()
                                    ));
                                    changed = true;
                                }
                                Err(err) => {
                                    *status_to_set =
                                        Some(format!("Failed to load sensor asset: {err}"));
                                }
                            }
                        }
                    }

                    if ui
                        .add_sized([44.0, 22.0], egui::Button::new("New"))
                        .clicked()
                    {
                        sensor.asset = default_sensor_asset();
                        changed = true;
                    }

                    if ui
                        .add_sized([44.0, 22.0], egui::Button::new("Save"))
                        .clicked()
                    {
                        let path = resolve_asset_path_text(
                            project_path,
                            &sensor.asset_path.to_string_lossy(),
                        );

                        match save_sensor_asset_to_file(&sensor.asset, &path) {
                            Ok(()) => {
                                sensor.asset_path = path.clone();
                                *status_to_set =
                                    Some(format!("Sensor asset saved to {}", path.display()));
                            }
                            Err(err) => {
                                *status_to_set =
                                    Some(format!("Failed to save sensor asset: {err}"));
                            }
                        }
                    }

                    if ui
                        .add_sized([62.0, 22.0], egui::Button::new("Save As"))
                        .clicked()
                    {
                        let dir = ensure_asset_dir("RobotAssets/Sensors");
                        let suggested =
                            json_file_name_from_name(&sensor.asset.name, "sensor_asset");

                        if let Some(path) = rfd::FileDialog::new()
                            .set_title("Save sensor asset JSON")
                            .set_file_name(suggested)
                            .set_directory(&dir)
                            .add_filter("JSON", &["json"])
                            .save_file()
                        {
                            match save_sensor_asset_to_file(&sensor.asset, &path) {
                                Ok(()) => {
                                    sensor.asset_path = path.clone();
                                    *status_to_set =
                                        Some(format!("Sensor asset saved as {}", path.display()));
                                    changed = true;
                                }
                                Err(err) => {
                                    *status_to_set =
                                        Some(format!("Failed to save sensor asset: {err}"));
                                }
                            }
                        }
                    }
                });

                egui::CollapsingHeader::new("Technical model information")
                    .id_source(format!("sensor_asset_params_{idx}"))
                    .default_open(false)
                    .show(ui, |ui| {
                        changed |= edit_sensor_asset_ui(ui, &mut sensor.asset, idx);
                    });
            }
        });
        ui.add_space(4.0);
    }

    if let Some(action) = action {
        match action {
            SensorInstanceAction::Duplicate(idx) => {
                if let Some(sensor) = sensors.get(idx).cloned() {
                    let mut copy = sensor;
                    copy.name = format!("{} copy", copy.name);
                    copy.position_m.y = (copy.position_m.y + 0.010).clamp(-0.125, 0.125);
                    copy.id = crate::io::assets::new_instance_id();
                    sensors.insert(idx + 1, copy);
                    changed = true;
                }
            }
            SensorInstanceAction::Remove(idx) => {
                if idx < sensors.len() {
                    sensors.remove(idx);
                    changed = true;
                }
            }
        }
    }
    changed
}

pub(super) fn edit_sensor_asset_ui(ui: &mut egui::Ui, asset: &mut SensorAsset, idx: usize) -> bool {
    let mut changed = false;
    ui.horizontal_wrapped(|ui| {
        if ui.text_edit_singleline(&mut asset.name).changed() {
            changed = true;
        }
        ui.label("Model");
        if ui.text_edit_singleline(&mut asset.model).changed() {
            changed = true;
        }
        let old_type = asset.sensor_type;
        egui::ComboBox::from_id_source(format!("sensor_type_{idx}"))
            .selected_text(asset.sensor_type.as_str())
            .show_ui(ui, |ui| {
                for ty in [
                    SensorType::LineAnalog,
                    SensorType::LineDigital,
                    SensorType::DistanceInfrared,
                    SensorType::DistanceToF,
                    SensorType::Ultrasonic,
                    SensorType::Color,
                    SensorType::Encoder,
                    SensorType::Gyro,
                    SensorType::Accelerometer,
                    SensorType::Custom,
                ] {
                    ui.selectable_value(&mut asset.sensor_type, ty, ty.as_str());
                }
            });
        if asset.sensor_type != old_type {
            changed = true;
        }
    });
    ui.horizontal_wrapped(|ui| {
        let mut visual_w_mm = asset.visual_width_m * 1000.0;
        let mut visual_h_mm = asset.visual_height_m * 1000.0;
        let mut visual_r_mm = asset.visual_radius_m * 1000.0;
        ui.label("Visual width/height/radius [mm]");
        if ui
            .add(
                egui::DragValue::new(&mut visual_w_mm)
                    .speed(0.2)
                    .clamp_range(0.0..=250.0),
            )
            .changed()
        {
            asset.visual_width_m = visual_w_mm / 1000.0;
            changed = true;
        }
        if ui
            .add(
                egui::DragValue::new(&mut visual_h_mm)
                    .speed(0.2)
                    .clamp_range(0.0..=250.0),
            )
            .changed()
        {
            asset.visual_height_m = visual_h_mm / 1000.0;
            changed = true;
        }
        if ui
            .add(
                egui::DragValue::new(&mut visual_r_mm)
                    .speed(0.2)
                    .clamp_range(0.0..=125.0),
            )
            .changed()
        {
            asset.visual_radius_m = visual_r_mm / 1000.0;
            changed = true;
        }
    });
    changed |= edit_sensor_detection_area_ui(ui, &mut asset.detection_area, idx);
    changed |= edit_sensor_response_model_ui(ui, &mut asset.response_model, idx);
    ui.horizontal_wrapped(|ui| {
        ui.label("Notes");
        if ui.text_edit_singleline(&mut asset.notes).changed() {
            changed = true;
        }
    });
    changed
}

pub(super) fn sensor_detection_kind(area: &SensorDetectionArea) -> &'static str {
    match area {
        SensorDetectionArea::Point { .. } => "Point",
        SensorDetectionArea::Rectangle { .. } => "Rectangle",
        SensorDetectionArea::Circle { .. } => "Circle",
        SensorDetectionArea::Cone { .. } => "Cone",
        SensorDetectionArea::CustomPolygon { .. } => "CustomPolygon",
    }
}

pub(super) fn default_detection_area_kind(kind: &str) -> SensorDetectionArea {
    match kind {
        "Point" => SensorDetectionArea::Point { radius_m: 0.002 },
        "Rectangle" => SensorDetectionArea::Rectangle {
            width_m: 0.005,
            height_m: 0.002,
        },
        "Circle" => SensorDetectionArea::Circle { radius_m: 0.004 },
        "Cone" => SensorDetectionArea::Cone {
            range_m: 0.080,
            angle_deg: 25.0,
        },
        "CustomPolygon" => SensorDetectionArea::CustomPolygon {
            points_m: vec![
                Vec2::new(0.0, -0.003),
                Vec2::new(0.010, 0.0),
                Vec2::new(0.0, 0.003),
            ],
        },
        _ => SensorDetectionArea::Point { radius_m: 0.002 },
    }
}

pub(super) fn edit_sensor_detection_area_ui(
    ui: &mut egui::Ui,
    area: &mut SensorDetectionArea,
    idx: usize,
) -> bool {
    let mut changed = false;
    let current = sensor_detection_kind(area).to_string();
    let mut selected = current.clone();
    ui.horizontal_wrapped(|ui| {
        ui.label("Detection area");
        egui::ComboBox::from_id_source(format!("sensor_detection_{idx}"))
            .selected_text(selected.as_str())
            .show_ui(ui, |ui| {
                for kind in ["Point", "Rectangle", "Circle", "Cone", "CustomPolygon"] {
                    ui.selectable_value(&mut selected, kind.to_string(), kind);
                }
            });
    });
    if selected != current {
        *area = default_detection_area_kind(&selected);
        changed = true;
    }
    ui.horizontal_wrapped(|ui| match area {
        SensorDetectionArea::Point { radius_m } => {
            let mut radius_mm = *radius_m * 1000.0;
            ui.label("radius [mm]");
            if ui
                .add(
                    egui::DragValue::new(&mut radius_mm)
                        .speed(0.1)
                        .clamp_range(0.0..=250.0),
                )
                .changed()
            {
                *radius_m = radius_mm / 1000.0;
                changed = true;
            }
        }
        SensorDetectionArea::Rectangle { width_m, height_m } => {
            let mut w_mm = *width_m * 1000.0;
            let mut h_mm = *height_m * 1000.0;
            ui.label("width/height [mm]");
            if ui
                .add(
                    egui::DragValue::new(&mut w_mm)
                        .speed(0.1)
                        .clamp_range(0.0..=250.0),
                )
                .changed()
            {
                *width_m = w_mm / 1000.0;
                changed = true;
            }
            if ui
                .add(
                    egui::DragValue::new(&mut h_mm)
                        .speed(0.1)
                        .clamp_range(0.0..=250.0),
                )
                .changed()
            {
                *height_m = h_mm / 1000.0;
                changed = true;
            }
        }
        SensorDetectionArea::Circle { radius_m } => {
            let mut radius_mm = *radius_m * 1000.0;
            ui.label("radius [mm]");
            if ui
                .add(
                    egui::DragValue::new(&mut radius_mm)
                        .speed(0.1)
                        .clamp_range(0.0..=500.0),
                )
                .changed()
            {
                *radius_m = radius_mm / 1000.0;
                changed = true;
            }
        }
        SensorDetectionArea::Cone { range_m, angle_deg } => {
            let mut range_mm = *range_m * 1000.0;
            ui.label("range [mm]");
            if ui
                .add(
                    egui::DragValue::new(&mut range_mm)
                        .speed(1.0)
                        .clamp_range(0.0..=2000.0),
                )
                .changed()
            {
                *range_m = range_mm / 1000.0;
                changed = true;
            }
            ui.label("angle [deg]");
            if ui
                .add(
                    egui::DragValue::new(angle_deg)
                        .speed(1.0)
                        .clamp_range(0.0..=180.0),
                )
                .changed()
            {
                changed = true;
            }
        }
        SensorDetectionArea::CustomPolygon { points_m } => {
            if ui.button("Add point").clicked() {
                points_m.push(Vec2::new(0.0, 0.0));
                changed = true;
            }
            let mut remove = None;
            for (i, p) in points_m.iter_mut().enumerate() {
                let mut x_mm = p.x * 1000.0;
                let mut y_mm = p.y * 1000.0;
                ui.label(format!("p{i}"));
                if ui.add(egui::DragValue::new(&mut x_mm).speed(0.5)).changed() {
                    p.x = x_mm / 1000.0;
                    changed = true;
                }
                if ui.add(egui::DragValue::new(&mut y_mm).speed(0.5)).changed() {
                    p.y = y_mm / 1000.0;
                    changed = true;
                }
                if ui.small_button("x").clicked() {
                    remove = Some(i);
                }
            }
            if let Some(i) = remove {
                points_m.remove(i);
                changed = true;
            }
        }
    });
    changed
}

pub(super) fn response_model_kind(model: &SensorResponseModel) -> &'static str {
    match model {
        SensorResponseModel::Ideal => "Ideal",
        SensorResponseModel::Threshold { .. } => "Threshold",
        SensorResponseModel::Linear { .. } => "Linear",
        SensorResponseModel::Polynomial { .. } => "Polynomial",
        SensorResponseModel::LookupTable { .. } => "LookupTable",
        SensorResponseModel::Custom { .. } => "Custom",
    }
}

pub(super) fn default_response_model_kind(kind: &str) -> SensorResponseModel {
    match kind {
        "Threshold" => SensorResponseModel::Threshold { threshold: 0.5 },
        "Linear" => SensorResponseModel::Linear {
            gain: 1.0,
            offset: 0.0,
        },
        "Polynomial" => SensorResponseModel::Polynomial {
            coefficients: vec![0.0, 1.0],
        },
        "LookupTable" => SensorResponseModel::LookupTable {
            points: vec![
                SensorResponsePoint {
                    input: 0.0,
                    output: 0.0,
                },
                SensorResponsePoint {
                    input: 1.0,
                    output: 1.0,
                },
            ],
        },
        "Custom" => SensorResponseModel::Custom {
            description: String::new(),
        },
        _ => SensorResponseModel::Ideal,
    }
}

pub(super) fn edit_sensor_response_model_ui(
    ui: &mut egui::Ui,
    model: &mut SensorResponseModel,
    idx: usize,
) -> bool {
    let mut changed = false;
    let current = response_model_kind(model).to_string();
    let mut selected = current.clone();
    ui.horizontal_wrapped(|ui| {
        ui.label("Response model");
        egui::ComboBox::from_id_source(format!("sensor_response_{idx}"))
            .selected_text(selected.as_str())
            .show_ui(ui, |ui| {
                for kind in [
                    "Ideal",
                    "Threshold",
                    "Linear",
                    "Polynomial",
                    "LookupTable",
                    "Custom",
                ] {
                    ui.selectable_value(&mut selected, kind.to_string(), kind);
                }
            });
    });
    if selected != current {
        *model = default_response_model_kind(&selected);
        changed = true;
    }
    ui.horizontal_wrapped(|ui| match model {
        SensorResponseModel::Ideal => {
            ui.label("Ideal response, no editable parameters.");
        }
        SensorResponseModel::Threshold { threshold } => {
            ui.label("threshold");
            if ui
                .add(egui::DragValue::new(threshold).speed(0.01))
                .changed()
            {
                changed = true;
            }
        }
        SensorResponseModel::Linear { gain, offset } => {
            ui.label("gain/offset");
            if ui.add(egui::DragValue::new(gain).speed(0.01)).changed() {
                changed = true;
            }
            if ui.add(egui::DragValue::new(offset).speed(0.01)).changed() {
                changed = true;
            }
        }
        SensorResponseModel::Polynomial { coefficients } => {
            if ui.button("Add coefficient").clicked() {
                coefficients.push(0.0);
                changed = true;
            }
            let mut remove = None;
            for (i, coefficient) in coefficients.iter_mut().enumerate() {
                ui.label(format!("c{i}"));
                if ui
                    .add(egui::DragValue::new(coefficient).speed(0.01))
                    .changed()
                {
                    changed = true;
                }
                if ui.small_button("x").clicked() {
                    remove = Some(i);
                }
            }
            if let Some(i) = remove {
                coefficients.remove(i);
                changed = true;
            }
        }
        SensorResponseModel::LookupTable { points } => {
            if ui.button("Add point").clicked() {
                points.push(SensorResponsePoint {
                    input: 0.0,
                    output: 0.0,
                });
                changed = true;
            }
            let mut remove = None;
            for (i, point) in points.iter_mut().enumerate() {
                ui.label(format!("p{i}"));
                if ui
                    .add(egui::DragValue::new(&mut point.input).speed(0.01))
                    .changed()
                {
                    changed = true;
                }
                if ui
                    .add(egui::DragValue::new(&mut point.output).speed(0.01))
                    .changed()
                {
                    changed = true;
                }
                if ui.small_button("x").clicked() {
                    remove = Some(i);
                }
            }
            if let Some(i) = remove {
                points.remove(i);
                changed = true;
            }
        }
        SensorResponseModel::Custom { description } => {
            ui.label("description");
            if ui.text_edit_singleline(description).changed() {
                changed = true;
            }
        }
    });
    changed
}

pub(super) fn ensure_asset_dir(dir: &str) -> PathBuf {
    let path = std::env::current_dir()
        .unwrap_or_else(|_| PathBuf::from("."))
        .join(dir);
    let _ = fs::create_dir_all(&path);
    path
}

pub(super) fn draw_robot_preview(
    ui: &mut egui::Ui,
    robot: &mut RobotConfig,
    available_height: f32,
    camera: &mut RobotPreviewCamera,
    selection: &mut Option<String>,
) {
    let desired = egui::vec2(ui.available_width(), available_height.max(260.0));
    let (response, painter) = ui.allocate_painter(desired, egui::Sense::click_and_drag());
    let rect = response.rect;
    painter.rect_filled(
        rect,
        egui::Rounding::same(6.0),
        egui::Color32::from_gray(22),
    );
    painter.rect_stroke(
        rect,
        egui::Rounding::same(6.0),
        egui::Stroke::new(1.0, egui::Color32::from_gray(70)),
    );

    let base_bounds = robot_preview_base_bounds(robot);
    camera.zoom = camera.zoom.clamp(camera.min_zoom, camera.max_zoom);
    let mut bounds = camera.viewport_bounds(base_bounds);

    if response.hovered() {
        let scroll_y = ui.input(|i| i.raw_scroll_delta.y);
        if scroll_y.abs() > 0.0 {
            let pointer = ui.input(|i| i.pointer.hover_pos()).unwrap_or(rect.center());
            let before = camera.screen_to_world(rect, bounds, pointer);
            let factor = (scroll_y * 0.0015).exp();
            camera.zoom = (camera.zoom * factor).clamp(camera.min_zoom, camera.max_zoom);
            bounds = camera.viewport_bounds(base_bounds);
            let after = camera.screen_to_world(rect, bounds, pointer);
            camera.pan_m.x += before.x - after.x;
            camera.pan_m.y += before.y - after.y;
            bounds = camera.viewport_bounds(base_bounds);
            ui.ctx().request_repaint();
        }
    }

    let pan_button_down = ui.input(|i| {
        i.pointer.button_down(egui::PointerButton::Middle)
            || i.pointer.button_down(egui::PointerButton::Secondary)
    });
    if response.dragged() && pan_button_down {
        let delta = ui.input(|i| i.pointer.delta());
        let scale = world_screen_scale(rect, bounds).max(1e-9);
        camera.pan_m.x -= delta.x as f64 / scale;
        camera.pan_m.y += delta.y as f64 / scale;
        bounds = camera.viewport_bounds(base_bounds);
        ui.ctx().request_repaint();
    }

    ui.allocate_ui_at_rect(
        egui::Rect::from_min_size(
            rect.left_top() + egui::vec2(8.0, 8.0),
            egui::vec2(260.0, 24.0),
        ),
        |ui| {
            ui.horizontal(|ui| {
                if ui.small_button("Reset Zoom").clicked() {
                    camera.reset();
                }
                if ui.small_button("Fit Robot").clicked() {
                    camera.fit_rect();
                }
                if ui.small_button("Center View").clicked() {
                    camera.center();
                }
            });
        },
    );

    if response.clicked() || response.drag_started_by(egui::PointerButton::Primary) {
        if let Some(pointer) = response.interact_pointer_pos() {
            *selection = component_ids(robot)
                .into_iter()
                .filter_map(|id| {
                    component_pose(robot, &id).map(|p| {
                        (
                            id,
                            world_to_screen(rect, bounds, Vec2::new(p.x, p.y)).distance(pointer),
                        )
                    })
                })
                .filter(|(_, d)| *d < 18.0)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(id, _)| id);
        }
    }
    if response.dragged_by(egui::PointerButton::Primary) {
        if let Some(id) = selection.as_deref() {
            if let Some(mut pose) = component_pose(robot, id) {
                let delta = ui.input(|i| i.pointer.delta());
                let scale = world_screen_scale(rect, bounds);
                pose.x += delta.x as f64 / scale;
                pose.y -= delta.y as f64 / scale;
                let _ = set_component_pose(robot, id, pose);
            }
        }
    }
    draw_robot_preview_grid(&painter, rect, bounds);
    draw_robot_envelope(&painter, rect, bounds);

    let half_l = robot.chassis.length_m.max(0.001) * 0.5;
    let half_w = robot.chassis.width_m.max(0.001) * 0.5;
    let origin = camera.world_to_screen(rect, bounds, Vec2::new(0.0, 0.0));
    let x_axis = camera.world_to_screen(rect, bounds, Vec2::new(0.150, 0.0));
    let y_axis = camera.world_to_screen(rect, bounds, Vec2::new(0.0, 0.150));
    painter.line_segment(
        [origin, x_axis],
        egui::Stroke::new(2.0, egui::Color32::from_rgb(210, 120, 60)),
    );
    painter.line_segment(
        [origin, y_axis],
        egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 160, 220)),
    );
    painter.text(
        x_axis,
        egui::Align2::LEFT_CENTER,
        "+X front",
        egui::FontId::proportional(11.0),
        egui::Color32::from_rgb(230, 160, 100),
    );
    painter.text(
        y_axis,
        egui::Align2::CENTER_BOTTOM,
        "+Y left",
        egui::FontId::proportional(11.0),
        egui::Color32::from_rgb(120, 190, 240),
    );

    let chassis = [
        Vec2::new(half_l, half_w),
        Vec2::new(half_l, -half_w),
        Vec2::new(-half_l, -half_w),
        Vec2::new(-half_l, half_w),
    ];
    let chassis_points: Vec<egui::Pos2> = chassis
        .iter()
        .map(|p| world_to_screen(rect, bounds, *p))
        .collect();
    painter.add(egui::Shape::convex_polygon(
        chassis_points.clone(),
        egui::Color32::from_rgb(48, 55, 64),
        egui::Stroke::new(2.0, egui::Color32::from_rgb(170, 190, 210)),
    ));
    painter.add(egui::Shape::closed_line(
        chassis_points,
        egui::Stroke::new(2.0, egui::Color32::from_rgb(210, 220, 235)),
    ));

    for area in robot.line_validity_areas.iter().filter(|area| area.enabled) {
        let points: Vec<_> = validity_area_points(area)
            .iter()
            .map(|point| world_to_screen(rect, bounds, *point))
            .collect();
        painter.add(egui::Shape::convex_polygon(
            points.clone(),
            egui::Color32::from_rgba_premultiplied(70, 220, 120, 45),
            egui::Stroke::new(2.0, egui::Color32::from_rgb(80, 230, 135)),
        ));
        if let Some(center) = points
            .first()
            .map(|_| world_to_screen(rect, bounds, area.position_m))
        {
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                area.name.as_str(),
                egui::FontId::proportional(10.0),
                egui::Color32::from_rgb(150, 245, 180),
            );
        }
    }

    if robot.normal_force.chamber_area_m2 > 0.0 {
        let side = robot
            .normal_force
            .chamber_area_m2
            .sqrt()
            .clamp(0.005, robot.chassis.width_m.max(0.005));
        draw_preview_rect(
            &painter,
            rect,
            bounds,
            robot.normal_force.position_m,
            side,
            side,
            egui::Color32::from_rgba_premultiplied(80, 120, 210, 70),
            egui::Color32::from_rgb(90, 150, 240),
        );
    }

    let assembly = RobotAssembly::effective(robot);
    let hull: Vec<_> = assembly
        .support_polygon()
        .iter()
        .map(|p| world_to_screen(rect, bounds, *p))
        .collect();
    if hull.len() >= 3 {
        painter.add(egui::Shape::closed_line(
            hull,
            egui::Stroke::new(1.5, egui::Color32::GREEN),
        ));
    }
    for wheel in &assembly.wheels {
        let points = wheel_corners(wheel, Pose2::default())
            .iter()
            .map(|p| world_to_screen(rect, bounds, *p))
            .collect();
        painter.add(egui::Shape::closed_line(
            points,
            egui::Stroke::new(2.0, egui::Color32::WHITE),
        ));
        let p = world_to_screen(rect, bounds, wheel_world_position(wheel, Pose2::default()));
        painter.circle_filled(p, 3.0, egui::Color32::GREEN);
        painter.text(
            p,
            egui::Align2::LEFT_BOTTOM,
            &wheel.id,
            egui::FontId::proportional(9.0),
            egui::Color32::WHITE,
        );
    }

    for sensor in robot
        .sensors
        .iter()
        .filter(|sensor| sensor.enabled && sensor.visible_in_preview)
    {
        draw_sensor_instance(&painter, rect, bounds, sensor);
    }

    let com = world_to_screen(
        rect,
        bounds,
        assembly
            .mass_properties(robot)
            .map(|m| m.center_m)
            .unwrap_or(robot.chassis.center_of_mass_m),
    );
    painter.circle_stroke(
        com,
        7.0,
        egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 220, 70)),
    );
    painter.line_segment(
        [com + egui::vec2(-7.0, 0.0), com + egui::vec2(7.0, 0.0)],
        egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 220, 70)),
    );
    painter.line_segment(
        [com + egui::vec2(0.0, -7.0), com + egui::vec2(0.0, 7.0)],
        egui::Stroke::new(1.5, egui::Color32::from_rgb(255, 220, 70)),
    );
    painter.text(
        com + egui::vec2(8.0, 8.0),
        egui::Align2::LEFT_TOP,
        "COM",
        egui::FontId::proportional(11.0),
        egui::Color32::from_rgb(255, 220, 70),
    );

    let nf = world_to_screen(rect, bounds, robot.normal_force.position_m);
    let diamond = vec![
        nf + egui::vec2(0.0, -7.0),
        nf + egui::vec2(7.0, 0.0),
        nf + egui::vec2(0.0, 7.0),
        nf + egui::vec2(-7.0, 0.0),
    ];
    painter.add(egui::Shape::convex_polygon(
        diamond,
        egui::Color32::from_rgb(130, 100, 240),
        egui::Stroke::new(1.0, egui::Color32::from_rgb(210, 200, 255)),
    ));
    painter.text(
        nf + egui::vec2(8.0, -8.0),
        egui::Align2::LEFT_BOTTOM,
        "Normal/downforce",
        egui::FontId::proportional(10.0),
        egui::Color32::from_rgb(210, 200, 255),
    );

    for (idx, fan) in robot.normal_force.fans.iter().enumerate() {
        let fan_bounds = circle_bounds(fan.position_m, fan.visual_radius_m.max(0.012));
        let inside = bounds_inside_envelope(fan_bounds);
        let color = if inside {
            egui::Color32::from_rgb(120, 200, 255)
        } else {
            egui::Color32::from_rgb(255, 90, 90)
        };
        let p = world_to_screen(rect, bounds, fan.position_m);
        let r = world_len_to_screen(rect, bounds, fan.visual_radius_m.max(0.012)).clamp(6.0, 22.0);
        let action_r =
            world_len_to_screen(rect, bounds, fan.action_radius_m.max(fan.visual_radius_m));
        painter.circle_stroke(
            p,
            action_r,
            egui::Stroke::new(
                1.0,
                egui::Color32::from_rgba_premultiplied(120, 200, 255, 80),
            ),
        );
        painter.circle_stroke(p, r, egui::Stroke::new(2.0, color));
        painter.line_segment(
            [p + egui::vec2(-r, 0.0), p + egui::vec2(r, 0.0)],
            egui::Stroke::new(1.0, color),
        );
        painter.line_segment(
            [p + egui::vec2(0.0, -r), p + egui::vec2(0.0, r)],
            egui::Stroke::new(1.0, color),
        );
        painter.text(
            p + egui::vec2(r + 3.0, 0.0),
            egui::Align2::LEFT_CENTER,
            format!("Fan {idx}"),
            egui::FontId::proportional(10.0),
            color,
        );
    }

    let outside_count = robot_components_outside_count(robot);
    if outside_count > 0 {
        painter.text(
            rect.right_top() + egui::vec2(-10.0, 10.0),
            egui::Align2::RIGHT_TOP,
            format!("{outside_count} physical component(s) outside 250 × 250 mm"),
            egui::FontId::proportional(12.0),
            egui::Color32::from_rgb(255, 105, 90),
        );
    }

    if let Some(id) = selection.as_deref() {
        if let Some(p) = component_pose(robot, id) {
            painter.circle_stroke(
                world_to_screen(rect, bounds, Vec2::new(p.x, p.y)),
                10.,
                egui::Stroke::new(2., egui::Color32::YELLOW),
            );
        }
    }
    draw_preview_scale(&painter, rect, bounds);
    painter.text(
        rect.left_bottom() + egui::vec2(10.0, -10.0),
        egui::Align2::LEFT_BOTTOM,
        "Scroll: zoom | middle/right drag: pan | envelope: 250 mm × 250 mm",
        egui::FontId::proportional(11.0),
        egui::Color32::from_gray(150),
    );
}

const ROBOT_ENVELOPE_HALF_M: f64 = 0.125;

pub(super) fn robot_preview_base_bounds(robot: &RobotConfig) -> Bounds {
    let mut b = Bounds {
        min_x: -ROBOT_ENVELOPE_HALF_M,
        max_x: ROBOT_ENVELOPE_HALF_M,
        min_y: -ROBOT_ENVELOPE_HALF_M,
        max_y: ROBOT_ENVELOPE_HALF_M,
    };
    include_bounds(
        &mut b,
        rect_bounds(
            Vec2::default(),
            robot.chassis.length_m,
            robot.chassis.width_m,
        ),
    );

    for wheel in &RobotAssembly::effective(robot).wheels {
        include_bounds(
            &mut b,
            tight_bounds_from_points(&wheel_corners(wheel, Pose2::default())),
        );
    }

    if robot.normal_force.chamber_area_m2 > 0.0 {
        let side = robot
            .normal_force
            .chamber_area_m2
            .sqrt()
            .clamp(0.005, 0.250);
        include_bounds(
            &mut b,
            rect_bounds(robot.normal_force.position_m, side, side),
        );
    }
    for fan in &robot.normal_force.fans {
        include_bounds(
            &mut b,
            circle_bounds(fan.position_m, fan.visual_radius_m.max(0.012)),
        );
    }
    for sensor in robot.sensors.iter().filter(|sensor| sensor.enabled) {
        include_bounds(&mut b, sensor_physical_bounds(sensor));
    }
    for area in robot.line_validity_areas.iter().filter(|area| area.enabled) {
        include_bounds(
            &mut b,
            tight_bounds_from_points(&validity_area_points(area)),
        );
    }

    let margin_x = ((b.max_x - b.min_x) * 0.12).max(0.035);
    let margin_y = ((b.max_y - b.min_y) * 0.12).max(0.035);
    b.min_x -= margin_x;
    b.max_x += margin_x;
    b.min_y -= margin_y;
    b.max_y += margin_y;
    b
}

pub(super) fn rect_bounds(center: Vec2, length_m: f64, width_m: f64) -> Bounds {
    let half_l = length_m.max(0.0) * 0.5;
    let half_w = width_m.max(0.0) * 0.5;
    Bounds {
        min_x: center.x - half_l,
        max_x: center.x + half_l,
        min_y: center.y - half_w,
        max_y: center.y + half_w,
    }
}

pub(super) fn circle_bounds(center: Vec2, radius_m: f64) -> Bounds {
    let r = radius_m.max(0.0);
    Bounds {
        min_x: center.x - r,
        max_x: center.x + r,
        min_y: center.y - r,
        max_y: center.y + r,
    }
}

pub(super) fn include_bounds(dst: &mut Bounds, src: Bounds) {
    dst.min_x = dst.min_x.min(src.min_x);
    dst.max_x = dst.max_x.max(src.max_x);
    dst.min_y = dst.min_y.min(src.min_y);
    dst.max_y = dst.max_y.max(src.max_y);
}

pub(super) fn tight_bounds_from_points(points: &[Vec2]) -> Bounds {
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
    if !min_x.is_finite() || !max_x.is_finite() || !min_y.is_finite() || !max_y.is_finite() {
        return Bounds {
            min_x: 0.0,
            max_x: 0.0,
            min_y: 0.0,
            max_y: 0.0,
        };
    }
    Bounds {
        min_x,
        max_x,
        min_y,
        max_y,
    }
}

pub(super) fn bounds_inside_envelope(bounds: Bounds) -> bool {
    let eps = 1e-9;
    bounds.min_x >= -ROBOT_ENVELOPE_HALF_M - eps
        && bounds.max_x <= ROBOT_ENVELOPE_HALF_M + eps
        && bounds.min_y >= -ROBOT_ENVELOPE_HALF_M - eps
        && bounds.max_y <= ROBOT_ENVELOPE_HALF_M + eps
}

pub(super) fn robot_components_outside_count(robot: &RobotConfig) -> usize {
    let mut outside = 0;
    if !bounds_inside_envelope(rect_bounds(
        Vec2::default(),
        robot.chassis.length_m,
        robot.chassis.width_m,
    )) {
        outside += 1;
    }
    for wheel in &RobotAssembly::effective(robot).wheels {
        if !bounds_inside_envelope(tight_bounds_from_points(&wheel_corners(
            wheel,
            Pose2::default(),
        ))) {
            outside += 1;
        }
    }
    for fan in &robot.normal_force.fans {
        if !bounds_inside_envelope(circle_bounds(
            fan.position_m,
            fan.visual_radius_m.max(0.012),
        )) {
            outside += 1;
        }
    }
    for sensor in robot.sensors.iter().filter(|sensor| sensor.enabled) {
        if !bounds_inside_envelope(sensor_physical_bounds(sensor)) {
            outside += 1;
        }
    }
    for area in robot.line_validity_areas.iter().filter(|area| area.enabled) {
        if !bounds_inside_envelope(tight_bounds_from_points(&validity_area_points(area))) {
            outside += 1;
        }
    }
    outside
}

pub(super) fn draw_robot_preview_grid(painter: &egui::Painter, rect: egui::Rect, bounds: Bounds) {
    let scale = world_screen_scale(rect, bounds).max(1e-9);
    let target_px = 44.0;
    let raw_step_m = target_px / scale;
    let step_m = nice_step_m(raw_step_m).clamp(0.001, 1.0);
    let first_x = (bounds.min_x / step_m).floor() as i64 - 1;
    let last_x = (bounds.max_x / step_m).ceil() as i64 + 1;
    let first_y = (bounds.min_y / step_m).floor() as i64 - 1;
    let last_y = (bounds.max_y / step_m).ceil() as i64 + 1;
    for i in first_x..=last_x {
        let x = i as f64 * step_m;
        let p0 = world_to_screen(rect, bounds, Vec2::new(x, bounds.min_y));
        let p1 = world_to_screen(rect, bounds, Vec2::new(x, bounds.max_y));
        let color = if x.abs() < step_m * 0.5 {
            egui::Color32::from_gray(80)
        } else {
            egui::Color32::from_gray(42)
        };
        painter.line_segment([p0, p1], egui::Stroke::new(1.0, color));
    }
    for i in first_y..=last_y {
        let y = i as f64 * step_m;
        let p0 = world_to_screen(rect, bounds, Vec2::new(bounds.min_x, y));
        let p1 = world_to_screen(rect, bounds, Vec2::new(bounds.max_x, y));
        let color = if y.abs() < step_m * 0.5 {
            egui::Color32::from_gray(80)
        } else {
            egui::Color32::from_gray(42)
        };
        painter.line_segment([p0, p1], egui::Stroke::new(1.0, color));
    }
}

pub(super) fn nice_step_m(raw_m: f64) -> f64 {
    let raw = raw_m.max(1e-6);
    let exp = raw.log10().floor();
    let base = 10_f64.powf(exp);
    for mul in [1.0, 2.0, 5.0, 10.0] {
        let step = base * mul;
        if step >= raw {
            return step;
        }
    }
    base * 10.0
}

pub(super) fn draw_robot_envelope(painter: &egui::Painter, rect: egui::Rect, bounds: Bounds) {
    let corners = [
        Vec2::new(ROBOT_ENVELOPE_HALF_M, ROBOT_ENVELOPE_HALF_M),
        Vec2::new(ROBOT_ENVELOPE_HALF_M, -ROBOT_ENVELOPE_HALF_M),
        Vec2::new(-ROBOT_ENVELOPE_HALF_M, -ROBOT_ENVELOPE_HALF_M),
        Vec2::new(-ROBOT_ENVELOPE_HALF_M, ROBOT_ENVELOPE_HALF_M),
    ];
    let points: Vec<_> = corners
        .iter()
        .map(|p| world_to_screen(rect, bounds, *p))
        .collect();
    painter.add(egui::Shape::closed_line(
        points.clone(),
        egui::Stroke::new(2.0, egui::Color32::from_rgb(140, 170, 190)),
    ));
    let label_pos = points.first().copied().unwrap_or(rect.left_top()) + egui::vec2(4.0, -4.0);
    painter.text(
        label_pos,
        egui::Align2::LEFT_BOTTOM,
        "250 × 250 mm",
        egui::FontId::proportional(11.0),
        egui::Color32::from_rgb(170, 195, 215),
    );
}

pub(super) fn draw_preview_scale(painter: &egui::Painter, rect: egui::Rect, bounds: Bounds) {
    let scale = world_screen_scale(rect, bounds).max(1e-9);
    let desired_m = 100.0 / scale;
    let scale_len_m = nice_step_m(desired_m).clamp(0.01, 0.5);
    let len_px = (scale_len_m * scale) as f32;
    let y = rect.bottom() - 28.0;
    let x = rect.right() - len_px - 18.0;
    let p0 = egui::pos2(x, y);
    let p1 = egui::pos2(x + len_px, y);
    painter.line_segment(
        [p0, p1],
        egui::Stroke::new(2.0, egui::Color32::from_gray(210)),
    );
    painter.line_segment(
        [p0 + egui::vec2(0.0, -5.0), p0 + egui::vec2(0.0, 5.0)],
        egui::Stroke::new(2.0, egui::Color32::from_gray(210)),
    );
    painter.line_segment(
        [p1 + egui::vec2(0.0, -5.0), p1 + egui::vec2(0.0, 5.0)],
        egui::Stroke::new(2.0, egui::Color32::from_gray(210)),
    );
    let label = if scale_len_m < 1.0 {
        format!("{:.0} mm", scale_len_m * 1000.0)
    } else {
        format!("{:.1} m", scale_len_m)
    };
    painter.text(
        p0 + egui::vec2(len_px * 0.5, -7.0),
        egui::Align2::CENTER_BOTTOM,
        label,
        egui::FontId::proportional(11.0),
        egui::Color32::from_gray(220),
    );
}

pub(super) fn sensor_center(sensor: &RobotSensorInstance) -> Vec2 {
    sensor_world_position(sensor, Pose2::default())
}

pub(super) fn validity_area_points(area: &RobotLineValidityArea) -> [Vec2; 4] {
    let half_l = area.length_m.max(0.0001) * 0.5;
    let half_w = area.width_m.max(0.0001) * 0.5;
    let angle = area.angle_deg.to_radians();
    let (s, c) = angle.sin_cos();
    let transform = |x: f64, y: f64| {
        Vec2::new(
            area.position_m.x + x * c - y * s,
            area.position_m.y + x * s + y * c,
        )
    };
    [
        transform(half_l, half_w),
        transform(half_l, -half_w),
        transform(-half_l, -half_w),
        transform(-half_l, half_w),
    ]
}

pub(super) fn sensor_rotate(sensor: &RobotSensorInstance, local: Vec2) -> Vec2 {
    let a = sensor.angle_deg.to_radians();
    let (s, c) = a.sin_cos();
    Vec2::new(
        sensor.position_m.x + local.x * c - local.y * s,
        sensor.position_m.y + local.x * s + local.y * c,
    )
}

pub(super) fn sensor_oriented_rect_points(
    sensor: &RobotSensorInstance,
    length_m: f64,
    width_m: f64,
) -> [Vec2; 4] {
    let half_l = length_m.max(0.001) * 0.5;
    let half_w = width_m.max(0.001) * 0.5;
    [
        sensor_rotate(sensor, Vec2::new(half_l, half_w)),
        sensor_rotate(sensor, Vec2::new(half_l, -half_w)),
        sensor_rotate(sensor, Vec2::new(-half_l, -half_w)),
        sensor_rotate(sensor, Vec2::new(-half_l, half_w)),
    ]
}

pub(super) fn sensor_physical_bounds(sensor: &RobotSensorInstance) -> Bounds {
    if sensor.asset.visual_radius_m > 0.0 {
        return circle_bounds(sensor_center(sensor), sensor.asset.visual_radius_m);
    }
    let pts = sensor_oriented_rect_points(
        sensor,
        sensor.asset.visual_height_m.max(0.001),
        sensor.asset.visual_width_m.max(0.001),
    );
    tight_bounds_from_points(&pts)
}

pub(super) fn sensor_type_color(sensor_type: SensorType) -> egui::Color32 {
    match sensor_type {
        SensorType::LineAnalog | SensorType::LineDigital => egui::Color32::from_rgb(95, 210, 120),
        SensorType::DistanceInfrared | SensorType::DistanceToF | SensorType::Ultrasonic => {
            egui::Color32::from_rgb(240, 185, 80)
        }
        SensorType::Color => egui::Color32::from_rgb(220, 120, 220),
        SensorType::Encoder => egui::Color32::from_rgb(150, 200, 255),
        SensorType::Gyro | SensorType::Accelerometer => egui::Color32::from_rgb(175, 150, 255),
        SensorType::Custom => egui::Color32::from_rgb(210, 210, 210),
    }
}

pub(super) fn draw_sensor_instance(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    sensor: &RobotSensorInstance,
) {
    draw_sensor_detection_area(painter, rect, bounds, sensor);
    let sensor_bounds = sensor_physical_bounds(sensor);
    let inside = bounds_inside_envelope(sensor_bounds);
    let base = sensor_type_color(sensor.asset.sensor_type);
    let stroke = if inside {
        base
    } else {
        egui::Color32::from_rgb(255, 95, 90)
    };
    let center = world_to_screen(rect, bounds, sensor.position_m);
    let angle = sensor.angle_deg.to_radians();
    let forward = egui::vec2(angle.cos() as f32, -angle.sin() as f32);
    if sensor.asset.visual_radius_m > 0.0 {
        let r = world_len_to_screen(rect, bounds, sensor.asset.visual_radius_m).clamp(4.0, 18.0);
        painter.circle_filled(
            center,
            r,
            egui::Color32::from_rgba_premultiplied(base.r(), base.g(), base.b(), 55),
        );
        painter.circle_stroke(center, r, egui::Stroke::new(1.5, stroke));
        painter.line_segment(
            [center, center + forward * r],
            egui::Stroke::new(1.4, stroke),
        );
    } else {
        let pts = sensor_oriented_rect_points(
            sensor,
            sensor.asset.visual_height_m.max(0.001),
            sensor.asset.visual_width_m.max(0.001),
        );
        let points: Vec<_> = pts
            .iter()
            .map(|p| world_to_screen(rect, bounds, *p))
            .collect();
        painter.add(egui::Shape::convex_polygon(
            points,
            egui::Color32::from_rgba_premultiplied(base.r(), base.g(), base.b(), 55),
            egui::Stroke::new(1.5, stroke),
        ));
        painter.line_segment(
            [
                center,
                center
                    + forward
                        * world_len_to_screen(
                            rect,
                            bounds,
                            sensor.asset.visual_height_m.max(0.008) * 0.5,
                        ),
            ],
            egui::Stroke::new(1.2, stroke),
        );
    }
    let min_size_px = world_len_to_screen(
        rect,
        bounds,
        sensor
            .asset
            .visual_width_m
            .max(sensor.asset.visual_height_m)
            .max(sensor.asset.visual_radius_m),
    );
    if min_size_px > 12.0 && !sensor.name.is_empty() {
        painter.text(
            center + egui::vec2(4.0, -4.0),
            egui::Align2::LEFT_BOTTOM,
            sensor.name.as_str(),
            egui::FontId::proportional(9.5),
            stroke,
        );
    }
}

pub(super) fn draw_sensor_detection_area(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    sensor: &RobotSensorInstance,
) {
    let color = sensor_type_color(sensor.asset.sensor_type);
    let stroke = egui::Stroke::new(
        1.0,
        egui::Color32::from_rgba_premultiplied(color.r(), color.g(), color.b(), 90),
    );
    match &sensor.asset.detection_area {
        SensorDetectionArea::Point { radius_m } | SensorDetectionArea::Circle { radius_m } => {
            painter.circle_stroke(
                world_to_screen(rect, bounds, sensor.position_m),
                world_len_to_screen(rect, bounds, *radius_m).max(2.0),
                stroke,
            );
        }
        SensorDetectionArea::Rectangle { width_m, height_m } => {
            let pts = sensor_oriented_rect_points(sensor, *height_m, *width_m);
            let points: Vec<_> = pts
                .iter()
                .map(|p| world_to_screen(rect, bounds, *p))
                .collect();
            painter.add(egui::Shape::closed_line(points, stroke));
        }
        SensorDetectionArea::Cone { range_m, angle_deg } => {
            let center = world_to_screen(rect, bounds, sensor.position_m);
            let steps = 18;
            let half = angle_deg.to_radians() * 0.5;
            let base = sensor.angle_deg.to_radians();
            let mut points = vec![center];
            for i in 0..=steps {
                let t = -half + (2.0 * half) * (i as f64 / steps as f64);
                let a = base + t;
                let world = Vec2::new(
                    sensor.position_m.x + range_m * a.cos(),
                    sensor.position_m.y + range_m * a.sin(),
                );
                points.push(world_to_screen(rect, bounds, world));
            }
            painter.add(egui::Shape::closed_line(points, stroke));
        }
        SensorDetectionArea::CustomPolygon { points_m } => {
            if points_m.len() >= 2 {
                let points: Vec<_> = points_m
                    .iter()
                    .map(|p| world_to_screen(rect, bounds, sensor_rotate(sensor, *p)))
                    .collect();
                painter.add(egui::Shape::closed_line(points, stroke));
            }
        }
    }
}

pub(super) fn draw_preview_rect(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    center: Vec2,
    length_m: f64,
    width_m: f64,
    fill: egui::Color32,
    stroke: egui::Color32,
) {
    let half_l = length_m * 0.5;
    let half_w = width_m * 0.5;
    let corners = [
        Vec2::new(center.x + half_l, center.y + half_w),
        Vec2::new(center.x + half_l, center.y - half_w),
        Vec2::new(center.x - half_l, center.y - half_w),
        Vec2::new(center.x - half_l, center.y + half_w),
    ];
    let points: Vec<egui::Pos2> = corners
        .iter()
        .map(|p| world_to_screen(rect, bounds, *p))
        .collect();
    painter.add(egui::Shape::convex_polygon(
        points,
        fill,
        egui::Stroke::new(1.0, stroke),
    ));
}

pub(super) fn edit_assembly_panel(
    ui: &mut egui::Ui,
    robot: &mut RobotConfig,
    selection: &mut Option<String>,
) -> bool {
    let before = robot_json(robot);
    edit_sensing_panel(ui, robot);
    egui::CollapsingHeader::new("Fidelidade da física").default_open(true).show(ui,|ui| {
        let mut selected=robot.physics.as_ref().map(|f|f.preset.clone()).unwrap_or_else(||"reference".into());
        let old=selected.clone();
        egui::ComboBox::from_id_source("physics_preset").selected_text(&selected).show_ui(ui,|ui| {for (id,label) in [("ideal","Ideal — cinemática"),("simplified","Simplificado — contato por roda"),("realistic","Realista — pneu reduzido calibrável"),("reference","Referência anterior — agregado por lado")] {ui.selectable_value(&mut selected,id.into(),label);}});
        if old!=selected {robot.physics=if selected=="reference" {None}else{Some(crate::models::fidelity::FidelityConfig::preset(&selected).unwrap())};}
        if let Some(f)=&mut robot.physics {
            ui.label("Realista é um modelo reduzido; os parâmetros exigem calibração. Não há dinâmica vertical.");
            egui::ComboBox::from_id_source("contact_model").selected_text(&f.contact).show_ui(ui,|ui|{for id in ["ideal","coulomb","brush"] {ui.selectable_value(&mut f.contact,id.into(),id);}});
            egui::ComboBox::from_id_source("normal_model").selected_text(&f.normal).show_ui(ui,|ui|{for id in ["static","quasi_static"] {ui.selectable_value(&mut f.normal,id.into(),id);}});
            ui.checkbox(&mut f.rolling,"Resistência de rolamento");
            for (label,value) in [("Coeficiente longitudinal (N·s/m)",&mut f.longitudinal_stiffness),("Coeficiente lateral (N·s/m)",&mut f.lateral_stiffness),("Relaxação (s)",&mut f.relaxation_s),("Expoente de sensibilidade à carga",&mut f.load_exponent),("Carga de referência (N)",&mut f.reference_load_n),("Rigidez radial (N/m; zero desliga)",&mut f.radial_stiffness_n_m),("Trail do caster (m)",&mut f.caster_trail_m)] {ui.horizontal(|ui|{ui.label(label);ui.add(egui::DragValue::new(value).speed(0.001));});}
            if let Err(e)=f.validate() {ui.colored_label(egui::Color32::RED,e);}
        }
    });
    egui::CollapsingHeader::new("Motores e alimentação acoplados").default_open(true).show(ui,|ui| {
        let mut enabled=robot.powertrain.is_some();if ui.checkbox(&mut enabled,"Usar cadeia elétrica com estado").changed(){robot.powertrain=enabled.then(crate::models::electrical::PowertrainConfig::default);}
        if let Some(p)=&mut robot.powertrain {
            ui.label("Exige física dinâmica por roda e uma roda motriz por motor; outros apoios podem ser passivos. Estes parâmetros substituem os motores simples da referência anterior.");
            for (side,m) in p.motors.iter_mut().enumerate(){ui.push_id(side,|ui|{egui::CollapsingHeader::new(if side==0 {"Motor esquerdo"}else{"Motor direito"}).show(ui,|ui|{
                egui::ComboBox::from_id_source("electrical_model").selected_text(&m.model).show_ui(ui,|ui|{for id in ["dc_simple","dc_electrical"]{ui.selectable_value(&mut m.model,id.into(),id);}});
                egui::ComboBox::from_id_source("parameter_shaft").selected_text(&m.parameter_shaft).show_ui(ui,|ui|{for id in ["motor","output"]{ui.selectable_value(&mut m.parameter_shaft,id.into(),id);}});
                for(label,value)in[("R (ohm)",&mut m.resistance_ohm),("L (H; zero no DC simples)",&mut m.inductance_h),("Ke (V·s/rad)",&mut m.ke_v_s_rad),("Kt (N·m/A)",&mut m.kt_nm_a),("Inércia do rotor (kg·m²)",&mut m.rotor_inertia_kg_m2),("Perda viscosa (N·m·s)",&mut m.viscous_nm_s),("Redução (motor/roda)",&mut m.ratio),("Eficiência mecânica",&mut m.efficiency)]{ui.horizontal(|ui|{ui.label(label);ui.add(egui::DragValue::new(value).speed(0.00001));});}
            });});}
            egui::ComboBox::from_id_source("battery_source").selected_text(&p.source).show_ui(ui,|ui|{ui.selectable_value(&mut p.source,"ideal".into(),"Fonte ideal");ui.selectable_value(&mut p.source,"thevenin".into(),"Bateria com RC");});
            ui.checkbox(&mut p.regeneration,"Permitir regeneração na bateria");
            for(label,value)in[("Resistência RC (ohm)",&mut p.rc_resistance_ohm),("Tempo RC (s)",&mut p.rc_time_s),("Fiação (ohm)",&mut p.wiring_resistance_ohm),("Lógica e sensores (W)",&mut p.auxiliary_power_w),("Eficiência regulador",&mut p.regulator_efficiency),("Ponte (ohm)",&mut p.bridge_resistance_ohm),("Corrente máxima de carga (A)",&mut p.charge_limit_a),("Subtensão (V)",&mut p.undervoltage_v),("Capacidade térmica (J/K)",&mut p.thermal_capacity_j_k),("Resistência térmica (K/W)",&mut p.thermal_resistance_k_w),("Ambiente (°C)",&mut p.ambient_c),("Limite térmico (°C)",&mut p.max_temperature_c),("Coeficiente térmico de R (1/K)",&mut p.resistance_temp_coefficient),("Folga fixa da sucção (m)",&mut p.suction_gap_m),("Vazamento por folga (1/m)",&mut p.suction_leak_per_m),("Tolerância do circuito (V)",&mut p.voltage_tolerance_v)]{ui.horizontal(|ui|{ui.label(label);ui.add(egui::DragValue::new(value).speed(0.001));});}
            ui.horizontal(|ui|{ui.label("Latência do driver (µs)");ui.add(egui::DragValue::new(&mut p.command_latency_us));ui.label("Máximo de iterações");ui.add(egui::DragValue::new(&mut p.max_iterations));});
            for(label,curve,defaults)in[("OCV × SoC",&mut p.ocv_curve,[(0.,robot.battery.empty_voltage_v),(1.,robot.battery.full_voltage_v)]),("Resistência × SoC",&mut p.resistance_curve,[(0.,robot.battery.internal_resistance_ohm),(1.,robot.battery.internal_resistance_ohm)])]{ui.push_id(label,|ui|{egui::CollapsingHeader::new(label).show(ui,|ui|{if ui.button("Preencher extremos").clicked(){*curve=defaults.to_vec();}if ui.button("Inserir ponto intermediário").clicked(){curve.push((0.5,(defaults[0].1+defaults[1].1)/2.));curve.sort_by(|a,b|a.0.total_cmp(&b.0));}if ui.button("Usar parâmetros básicos").clicked(){curve.clear();}let mut remove=None;for(i,(soc,value))in curve.iter_mut().enumerate(){ui.horizontal(|ui|{ui.label("SoC");ui.add(egui::DragValue::new(soc).speed(0.01));ui.add(egui::DragValue::new(value).speed(0.01));if ui.button("Remover").clicked(){remove=Some(i);}});}if let Some(i)=remove{curve.remove(i);}});});}
            if let Err(e)=p.validate(){ui.colored_label(egui::Color32::RED,e);}
        }
    });
    if robot.assembly.is_none() {
        robot.assembly = Some(RobotAssembly::from_legacy(robot));
    }
    egui::CollapsingHeader::new("Montagem física").default_open(true).show(ui,|ui|{
        ui.label("Origem: centro do retângulo de apoio. X frente, Y esquerda, Z acima da pista.");
        ui.label("Alturas e áreas ópticas são preservadas; o cálculo atual é planar e a leitura óptica é pontual.");
        let ids=component_ids(robot);
        egui::ComboBox::from_id_source("assembly_selection").selected_text(selection.as_deref().unwrap_or("Selecionar componente")).show_ui(ui,|ui|{for id in &ids {ui.selectable_value(selection,Some(id.clone()),id);}});
        ui.horizontal_wrapped(|ui|{
            if ui.button("Adicionar roda/apoio").clicked(){let a=robot.assembly.as_mut().unwrap();let mut w=RobotAssembly::from_legacy(&default_robot_config()).wheels.remove(0);w.id=crate::io::assets::new_instance_id();w.kind=WheelKind::Passive;w.motor=None;*selection=Some(w.id.clone());a.wheels.push(w);}
            if ui.button("Adicionar sensor").clicked(){let s=default_sensor_instance();*selection=Some(s.id.clone());robot.sensors.push(s);}
            if ui.button("Adicionar fan").clicked(){let f=default_fan_config(robot.battery.nominal_voltage_v);*selection=Some(f.id.clone());robot.normal_force.fans.push(f);}
        });
        if let Some(id)=selection.clone(){
            ui.horizontal(|ui|{
                if ui.button("Duplicar").clicked(){if let Ok(new)=duplicate_component(robot,&id){*selection=Some(new);}}
                if ui.button("Remover").clicked(){let _=remove_component(robot,&id);*selection=None;}
            });
            if let Some(mut pose)=component_pose(robot,&id){
                edit_mm(ui,"X [mm]",&mut pose.x);edit_mm(ui,"Y [mm]",&mut pose.y);
                if !robot.normal_force.fans.iter().any(|f|f.id==id){let mut angle=pose.yaw.to_degrees();ui.add(egui::DragValue::new(&mut angle).suffix(" graus").speed(1.));pose.yaw=angle.to_radians();}
                ui.horizontal_wrapped(|ui|{
                    if ui.button("Alinhar X=0").clicked(){pose.x=0.;}
                    if ui.button("Alinhar Y=0").clicked(){pose.y=0.;}
                    if ui.button("Grade 1 mm").clicked(){pose.x=(pose.x*1000.).round()/1000.;pose.y=(pose.y*1000.).round()/1000.;}
                });
                ui.label(format!("Distância à origem: {:.2} mm",(pose.x*pose.x+pose.y*pose.y).sqrt()*1000.));
                let _=set_component_pose(robot,&id,pose);
            }
            if let Some(w)=robot.assembly.as_mut().unwrap().wheels.iter_mut().find(|w|w.id==id){
                ui.label(format!("ID: {}",w.id));
                egui::ComboBox::from_id_source("wheel_kind").selected_text(format!("{:?}",w.kind)).show_ui(ui,|ui|{for k in [WheelKind::Driven,WheelKind::Passive,WheelKind::Caster]{if ui.selectable_value(&mut w.kind,k,format!("{k:?}")).changed(){w.motor=if k==WheelKind::Driven {Some("motor:left".into())}else{None};}}});
                if w.kind==WheelKind::Driven {egui::ComboBox::from_id_source("wheel_motor").selected_text(w.motor.as_deref().unwrap_or("Motor")).show_ui(ui,|ui|{for m in ["motor:left","motor:right"]{ui.selectable_value(&mut w.motor,Some(m.into()),m);}});}
                edit_mm(ui,"Raio [mm]",&mut w.radius_m);edit_mm(ui,"Largura [mm]",&mut w.width_m);edit_mm(ui,"Contato Z [mm] (sem efeito vertical)",&mut w.height_m);
                ui.label("Inércia desta roda [kg m²]");ui.add(egui::DragValue::new(&mut w.inertia_kg_m2).speed(1e-8).max_decimals(10));
                ui.label("Material");ui.text_edit_singleline(&mut w.material);
                ui.label("Atrito longitudinal / lateral");ui.add(egui::DragValue::new(&mut w.tire.mu_longitudinal).speed(0.01));ui.add(egui::DragValue::new(&mut w.tire.mu_lateral).speed(0.01));
                ui.label("Resistência ao rolamento");ui.add(egui::DragValue::new(&mut w.tire.rolling_resistance).speed(0.001));
                if ui.button("Aplicar pneu/material a todas as rodas").clicked(){let tire=w.tire.clone();let material=w.material.clone();for w in &mut robot.assembly.as_mut().unwrap().wheels{w.tire=tire.clone();w.material=material.clone();}}
            }
            if let Some(s)=robot.sensors.iter_mut().find(|s|s.id==id){edit_mm(ui,"Altura sensor [mm] (metadado)",&mut s.height_m);}
        }
        ui.separator();
        let a=robot.assembly.as_mut().unwrap();
        ui.label("Distribuição de massa (modos exclusivos)");
        ui.radio_value(&mut a.mass_mode,MassMode::Measured,"Massa/COM/inércia medidos no painel Chassis");
        ui.radio_value(&mut a.mass_mode,MassMode::Components,"Calcular somente pela soma dos componentes");
        edit_mm(ui,"COM Z medido [mm]",&mut a.measured_com_height_m);
        ui.label("Na conversão, a massa global fica em chassis. Redistribua-a antes de acrescentar massas de componentes para não contar o mesmo material duas vezes.");
        if ui.button("Adicionar massa estrutural").clicked(){a.masses.push(MassElement{id:crate::io::assets::new_instance_id(),component:None,mass_kg:0.01,position_m:Vec2::default(),height_m:0.,inertia_kg_m2:1e-7});}
        let choices:Vec<_>=["chassis","battery","motor:left","motor:right","driver","controller","encoder","gyro"].iter().map(|v|v.to_string()).chain(ids).collect();
        let mut remove=None;
        for (i,m) in a.masses.iter_mut().enumerate(){ui.push_id(&m.id,|ui|{
            egui::CollapsingHeader::new(m.component.as_deref().unwrap_or(&m.id)).show(ui,|ui|{
                egui::ComboBox::from_id_source("mass_component").selected_text(m.component.as_deref().unwrap_or("Parte independente")).show_ui(ui,|ui|{ui.selectable_value(&mut m.component,None,"Parte independente");for id in &choices {ui.selectable_value(&mut m.component,Some(id.clone()),id);}});
                let mut grams=m.mass_kg*1000.;ui.add(egui::DragValue::new(&mut grams).suffix(" g").speed(0.1));m.mass_kg=grams/1000.;
                ui.label("Posição própria para estrutura, bateria e motores; rodas/sensores/fans seguem sua montagem.");
                edit_mm(ui,"X [mm]",&mut m.position_m.x);edit_mm(ui,"Y [mm]",&mut m.position_m.y);edit_mm(ui,"Z [mm]",&mut m.height_m);
                ui.label("Inércia própria em yaw [kg m²]");ui.add(egui::DragValue::new(&mut m.inertia_kg_m2).speed(1e-8).max_decimals(10));
                if ui.button("Remover massa").clicked(){remove=Some(i);}
            });
        });}
        if let Some(i)=remove{a.masses.remove(i);}
        let a=RobotAssembly::effective(robot);
        if let Ok(m)=a.mass_properties(robot){ui.label(format!("Efetivo: {:.2} g | COM ({:.2}, {:.2}, {:.2}) mm | Iz {:.8} kg m²",m.mass_kg*1000.,m.center_m.x*1000.,m.center_m.y*1000.,m.height_m*1000.,m.inertia_kg_m2));}
        match a.validate(robot){Ok(warnings)=>{for w in warnings{ui.colored_label(egui::Color32::YELLOW,w);}},Err(e)=>{ui.colored_label(egui::Color32::RED,e);}}
        ui.label(format!("Modelos: corpo planar, {}, {}, {}. Contatos verdes; apoio verde; COM amarelo.",robot.motor_left.model,robot.tire.model,robot.normal_force.model.as_str()));
    });
    before != robot_json(robot)
}
fn edit_mm(ui: &mut egui::Ui, label: &str, value: &mut f64) {
    ui.horizontal(|ui| {
        ui.label(label);
        let mut mm = *value * 1000.;
        if ui.add(egui::DragValue::new(&mut mm).speed(0.1)).changed() {
            *value = mm / 1000.;
        }
    });
}

/// Uses exactly the body-frame transform used by sensor sampling and wheel geometry.
pub(super) fn draw_assembly_world(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    robot: &RobotConfig,
    pose: Pose2,
) {
    draw_robot(
        painter,
        rect,
        bounds,
        pose,
        robot.chassis.length_m,
        robot.chassis.width_m,
    );
    let a = RobotAssembly::effective(robot);
    for w in &a.wheels {
        painter.add(egui::Shape::closed_line(
            wheel_corners(w, pose)
                .iter()
                .map(|p| world_to_screen(rect, bounds, *p))
                .collect(),
            egui::Stroke::new(1., egui::Color32::WHITE),
        ));
    }
    for s in robot
        .sensors
        .iter()
        .filter(|s| s.enabled && s.visible_in_preview)
    {
        let p = world_to_screen(rect, bounds, sensor_world_position(s, pose));
        painter.circle_filled(p, 3., sensor_type_color(s.asset.sensor_type));
    }
    for f in &robot.normal_force.fans {
        painter.circle_stroke(
            world_to_screen(rect, bounds, pose.transform_point(f.position_m)),
            world_len_to_screen(rect, bounds, f.visual_radius_m),
            egui::Stroke::new(1., egui::Color32::LIGHT_BLUE),
        );
    }
    if let Ok(m) = a.mass_properties(robot) {
        painter.circle_filled(
            world_to_screen(rect, bounds, pose.transform_point(m.center_m)),
            3.,
            egui::Color32::YELLOW,
        );
    }
}
#[cfg(test)]
mod stage4_ui_tests {
    use super::*;
    #[test]
    fn assembly_editor_and_previews_render_without_a_window() {
        let context = egui::Context::default();
        let mut robot = default_robot_config();
        robot.assembly = Some(RobotAssembly::from_legacy(&robot));
        let mut camera = RobotPreviewCamera::default();
        let mut selected = Some(robot.sensors[0].id.clone());
        for frame in 0..4 {
            let mut input = egui::RawInput::default();
            input.screen_rect = Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1400., 1000.),
            ));
            let output = context.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    if frame == 0 {
                        edit_assembly_panel(ui, &mut robot, &mut selected);
                    } else if frame == 1 {
                        selected = Some(robot.assembly.as_ref().unwrap().wheels[0].id.clone());
                        edit_assembly_panel(ui, &mut robot, &mut selected);
                    } else if frame == 2 {
                        draw_robot_preview(ui, &mut robot, 700., &mut camera, &mut selected);
                    } else {
                        let (rect, _) =
                            ui.allocate_exact_size(egui::vec2(700., 700.), egui::Sense::hover());
                        draw_assembly_world(
                            ui.painter(),
                            rect,
                            Bounds {
                                min_x: -1.,
                                max_x: 1.,
                                min_y: -1.,
                                max_y: 1.,
                            },
                            &robot,
                            Pose2::new(0.3, -0.2, 1.2),
                        );
                    }
                });
            });
            assert!(!output.shapes.is_empty());
        }
    }
}

#[cfg(test)]
mod stage6_ui_tests {
    use super::*;
    #[test]
    fn all_fidelity_presets_render_without_window_and_keep_overrides() {
        let ctx = egui::Context::default();
        let mut robot = default_robot_config();
        let mut selected = None;
        for preset in ["ideal", "simplified", "realistic"] {
            robot.physics = Some(crate::models::fidelity::FidelityConfig::preset(preset).unwrap());
            let expected = robot.physics.clone();
            let out = ctx.run(egui::RawInput::default(), |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    edit_assembly_panel(ui, &mut robot, &mut selected);
                });
            });
            assert!(!out.shapes.is_empty());
            assert_eq!(robot.physics, expected);
        }
    }
}

#[cfg(test)]
mod stage7_ui_tests {
    use super::*;
    #[test]
    fn electrical_editor_renders_and_preserves_configuration() {
        let ctx = egui::Context::default();
        let mut robot =
            crate::config::load_project(std::path::Path::new("examples/power/projeto.rtsim"))
                .unwrap()
                .robot;
        let expected = robot.powertrain.clone();
        let mut selected = None;
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                edit_assembly_panel(ui, &mut robot, &mut selected);
            });
        });
        assert!(!out.shapes.is_empty());
        assert_eq!(robot.powertrain, expected);
    }
}

fn edit_sensing_panel(ui: &mut egui::Ui, robot: &mut RobotConfig) {
    egui::CollapsingHeader::new("Sensores e controle").default_open(true).show(ui,|ui|{
  ui.label("Cada sensor tem aquisição e entrega próprias. Tempos devem respeitar o passo físico; período zero usa o período do projeto.");
  for sensor in &robot.sensors {ui.push_id(&sensor.id,|ui|{egui::CollapsingHeader::new(&sensor.name).show(ui,|ui|{
   let mut custom=robot.sensing.optical.contains_key(&sensor.id);if ui.checkbox(&mut custom,"Configurar aquisição individual").changed(){if custom{robot.sensing.optical.insert(sensor.id.clone(),Default::default());}else{robot.sensing.optical.remove(&sensor.id);}}
   if let Some(a)=robot.sensing.optical.get_mut(&sensor.id){let mut value=a.value();if edit_sensing_numbers(ui,&mut value){match crate::models::sensing::Acquisition::from_value(&value){Ok(next)=>*a=next,Err(e)=>{ui.colored_label(egui::Color32::RED,e);}}}}
  });});}
  egui::CollapsingHeader::new("Encoder").show(ui,|ui|{ui.label("Razão 1: eixo da roda. Para eixo do motor, informe sua redução. Resolução efetiva = pulsos/volta × quadratura × razão.");let mut value=robot.sensing.encoder.value();if edit_sensing_numbers(ui,&mut value){if let Ok(next)=crate::models::sensing::EncoderSettings::from_value(&value){robot.sensing.encoder=next;}}});
  egui::CollapsingHeader::new("IMU").show(ui,|ui|{ui.label("Aceleração planar no centro do corpo; rotação de montagem em yaw. Drift do gyro é densidade por raiz de segundo.");let mut value=robot.sensing.imu.value();if edit_sensing_numbers(ui,&mut value){if let Ok(next)=crate::models::sensing::ImuSettings::from_value(&value){robot.sensing.imu=next;}}});
  egui::CollapsingHeader::new("Controle e estimação").show(ui,|ui|{ui.label("Modo de velocidade: 0 usa PWM base; 1 controla velocidade medida. Marcas largas nos canais ativos delimitam voltas estimadas; isso não substitui a arbitragem da pista.");let mut value=robot.sensing.control.value();if edit_sensing_numbers(ui,&mut value){if let Ok(next)=crate::models::sensing::ControlSettings::from_value(&value){robot.sensing.control=next;}}});
  let mut replay=robot.sensing.replay_csv.is_some();if ui.checkbox(&mut replay,"Repetir comandos registrados").changed(){robot.sensing.replay_csv=if replay{Some("t_us,pwm_left,pwm_right,downforce_pwm,mode_left,mode_right\n0,0,0,0,drive,drive\n".into())}else{None};}if let Some(text)=&mut robot.sensing.replay_csv{ui.label("Cole o conteúdo de um arquivo .commands.csv; substitui o controlador interno.");ui.add(egui::TextEdit::multiline(text).desired_rows(5));}
 });
}
fn edit_sensing_numbers(ui: &mut egui::Ui, value: &mut crate::json::JsonValue) -> bool {
    let crate::json::JsonValue::Object(fields) = value else {
        return false;
    };
    let mut changed = false;
    for (key, value) in fields {
        if let crate::json::JsonValue::Number(n) = value {
            ui.push_id(key.as_str(), |ui| {
                ui.horizontal(|ui| {
                    let label = match key.as_str() {
                        "period_us" => "Período (µs; 0 = projeto)",
                        "phase_us" => "Fase / canal sequencial (µs)",
                        "mux_group" => "Multiplexador (0 = independente)",
                        "conversion_us" => "Conversão (µs)",
                        "latency_us" => "Atraso de entrega (µs)",
                        "filter_tau_s" => "Constante do filtro (s)",
                        "threshold" => "Limiar digital",
                        "hysteresis" => "Largura da histerese",
                        "area_samples" => "Divisões por eixo da área",
                        "shaft_ratio" => "Razão entre eixo e roda",
                        "quadrature" => "Quadratura (1, 2 ou 4)",
                        "loss_probability" => "Probabilidade de perder pulso",
                        "seed" => "Seed de ruído",
                        "derivative_tau_s" => "Filtro da derivada (s)",
                        "integral_limit" => "Limite das integrais",
                        "speed_mode" => "Controle de velocidade (0/1)",
                        "target_speed_m_s" => "Velocidade desejada (m/s)",
                        "speed_kp" => "Ganho proporcional de velocidade",
                        "speed_ki" => "Ganho integral de velocidade",
                        "recovery_pwm" => "PWM de busca da linha",
                        "loss_timeout_us" => "Espera antes da busca (µs)",
                        "gyro_weight" => "Peso do gyro na odometria",
                        "mark_threshold" => "Sinal mínimo da marca",
                        "mark_min_channels" => "Canais mínimos da marca",
                        "mark_refractory_us" => "Intervalo mínimo entre marcas (µs)",
                        "lap_min_distance_m" => "Distância mínima entre voltas (m)",
                        "profile_enabled" => "Usar mapa aprendido (0/1)",
                        "curve_speed_factor" => "Fator de velocidade em curvas",
                        "drift_std_rad_s_sqrt_s" => "Drift do gyro (rad/s/√s)",
                        "yaw_misalignment_deg" => "Ângulo de montagem da IMU (graus)",
                        "accel_noise_std_m_s2" => "Ruído da aceleração (m/s²)",
                        "accel_bias_x_m_s2" => "Bias da aceleração X (m/s²)",
                        "accel_bias_y_m_s2" => "Bias da aceleração Y (m/s²)",
                        "accel_limit_m_s2" => "Saturação da aceleração (m/s²)",
                        _ => key.as_str(),
                    };
                    ui.label(label);
                    let integer = key.ends_with("_us")
                        || [
                            "quadrature",
                            "mux_group",
                            "area_samples",
                            "seed",
                            "speed_mode",
                            "profile_enabled",
                            "mark_min_channels",
                        ]
                        .contains(&key.as_str());
                    let edit = egui::DragValue::new(n).speed(if integer { 1. } else { 0.001 });
                    changed |= ui
                        .add(if integer { edit.max_decimals(0) } else { edit })
                        .changed();
                });
            });
        }
    }
    changed
}
#[cfg(test)]
mod stage8_ui_tests {
    use super::*;
    #[test]
    fn acquisition_editor_renders_without_changing_settings() {
        let ctx = egui::Context::default();
        let mut robot = default_robot_config();
        let before = robot.sensing.clone();
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| edit_sensing_panel(ui, &mut robot));
        });
        assert!(!out.shapes.is_empty());
        assert_eq!(robot.sensing, before);
    }
}
