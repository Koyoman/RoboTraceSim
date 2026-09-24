use super::*;
use crate::track::{
    definition::*,
    runtime::{TrackRuntime, TrackRuntimeCache},
};
pub(super) fn cached_track(
    ui_ctx: &egui::Context,
    track: &TrackConfig,
) -> Result<TrackRuntime, String> {
    let id = egui::Id::new("track_runtime_preview_cache");
    ui_ctx.data_mut(|data| {
        let cache = data.get_temp_mut_or_default::<TrackRuntimeCache>(id);
        cache.get(track)
    })
}
fn mm(ui: &mut egui::Ui, label: &str, v: &mut f64) {
    ui.horizontal(|ui| {
        ui.label(label);
        let mut x = *v * 1000.;
        if ui.add(egui::DragValue::new(&mut x).speed(0.1)).changed() {
            *v = x / 1000.;
        }
    });
}
fn rect_edit(ui: &mut egui::Ui, r: &mut RectArea, grid: f64) {
    mm(ui, "Centro X [mm]", &mut r.center_m.x);
    mm(ui, "Centro Y [mm]", &mut r.center_m.y);
    mm(ui, "Comprimento [mm]", &mut r.size_m.x);
    mm(ui, "Largura [mm]", &mut r.size_m.y);
    ui.add(egui::DragValue::new(&mut r.angle_deg).suffix(" graus"));
    if ui.button("Ajustar centro à grade").clicked() && grid > 0. {
        r.center_m.x = (r.center_m.x / grid).round() * grid;
        r.center_m.y = (r.center_m.y / grid).round() * grid;
    }
}
fn new_area() -> RectArea {
    RectArea {
        center_m: Vec2::new(0.5, 0.5),
        size_m: Vec2::new(0.1, 0.1),
        angle_deg: 0.,
    }
}
fn reorder_buttons(
    ui: &mut egui::Ui,
    i: usize,
    len: usize,
    action: &mut Option<(usize, usize)>,
    remove: &mut Option<usize>,
) {
    ui.horizontal(|ui| {
        if ui.add_enabled(i > 0, egui::Button::new("Subir")).clicked() {
            *action = Some((i, i - 1));
        }
        if ui
            .add_enabled(i + 1 < len, egui::Button::new("Descer"))
            .clicked()
        {
            *action = Some((i, i + 1));
        }
        if ui.button("Remover").clicked() {
            *remove = Some(i);
        }
    });
}
fn apply_order<T>(items: &mut Vec<T>, action: Option<(usize, usize)>, remove: Option<usize>) {
    if let Some(i) = remove {
        items.remove(i);
    } else if let Some((a, b)) = action {
        let _ = move_item(items, a, b);
    }
}
pub(super) fn edit_track_environment(
    ui: &mut egui::Ui,
    track: &mut TrackConfig,
    project_pose: &mut Pose2,
) -> bool {
    let before = track_json(track);
    let previous_pose = *project_pose;
    let grid = track
        .parametric
        .as_ref()
        .map(|p| p.area.grid_mm / 1000.)
        .unwrap_or(0.001);
    egui::CollapsingHeader::new("Superfície, marcas e corrida").default_open(true).show(ui,|ui|{
  ui.label("Grade, eixos, contorno de largada e portais amarelos são guias visuais; não são pintura óptica.");
  ui.label("Última região sobreposta prevalece. A linha cobre o substrato; marcas são aplicadas depois, na ordem da lista. Cores não definem refletância.");
  ui.label("Origem da pose inicial");ui.radio_value(&mut track.environment.start_source,StartSource::Track,"Largada da pista");ui.radio_value(&mut track.environment.start_source,StartSource::Project,"Pose do projeto");
  if track.environment.start_source==StartSource::Project{mm(ui,"X inicial [mm]",&mut project_pose.x);mm(ui,"Y inicial [mm]",&mut project_pose.y);let mut deg=project_pose.yaw.to_degrees();ui.add(egui::DragValue::new(&mut deg).suffix(" graus"));project_pose.yaw=deg.to_radians();}
  match effective_start_pose(track,*project_pose){Ok(p)=>{ui.label(format!("Pose efetiva: X {:.2} mm, Y {:.2} mm, ângulo {:.2}°",p.x*1000.,p.y*1000.,p.yaw.to_degrees()));},Err(e)=>{ui.colored_label(egui::Color32::RED,e);}}
  let e=&mut track.environment;
  ui.horizontal(|ui|{ui.label("Material fora da mesa");ui.text_edit_singleline(&mut e.outside_material);});ui.add(egui::DragValue::new(&mut e.outside_mu).prefix("Atrito externo ").speed(0.01));ui.add(egui::DragValue::new(&mut e.outside_reflectance).prefix("Refletância externa ").speed(0.01));
  if ui.button("Adicionar região").clicked(){e.regions.push(SurfaceRegion{id:crate::io::assets::new_instance_id(),area:new_area(),material:"material".into(),mu:1.,reflectance:0.08,color:[40,40,40],height_m:0.,roughness_m:0.});}
  let mut action=None;let mut remove=None;let len=e.regions.len();
  for(i,r)in e.regions.iter_mut().enumerate(){ui.push_id(&r.id,|ui|{egui::CollapsingHeader::new(format!("Região {}: {}",i+1,r.material)).show(ui,|ui|{ui.label(&r.id);rect_edit(ui,&mut r.area,grid);ui.text_edit_singleline(&mut r.material);ui.add(egui::DragValue::new(&mut r.mu).prefix("Atrito ").speed(0.01));ui.add(egui::DragValue::new(&mut r.reflectance).prefix("Refletância ").speed(0.01));ui.color_edit_button_srgb(&mut r.color);mm(ui,"Altura (metadado)",&mut r.height_m);mm(ui,"Rugosidade (metadado)",&mut r.roughness_m);reorder_buttons(ui,i,len,&mut action,&mut remove);});});}apply_order(&mut e.regions,action,remove);
  if ui.button("Adicionar marca/falha").clicked(){e.marks.push(OpticalMark{id:crate::io::assets::new_instance_id(),area:new_area(),kind:MarkKind::Paint,reflectance:0.86,color:[220,220,220]});}
  let mut action=None;let mut remove=None;let len=e.marks.len();for(i,m)in e.marks.iter_mut().enumerate(){ui.push_id(&m.id,|ui|{egui::CollapsingHeader::new(format!("Marca {}: {:?}",i+1,m.kind)).show(ui,|ui|{ui.label(&m.id);ui.radio_value(&mut m.kind,MarkKind::Paint,"Pintura");ui.radio_value(&mut m.kind,MarkKind::Gap,"Falha: expor substrato");rect_edit(ui,&mut m.area,grid);if m.kind==MarkKind::Paint{ui.add(egui::DragValue::new(&mut m.reflectance).prefix("Refletância ").speed(0.01));ui.color_edit_button_srgb(&mut m.color);}reorder_buttons(ui,i,len,&mut action,&mut remove);});});}apply_order(&mut e.marks,action,remove);
  ui.checkbox(&mut e.race_enabled,"Eventos de corrida e parada na chegada");ui.checkbox(&mut e.stop_on_exit,"Parar ao sair da mesa (contorno do robô)");ui.add(egui::DragValue::new(&mut e.laps).prefix("Voltas alvo ").clamp_range(1..=100000));
  ui.label("Sem portais próprios: largada/chegada automáticas. Ao criar uma lista própria, inclua um Start e um Finish; checkpoints seguem a ordem da lista. Direção é a seta do portal.");
  if ui.button("Adicionar portal").clicked(){e.gates.push(RaceGate{id:crate::io::assets::new_instance_id(),kind:GateKind::Checkpoint,center_m:Vec2::new(0.5,0.5),heading_deg:0.,half_width_m:0.2});}
  let mut action=None;let mut remove=None;let len=e.gates.len();for(i,g)in e.gates.iter_mut().enumerate(){ui.push_id(&g.id,|ui|{egui::CollapsingHeader::new(format!("Portal {}: {:?}",i+1,g.kind)).show(ui,|ui|{ui.label(&g.id);for k in [GateKind::Start,GateKind::Checkpoint,GateKind::Finish]{ui.radio_value(&mut g.kind,k,format!("{k:?}"));}mm(ui,"Centro X",&mut g.center_m.x);mm(ui,"Centro Y",&mut g.center_m.y);mm(ui,"Meia largura",&mut g.half_width_m);ui.add(egui::DragValue::new(&mut g.heading_deg).suffix(" graus"));if ui.button("Ajustar portal à grade").clicked()&&grid>0.{g.center_m.x=(g.center_m.x/grid).round()*grid;g.center_m.y=(g.center_m.y/grid).round()*grid;}reorder_buttons(ui,i,len,&mut action,&mut remove);});});}apply_order(&mut e.gates,action,remove);
  ui.checkbox(&mut e.relief_enabled,"Solicitar relevo vertical (solver atual recusa)");ui.label("Alturas/rugosidade são guardadas; nenhuma força vertical é calculada.");
  if let Some(p)=&mut track.parametric{ui.label("Fonte do regulamento (URL ou referência)");ui.text_edit_singleline(&mut p.rules.source);ui.label("Edição/data");ui.text_edit_singleline(&mut p.rules.edition);ui.label("Preencher a fonte não certifica conformidade oficial.");
   ui.label("Reordenar segmentos preserva IDs. Referências de largada continuam apontando ao mesmo segmento.");let mut action=None;let mut remove=None;let len=p.segments.len();for(i,seg)in p.segments.iter().enumerate(){ui.push_id(("order",seg.id()),|ui|{ui.horizontal(|ui|{ui.label(seg.id());reorder_buttons(ui,i,len,&mut action,&mut remove);});});}if let Some(i)=remove{if p.segments[i].id()==p.markings.start_finish.segment_id {ui.colored_label(egui::Color32::RED,"Mude a referência de largada antes de remover esse segmento.");}else{p.segments.remove(i);}}else if let Some((a,b))=action{let _=move_item(&mut p.segments,a,b);}
   if ui.button("Ajustar origem à grade").clicked()&&grid>0.{p.origin.x_mm=(p.origin.x_mm/(grid*1000.)).round()*grid*1000.;p.origin.y_mm=(p.origin.y_mm/(grid*1000.)).round()*grid*1000.;}
  }
  match validate_definition(track){Ok(warnings)=>{for w in warnings{ui.colored_label(egui::Color32::YELLOW,w);}},Err(e)=>{ui.colored_label(egui::Color32::RED,e);}}
 });
    before != track_json(track) || previous_pose != *project_pose
}
fn fill_polygon(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    points: &[Vec2],
    color: [u8; 3],
) {
    if points.len() >= 3 {
        painter.add(egui::Shape::convex_polygon(
            points
                .iter()
                .map(|p| world_to_screen(rect, bounds, *p))
                .collect(),
            egui::Color32::from_rgb(color[0], color[1], color[2]),
            egui::Stroke::NONE,
        ));
    }
}
fn within_table(runtime: &TrackRuntime, points: &[Vec2]) -> Vec<Vec2> {
    if let Some(p) = &runtime.definition().parametric {
        clip_polygon(
            points,
            &[
                Vec2::new(0., 0.),
                Vec2::new(p.area.width_mm / 1000., 0.),
                Vec2::new(p.area.width_mm / 1000., p.area.height_mm / 1000.),
                Vec2::new(0., p.area.height_mm / 1000.),
            ],
        )
    } else {
        points.to_vec()
    }
}
pub(super) fn draw_surface_regions(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    runtime: &TrackRuntime,
) {
    for r in &runtime.definition().environment.regions {
        fill_polygon(
            painter,
            rect,
            bounds,
            &within_table(runtime, &r.area.corners()),
            r.color,
        );
    }
}
pub(super) fn draw_optical_marks(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    runtime: &TrackRuntime,
) {
    for m in runtime.marks() {
        let polygon = within_table(runtime, &m.area.corners());
        if m.kind == MarkKind::Paint {
            fill_polygon(painter, rect, bounds, &polygon, m.color);
        } else {
            fill_polygon(
                painter,
                rect,
                bounds,
                &polygon,
                runtime
                    .definition()
                    .parametric
                    .as_ref()
                    .map(|p| display_color(&p.surface.base_color))
                    .unwrap_or([8, 8, 8]),
            );
            for r in &runtime.definition().environment.regions {
                fill_polygon(
                    painter,
                    rect,
                    bounds,
                    &clip_polygon(&polygon, &r.area.corners()),
                    r.color,
                );
            }
        }
    }
    if runtime.definition().environment.race_enabled {
        for g in runtime.gates() {
            let h = g.heading_deg.to_radians();
            let n = Vec2::new(h.cos(), h.sin());
            let side = Vec2::new(-n.y, n.x) * g.half_width_m;
            let center = world_to_screen(rect, bounds, g.center_m);
            painter.line_segment(
                [
                    world_to_screen(rect, bounds, g.center_m - side),
                    world_to_screen(rect, bounds, g.center_m + side),
                ],
                egui::Stroke::new(1., egui::Color32::YELLOW),
            );
            painter.arrow(
                center,
                world_to_screen(rect, bounds, g.center_m + n * 0.06) - center,
                egui::Stroke::new(1., egui::Color32::YELLOW),
            );
        }
    }
}

pub(super) fn draw_track_substrate(
    painter: &egui::Painter,
    rect: egui::Rect,
    bounds: Bounds,
    track: &TrackConfig,
) {
    if let Some(p) = &track.parametric {
        fill_polygon(
            painter,
            rect,
            bounds,
            &[
                Vec2::new(0., 0.),
                Vec2::new(p.area.width_mm / 1000., 0.),
                Vec2::new(p.area.width_mm / 1000., p.area.height_mm / 1000.),
                Vec2::new(0., p.area.height_mm / 1000.),
            ],
            display_color(&p.surface.base_color),
        );
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn region_and_mark_editor_renders_without_window_and_zoom_reuses_cache() {
        let ctx = egui::Context::default();
        let mut cfg = default_loaded_config(PathBuf::from("target/track-ui.rtsim"));
        let mut r = SurfaceRegion {
            id: "test-region".into(),
            area: new_area(),
            material: "rubber".into(),
            mu: 0.4,
            reflectance: 0.3,
            color: [30, 70, 60],
            height_m: 0.,
            roughness_m: 0.,
        };
        r.area.angle_deg = 30.;
        cfg.track.environment.regions.push(r);
        cfg.track.environment.marks.push(OpticalMark {
            id: "test-gap".into(),
            area: new_area(),
            kind: MarkKind::Gap,
            reflectance: 0.,
            color: [0, 0, 0],
        });
        let cached = cached_track(&ctx, &cfg.track).unwrap();
        for zoom in [0.5, 1., 3.] {
            let mut input = egui::RawInput::default();
            input.screen_rect = Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1200., 900.),
            ));
            let out = ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    edit_track_environment(ui, &mut cfg.track, &mut cfg.project.start_pose);
                    let mut pan = Vec2::default();
                    let mut z = zoom;
                    draw_track_view_with_height_zoomable(
                        ui,
                        &cfg.track,
                        None,
                        &[],
                        300.,
                        &mut z,
                        &mut pan,
                        None,
                    );
                });
            });
            assert!(!out.shapes.is_empty());
            assert!(cached.shares_geometry_with(&cached_track(&ctx, &cfg.track).unwrap()));
        }
    }
}
