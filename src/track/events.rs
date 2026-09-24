use super::{definition::*, runtime::TrackRuntime};
use crate::math::Vec2;
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RaceEventKind {
    Started,
    Checkpoint,
    Lap,
    Finished,
    WrongSequence,
    ReverseCrossing,
    ExitedArea,
    EnteredArea,
}
#[derive(Debug, Clone, PartialEq)]
pub struct RaceEvent {
    pub t_us: u64,
    pub kind: RaceEventKind,
    pub gate_id: Option<String>,
    pub lap: u32,
}
#[derive(Debug, Clone)]
pub struct RaceState {
    enabled: bool,
    stop_on_exit: bool,
    target_laps: u32,
    gates: Vec<RaceGate>,
    checkpoints: Vec<String>,
    next: usize,
    started: bool,
    lap_armed: bool,
    invalid_lap: bool,
    lap: u32,
    finished: bool,
    inside: bool,
    exited: bool,
    events: Vec<RaceEvent>,
}
impl RaceState {
    pub fn new(track: &TrackRuntime, inside: bool) -> Self {
        let e = &track.definition().environment;
        let gates = track.gates().to_vec();
        let checkpoints = gates
            .iter()
            .filter(|g| g.kind == GateKind::Checkpoint)
            .map(|g| g.id.clone())
            .collect();
        let mut s = Self {
            enabled: e.race_enabled,
            stop_on_exit: e.stop_on_exit,
            target_laps: e.laps,
            gates,
            checkpoints,
            next: 0,
            started: false,
            lap_armed: false,
            invalid_lap: false,
            lap: 0,
            finished: false,
            inside,
            exited: !inside,
            events: vec![],
        };
        if !inside {
            s.emit(0, RaceEventKind::ExitedArea, None);
        }
        s
    }
    fn emit(&mut self, t_us: u64, kind: RaceEventKind, id: Option<String>) {
        self.events.push(RaceEvent {
            t_us,
            kind,
            gate_id: id,
            lap: self.lap,
        });
    }
    pub fn events(&self) -> &[RaceEvent] {
        &self.events
    }
    pub fn laps(&self) -> u32 {
        self.lap
    }
    pub fn started(&self) -> bool {
        self.started
    }
    pub fn termination(&self) -> Option<&'static str> {
        if self.stop_on_exit && self.exited {
            Some("area_exit")
        } else if self.enabled && self.finished {
            Some("race_finished")
        } else {
            None
        }
    }
    pub fn update(&mut self, from: Vec2, to: Vec2, inside: bool, t_us: u64) {
        if inside != self.inside {
            self.inside = inside;
            if !inside {
                self.exited = true;
            }
            self.emit(
                t_us,
                if inside {
                    RaceEventKind::EnteredArea
                } else {
                    RaceEventKind::ExitedArea
                },
                None,
            );
        }
        if !self.enabled || self.finished {
            return;
        }
        let mut hits = Vec::new();
        for (i, g) in self.gates.iter().enumerate() {
            let h = g.heading_deg.to_radians();
            let normal = Vec2::new(h.cos(), h.sin());
            let a = (from - g.center_m).dot(normal);
            let b = (to - g.center_m).dot(normal);
            let forward = a < 0. && b >= 0.;
            let reverse = a > 0. && b <= 0.;
            if !(forward || reverse) {
                continue;
            }
            let fraction = a / (a - b);
            let point = from + (to - from) * fraction;
            let lateral = (point - g.center_m).dot(Vec2::new(-normal.y, normal.x));
            if lateral.abs() <= g.half_width_m {
                hits.push((fraction, i, forward));
            }
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (_, i, forward) in hits {
            let g = self.gates[i].clone();
            if !forward {
                self.emit(t_us, RaceEventKind::ReverseCrossing, Some(g.id));
                continue;
            }
            match g.kind {
                GateKind::Start if !self.lap_armed => {
                    self.started = true;
                    self.lap_armed = true;
                    self.next = 0;
                    self.invalid_lap = false;
                    self.emit(t_us, RaceEventKind::Started, Some(g.id));
                }
                GateKind::Checkpoint if self.started && self.lap_armed => {
                    if self.checkpoints.get(self.next) == Some(&g.id) {
                        self.next += 1;
                        self.emit(t_us, RaceEventKind::Checkpoint, Some(g.id));
                    } else {
                        self.invalid_lap = true;
                        self.emit(t_us, RaceEventKind::WrongSequence, Some(g.id));
                    }
                }
                GateKind::Finish if self.started && self.lap_armed => {
                    if self.next == self.checkpoints.len() && !self.invalid_lap {
                        self.lap += 1;
                        self.emit(t_us, RaceEventKind::Lap, Some(g.id.clone()));
                        if self.lap >= self.target_laps {
                            self.finished = true;
                            self.emit(t_us, RaceEventKind::Finished, Some(g.id));
                            break;
                        }
                    } else {
                        self.emit(t_us, RaceEventKind::WrongSequence, Some(g.id));
                    }
                    self.next = 0;
                    self.invalid_lap = false;
                    self.lap_armed = false;
                }
                _ => {}
            }
        }
    }
    pub fn to_json(&self, termination: &str) -> String {
        use crate::io::persistence::escape_json as esc;
        let events = self
            .events
            .iter()
            .map(|e| {
                format!(
                    r#"{{"t_us":{},"kind":"{:?}","gate_id":{},"lap":{}}}"#,
                    e.t_us,
                    e.kind,
                    e.gate_id
                        .as_ref()
                        .map(|v| format!("\"{}\"", esc(v)))
                        .unwrap_or("null".into()),
                    e.lap
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        format!(
            r#"{{"schema":"rtsim-race-events-v1","termination":"{}","laps":{},"events":[{}]}}"#,
            esc(termination),
            self.lap,
            events
        )
    }
}
