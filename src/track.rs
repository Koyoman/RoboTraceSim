use crate::config::TrackConfig;
use crate::math::{distance_point_segment, Vec2};

pub trait TrackModel {
    fn reflectance_at(&self, world_point: Vec2) -> f64;
    fn surface_mu_at(&self, world_point: Vec2) -> f64;
    fn distance_to_line_m(&self, world_point: Vec2) -> f64;
    fn base_reflectance(&self) -> f64;
    fn line_reflectance(&self) -> f64;
}

#[derive(Debug, Clone)]
pub struct VectorTrack {
    cfg: TrackConfig,
}

impl VectorTrack {
    pub fn new(cfg: TrackConfig) -> Self {
        Self { cfg }
    }
}

impl TrackModel for VectorTrack {
    fn reflectance_at(&self, world_point: Vec2) -> f64 {
        if self.distance_to_line_m(world_point) <= self.cfg.line_width_m * 0.5 {
            self.cfg.line_reflectance
        } else {
            self.cfg.base_reflectance
        }
    }

    fn surface_mu_at(&self, _world_point: Vec2) -> f64 {
        self.cfg.surface_mu
    }

    fn distance_to_line_m(&self, world_point: Vec2) -> f64 {
        self.cfg
            .centerline
            .windows(2)
            .map(|w| distance_point_segment(world_point, w[0], w[1]))
            .fold(f64::INFINITY, f64::min)
    }

    fn base_reflectance(&self) -> f64 {
        self.cfg.base_reflectance
    }

    fn line_reflectance(&self) -> f64 {
        self.cfg.line_reflectance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_line_reflectance_switches_on_width() {
        let track = VectorTrack::new(TrackConfig {
            schema: "rtsim-track-v1".to_string(),
            name: "test".to_string(),
            model: "VectorTrack".to_string(),
            line_width_m: 0.02,
            base_reflectance: 0.9,
            line_reflectance: 0.1,
            surface_mu: 1.0,
            centerline: vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
        });
        assert!((track.reflectance_at(Vec2::new(0.5, 0.0)) - 0.1).abs() < 1e-12);
        assert!((track.reflectance_at(Vec2::new(0.5, 0.02)) - 0.9).abs() < 1e-12);
    }
}
