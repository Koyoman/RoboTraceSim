#[cfg(test)]
use crate::config::TrackConfig;
use crate::math::Vec2;

pub trait TrackModel {
    fn reflectance_at(&self, world_point: Vec2) -> f64;
    fn surface_mu_at(&self, world_point: Vec2) -> f64;
    fn distance_to_line_m(&self, world_point: Vec2) -> f64;
    fn base_reflectance(&self) -> f64;
    fn line_reflectance(&self) -> f64;
}

pub mod definition;
pub mod events;
mod persistence;
pub mod runtime;
pub use runtime::{TrackRuntime, VectorTrack};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_line_reflectance_switches_on_width() {
        let track = VectorTrack::new(TrackConfig {
            environment: Default::default(),
            schema: "rtsim-track-v1".to_string(),
            name: "test".to_string(),
            model: "VectorTrack".to_string(),
            line_width_m: 0.02,
            base_reflectance: 0.9,
            line_reflectance: 0.1,
            surface_mu: 1.0,
            centerline: vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            parametric: None,
        });
        assert!((track.reflectance_at(Vec2::new(0.5, 0.0)) - 0.1).abs() < 1e-12);
        assert!((track.reflectance_at(Vec2::new(0.5, 0.02)) - 0.9).abs() < 1e-12);
    }
}

mod spatial;
