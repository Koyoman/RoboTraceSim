//! Reusable simulation API. The core never depends on GUI types or wall-clock time.
mod battery;
pub mod calibration;
mod cli;
pub mod config;
pub mod controller;
pub mod core;
pub mod encoder;
pub mod gyro;
pub mod io;
pub mod json;
pub mod math;
pub mod models;
mod motor;
mod normal_force;
pub mod replay;
mod rng;
pub mod rtsim_track;
pub mod sensor;
pub mod sim;
pub mod telemetry;
pub mod track;
mod ui;
mod wheel;

pub use cli::run_cli;
pub use ui::run_app;

pub mod control;

pub mod experiments;
