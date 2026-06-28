mod cli;
mod config;
mod controller;
mod json;
mod math;
mod motor;
mod sensor;
mod sim;
mod telemetry;
mod track;
mod wheel;

fn main() {
    if let Err(err) = cli::run_cli(std::env::args().collect()) {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}
