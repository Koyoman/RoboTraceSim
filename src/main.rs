mod battery;
mod cli;
mod config;
mod controller;
mod encoder;
mod gyro;
mod json;
mod math;
mod motor;
mod normal_force;
mod replay;
mod rng;
mod sensor;
mod sim;
mod telemetry;
mod track;
mod ui;
mod wheel;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let result = if args.len() < 2 {
        ui::run_app()
    } else {
        match args[1].as_str() {
            "ui" | "gui" | "app" => ui::run_app(),
            _ => cli::run_cli(args),
        }
    };

    if let Err(err) = result {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}
