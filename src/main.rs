fn main() {
    let args: Vec<String> = std::env::args().collect();
    let result = if args.len() < 2 {
        robotrace_sim::run_app()
    } else {
        match args[1].as_str() {
            "ui" | "gui" | "app" => robotrace_sim::run_app(),
            _ => robotrace_sim::run_cli(args),
        }
    };

    if let Err(err) = result {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}
