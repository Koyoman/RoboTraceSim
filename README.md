# Robotrace Sim v0.4 — Interface única

Simulador de robôs seguidores de linha em Rust com núcleo físico determinístico fixed-step, execução por terminal e primeira interface gráfica integrada em `egui/eframe`.

A v0.4 preserva os recursos das versões anteriores e adiciona uma aplicação única com:

- Home
- Editor de pista
- Editor de robô
- Simulador visual
- Replay viewer

A UI não substitui o núcleo de simulação: ela apenas configura, visualiza e aciona comandos. O core continua podendo rodar de forma automatizada por linha de comando.

## Compilar

```bash
cargo build --release
```

A interface gráfica é a feature padrão. Para compilar apenas o núcleo CLI, sem baixar `eframe`:

```bash
cargo build --release --no-default-features
```

## Abrir a interface gráfica

```bash
cargo run --release
```

ou:

```bash
cargo run --release -- ui
```

## Rodar simulação headless

```bash
cargo run --release -- run examples/basic/projeto.rtsim --headless --duration 10s
```

Com CSV e replay explícitos:

```bash
cargo run --release -- run examples/basic/projeto.rtsim --headless --duration 10s --csv examples/basic/resultado.csv --replay examples/basic/resultado.rtlog
```

## Benchmark

```bash
cargo run --release -- benchmark examples/basic/projeto.rtsim --duration 10s --physics-dt-us 500
```

## Exportar replay para CSV

```bash
cargo run --release -- export examples/basic/resultado.rtlog --format csv --output examples/basic/resultado_exportado.csv
```

## Recursos já implementados

### Base v0.1

- Projeto `.rtsim` com JSON versionado.
- Leitura de `robot.json` e `track.json`.
- Simulação fixed-step com tempo interno em microssegundos.
- Scheduler para física, sensores, controlador, IMU, encoder e log.
- Pista vetorial simples (`VectorTrack`).
- Consulta de refletância e atrito da pista.
- Sensor de linha com array de N sensores.
- Motor DC simples.
- Modelo diferencial 2D.
- Controlador PID built-in.
- Log CSV.
- Execução por terminal.
- Benchmark.

### Realismo básico v0.2

- `SlipRatioWheel`.
- `VoltageSagBattery`.
- `PwmHBridge` com PWM quantizado, queda de tensão, limite de corrente e brake/coast.
- `QuantizedEncoder`.
- `NoisyGyro`.
- `NoisyAdcSensor`.
- Replay binário `.rtlog` com exportação CSV.

### Downforce/sucção v0.3

- `NormalForceModel` modular.
- `FanDownforce` com múltiplos fans, posição no chassi e curvas PWM → força.
- `SuctionDownforce` com área de câmara, pressão diferencial, vazamento e resposta dinâmica.
- Distribuição de normal nas quatro rodas.
- Efeito da normal no atrito por `Fmax = μ * N`.
- PWM de fan/sucção via controlador.
- Consumo elétrico do sistema de downforce somado à bateria.
- Replay binário v3 (`RTSRPL03`) com campos de normal/downforce.

### Interface única v0.4

- `src/ui.rs` com app `egui/eframe`.
- Home para carregar, criar e salvar projetos.
- Editor de pista com canvas vetorial e tabela de pontos.
- Editor de robô com parâmetros físicos, eletrônicos, sensores, controle e downforce.
- `SimulationSession` incremental para visualização sem duplicar a física.
- Simulador visual com play/pause/step e painel de telemetria.
- Replay viewer com carregamento `.rtlog`, slider temporal, trajetória e exportação CSV.

## Formato do projeto `.rtsim`

```json
{
  "rtsim_schema": "rtsim-project-v1",
  "name": "basic-v0.4-demo",
  "robot": "robot.json",
  "track": "track.json",
  "time": {
    "physics_dt_us": 500,
    "controller_period_us": 1000,
    "sensor_period_us": 500,
    "imu_period_us": 500,
    "encoder_period_us": 500,
    "log_period_us": 1000,
    "render_period_us": 16667
  },
  "simulation": {
    "duration_s": 10.0,
    "start_pose_m": [0.0, 0.035, 0.0]
  },
  "log": {
    "csv": "resultado.csv",
    "replay": "resultado.rtlog"
  }
}
```

## Observações técnicas

- A física continua determinística e desacoplada da UI.
- A UI usa `egui::Painter` para renderização inicial, conforme a especificação.
- O parser JSON próprio foi mantido para preservar a base sem `serde`, mas a serialização manual da UI já salva os arquivos principais.
- O comando `batch` ainda permanece como próximo passo.

## Limitação conhecida deste pacote

O ambiente usado para montar esta versão não possui `cargo`/`rustc` instalado, então não foi possível executar `cargo check` ou `cargo test` aqui. A revisão foi estática e os JSONs de exemplo foram validados com `python -m json.tool`.
