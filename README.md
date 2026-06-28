# Robotrace Sim v0.1 — Headless

Primeira implementação do núcleo headless do simulador de seguidores de linha.

Esta versão ainda não possui interface gráfica. O objetivo é validar o formato `.rtsim`, a leitura de `robot.json` e `track.json`, a simulação determinística fixed-step e a execução por terminal.

## O que está implementado

- Projeto `.rtsim` com JSON versionado.
- Leitura de `robot.json`.
- Leitura de `track.json`.
- Simulação fixed-step com tempo interno em microssegundos.
- Scheduler simples para física, sensores, controlador e log.
- Modelo de pista vetorial simples por polilinha (`VectorTrack`).
- Consulta de refletância da pista para sensores.
- Consulta de atrito de superfície.
- Sensor de linha simples com array de N sensores (`SensorArray16`).
- Motor DC simples com curva linear torque x velocidade (`DcMotorSimple`).
- Roda com limite por atrito de Coulomb (`CoulombFrictionWheel`).
- Modelo de normal `NoDownforce` (`N = m*g`, distribuído entre lados esquerdo/direito).
- Controlador PID built-in.
- Log CSV.
- Execução por terminal.
- Benchmark por terminal.

## O que ficou como próximo passo

- UI com egui/eframe.
- Replay binário `.rrlog`.
- Comando `batch` completo.
- Comando `export` completo.
- Modelos avançados de motor, bateria, driver, sensor, slip ratio, fan/downforce e sucção.
- Colisão/saída de pista.
- Comparação com dados reais.

## Compilar

```bash
cargo build --release
```

## Rodar simulação headless

```bash
cargo run --release -- run examples/basic/projeto.rtsim --headless --duration 10s
```

Ou, depois de compilar:

```bash
./target/release/robotrace-sim run examples/basic/projeto.rtsim --headless --duration 10s
```

Por padrão, o exemplo escreve:

```text
examples/basic/resultado.csv
```

Para escolher outro CSV:

```bash
robotrace-sim run examples/basic/projeto.rtsim --headless --duration 10s --output resultado.csv
```

## Benchmark

```bash
cargo run --release -- benchmark examples/basic/projeto.rtsim --duration 10s --physics-dt-us 500
```

## Formato do projeto `.rtsim`

```json
{
  "rtsim_schema": "rtsim-project-v1",
  "name": "basic-headless-demo",
  "robot": "robot.json",
  "track": "track.json",
  "time": {
    "physics_dt_us": 500,
    "controller_period_us": 1000,
    "sensor_period_us": 500,
    "log_period_us": 1000,
    "render_period_us": 16667
  },
  "simulation": {
    "duration_s": 10.0,
    "start_pose_m": [0.0, 0.035, 0.0]
  },
  "log": {
    "csv": "resultado.csv"
  }
}
```

## Observações técnicas

A v0.1 usa apenas a biblioteca padrão do Rust. Isso mantém o projeto fácil de compilar em qualquer ambiente, sem dependências externas.

O parser JSON incluído é propositalmente mínimo, suficiente para os arquivos de configuração da v0.1. Em versões futuras, vale trocar por `serde`/`serde_json` quando o projeto passar a aceitar schemas maiores e validação mais detalhada.

A dinâmica 2D usa corpo rígido simplificado:

- `x`, `y`, `yaw` no mundo.
- `vx`, `vy`, `yaw_rate` no corpo.
- Força longitudinal por roda limitada por `Fmax = mu * N`.
- Força lateral simples para reduzir escorregamento lateral.
- Torque em yaw por diferença de força entre as rodas.

A intenção é manter a arquitetura modular desde o começo, mesmo usando modelos simples.
