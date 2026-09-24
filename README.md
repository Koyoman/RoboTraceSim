# RoboTraceSim 0.6.0

Simulador de robôs seguidores de linha em Rust com núcleo físico determinístico fixed-step, execução por terminal, interface gráfica `egui/eframe` e ferramentas de comparação entre simulação e robô real.

O aplicativo inclui:

- Importação/normalização de log real em CSV.
- Comparação simulação vs robô real.
- Erro de trajetória.
- Erro de sensores.
- Erro de velocidade.
- Relatório de métricas.
- Ajuste grosso de parâmetros por busca determinística.
- Tela de calibração integrada à interface única.

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

## Importar log real

O importador aceita CSV com `t_us`, `time_us`, `t_s`, `time_s`, `t_ms` ou `time_ms`, o tempo é normalizado para iniciar em 0 µs, além de colunas opcionais como `x_m`, `y_m`, `yaw_rad`, `vx_body_m_s`, `speed_m_s`, `line_position_m`, `line_error_m` e `sensor_00_adc` até `sensor_NN_adc`.

```bash
cargo run --release -- import-log examples/basic/real_log_demo.csv --output examples/basic/real_log_normalizado.csv
```

## Comparar simulação vs robô real

```bash
cargo run --release -- compare examples/basic/projeto.rtsim --real examples/basic/real_log_demo.csv --output examples/basic/comparacao_v05.csv --report examples/basic/comparacao_v05.txt
```

A comparação alinha os dados pelo tempo e calcula:

- RMS, média absoluta e máximo do erro de trajetória.
- RMS, média absoluta e máximo do erro de yaw.
- RMS, média absoluta e máximo do erro de velocidade.
- RMS, média absoluta e máximo do erro de sensores ADC.
- RMS, média absoluta e máximo do erro de linha.
- Score normalizado para calibração.

## Ajustar parâmetros

```bash
cargo run --release -- tune examples/basic/projeto.rtsim --real examples/basic/real_log_demo.csv --output examples/basic/ajuste_v05.json
```

O ajuste da v0.5 faz uma busca grossa determinística sobre:

- `tire.mu_longitudinal`
- escala de torque de stall dos motores esquerdo/direito
- escala de corrente de stall correspondente

Ele não sobrescreve automaticamente o `robot.json`; em vez disso, gera um JSON com os melhores valores e métricas para revisão.

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

### Comparação com dados reais v0.5

- `src/calibration.rs` com importação de CSV real, alinhamento temporal e métricas.
- Comando `import-log` para normalizar logs reais.
- Comando `compare` para executar a simulação e comparar contra o log real.
- Comando `tune`/`calibrate` para ajuste grosso de parâmetros.
- Tela `Calibração v0.5` dentro da interface única.
- Arquivo de exemplo `examples/basic/real_log_demo.csv`.
- Relatórios em CSV, TXT e JSON.

## Formato do projeto `.rtsim`

```json
{
  "rtsim_schema": "rtsim-project-v1",
  "name": "basic-v0.5-demo",
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
- O parser JSON próprio valida UTF-8, escapes Unicode, números finitos e sintaxe estrita.
- O ajuste de parâmetros da v0.5 é propositalmente simples e revisável; modelos mais avançados podem adicionar otimização multiobjetivo, bounds configuráveis e exportação direta do `robot.json`.
- O comando `batch` ainda permanece como próximo passo.

## Configuração e sensores

O schema atual de robô é `rtsim-robot-v8`. Cada sensor tem ID, pose, resposta, ADC, ruído e seed próprios. As leituras são geradas separadamente pela área e pose mundial de cada sensor, com aquisição, filtro, ADC e entrega individuais. Não existe compromisso de compatibilidade com APIs, formatos ou resultados antigos.

Projetos e assets usam caminhos relativos ao arquivo proprietário. Sensores são salvos com assets incorporados; a API `io::experiment::save_project_bundle` produz uma pasta portável. Cada run com logs também grava `.experiment.json` com sua configuração congelada.

Consulte [configuração e sensores](docs/etapa-3-configuracao-e-sensores.md), [contrato temporal](docs/etapa-1-executor-e-tempo.md), [dinâmica física](docs/etapa-2-dinamica-fisica.md) e [tarefas](tasks.md). As versões dos schemas são independentes da versão do aplicativo.

Verificação local: `cargo test --offline --no-default-features`, `cargo test --offline` e `cargo fmt --all -- --check`. A compilação da GUI não equivale a teste interativo.

### Montagem física — etapa 4

O editor usa instâncias de rodas/apoios e sensores, massa medida ou calculada por componentes, COM XYZ e inércia. Há seleção, movimento, rotação, duplicação, alinhamento, grade e desfazer/refazer. O solver atual aceita a montagem simétrica de quatro rodas; configurações incompatíveis são recusadas ao iniciar. Veja [montagem, editor e limites](docs/etapa-4-montagem-e-editor.md).

### Pista física e corrida — etapa 5

A pista agora contém regiões de material/atrito, marcas ópticas e falhas, portais de largada/checkpoint/chegada e escolha explícita da pose inicial. O editor oferece reordenação, grade e desfazer/refazer. Sensores consultam geometria analítica; rodas consultam suas superfícies locais. Eventos são gravados em `.events.json` junto aos logs. Relevo permanece como metadado e contato independente será entregue na etapa 6.

Veja [contratos, limites e testes](docs/etapa-5-pista-optica-e-corrida.md) e o [projeto de ensaio](examples/stage5/projeto.rtsim).

### Fidelidade e contato por roda — etapa 6

O editor permite escolher Ideal, Simplificado ou Realista reduzido e ajustar os subsistemas. Os novos modelos incluem forças individuais, aderência combinada, transferência quase estática de carga, rodas passivas, caster reduzido e refinamentos opcionais do pneu. Registros por roda acompanham CSV/replay em `.contacts.csv`.

Projetos de exemplo: [Ideal](examples/physics/ideal.rtsim), [Simplificado](examples/physics/simplified.rtsim) e [Realista](examples/physics/realistic.rtsim). Consulte [equações, limitações, testes e desempenho](docs/etapa-6-fidelidade-e-contatos.md). O preset Realista exige calibração; a etapa 9 otimiza o contato e registra novas medições a 50 µs.

```powershell
cargo run --release --offline --no-default-features -- run examples/physics/simplified.rtsim --headless --csv target/stage6.csv --replay target/stage6.rtlog
cargo run --release --offline --no-default-features -- benchmark examples/physics/realistic.rtsim --duration 100ms
```

### Motores, alimentação e downforce — etapa 7

O bloco opcional `powertrain` habilita motores DC simples/elétricos, transmissão explícita, driver médio, bateria com curvas e RC, regeneração/proteções e fan/sucção acoplados à tensão. O editor permite ajustar parâmetros e a simulação registra `.power.csv` e `.power.events.json` junto aos resultados.

Veja o [exemplo executável](examples/power/projeto.rtsim) e os [contratos, limites e testes](docs/etapa-7-motores-alimentacao-e-downforce.md). O modelo exige uma roda motriz por motor e ainda custa mais que tempo real no ensaio a 50 µs. A etapa 9 entrega cálculo/reprodução separados e documenta o custo restante.

### Sensoriamento e controle — etapa 8

O painel “Sensores e controle” configura aquisição individual/multiplexada, latência, filtros, encoder e IMU. O controlador recebe somente leituras entregues e pode usar controle de velocidade, recuperação da linha, odometria e perfil aprendido. Também há repetição de comandos e API de firmware em processo, sem carregar DLLs.

O [exemplo de aquisição sequencial](examples/sensing/projeto.rtsim) usa física a 50 µs e controle a 1 ms. Registros `.sensors.jsonl` e `.commands.csv` acompanham os resultados. Veja os [contratos, aproximações e testes](docs/etapa-8-sensoriamento-e-controle.md). Validação: 135 testes sem GUI e 140 com GUI.

### Cálculo, replay e batch — etapa 9

A simulação roda em worker com pausa, retomada, passo e cancelamento. **Calcular e reproduzir** abre o resultado em um player por tempo, com velocidade e busca. Replay v4 registra a configuração congelada, canais, integridade e motivo de término; a leitura usa cache limitado. Importação, exportação, comparação e ajuste executam em segundo plano.

```powershell
cargo run --release --offline --no-default-features -- batch examples/batch/sweep.json --out target/sweep-001 --jobs 2
```

Veja [contratos, checkpoint e limites](docs/etapa-9-calculo-replay-e-experimentos.md) e [benchmarks reproduzíveis](docs/etapa-9-benchmarks.md). O checkpoint atual retoma somente em memória; não há persistência de estado físico em disco. A validação experimental dos modelos permanece na etapa 10.

### Calibração e qualificação — etapa 10

Há estudos versionados com dados de calibração/validação separados, objetivos e bounds configuráveis, sincronização explícita, métricas com cobertura e ensaios numéricos a 100/50/25 µs. A interface executa estudos e robustez em segundo plano. A CI verifica núcleo, GUI, formatos e regressões de software.

```powershell
cargo run --release --offline --no-default-features --example stage10_fixture -- target/stage10-fixture
cargo run --release --offline --no-default-features -- qualify target/stage10-fixture/study.json target/stage10-qualification.json
python scripts/check-qualification.py target/stage10-qualification.json
```

A fixture é **sintética**. Não há medições físicas qualificadas no repositório; `real_log_demo.csv` tem proveniência desconhecida. O Realista ainda não é validado experimentalmente. Veja [entregas e limites](docs/etapa-10-calibracao-e-qualificacao.md), [protocolo de bancada](docs/etapa-10-protocolo-experimental.md) e [pendências](tasks.md).
