# Etapa 8 — Sensoriamento e controle observável

Implementação verificada em 23/09/2026. Esta entrega mantém física e lógica com períodos separados e cada sensor como uma instância posicionada no robô. O modelo efetivo de aquisição é `individual-area-timed-v1`; o controlador é identificado como `observable-pid-v1`, `replay-v1` ou `native-api-v1`.

## Configuração e estruturas

O bloco `robot.sensing` acrescenta configuração de aquisição por ID em `optical`, extensões de `encoder` e `imu`, parâmetros de `control` e o texto opcional `replay_csv`. Ele é persistido integralmente no robô e no snapshot congelado do experimento. Campos desconhecidos são rejeitados. Os parâmetros básicos de ADC, ruído e seed continuam em cada instância de sensor; resolução básica do encoder e ruído/bias do gyro continuam nos seus blocos originais.

| Estrutura | Responsabilidade implementada |
|---|---|
| `src/models/sensing.rs` | Configuração, defaults, parsing, validação temporal e multiplexador |
| `src/sensor.rs` | Área e transformação por sensor, resposta, filtro, ADC, histerese e filas |
| `src/encoder.rs`, `src/gyro.rs` | Aquisição, quantização, ruído, filtro e entrega dos dispositivos |
| `src/control/mod.rs` | `SensorFrame`, `ControllerInput`, feedback e `TimedCommand` |
| `src/controller.rs` | PID de linha, anti-windup, recuperação e PI de velocidade |
| `src/control/estimator.rs` | Odometria, fusão de yaw, marcas e perfil aprendido |
| `src/control/replay_controller.rs` | Comandos gravados com retenção temporal |
| `src/control/native.rs` | Adaptador seguro de firmware e tipos para futura ABI C |
| `src/sim.rs` | Aquisição/entrega em cada tick, controle e seleção de adaptador |
| `src/io/`, `src/config.rs` | Persistência e resolução; recusa de modelos não executáveis |
| `src/app/robot_editor.rs`, `src/ui.rs` | Edição dos parâmetros e inspeção de leituras/estimativa |
| `src/telemetry.rs` | Diagnóstico de sensores e gravação de comandos |
| `tests/stage8_sensing.rs`, `examples/sensing/` | Ensaios e exemplo reproduzível |

O painel “Sensores e controle” configura cada dispositivo, períodos/fases, multiplexador, filtros, encoder/IMU e controle. O painel de simulação mostra ADC, validade, aquisição, disponibilidade, idade e odometria estimada. Edição, duplicação e remoção preservam ou removem corretamente as configurações associadas ao ID; desfazer/refazer continua usando snapshots completos.

## 8.1–8.2 — Pipeline óptico individual

Cada aquisição transforma pontos da área local pelo ângulo e posição do sensor e depois pela pose do robô. Integra a refletância do runtime da pista sobre essa área. O pipeline é:

1. Média espacial da refletância.
2. Resposta ideal, linear, polinomial, tabela ou limiar.
3. Ruído gaussiano por amostra e saturação em [0,1].
4. Filtro passa-baixa de primeira ordem.
5. Ruído do ADC em LSB, arredondamento e saturação na resolução configurada.
6. Comparador com histerese, quando o dispositivo é `LineDigital`.
7. Conversão/latência e entrega da leitura retida.

O filtro usa `y_n=y_anterior+(x_n−y_anterior)(1−exp(−Δt/τ))`, com o intervalo real entre aquisições daquele dispositivo. A primeira amostra inicializa o filtro; τ=0 o desliga.

`Point` é uma consulta infinitesimal; seu raio não aumenta a área física. Retângulos usam quadratura de pontos médios N×N. Círculos e setores usam distribuição polar de igual área; círculo com N=1 usa o centro. Polígonos usam a grade do retângulo envolvente, retendo pontos internos. Uma grade que não resolve o polígono é rejeitada, pedindo maior resolução. `area_samples` permite 1–64 divisões por eixo. O ângulo do sensor afeta a área; uma área circular não deve ganhar dependência física de orientação, mas erro discreto de quadratura pode variar com N.

A integração é numérica, não uma interseção analítica exata entre todas as áreas e marcas. Faixas menores que o espaçamento dos pontos exigem aumentar N e verificar convergência. `Cone` aqui é apenas um setor 2D no plano da pista, sem simulação de distância, lente ou propagação de luz. Altura continua metadado; iluminação, sombra, inclinação e dinâmica vertical não estão implementadas.

Sensores digitais retornam um booleano e ADC 0/máximo. O limiar vem da resposta `Threshold`, se selecionada, ou de `sensing.optical[id].threshold`. Liga em limiar+histerese/2 e desliga abaixo de limiar−histerese/2. Em sensor digital, `Threshold` alimenta o comparador com a intensidade, evitando perder a histerese por binarização antecipada. Em sensor analógico, `Threshold` continua uma resposta binária sem estado.

A posição da linha e as marcas usam os valores de ADC entregues, normalizados pelos extremos de calibração de fundo/linha após a resposta do dispositivo. Isso também permite resposta invertida. Esses extremos vêm da configuração estática da pista, como calibração conhecida; nenhuma consulta à posição real entra no controlador. Se os dois extremos resultam iguais, o canal não fornece contraste útil. Custom sem implementação, ToF, ultrassom e outros tipos não ópticos de linha habilitados são recusados na execução.

## 8.3 — Agenda, conversão e entrega

Por dispositivo:

| Parâmetro | Significado |
|---|---|
| `period_us` | Período próprio; 0 herda `project.time.sensor_period_us` |
| `phase_us` | Instante da primeira aquisição e fase dentro do período |
| `conversion_us` | Tempo de conversão após capturar a entrada |
| `latency_us` | Atraso adicional até disponibilizar a amostra |
| `mux_group` | 0: aquisição independente; outro número: conversor compartilhado |
| `filter_tau_s`, `area_samples` | Resposta temporal e resolução espacial |
| `threshold`, `hysteresis` | Comparador digital |

Aquisições ocorrem em `phase_us + k × period_us`. A pose usada é a do início da aquisição. Conversão é um atraso de resultado, sem integração temporal durante uma janela de exposição. Entrega acontece em `aquisição + conversão + latência`; o núcleo a verifica a cada passo físico, mesmo fora do tick do controlador ou do período padrão dos sensores.

Tempos precisam ser múltiplos do passo físico. A fase fica dentro do período. Em um grupo multiplexado, todos os canais habilitados têm o mesmo período, conversão positiva e intervalos de conversão sem sobreposição, inteiramente dentro do período. Essa é uma agenda estática de slots, sem arbitragem dinâmica de barramento. Grupos distintos podem adquirir simultaneamente. O exemplo usa fases 0, 50, 100 e 150 µs, conversão de 50 µs e latência de 100 µs.

Antes da primeira entrega, o canal é inválido. Depois, mantém a última leitura, com idade calculada desde a aquisição. O frame apresenta os canais habilitados na ordem do robô, identificados por ID; desabilitar um canal não renumera seus vizinhos conceitualmente, embora reduza o vetor enviado ao firmware. Não há expiração automática da leitura: validade significa que uma amostra foi entregue; idade permite ao firmware decidir se está velha.

A ordem no tick é física → aquisição/entrega óptica, encoder e IMU → controlador, se devido → observação. O comando novo atua no intervalo físico seguinte e respeita a latência de driver da etapa 7. No instante zero, só aquisições com fase zero ocorrem e somente entregas de atraso zero são visíveis. `EventCounts.sensor` conserva a contagem dos ticks globais do sensor; os timestamps por canal são a evidência da aquisição individual, inclusive quando usa outro período.

## 8.4–8.5 — Encoder, IMU e ruído

O encoder distingue o eixo observado por `shaft_ratio`: 1 mede a roda; uma redução g mede o eixo do motor rigidamente ligado à roda. A resolução efetiva por volta da roda é `ticks_per_rev × quadrature × shaft_ratio`. Quadratura admite 1, 2 ou 4. A velocidade entregue é normalizada para rad/s da roda; inversões de canal são preservadas e compensadas pela odometria conforme a configuração conhecida. Não há estado de rotor independente por elasticidade de transmissão.

A perda de pulsos é probabilística, por lado, sobre o deslocamento quantizado entre aquisições. Até 4.096 pulsos por lote usa ensaios Bernoulli; acima disso usa aproximação normal limitada da binomial para conter o custo. Não reconstrói bordas eletrônicas A/B nem captura inversões ocorridas inteiramente entre duas amostras. Contadores cumulativos permitem ao controlador mais lento observar todo deslocamento entregue sem recontar a mesma amostra. Velocidade tem filtro e o pacote tem latência própria.

A IMU acrescenta aceleração planar no referencial do corpo, filtro, latência, bias X/Y, ruído e saturação. A aceleração é estimada entre aquisições com termo de transporte do referencial girante. A posição de montagem é o centro do corpo; não há braço de alavanca, gravidade projetada por roll/pitch ou IMU 3D. O desalinhamento é yaw da montagem: gira os eixos do acelerômetro, mas não o eixo z do gyro.

O gyro mantém bias inicial e ruído por amostra, e ganha passeio aleatório de bias: incremento gaussiano com desvio `drift_std_rad_s_sqrt_s × sqrt(Δt)`. Esse parâmetro é densidade de passeio aleatório; os demais desvios de refletância, ADC, gyro e aceleração são por amostra, não densidades espectrais.

Há estados RNG separados por sensor, ruído óptico/ADC, lado do encoder, ruído/drift do gyro e eixo do acelerômetro. Desligar ou reordenar outro sensor não consome sua sequência. Seeds explícitas são persistidas; quando omitida na leitura de uma instância, a seed óptica deriva de hash estável do ID. IDs também precisam ser estáveis para isso; IDs ausentes são gerados a partir da posição no arquivo. Seeds iguais ainda podem produzir sequências correlacionadas — streams separados não substituem escolher seeds distintas quando se deseja independência estatística.

## 8.6–8.7 — Fronteira do controlador e controles internos

`ControllerInput` contém um `SensorFrame` com leituras entregues e feedback do driver: PWM aplicado e flags de corrente limitada. Não contém pose real, geometria da pista, força de contato, refletância ideal ou bias verdadeiro do gyro. `OpticalSample` inclui ID, ADC/digital, timestamps, idade e validade. IMU e encoder também expõem validade e tempo. A API de diagnóstico `sensor_readings()` e os registros de depuração são separados dessa entrada.

`TimedCommand` contém timestamp de emissão, dois PWMs, modos Drive/Brake/Coast e PWM de downforce. O circuito acoplado executa os modos explícitos. O modelo mecânico de referência conserva seu modo global de driver: o controlador interno emite Drive e o zero segue a configuração existente. Replay/firmware com Brake/Coast explícitos exigem `powertrain`, impedindo ignorar silenciosamente esses comandos.

O controlador interno inclui:

- PID de linha com limite da integral, integração condicional em saturação e filtro da derivada.
- Pausa da integração quando o driver informa limitação de corrente.
- PI independente de velocidade por roda quando `speed_mode=1`, com feedback do encoder entregue. `speed_mode=0` usa PWM base.
- Em perda de linha, zeragem das integrais e da derivada, comando de parada e, após `loss_timeout_us`, giro de busca com `recovery_pwm` na direção do último erro. Ao reencontrar a linha, reinicia a derivada sem salto da perda.
- Sem encoder válido, o modo de velocidade não comanda avanço. Sem linha válida/visível, aplica a política de perda.

O alvo de velocidade é m/s; no modo de velocidade a correção de linha altera os alvos das rodas em m/s. No modo PWM essa correção é adimensional, portanto os ganhos de linha precisam ser ajustados ao modo escolhido. Recuperação é uma política reduzida de busca, não garantia de reencontro para toda geometria. Bias, escorregamento e atraso continuam podendo prejudicar o controlador.

## 8.8 — Estimação e perfil de volta

A odometria começa em (0,0,0) relativo à inicialização, sem receber a pose inicial real. Usa diferenças dos contadores cumulativos, raios/bitola conhecidos e integração de posição pela orientação intermediária. Yaw combina incremento de encoder e taxa entregue do gyro com peso configurável. Amostras são retidas quando o controlador roda mais rápido; não há sincronização retroativa por atraso nem estimador Kalman. O usuário deve considerar os atrasos relativos ao calibrar o peso.

Marcas são detectadas por intensidade calibrada em número mínimo de canais, borda de entrada e período refratário. Uma marca que cobre todos os canais ativos estabelece a referência de volta; a próxima só incrementa a volta após distância mínima estimada. Não são usados os portais reais da pista nessa decisão. As regras oficiais de largada, checkpoints, direção e término continuam no árbitro da pista, separado do controlador. O detector reduzido exige que a pista/sensores diferenciem marcas largas da linha normal; cruzamentos podem ser ambíguos.

Entre a primeira marca larga e o encerramento da primeira volta, armazena até 4.096 pontos de distância/perfil, com espaçamento mínimo de 1 cm. Curvatura estimada acima de 2 rad/m aplica `curve_speed_factor`; outros trechos usam fator 1. Em voltas posteriores, consulta o perfil pela distância percorrida desde a marca e ajusta o alvo de velocidade. `profile_enabled=1` exige controle de velocidade. O mapa usa apenas medições; não incorpora a geometria da pista. Não há otimização de frenagem antecipada ou planejamento ótimo nesta etapa.

## 8.9 — Replay e firmware nativo

`ReplayController` carrega CSV com cabeçalho:

```text
t_us,pwm_left,pwm_right,downforce_pwm,mode_left,mode_right
0,0.1,0.1,0,drive,drive
1000,0.2,0.2,0,drive,drive
```

Exige registro inicial em zero, tempos estritamente crescentes na grade física, valores finitos e modos válidos. Retém o comando anterior, sem interpolar e sem ler comando futuro, inclusive ao consultar um instante anterior. No runtime a agenda de replay é verificada em cada tick físico; não fica restrita ao período do controlador. O texto completo em `sensing.replay_csv` é congelado no experimento; não há leitura de arquivo mutável durante o cálculo. O editor permite colar o conteúdo de `.commands.csv`.

Para firmware compilado junto ao host, `Firmware` define `reset`, `step(&ControllerInput) -> Result<TimedCommand, String>` e `stop`. `SimulationCore::install_firmware` instala antes de avançar o tempo; o adaptador inicializa uma vez, chama em zero e nos ticks de controle, valida timestamp/PWM e encerra ao ser descartado. Há limite de 256 canais. Falha de inicialização também chama `stop`; erro de execução interrompe o run. Esse código roda no processo do simulador e deve retornar sem depender de tempo de parede; não é um sandbox para firmware arbitrário.

A DLL continua opcional e **não é carregada**. Para integração futura, `native.rs` já declara layouts `repr(C)` de cabeçalho, amostra óptica, frame e comando. O contrato proposto de ABI é:

- Versão 1, execução nativa na mesma arquitetura do host; validar versão e tamanho de cada estrutura antes de acessar campos.
- Buffers de canais pertencem ao host, com capacidade/contagem explícitas e validade limitada à chamada; o plugin não guarda ponteiros. IDs e ordem são negociados na inicialização e ficam imutáveis no run.
- Ciclo futuro `create/reset → step → destroy`, handle opaco pertencente ao plugin e destruição pelo mesmo módulo que o criou. Sem alocação Rust, String, Vec, bool de linguagem ou exceção atravessando a fronteira C.
- Flags ópticas propostas: bit 0 válido, bit 1 dispositivo digital, bit 2 nível digital. Flags de frame: bits 0/1 encoder/IMU válidos, bits 2/3 limitação de corrente esquerda/direita.
- Modos de saída 0 Drive, 1 Brake, 2 Coast. Erro 0 sucesso; 1 versão/tamanho; 2 entrada/capacidade; 3 execução interna; 4 comando inválido. Em erro, não aplicar o comando e interromper com diagnóstico.
- Nenhuma biblioteca será considerada compatível até existirem negociação, validação de buffers, teste de layout em C e política de falhas no carregador. Os tipos atuais são preparação da ABI, não promessa de suporte binário já operacional.

## Registros, calibração e limites do replay visual

Junto ao CSV/replay são gravados:

- `.sensors.jsonl`: uma observação por tick de log, com canais, aquisição/disponibilidade, validade, idade, ADC/digital, refletância/filtrado explicitamente de depuração, encoder/IMU entregues, pose estimada, marcas, volta e mapa aprendido.
- `.commands.csv`: comando inicial e mudanças, capturadas em todos os ticks físicos; não depende da frequência de log científico para preservar os instantes de atuação solicitada.
- Snapshot do experimento e metadados com seletores efetivos e parâmetros/seeds completos.

O JSONL mostra a última amostra entregue em cada observação; não é gravação de todas as aquisições caso a taxa de log seja menor. Os valores anteriores permanecem retidos. Reflexão ideal e bias real não entram no firmware. O replay binário v3 continua com seus canais agregados; player enriquecido, worker, índices, checkpoints e políticas de gravação ficam na etapa 9. Calibração existente continua usando o mesmo núcleo; os novos registros permitem comparar dados, mas ajuste automático conjunto de todos os novos parâmetros não foi adicionado.

## Evidências finais

- **135 testes sem GUI e 140 com GUI passaram.** São 18 novos testes de integração e um de renderização do painel sem janela.
- Casos cobrem área/rotação/pose e polaridade, respostas/filtro/saturação, histerese digital, fase/conversão/latência/retenção, conflito de multiplexador, ruído independente, encoder/IMU, anti-windup, perda/reentrada da linha, controle de velocidade, odometria/fusão, marcas/perfil, persistência, replay e ciclo de vida do firmware.
- Replay dos comandos gravados reproduziu posição X e yaw do ensaio elétrico dentro de 1e-12. Modelos inválidos, leitura fora da grade e comando de firmware com timestamp incorreto foram recusados.
- `examples/sensing/projeto.rtsim`: 2 s simulados, **40.000 passos físicos a 50 µs**, controle a 1 ms, 2.001 observações e **8.004 registros ópticos** conferidos sem entrega futura. Velocidade final 0,038488 m/s para alvo 0,04 m/s; erro menor que 4%. Odometria de deslocamento X aproximadamente 0,059373 m.
- Benchmark local único desse exemplo, release e sem gravação: 0,105254 s de parede, cerca de 380.034 passos/s e **19,00× tempo real**. Usa a física de referência, não o circuito/contato avançado da etapa 7; não invalida a necessidade de otimizar aquele cenário. Não é um benchmark representativo de todos os modelos.
- Formatação e diff verificados. Não houve teste manual de janela nem validação experimental com um robô real.

```powershell
cargo test --offline --no-default-features
cargo test --offline
cargo fmt --all -- --check
cargo run --release --offline --no-default-features -- run examples/sensing/projeto.rtsim --headless --csv target/stage8-final.csv --replay target/stage8-final.rtlog
cargo run --release --offline --no-default-features -- benchmark examples/sensing/projeto.rtsim --duration 2s
```
