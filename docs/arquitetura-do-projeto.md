# Arquitetura do RoboTraceSim

> Documento de arquitetura e direção técnica do projeto.
> Revisão desta auditoria: **24/09/2026**.
> Escopo: simulador configurável de robô seguidor de linha / Robotrace, com física determinística, relógios independentes, níveis de fidelidade, editores de robô e pista, cálculo desacoplado da visualização e comparação com medições reais.

## 1. Objetivo do projeto

O RoboTraceSim deve permitir construir um robô e uma pista virtual, executar o mesmo controlador que seria usado no robô real e observar como o conjunto se comportaria sob diferentes níveis de fidelidade física.

O objetivo não é apenas produzir uma animação plausível. O simulador deve ser útil para:

- desenvolver e testar algoritmos de controle;
- estudar sensibilidade a parâmetros mecânicos, elétricos e ópticos;
- comparar projetos de robô;
- testar pistas e estratégias de corrida;
- reproduzir experimentos de forma determinística;
- executar varreduras e lotes sem interface gráfica;
- comparar simulação e medições do robô real;
- aumentar gradualmente o realismo sem obrigar todo usuário a pagar o custo computacional do modelo mais complexo.

A referência temporal desejada para casos de alta fidelidade é:

```text
passo físico:       50 us  = 20 kHz
passo do controle:   1 ms  =  1 kHz
```

Isso significa que, em um intervalo lógico de 1 ms, o mundo físico pode avançar 20 vezes enquanto o controlador mantém o último comando entregue.

## 2. Princípios arquiteturais obrigatórios

### 2.1 Física e lógica não compartilham obrigatoriamente a mesma frequência

O simulador deve ter um relógio físico de passo fixo e agendas independentes para controle, sensores, encoder, IMU, log e visualização.

A regra atual, adequada ao objetivo, é que eventos lógicos ocorram em múltiplos inteiros do passo físico. Assim, 50 us de física e 1.000 us de controle são uma combinação válida e determinística.

A renderização não pertence ao relógio científico. Um monitor a 60 Hz não deve mudar uma integração a 20 kHz.

### 2.2 O cálculo é a fonte de verdade; a visualização é uma reconstrução

A interface gráfica não deve possuir sua própria física. Ela configura uma execução, envia comandos ao núcleo e apresenta snapshots/replays calculados pelo mesmo executor usado pela CLI.

Este princípio já está materializado na arquitetura atual e deve ser preservado:

```text
Projeto congelado
      |
      v
SimulationCore ------------------------------+
      |                                      |
      | snapshots/telemetria                 | resultados científicos
      v                                      v
preview descartável                    CSV / RTLOG / sidecars
      |                                      |
      v                                      v
GUI ao vivo                         Replay / comparação / análise
```

A reprodução pode interpolar visualmente entre amostras, mas essa interpolação nunca altera estados físicos ou resultados científicos.

### 2.3 Realismo deve ser selecionável por subsistema

`Ideal`, `Simplificado` e `Realista` devem ser entendidos como presets convenientes, não como três engines diferentes.

A arquitetura-alvo é uma composição de modelos:

```text
Robô
├── chassi / normal
├── contato e pneus
├── rodas / transmissão
├── motores
├── driver
├── bateria / barramento
├── fan / sucção
├── sensores
└── controlador

Pista
├── geometria
├── material / atrito
├── campo óptico
├── marcas e falhas
├── relevo, quando suportado
└── eventos de corrida
```

Cada bloco deve ter uma implementação simples e, quando fizer sentido, implementações progressivamente mais completas.

### 2.4 Complexidade não equivale a fidelidade

Um modelo só deve ser chamado de mais realista quando:

1. representa um efeito que pode ser relevante no Robotrace;
2. possui parâmetros identificáveis ou mensuráveis;
3. é numericamente estável na faixa prevista;
4. melhora a comparação com dados físicos independentes;
5. seu custo computacional é conhecido.

O preset chamado `realistic` no código atual é um **preset avançado**, mas ainda não está qualificado experimentalmente.

### 2.5 Nenhuma opção física pode ser silenciosamente ignorada

Uma configuração deve seguir uma destas três regras:

- é executada pelo runtime;
- é explicitamente apenas metadado/visual;
- é rejeitada por ainda não ser suportada.

Essa política já aparece em relevo, tipos de sensor não implementados, modelos elétricos e combinações de montagem incompatíveis e deve continuar como regra arquitetural.

## 3. Estado da auditoria

Esta revisão foi feita por inspeção do código, schemas, exemplos, testes, documentação e resultados preservados no repositório enviado.

O ambiente desta auditoria não possui `cargo`; portanto, **não foi executada uma compilação nova desta cópia**. Contagens de testes e benchmarks citadas abaixo são evidências registradas pelo próprio projeto em 22–23/09/2026, e não uma nova certificação produzida nesta revisão.

O repositório está substancialmente à frente do README histórico de v0.5/v0.6. As etapas 1 a 9 de `tasks.md` estão marcadas como concluídas; as pendências formais concentram-se na qualificação física da etapa 10.

Também existe uma divergência de nomenclatura de versão: `Cargo.toml` declara `0.6.0`, enquanto partes da interface/documentação exibem v0.09. A versão do aplicativo e as versões de schema devem continuar independentes, mas a versão pública do produto deve ser unificada.

## 4. Arquitetura atual do código

### 4.1 Entrada e API

```text
src/main.rs
  └── seleciona GUI ou CLI

src/lib.rs
  ├── config
  ├── core
  ├── models
  ├── track
  ├── control
  ├── experiments
  ├── io
  ├── sim
  ├── replay / telemetry
  └── run_app / run_cli
```

`main.rs` é fino. A simulação é reutilizável como biblioteca, o que é correto para GUI, CLI, testes, batch e ferramentas de calibração.

### 4.2 Núcleo temporal

| Arquivo | Responsabilidade atual |
|---|---|
| `src/core/clock.rs` | tempo simulado inteiro em microssegundos, duração e grade física |
| `src/core/scheduler.rs` | agenda de sensores, encoder, IMU, controlador e log |
| `src/core/integrator.rs` | integração mecânica |
| `src/core/power_step.rs` | solução elétrica/mecânica acoplada |
| `src/sim.rs` | `SimulationCore`, ciclo causal, sessão, runner, checkpoint e execução completa |

`SimulationSession` é hoje um alias de `SimulationCore`, evitando dois motores de simulação diferentes.

A API já oferece operações equivalentes a:

```text
sample()
step()
advance_steps()
advance_until()
is_finished()
checkpoint()/restore()
```

Observar o estado não deve consumir RNG, integrar física ou executar o controlador.

### 4.3 Modelos físicos

| Arquivo | Responsabilidade atual |
|---|---|
| `src/models/robot.rs` | montagem física, rodas, massas, COM e validação |
| `src/models/chassis.rs` | equilíbrio de cargas normais e transferência quase-estática |
| `src/models/contact.rs` | contato individual, Coulomb, brush reduzido, caster e rolamento |
| `src/models/fidelity.rs` | presets e seleção explícita de modelos |
| `src/models/electrical.rs` | motor DC elétrico, transmissão e parâmetros do barramento |
| `src/models/power.rs` | estados elétricos, comandos, proteção e energia |
| `src/normal_force.rs` | fan/downforce/sucção |
| `src/motor.rs` | modelo de motor usado pelo solver mecânico |
| `src/battery.rs` | referência histórica do modelo de bateria |

Ainda existem módulos históricos na raiz (`battery.rs`, `motor.rs`, `wheel.rs`, etc.) ao lado dos módulos novos. Parte deles continua sendo usada como implementação interna ou compatibilidade. A direção correta é continuar migrando responsabilidades para `models/`, `core/`, `track/`, `control/`, `io/` e `experiments/`, sem manter dois caminhos físicos equivalentes indefinidamente.

### 4.4 Pista

```text
src/rtsim_track.rs       geometria paramétrica v2 e regras
src/track/definition.rs  ambiente, regiões, marcas, portais e validação
src/track/runtime.rs     snapshot imutável, primitivas e consultas
src/track/spatial.rs     índice espacial
src/track/events.rs      estado da corrida
src/track/persistence.rs persistência do ambiente
```

A definição editável é congelada antes de uma execução. O runtime contém caches e índices para evitar reconstrução por tick.

### 4.5 Sensoriamento e controle

```text
src/models/sensing.rs
src/sensor.rs
src/encoder.rs
src/gyro.rs
src/control/mod.rs
src/controller.rs
src/control/estimator.rs
src/control/replay_controller.rs
src/control/native.rs
```

O controlador recebe observáveis entregues, não a verdade do simulador. Isso é fundamental para evitar um controlador artificialmente onisciente.

### 4.6 Execução, replay e experimentos

```text
src/experiments/jobs.rs       worker, pausa, cancelamento e preview
src/experiments/batch.rs      expansão e execução de lotes
src/experiments/calibration.rs estudos versionados e ajuste
src/experiments/metrics.rs    comparação e métricas
src/experiments/robustness.rs refinamento e robustez
src/io/replay_v4.rs           replay indexado e cache limitado
src/io/experiment.rs          snapshot/configuração congelada
src/telemetry.rs              canais e sidecars
```

Essa separação implementa a estratégia desejada de calcular independentemente do desenho.

## 5. Contrato temporal

### 5.1 Configuração

O projeto já possui períodos independentes:

```json
{
  "time": {
    "physics_dt_us": 50,
    "controller_period_us": 1000,
    "sensor_period_us": 500,
    "imu_period_us": 500,
    "encoder_period_us": 500,
    "log_period_us": 1000,
    "render_period_us": 16667
  }
}
```

O caso acima executa 20 passos físicos entre chamadas consecutivas do controlador.

### 5.2 Ordem causal

A arquitetura deve preservar a distinção entre:

1. estado físico no início do tick;
2. integração do intervalo físico;
3. aquisições e entregas de sensores devidas naquele instante;
4. controlador, quando devido;
5. comando entregue ao atuador conforme sua latência;
6. observação/log;
7. próximo intervalo físico.

O detalhe exato da fase temporal é um contrato científico e não deve ser alterado por refatorações de UI.

### 5.3 Extensão futura: subpassos internos

O passo físico global não precisa obrigatoriamente resolver todo fenômeno futuro.

Por exemplo, se algum dia for implementado chaveamento explícito de um driver a centenas de kHz, existem duas opções corretas:

- manter um modelo médio no passo de 50 us; ou
- criar subpassos internos somente para o subsistema elétrico.

Não se deve reduzir todo o simulador para o menor tempo do fenômeno mais rápido sem necessidade, pois isso destruiria desempenho.

## 6. Níveis de fidelidade

### 6.1 Presets atualmente implementados

| Preset | Contato | Normal | Rolamento | Interpretação |
|---|---|---|---|---|
| `ideal` | cinemática diferencial | estática | desligado | controle/algoritmo com mínimo custo físico |
| `simplified` | Coulomb por roda + elipse | quase-estática | ligado | forças, saturação e perdas principais |
| `realistic` | brush viscoelástico reduzido | quase-estática | ligado | slip regularizado, relaxação e parâmetros calibráveis |

O usuário pode sobrescrever subsistemas, portanto o preset deve ser salvo junto com a configuração efetivamente resolvida.

### 6.2 Evolução recomendada

Não criar `IdealEngine`, `SimpleEngine` e `RealEngine`. Criar um registro de modelos por subsistema, por exemplo:

```text
contact:
  ideal
  coulomb
  brush
  measured_tire          futuro

chassis_vertical:
  none
  quasi_static
  planar_pitch_roll      futuro
  full_3d                somente se justificado

motor:
  ideal
  dc_simple
  dc_electrical
  measured_curve         futuro
  bldc_average           futuro

sensor_optical:
  ideal_point
  area_2d
  measured_response
  optical_height_light   futuro
```

Cada descritor de modelo deve informar:

- parâmetros obrigatórios;
- estado adicional;
- custo esperado;
- dependências;
- limites de validade;
- canais observáveis;
- ensaio de calibração recomendado.

## 7. Catálogo físico: implementado, faltante e prioridade

As tabelas desta seção são o inventário central de realismo do projeto.

### 7.1 Chassi, massa e movimento

| Efeito | Estado atual | Direção |
|---|---|---|
| massa total | implementado | manter |
| COM X/Y/Z | implementado/persistido | manter |
| inércia de yaw | implementada | manter |
| massa por componente + eixo paralelo | implementado | manter |
| corpo rígido planar X/Y/yaw | implementado | base principal |
| cargas por apoio | implementado | manter |
| transferência de carga por aceleração | quase-estática implementada | calibrar/validar |
| pitch e roll | não implementados | P2 se carga/óptica/sucção exigirem |
| heave / movimento vertical | não implementado | P2 condicionado a dados |
| tombamento dinâmico | não implementado | P3; hoje execução é recusada quando equilíbrio não é possível |
| flexão do chassi | não implementada | P3/P4 |
| vibração estrutural | não implementada | P3/P4 |
| arrasto aerodinâmico | não implementado | **P1/P2**, fácil de identificar por coast-down |
| vento | não implementado | P4 |

Para Robotrace de baixa altura, a física planar continua uma escolha eficiente. Pitch/roll devem entrar quando houver evidência de efeito relevante sobre distribuição de normal, altura dos sensores ou vedação de sucção.

### 7.2 Rodas, pneus e contato

| Efeito | Estado atual | Direção |
|---|---|---|
| geometria por roda | implementada | manter |
| inércia por roda | implementada | manter |
| roda motriz/passiva/caster | implementado | manter |
| atrito longitudinal/lateral | implementado | calibrar |
| atrito local da pista | implementado | calibrar |
| limite combinado Fx/Fy | elipse implementada | manter |
| slip longitudinal | implementado | validar com bancada |
| slip angle/lateral | implementado | validar |
| resistência ao rolamento | implementada, reduzida | P1: identificar dependência com carga/velocidade se necessária |
| sensibilidade à carga | implementada por expoente | calibrar |
| relaxação tangencial | implementada no brush reduzido | calibrar |
| compressão radial | quase-estática implementada | validar |
| caster passivo | reduzido implementado | validar apenas se usado |
| curva medida de pneu | não implementada como modelo dedicado | **P1** após dados de bancada |
| deformação avançada da carcaça | não implementada | P3 |
| temperatura do pneu | não implementada | P3 |
| desgaste | não implementado | P3/P4 |
| contaminante acumulado/sujeira no pneu | não implementado | P4 |
| hidro/fluido | fora do objetivo normal | não priorizar |

A próxima melhoria física importante não é necessariamente adicionar mais estados ao pneu. Primeiro devem ser obtidas curvas força × slip × normal na superfície real e verificado se o `brush` reduzido é suficiente.

### 7.3 Motores e transmissão

| Efeito | Estado atual | Direção |
|---|---|---|
| DC quase-estático | implementado | manter |
| R, L, Ke, Kt | implementados | calibrar |
| inércia do rotor | implementada | calibrar |
| redução | implementada | manter |
| eficiência | implementada | substituir por mapa se dados exigirem |
| perda viscosa | implementada | calibrar |
| temperatura do motor | implementada por corpo térmico | calibrar |
| R(T) | implementada | calibrar |
| reversão/frenagem | implementada | manter |
| backlash | não implementado | **P2** se observado em reversões/controle agressivo |
| elasticidade do eixo/engrenagem | não implementada | P2/P3 |
| atrito de transmissão dependente de velocidade/carga | reduzido | P2 após ensaio |
| mapa de eficiência | não implementado | P2 |
| motor por curva medida | não dedicado | P1/P2 |
| BLDC médio | não implementado | P3 se houver robô BLDC |
| FOC/chaveamento explícito | não implementado | P4, somente para perguntas elétricas específicas |

### 7.4 Driver, eletrônica e bateria

| Efeito | Estado atual | Direção |
|---|---|---|
| PWM médio | implementado | adequado ao passo de 50 us |
| quantização do PWM | implementada | manter |
| deadband | implementado | calibrar |
| queda fixa da ponte | implementada | calibrar |
| resistência da ponte | implementada | calibrar |
| limite de corrente | acoplado ao torque | manter |
| latência do comando | implementada | medir |
| Drive/Brake/Coast | implementados | manter |
| regeneração | implementada com limites | validar |
| barramento único | implementado | adequado à maioria dos robôs |
| fonte ideal / Thevenin | implementada | manter |
| OCV × SoC e R × SoC | implementados | medir |
| ramo RC de polarização | implementado | medir |
| fiação e consumo auxiliar | implementados | calibrar |
| proteção de subtensão/corrente/temperatura | implementada | parametrizar pelo hardware real |
| térmica da bateria | não implementada | P3 |
| reguladores dinâmicos separados | não implementados | P3 |
| MOSFET/diodos/avalanche | não implementados | P4 |
| bordas de chaveamento/EMI | não implementadas | fora do uso normal de dinâmica do robô |

### 7.5 Fan, downforce e sucção

| Efeito | Estado atual | Direção |
|---|---|---|
| múltiplos fans posicionados | implementado | manter |
| curvas PWM → força | implementadas | usar curvas medidas |
| resposta temporal | primeira ordem implementada | calibrar |
| dependência de tensão | reduzida implementada | calibrar |
| corrente/consumo | implementados | medir |
| sucção pressão × área | implementada | manter |
| vazamento | implementado de forma reduzida | calibrar |
| folga configurada | implementada, fixa | manter por enquanto |
| momento causado pela posição | implementado | manter |
| RPM mecânico independente do fan | não implementado | P2 se transitórios exigirem |
| folga variando com pitch/heave | não implementada | **P2** junto à dinâmica vertical |
| selo deformável | não implementado | P3 |
| CFD/fluxo detalhado | não implementado | P4, provavelmente desnecessário |
| efeito de velocidade do ar/chão | reduzido ou ausente | P3 se medições mostrarem necessidade |

### 7.6 Sensores ópticos

| Efeito | Estado atual | Direção |
|---|---|---|
| posição individual | implementada | manter |
| orientação individual | implementada | manter |
| footprint 2D | implementado | manter |
| integração espacial | implementada por quadratura | otimizar em lotes |
| resposta ideal/linear/polinomial/tabela/limiar | implementada | calibrar por sensor |
| ADC e saturação | implementados | manter |
| ruído óptico/ADC | implementados | medir distribuição real |
| filtro | implementado | configurar conforme firmware |
| aquisição/conversão/latência | implementadas | medir |
| multiplexação estática | implementada | manter |
| histerese digital | implementada | manter |
| altura do sensor | persistida, sem efeito óptico | **P2** |
| iluminação ambiente | não implementada | **P2** |
| sombra/reflexo/ângulo de incidência | não implementados | P2/P3 |
| lente/óptica 3D | não implementada | P3 |
| integração durante tempo de exposição | não implementada | P3 |
| temperatura do emissor/receptor | não implementada | P4 |

O primeiro passo para aumentar realismo óptico deve ser medir ADC versus posição, altura, material e iluminação. Um modelo empírico medido provavelmente entrega mais fidelidade por custo que um renderizador óptico 3D.

### 7.7 Encoder, IMU e firmware observável

Já existem quantização, quadratura, relação do eixo, perda de pulsos, latência, filtro, ruído, bias, drift de gyro, aceleração planar e RNGs separados.

Ainda faltam, se necessários:

- erros sistemáticos de escala por roda;
- excentricidade/erro periódico do encoder;
- jitter de timestamp;
- IMU em posição diferente do centro, com braço de alavanca;
- orientação 3D, gravidade em pitch/roll;
- modelo de barramento real (SPI/I2C/ADC DMA) além dos slots estáticos.

Esses efeitos devem ser P2/P3 e orientados pelo firmware/hardware que se pretende reproduzir.

### 7.8 Pista e ambiente

| Efeito | Estado atual | Direção |
|---|---|---|
| retas | implementadas | manter |
| arcos | implementados | manter |
| pista fechada e validação geométrica | implementadas | manter |
| linha com largura real | implementada | manter |
| regiões de material/atrito | implementadas | ampliar formas quando necessário |
| refletância do substrato | implementada | calibrar |
| marcas pintadas e falhas | implementadas | manter |
| largada/checkpoints/chegada | implementados | manter |
| regras/perfis | implementados como configuração | não confundir com certificação oficial |
| índice espacial | implementado | manter/otimizar |
| spline/Bézier | não implementado no TrackV2 | **P1** para editor realmente livre |
| clothoid/transição de curvatura | não implementada | P2 |
| desenho livre/importação vetorial | não implementado | **P1/P2** |
| regiões por polígono arbitrário | não implementadas; atuais são retângulos orientados | P2 |
| mapa raster de refletância | não é o runtime principal atual | P2 para pistas medidas/fotos calibradas |
| rugosidade física | armazenada como metadado | P2/P3 |
| altura/relevo | metadado; solver rejeita relevo ativo | P2/P3 |
| inclinação/normal 3D | não implementada | P3 |
| sujeira espacial procedural/temporal | não implementada | P3 |
| temperatura/umidade | não implementadas | P4 |

Para cumprir literalmente “montar da forma que o usuário quiser”, a maior lacuna funcional do editor de pista é a geometria: o formato paramétrico moderno aceita apenas retas e arcos. O próximo salto útil é adicionar spline/Bézier e importação de caminho vetorial, mantendo consultas analíticas ou uma discretização com tolerância explícita.

## 8. Editor de robô

### 8.1 O que já existe

A arquitetura atual já possui uma base forte para o editor solicitado:

- rodas/apoios com ID, posição, ângulo, raio, largura e inércia;
- tipos Driven, Passive e Caster;
- associação a motor esquerdo/direito;
- material/pneu por roda;
- posição de sensores;
- posição de fans;
- massas por componente;
- massa medida ou calculada;
- COM e inércia;
- altura persistida;
- seleção, arraste, rotação, duplicação, remoção e snapping;
- undo/redo;
- preview top-down usando o mesmo referencial do runtime;
- edição dos modelos elétricos, sensores e downforce.

### 8.2 Arquitetura-alvo do editor

O editor não deve editar diretamente structs do solver. Ele deve editar uma **definição de robô**, validável independentemente da GUI:

```text
RobotDefinition
├── Geometry
│   ├── chassis outline
│   ├── contacts/wheels
│   ├── sensors
│   ├── fans/suction
│   └── optional collision/validity areas
├── MassModel
├── Tires
├── Motors/Transmission
├── ElectricalSystem
├── Sensing
├── Control interface
└── Fidelity selections
```

Antes do run:

```text
RobotDefinition
      |
      | validation + model resolution
      v
ResolvedRobot / FrozenExperiment
      |
      v
SimulationCore
```

A execução nunca deve consultar um objeto que a interface ainda esteja editando.

### 8.3 Melhorias prioritárias do editor

1. mostrar claramente quais parâmetros pertencem ao preset e quais são overrides;
2. exibir custo/limitação de cada modelo de fidelidade;
3. indicar visualmente parâmetros que são somente metadados;
4. adicionar assistentes de importação de curvas medidas de motor, pneu, bateria, fan e sensor;
5. permitir desenho de contorno de chassi sem transformá-lo automaticamente em física 3D;
6. fornecer diagnóstico de identificabilidade: “este parâmetro não tem dados para ser calibrado”.

## 9. Editor de pista

### 9.1 Fonte de verdade

O editor deve produzir uma única `TrackDefinition`; o `TrackRuntime` é um snapshot otimizado dessa definição.

A pista deve manter camadas separadas:

```text
TrackDefinition
├── Centerline/Geometry
├── Table/Bounds
├── OpticalLayer
├── SurfaceLayer
├── RaceLayer
└── VerticalLayer (futuro)
```

### 9.2 Prioridades para maior liberdade

A sequência recomendada é:

1. retas e arcos — já implementados;
2. spline/Bézier com controle de tangência;
3. clothoid opcional para transições de curvatura;
4. importação SVG/DXF ou polilinha com escala e simplificação;
5. regiões de superfície poligonais;
6. mapa raster opcional de refletância para reproduzir uma pista medida;
7. height map/relevo somente quando houver solver vertical.

O desenho da tela pode ser rasterizado para velocidade, mas a fonte física deve continuar vetorial/analítica ou possuir uma discretização cuja resolução e erro sejam explícitos.

## 10. Cálculo, visualização e desempenho

### 10.1 A estratégia de “calcular antes e reconstruir depois” é correta

A ideia descrita para uma versão anterior do projeto é arquiteturalmente adequada e já foi implementada em grande parte na etapa 9:

- worker separado da thread da interface;
- preview com snapshots descartáveis;
- pausa, passo, retomada e cancelamento;
- cálculo headless;
- replay v4 indexado;
- cache limitado;
- player com seek e velocidade independente;
- batch paralelo de experimentos independentes.

Essa estratégia deve permanecer o padrão para simulações pesadas.

Também pode existir um modo “ao vivo”, mas ele é apenas uma política de consumo:

```text
while wall_frame:
    avançar quantos ticks físicos couberem/forem desejados
    publicar o snapshot mais recente
    desenhar 1 frame
```

Não é necessário desenhar os 20.000 estados físicos de cada segundo simulado.

### 10.2 Evidência de desempenho existente

O benchmark preservado da etapa 9, em Windows x86_64, release, passo de 50 us e runs curtos de 100 ms, registrou aproximadamente:

| Cenário | Fator simulado/real sem logs |
|---|---:|
| referência agregada | 7,3x |
| 16 sensores | 2,0x |
| Simplificado 4 contatos | 1,87x |
| Realista reduzido 4 contatos | 4,8x |
| circuito elétrico acoplado | 0,34x |
| 64 sensores + pista complexa | 0,14x |

Esses números não são garantia universal, mas mostram duas coisas importantes:

1. 50 us é viável em tempo real para vários cenários já existentes;
2. os gargalos restantes são específicos de algoritmo/subsistema, especialmente solução elétrica acoplada e óptica densa.

### 10.3 Deve-se usar DLL de funções em C?

**Não como estratégia padrão de aceleração.**

O núcleo atual em Rust já é compilado para código nativo. Reescrever a mesma equação em C e atravessar uma ABI/DLL não cria automaticamente um algoritmo mais rápido e ainda adiciona:

- custo de FFI em chamadas muito pequenas;
- duplicação de tipos e validação;
- risco de divergência numérica;
- gerenciamento manual de memória/ownership;
- maior complexidade de build e distribuição;
- mais dificuldade para checkpoint e determinismo.

Portanto, a decisão arquitetural é:

> **Manter Rust como kernel físico principal. Só introduzir C/C++/DLL quando um perfil reproduzível mostrar um kernel específico no qual outra implementação entregue ganho material, ou quando a DLL for necessária para reutilizar firmware/código externo.**

### 10.4 Onde uma ABI C faz sentido

Há uma aplicação válida e diferente: integrar um controlador/firmware nativo.

`src/control/native.rs` já prepara layouts `repr(C)`. A evolução pode carregar opcionalmente uma biblioteca que implemente uma ABI versionada semelhante a:

```text
create(config)
reset(seed)
step(sensor_frame) -> actuator_command
serialize_state()      opcional
restore_state()        opcional
shutdown()
```

Nesse caso, a DLL existe por **reuso e integração**, não por presumir que C é mais rápido que Rust.

Para firmware não confiável ou sujeito a crash, um processo separado pode ser mais seguro que uma DLL no processo do simulador.

### 10.5 Ordem recomendada de otimização

Antes de qualquer troca de linguagem:

1. perfilar por subsistema em release;
2. remover clones e resolução de configuração dentro do tick;
3. usar dados resolvidos e compactos no hot path;
4. warm-start de solvers iterativos quando matematicamente seguro;
5. reduzir chamadas repetidas do solver mecânico durante a busca elétrica;
6. agrupar consultas ópticas de muitos sensores;
7. reutilizar pontos de quadratura transformados;
8. melhorar locality/SoA nos canais muito numerosos;
9. vetorizar/SIMD somente os kernels comprovadamente dominantes;
10. GPU apenas para lotes grandes o bastante para amortizar transferência e despacho.

A paralelização mais simples e robusta continua sendo entre **runs independentes** de batch/calibração.

### 10.6 Orçamento de desempenho

Definir metas por preset e não uma única meta global.

Sugestão de contrato:

| Perfil | Meta inicial |
|---|---|
| Ideal | muito acima de tempo real |
| Simplificado | >= 1x em 50 us na máquina de referência |
| Realista sem circuito pesado | alvo >= 1x quando possível |
| Realista completo | pode calcular offline; custo deve ser reportado |
| batch | maximizar throughput total, não FPS |

Um modelo não deve reduzir silenciosamente resolução ou fidelidade para “bater tempo real”.

## 11. Replay, telemetria e reprodutibilidade

### 11.1 Snapshot imutável

Cada run deve congelar:

- definição resolvida do robô;
- definição resolvida da pista;
- seleção efetiva de modelos;
- parâmetros;
- períodos;
- seed mestre e seeds derivados;
- duração;
- versão dos schemas e aplicativo;
- controlador/adaptador usado.

A UI pode continuar sendo editada enquanto o worker roda sem alterar o experimento em curso.

### 11.2 Replay científico versus visual

O replay precisa conservar canais necessários para reconstrução e análise, mas não precisa registrar cada tick físico.

Arquitetura correta:

```text
20 kHz physics
   |
   +--> estados internos do solver (não gravar todos)
   |
   +--> log científico 1 kHz ou configurável
             |
             +--> replay indexado
             +--> CSV/sidecars
             +--> visualização interpolada
```

Os sidecars atuais de contatos, potência, sensores, comandos e eventos são úteis. Em uma evolução de formato, podem ser consolidados em blocos/canais do replay sem obrigar todo leitor a carregar tudo em RAM.

### 11.3 Checkpoint

O checkpoint atual é process-local. Persistência de checkpoint em disco é uma melhoria útil para runs muito longos, mas exige serializar absolutamente todos os estados causais: solver, motor, corrente, temperatura, SoC, pressão/fan, filtros, RNG, filas de sensores, controlador, comandos pendentes e eventos.

Não reconstruir checkpoint a partir de amostras de replay.

## 12. Persistência e schemas

A arquitetura correta é manter schemas versionados por domínio, independentes da versão do aplicativo.

Atualmente existem, entre outros:

- `rtsim-project-v1`;
- `rtsim-robot-v8`;
- `rtsim-track-v2`;
- `rtsim-track-environment-v1`;
- schemas de perfis de componentes;
- configuração congelada de experimento;
- replay v4;
- estudo `rtsim-study-v1`.

Regras a preservar:

- caminhos relativos ao projeto;
- assets incorporáveis para portabilidade;
- Unicode válido;
- números finitos;
- campos desconhecidos rejeitados quando implicam física não suportada;
- migração explícita de formatos conhecidos;
- snapshot completo da configuração efetiva.

Não existe necessidade de manter compatibilidade eterna com protótipos antigos, mas qualquer quebra deliberada deve ser identificável e conversível quando possível.

## 13. Calibração e validação física

Esta é hoje a maior lacuna entre “software sofisticado” e “simulador fisicamente confiável”.

A infraestrutura já existe para:

- importar logs;
- alinhar por tempo;
- comparar sinais;
- calcular métricas e cobertura;
- rodar estudos de parâmetros;
- separar configuração congelada;
- executar robustez 100/50/25 us;
- manter dados de calibração e validação separados no manifesto.

O que ainda falta é dado físico qualificado.

### 13.1 Ordem recomendada de identificação

Não ajustar todos os parâmetros ao tempo de volta. Identificar subsistemas primeiro:

1. **motor + driver** — tensão, corrente, RPM, torque/carga;
2. **transmissão** — redução, perdas e eventual backlash;
3. **bateria** — OCV, resistência, recuperação RC;
4. **fan/sucção** — força, corrente, tensão, folga e transitório;
5. **sensor** — ADC por posição/altura/luz/material e latência;
6. **pneu** — força versus slip, normal e superfície;
7. **robô completo** — trajetórias e voltas reservadas.

### 13.2 Critério para chamar um modelo de realista

O preset avançado só deve ser promovido a “realista qualificado” quando:

- dados de calibração e validação forem fisicamente independentes;
- tolerâncias forem definidas antes de avaliar o conjunto reservado;
- parâmetros tiverem faixa de validade;
- incerteza/repetibilidade forem registradas;
- o modelo avançado demonstrar ganho útil sobre o simplificado;
- o custo adicional também for publicado.

Até isso ocorrer, o software pode ser numericamente verificado, mas não deve prometer precisão física universal.

## 14. O que já está concluído em relação ao objetivo

### Núcleo e tempo

- executor único;
- fixed-step determinístico;
- microssegundos inteiros;
- 50 us / 1 ms suportado;
- agendas independentes;
- configuração efetiva registrada;
- checkpoint em memória.

### Física

- corpo rígido planar;
- montagem explícita;
- contatos por roda;
- normal por apoio;
- transferência quase-estática;
- Coulomb e brush reduzido;
- atrito combinado;
- rolling/caster;
- motor DC simples e elétrico;
- circuito/bateria acoplados;
- térmica do motor;
- regeneração/proteções;
- múltiplos fans e sucção.

### Sensores/controle

- sensores posicionados individualmente;
- footprint 2D;
- ruído, ADC, filtro, latência e multiplexação;
- encoder e IMU;
- controlador observável;
- estimador/odometria;
- replay de comandos;
- contrato para integração de firmware nativo.

### Editores

- editor de robô funcional;
- montagem e massa/COM;
- editor de pista;
- retas/arcos;
- regiões, marcas e portais;
- undo/redo;
- preview coerente com o runtime.

### Execução e análise

- GUI e CLI;
- worker em segundo plano;
- cálculo e reprodução separados;
- replay indexado;
- batch;
- benchmark;
- calibração e comparação;
- CI/testes preservados no repositório.

## 15. O que falta para atingir plenamente o objetivo

### Prioridade P0 — fechar confiabilidade física, não adicionar complexidade

1. coletar e versionar dados reais de subsistemas;
2. concluir a etapa 10 de validação independente;
3. definir tolerâncias físicas e incerteza;
4. publicar quais parâmetros são medidos, estimados ou apenas defaults;
5. investigar a não convergência conhecida com oito contatos sob controle fechado;
6. manter regressões numéricas separadas de validação física.

### Prioridade P1 — maior retorno de fidelidade/uso

1. adicionar spline/Bézier e importação vetorial ao editor de pista;
2. criar modelos por curvas medidas para pneu/motor/fan/sensor;
3. implementar arrasto/resistências identificadas por coast-down;
4. otimizar o solver elétrico acoplado;
5. otimizar aquisição óptica em grande número de sensores;
6. melhorar tooling de importação/calibração de componentes reais.

### Prioridade P2 — efeitos que podem ser importantes em robôs extremos

1. backlash e elasticidade da transmissão;
2. altura/iluminação/ângulo na óptica;
3. dinâmica vertical reduzida de heave/pitch/roll;
4. folga de sucção acoplada à atitude/altura;
5. regiões de pista poligonais e raster calibrado;
6. rolling resistance dependente de velocidade/carga;
7. IMU com posição/3D quando necessário.

### Prioridade P3/P4 — somente com evidência

- térmica e desgaste do pneu;
- carcaça avançada;
- suspensão/flexão/vibração completa;
- selo deformável;
- CFD;
- BLDC/FOC detalhado;
- chaveamento de semicondutores;
- ambiente térmico/umidade;
- engine 3D completa.

Esses itens não são pré-requisitos para um simulador Robotrace de alta utilidade. Podem até reduzir a confiabilidade se forem adicionados sem parâmetros reais.

## 16. Roadmap arquitetural recomendado

### Marco A — “Realista mensurável”

- concluir protocolo e dados da etapa 10;
- calibrar motor/bateria/fan/sensor/pneu;
- comparar Simplificado × Realista em holdout;
- publicar custo e erro por subsistema.

**Saída:** primeira versão em que `realistic` possui significado experimental declarado.

### Marco B — “Editor de pista livre”

- spline/Bézier;
- importação SVG/DXF/polilinha;
- regiões poligonais;
- cache/índice compatíveis;
- testes de erro geométrico.

**Saída:** usuário consegue reproduzir praticamente qualquer pista 2D sem aproximá-la manualmente por muitos arcos pequenos.

### Marco C — “Realismo extremo 2.5D”

- drag identificado;
- transmissão elástica opcional;
- modelo óptico com altura/luz;
- pitch/roll/heave reduzidos;
- folga de sucção acoplada.

**Saída:** efeitos verticais entram sem transformar imediatamente o projeto em um simulador 3D genérico.

### Marco D — “Integração de firmware”

- loader opcional de plugin nativo ou processo isolado;
- ABI versionada;
- checkpoint/state contract opcional;
- watchdog e erros explícitos.

**Saída:** algoritmo real pode ser ensaiado no simulador sem contaminar o núcleo físico.

### Marco E — “Escala e otimização”

- profiling automatizado por subsistema;
- benchmarks de regressão;
- kernels em lote/SIMD onde medidos;
- compressão/canais seletivos do replay se I/O virar gargalo;
- C/C++/GPU somente se uma comparação real justificar.

## 17. Critérios de aceitação do projeto

O simulador pode ser considerado arquiteturalmente maduro quando os seguintes contratos forem continuamente verificáveis:

| Área | Critério |
|---|---|
| tempo | 50 us de física / 1 ms de controle produz exatamente 20 integrações por intervalo lógico |
| determinismo | mesma configuração + seed produz o mesmo resultado dentro do contrato numérico |
| GUI | FPS e velocidade do player não alteram o run |
| executor | GUI, CLI, worker, batch e calibração chamam o mesmo núcleo |
| editor | todo parâmetro suportado salvo afeta o runtime ou é marcado como visual/metadado |
| contatos | forças respeitam normal, superfície, slip e limite combinado |
| energia | motor/driver/bateria/fan fecham um balanço documentado dentro da tolerância numérica |
| sensores | controlador recebe apenas observações entregues e temporizadas |
| pista | geometria e propriedades consultadas são as mesmas vistas pelo editor/runtime |
| replay | configuração congelada e canais permitem reproduzir/inspecionar o resultado sem recalcular |
| desempenho | custo por preset é medido; nenhuma redução silenciosa de precisão |
| validação | “realista” é acompanhado de ensaios, faixa de validade e dados independentes |

## 18. Decisões arquiteturais registradas

### ADR-001 — Rust permanece o kernel principal

**Decisão:** manter o núcleo físico em Rust.
**Motivo:** já é código nativo e os benchmarks indicam que os maiores custos são algoritmos específicos, não a linguagem.
**Reavaliar quando:** um perfil e benchmark equivalente demonstrarem ganho material de outro kernel.

### ADR-002 — cálculo e renderização permanecem desacoplados

**Decisão:** resultado físico independe do FPS; simulações pesadas podem ser calculadas antes e reproduzidas depois.
**Motivo:** permite física a 50 us, batch, replay e UI responsiva.

### ADR-003 — tempo científico usa inteiros e fixed-step

**Decisão:** microssegundos inteiros e passo físico fixo.
**Motivo:** determinismo, agenda precisa e comparação entre runs.

### ADR-004 — fidelidade é composição de subsistemas

**Decisão:** presets apenas selecionam modelos/defaults; overrides permanecem possíveis e explícitos.
**Motivo:** um usuário pode querer pneu realista com sensor ideal ou o inverso.

### ADR-005 — física planar é a base

**Decisão:** manter 2D/2.5D como caminho principal; adicionar dinâmica vertical reduzida antes de qualquer engine 3D completa.
**Motivo:** melhor relação custo/benefício para Robotrace.

### ADR-006 — C ABI é integração, não aceleração presumida

**Decisão:** plugin C pode existir para firmware/controlador ou kernel comprovado; não migrar a física inteira para uma DLL apenas por desempenho presumido.

### ADR-007 — todo run usa configuração congelada

**Decisão:** editar a GUI não altera um run em curso.
**Motivo:** reprodutibilidade científica e segurança de batch/calibração.

### ADR-008 — efeitos avançados entram por evidência

**Decisão:** adicionar temperatura de pneu, desgaste, vibração, CFD, BLDC detalhado etc. apenas após observar erro sistemático que o modelo atual não explique.

## 19. Arquivos de referência do projeto

Para detalhes de implementação, esta arquitetura deve ser lida junto com:

- `../tasks.md` — plano e estado das entregas;
- `etapa-1-executor-e-tempo.md` — contrato temporal;
- `etapa-2-dinamica-fisica.md` — base mecânica;
- `etapa-3-configuracao-e-sensores.md` — schemas e portabilidade;
- `etapa-4-montagem-e-editor.md` — montagem e editor do robô;
- `etapa-5-pista-optica-e-corrida.md` — pista e eventos;
- `etapa-6-fidelidade-e-contatos.md` — presets e pneus/contato;
- `etapa-7-motores-alimentacao-e-downforce.md` — cadeia elétrica e normal;
- `etapa-8-sensoriamento-e-controle.md` — aquisição e controlador;
- `etapa-9-calculo-replay-e-experimentos.md` — worker/replay/batch;
- `etapa-9-benchmarks.md` — custos observados;
- `etapa-10-calibracao-e-qualificacao.md` — estado de qualificação;
- `etapa-10-protocolo-experimental.md` — ensaios físicos necessários.

## 20. Resumo executivo

A arquitetura fundamental necessária para o RoboTraceSim **já existe**: núcleo determinístico, física e lógica com períodos diferentes, modelos selecionáveis, editor de robô, editor de pista, execução headless, worker, replay, batch e infraestrutura de calibração.

O caminho correto a partir daqui não é reescrever o simulador em C nem transformar tudo em uma engine 3D. A prioridade é:

```text
1. medir o robô real;
2. qualificar os modelos existentes;
3. otimizar os gargalos medidos;
4. ampliar a liberdade geométrica do editor de pista;
5. adicionar apenas os efeitos físicos que os dados mostrarem necessários.
```

A separação “**calcular o mundo primeiro e apenas reconstruir sua visualização depois**” deve permanecer como um dos contratos centrais do projeto. Ela é compatível com 50 us de física, 1 ms de controle, replays fluidos e modelos futuros mais pesados sem tornar a interface dependente da velocidade do solver.
