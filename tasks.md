# Plano de implementação — RoboTraceSim

Este arquivo transforma o plano de 10 etapas em tarefas executáveis. A referência de diagnóstico e decisões é [Arquitetura do projeto](docs/arquitetura-do-projeto.md), especialmente as lacunas G01–G14. O plano descreve trabalho pendente; sua criação não significa que essas funcionalidades foram implementadas.

## Objetivo e regras de execução

Entregar um simulador configurável de robôs seguidores de linha/Robotrace, com editores de robô e pista, períodos independentes de física e lógica, níveis de fidelidade, reprodução visual e comparação com dados reais. O cenário temporal de referência é **50 µs para física e 1.000 µs para controle**.

- Manter Rust como núcleo principal. Separar cálculo e visualização; considerar DLL/FFI por necessidade comprovada de integração ou desempenho.
- Reutilizar os recursos existentes. Uma tarefa de evolução não implica reescrever o módulo inteiro.
- Não aceitar campos físicos que sejam silenciosamente ignorados. Toda opção deve funcionar, ser rejeitada ou ser identificada como exclusivamente visual.
- Não há obrigação de manter compatibilidade com formatos, APIs ou resultados antigos. Priorizar o modelo correto para o objetivo atual; atualizar exemplos e documentar quebras e conversões disponíveis.
- Não confundir maior complexidade com maior precisão. Modelos avançados precisam de parâmetros identificáveis e validação.
- Os caminhos em **estruturas propostas** ainda não existem necessariamente. Introduzi-los gradualmente, movendo responsabilidades e atualizando imports; não manter duas implementações concorrentes.
- Cada sensor deve gerar seu valor separadamente, conforme sua posição no robô e a posição/orientação do robô na pista. A estimativa de linha usada pelo controlador é um resultado agregado dessas leituras, não substitui os sensores individuais.
- Cada checkbox representa uma entrega verificável. Marcá-lo somente após implementação, verificação apropriada e registro da evidência.
- Efeitos opcionais devem ser marcados como implementados, adiados com justificativa ou não aplicáveis. Não impedem a entrega principal quando não houver necessidade experimental demonstrada.

## Sequência e dependências

| Etapa | Resultado principal | Dependências |
|---|---|---|
| 1 | Executor e agenda temporal únicos | Base atual |
| 2 | Dinâmica básica fisicamente consistente | 1 |
| 3 | Configuração tipada, validada e portável | 1; incorporar os contratos de 2 |
| 4 | Editor de robô fiel ao runtime | 2 e 3 |
| 5 | Pista física/óptica e eventos de corrida | 3 e contratos geométricos de 4 |
| 6 | Fidelidade selecionável e contato por roda | 2, 3, 4 e 5 |
| 7 | Motor, alimentação e downforce acoplados | 3 e 6 |
| 8 | Sensores completos e controle observável | 1, 4, 5 e interfaces de 7 |
| 9 | Worker, replay completo, batch e otimização | 1 e 3 para iniciar; 4–8 para validação final |
| 10 | Modelos calibrados e qualidade demonstrada | 6–9; coleta experimental pode começar antes |

A ordem é de consolidação. A infraestrutura de worker/cache da etapa 9 pode ser antecipada após estabilizar o executor; coleta de dados da etapa 10 também. Esses adiantamentos não substituem os critérios físicos das etapas anteriores.

## Etapa 1 — Unificar núcleo de execução e relógios

**Objetivo:** garantir que CLI, interface e calibração executem a mesma simulação, com a mesma inicialização e ordem de eventos. Resolver principalmente G07/G08 e preparar G09.

### Tarefas

- [x] **1.1 — Extrair uma API de biblioteca.** Criar `src/lib.rs`, expor somente os tipos necessários aos executores e deixar `main.rs` responsável pela escolha entre GUI e CLI. Evitar dependência do núcleo em egui ou dialogs.
- [x] **1.2 — Definir um único dono do estado.** Consolidar estado físico, sensores, controlador, modelos, RNGs e agenda em `SimulationCore` ou equivalente. Transformar `run_simulation` e `SimulationSession` em adaptadores desse núcleo; remover a duplicação da inicialização e do loop.
- [x] **1.3 — Formalizar a ordem temporal.** Definir aquisição, entrega de leituras, execução do controlador, aplicação de comandos, registro e integração. Inicializar cada dispositivo uma única vez em `t=0`; estabelecer uma ordenação estável para eventos simultâneos.
- [x] **1.4 — Validar períodos sem arredondamentos silenciosos.** Aceitar microssegundos inteiros positivos. Inicialmente exigir múltiplos do passo físico para controle, sensores e registro, com mensagem indicando combinações válidas. O período visual não participa dessa restrição.
- [x] **1.5 — Definir duração e amostra final.** Escolher e documentar política para duração não divisível pelo passo: rejeição, arredondamento explícito ou subpasso final. Separar número de integrações do número de amostras; corrigir o atual `steps + 1` no resumo.
- [x] **1.6 — Separar avançar de observar.** Criar operações equivalentes a `step`, `advance_until`, `sample` e `is_finished`. Uma leitura de telemetria não pode avançar RNG, controle ou física. Avançar além do fim deve ser impedido ou ter comportamento explícito.
- [x] **1.7 — Registrar a configuração efetiva.** Guardar passo, períodos, seed e duração realmente usados, inclusive overrides da CLI. Definir se a telemetria representa estado no início/fim do intervalo e a que intervalo suas forças se referem.
- [x] **1.8 — Adaptar os consumidores.** Fazer CLI, calibração e GUI chamarem essa API. A GUI pode continuar síncrona temporariamente, mas não terá uma agenda própria.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/sim.rs`: `SimulationSession`, `run_simulation`, `RobotState`, `RunSummary` | Consolidar executor, inicialização, avanço e contadores |
| `src/config.rs`: `TimeConfig`, `ProjectConfig` | Validação de períodos, duração e configuração efetiva |
| `src/calibration.rs`: `run_simulation_samples` | Coletar dados na mesma fase temporal usada pelo headless |
| `src/ui.rs`: reset/step/play e geração de replay | Consumir a API comum |
| `src/cli.rs`, `src/main.rs`, `Cargo.toml` | Entrada fina, exports de biblioteca e adaptação dos comandos |
| Estruturas propostas | `src/core/{clock,scheduler,state,runner}.rs`, `src/lib.rs` |

### Verificação e conclusão

- [x] Testar 20 integrações físicas de 50 µs por intervalo lógico de 1 ms, distinguindo a chamada inicial de controle em `t=0`.
- [x] Comparar estados e amostras de CLI, sessão incremental e calibração com mesma configuração/seed.
- [x] Testar duração zero, último instante, período inválido, timestamps crescentes e leitura sem efeitos colaterais.
- [x] Demonstrar que diferentes tamanhos de blocos de avanço não alteram o resultado.

**Concluída quando:** existe um executor único e os três consumidores concordam nos mesmos instantes simulados.

### Registro de conclusao

Implementada em 21/09/2026. Evidencias e contrato temporal: [Etapa 1 - executor e tempo](docs/etapa-1-executor-e-tempo.md).

| Tarefa | Evidencia principal |
|---|---|
| 1.1 | `src/lib.rs`, entrada fina em `main.rs`, `eframe`/`rfd` opcionais |
| 1.2 | `SimulationCore` e alias `SimulationSession`; removido o loop fisico duplicado |
| 1.3 | Ordem unica de eventos e teste do primeiro resultado dos RNGs em `sim::timing_tests` |
| 1.4 | Parser temporal estrito e validacao em `core/scheduler.rs`; testes de periodos invalidos |
| 1.5 | `Clock` rejeita duracao fora da grade; testes de zero/final; 43 integracoes e 4 amostras em 2150 us |
| 1.6 | `step`, `advance_until`, `advance_steps`, `sample`, `is_finished`; testes de pureza e blocos |
| 1.7 | `EffectiveRunConfig`, resumo e arquivos `.metadata.json`; override de CLI registrado |
| 1.8 | CLI, GUI e calibracao usam o mesmo core; igualdade de amostras e arquivos testada |

As verificacoes exercitam a API usada pela GUI e sua compilacao; nao substituem uma avaliacao interativa da interface. Equacoes fisicas da etapa 2 permanecem pendentes.

Validacao final: 33 testes aprovados com GUI e 33 sem GUI; `cargo fmt --all -- --check` aprovado. Detalhes no documento da etapa.

## Etapa 2 — Corrigir a dinâmica física básica

**Objetivo:** estabelecer uma base confiável de movimento e energia antes de aumentar a fidelidade. Resolver a base de G02–G05; os modelos completos de contato e circuito serão desenvolvidos nas etapas 6 e 7.

### Tarefas

- [x] **2.1 — Documentar referenciais e sinais.** Fixar X para frente, Y para esquerda e yaw positivo anti-horário, além da origem do corpo e sua relação com o COM. Definir sinais de torque, velocidade, corrente, frenagem e PWM reverso.
- [x] **2.2 — Corrigir a integração no referencial escolhido.** Se velocidades forem armazenadas no corpo, incorporar termos de rotação do referencial; alternativamente integrar velocidade no mundo e transformar para os contatos. Não misturar as duas formulações.
- [x] **2.3 — Separar forças, momentos e integração.** Calcular forças e momentos antes de atualizar o estado. Substituir amortecimentos numéricos arbitrários por parâmetros explícitos quando representarem perdas físicas; identificar separadamente estabilização numérica.
- [x] **2.4 — Revisar a cinemática e reação das rodas.** Tratar parada, reversão, saturação e aderência sem saltos injustificados de energia. Incorporar a inércia equivalente no modo de rolamento aderente, em vez de apenas sobrescrever a velocidade angular sem balanço.
- [x] **2.5 — Corrigir a dependência de tensão do motor simples.** Referenciar torque/velocidade a tensão nominal ou parâmetros equivalentes. Garantir que a queda de tensão afete o motor, sem se cancelar na normalização atual.
- [x] **2.6 — Corrigir os limites de alimentação.** Definir como limite de corrente reduz a atuação. Não limitar apenas a corrente contabilizada pela bateria enquanto os motores mantêm potência incompatível. Registrar a aproximação temporal usada no acoplamento.
- [x] **2.7 — Revisar resistências e repouso.** Impedir que rolamento/amortecimento causem reversão espontânea ou oscilações artificiais perto de zero. Distinguir força dissipativa de restrição de contato.
- [x] **2.8 — Adicionar diagnóstico numérico.** Detectar valores não finitos, estado inválido e divergência, encerrando o run com instante e subsistema responsáveis. Não esconder entrada inválida com pequenos denominadores artificiais.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/sim.rs`: `physics_step`, `update_wheel_kinematics`, `apply_rolling_resistance` | Formulação dinâmica, integração e diagnóstico |
| `src/math.rs`: `Pose2`, `Vec2` | Transformações e convenções testadas |
| `src/wheel.rs`: `TireInput`, `WheelForces` | Contrato de força/reação e transições básicas |
| `src/motor.rs`, `src/battery.rs` | Tensão nominal, sinais, potência e limites consistentes |
| `src/telemetry.rs` | Diagnóstico de balanço e falhas |
| Estruturas propostas | `src/core/integrator.rs`, `src/models/chassis.rs`, cenários em `tests/physics/` |

### Verificação e conclusão

- [x] Testar movimento livre, aceleração por força constante e resposta a torque conhecido.
- [x] Testar curva com demanda centrípeta e transformação entre velocidade no corpo e no mundo.
- [x] Testar parada, reversão, coast, brake e queda de tensão sem geração artificial de energia.
- [x] Comparar passos menores em cenários determinísticos simples e documentar tolerâncias numéricas.

**Concluída quando:** movimento, forças e potência passam nos cenários de referência, sem depender de ajustes do controlador para mascarar erro físico.

### Registro de conclusão da etapa 2

Implementada em 21/09/2026. Contratos, equações, limites e evidências: [Etapa 2 — dinâmica física](docs/etapa-2-dinamica-fisica.md).

| Tarefa | Evidência principal |
|---|---|
| 2.1 | Referenciais, origem/COM, sinais de rodas, PWM, torque e corrente documentados |
| 2.2 | Transporte de velocidade no mundo e recuperação da pose da origem; testes analíticos e de curva |
| 2.3 | `core/integrator.rs` separa solução de impulsos e pose; removido amortecimento arbitrário de yaw |
| 2.4 | Solver conjunto de motores/contatos com inércias e reações; testes de aderência, saturação e reversão |
| 2.5 | `nominal_voltage_v` no parser, modelo, editor, persistência e exemplos; teste de redução de torque |
| 2.6 | Orçamento de corrente altera atuação de motores/downforce; bateria registra consumo efetivo |
| 2.7 | Resistência de rolamento por impulso limitado; testes de repouso, coast e brake sem energia artificial |
| 2.8 | Validação física, balanço de energia, detecção de não finitos/divergência e erro com instante/subsistema |

Verificação: 48 testes (40 unitários e 8 de integração), com e sem GUI. Benchmark de 10 s a 50 µs concluído em aproximadamente 0,364 s no exemplo de sucção. A GUI foi compilada, sem validação interativa. Modelos avançados das etapas 6–7 e validação experimental permanecem pendentes.

## Etapa 3 — Consolidar configurações, schemas e componentes

**Objetivo:** tornar configurações previsíveis, portáveis e evolutivas. Resolver G06/G12 na camada de dados e preparar a seleção real de modelos.

### Tarefas

- [x] **3.1 — Definir tipos de domínio.** Separar definição editável de robô/pista, configuração resolvida do experimento e estado dinâmico. Introduzir IDs estáveis para instâncias de componentes e conexões.
- [x] **3.2 — Substituir seleção ambígua por tipos explícitos.** Usar enums/configurações específicas por modelo. Unificar `model` e `model_kind`; nomes desconhecidos devem retornar erro ou migração conhecida, nunca selecionar arbitrariamente outro modelo.
- [x] **3.3 — Implementar validação semântica.** Verificar finitude, faixas físicas, inteiros temporais, raios/inércias positivos, consistência de unidades, tabelas ordenadas e referências. Diferenciar erro impeditivo de aviso de faixa não calibrada.
- [x] **3.4 — Definir versões e migrações.** Documentar schemas suportados e caminho de atualização. Representar sensores como instâncias independentes, com ID, pose, aquisição e resposta próprios. Conversões de `line_sensor` legado podem facilitar a atualização dos exemplos, mas não existe obrigação de compatibilidade. Documentar versões rejeitadas e mudanças de comportamento.
- [x] **3.5 — Corrigir persistência JSON.** Escolher entre corrigir o parser próprio ou usar biblioteca consolidada; cobrir UTF-8, escapes, números inválidos e round-trip. Levar serialização para fora de `ui.rs`.
- [x] **3.6 — Resolver assets de forma uniforme.** Definir base dos caminhos relativos, detectar arquivos ausentes e remover fallback silencioso para componente padrão. Permitir transportar o projeto com seus assets sem depender do diretório de execução.
- [x] **3.7 — Preparar configuração congelada do run.** Expandir defaults/presets e referências em um `ResolvedExperiment`, preservando valores efetivos e hashes dos arquivos. Alterações no editor não modificam uma execução iniciada.
- [x] **3.8 — Atualizar exemplos e documentação.** Remover caminhos absolutos dos exemplos, unificar versão exibida do produto e documentar unidades de catálogo versus unidades internas. Separar versão do aplicativo da versão de cada formato.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/config.rs`: todos os tipos de configuração e loaders | Validação, enums, resolução, migração e IDs |
| `src/json.rs` | Parser conforme a decisão de persistência |
| `src/ui.rs`: funções `*_json` e `save_*` | Extrair para API de persistência compartilhada |
| `RobotAssets/`, `examples/`, `Tracks/`, `SurfaceProfiles/` | Exemplos migrados, referências e schemas documentados |
| `Cargo.toml`, `README.md` | Dependências necessárias e instruções atualizadas |
| Estruturas propostas | `src/io/{persistence,assets,models,validation,experiment}.rs`, fixtures legadas em `tests/fixtures/` |

### Verificação e conclusão

- [x] Salvar/reabrir projetos preservando todos os parâmetros físicos e IDs.
- [x] Validar o schema atual e a conversão explícita dos exemplos existentes, incluindo as 16 instâncias independentes do antigo array. Não exigir resultados históricos idênticos.
- [x] Testar nomes em português/japonês, curvas inválidas, números não finitos, modelo desconhecido e asset ausente.
- [x] Mover um projeto de teste para outro diretório e executá-lo sem ajustes manuais de caminhos.

**Concluída quando:** a configuração executada é explícita, validada e transportável, com migrações testadas.

### Entrega verificada em 22/09/2026

| Item | Implementação e evidência |
|---|---|
| 3.1 | Definição editável separada de `ResolvedExperiment` e estado do core; IDs e conexões validados, com testes de duplicação e persistência. |
| 3.2 | `io/models.rs` resolve modelos conhecidos; `NormalForceKind` substitui a seleção duplicada. Nomes desconhecidos geram erro. |
| 3.3 | Validação antes de salvar/executar: tipos, finitude, faixas, curvas, unidades e referências; avisos separados de erros impeditivos. |
| 3.4 | Robô v7 com sensores independentes; conversão explícita do array antigo para 16 instâncias testada. Sem obrigação de compatibilidade histórica. |
| 3.5 | Parser corrigido para Unicode, escapes, surrogates e sintaxe estrita; serialização extraída de `ui.rs` para `io/persistence.rs`, com round-trip. |
| 3.6 | Assets relativos ao arquivo do robô, referências ausentes rejeitadas; `save_project_bundle` incorpora sensores. Projeto movido e executado em teste. |
| 3.7 | Core executa cópia congelada, com defaults e overrides resolvidos; sidecars `.experiment.json` registram definições, conexões e fingerprints FNV-1a. |
| 3.8 | Exemplos atualizados, caminhos relativos, versão 0.6.0 derivada do Cargo na interface; schemas e unidades documentados. |

Passaram **59 testes** (40 unitários + 8 da etapa 1 + 11 da etapa 3), com e sem a feature GUI. A execução CLI de 2 ms completou 40 passos de 50 µs, com controle de 1 ms, e gerou CSV/replay com snapshots de configuração conferidos. Formatação e whitespace verificados. A GUI compilou; não houve teste interativo.

Contrato e limitações: [Configuração e sensores independentes](docs/etapa-3-configuracao-e-sensores.md). A leitura por posição foi antecipada da etapa 4; área óptica, latência, preview integrado e sensores avançados continuam pendentes nas etapas 4/8. A exportação portátil é uma API, ainda sem botão dedicado.

## Etapa 4 — Completar o editor de robô e a montagem física

**Objetivo:** garantir que posições e características editadas representem os mesmos componentes usados na simulação. Resolver a estrutura de G01/G04 e preparar contatos/sensores avançados.

### Tarefas

- [x] **4.1 — Introduzir instâncias de rodas e apoios.** Definir posição/orientação, raio, largura, inércia, material/pneu, motor associado e tipo motriz/passivo/caster. Migrar o drivetrain agregado atual para uma montagem equivalente explicitamente identificada.
- [x] **4.2 — Modelar distribuição de massa.** Permitir massa e posição por componente, incluindo bateria, motores e fans. Calcular COM/inércia quando solicitado e permitir valores medidos como override; impedir dupla contagem entre massa global e componentes.
- [x] **4.3 — Incluir altura e referência geométrica.** Acrescentar COM Z, altura dos sensores e apoios. Documentar origem do desenho, origem dinâmica e transformações entre elas; não habilitar efeitos verticais sem modelo compatível.
- [x] **4.4 — Completar a montagem de sensores independentes.** Cada sensor é um componente separado com ID, pose e leitura próprios. Calcular sua posição mundial a partir da pose no projeto do robô e da pose atual do robô na pista; consultar a pista nesse ponto e aplicar sua resposta/aquisição individual. Não reduzir a montagem a uma barra uniforme nem copiar leituras de outro sensor. A leitura pontual antecipada na etapa 3 deve ser integrada e verificada no editor/preview; área de detecção, orientação óptica, latência e resposta completa ficam na etapa 8.
- [x] **4.5 — Evoluir ferramentas de edição.** Inserir, mover, girar, duplicar e remover componentes, com seleção, alinhamento, medidas e undo/redo. Reutilizar preview e biblioteca de assets existentes.
- [x] **4.6 — Mostrar implicações físicas.** Desenhar COM, polígono de apoio, contatos, área dos sensores e posição de fans. Identificar parâmetros exclusivamente visuais e indicar quais modelos participam do experimento.
- [x] **4.7 — Validar a montagem.** Detectar componentes sem referência, associação de motor inválida, geometria degenerada e montagem incompatível com o solver. Usar avisos para situações fisicamente possíveis, mas fora da faixa de suporte.
- [x] **4.8 — Separar o painel do domínio.** Extrair editor/preview de robô de `ui.rs`, mantendo cálculos de massa, transformações e validações em módulos testáveis sem GUI.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/config.rs`: `RobotConfig`, `ChassisConfig`, `DrivetrainConfig`, `RobotSensorInstance`, `FanConfig` | Montagem por instâncias, COM XYZ e referências |
| `src/ui.rs`: editor, `draw_robot_preview`, editores de sensores/assets | Manipulação e visualização da montagem |
| `src/sim.rs`: `RobotState`, construção de modelos e `runtime_line_sensor_config` | Estados por instância e remoção da redução para barra uniforme |
| `src/sensor.rs`, `src/normal_force.rs`, `src/wheel.rs` | Consumir poses e contatos definidos no robô |
| `RobotAssets/`, `Robots/`, exemplos de robô | Componentes e montagens de referência |
| Estruturas propostas | `src/models/robot.rs`, `src/app/robot_editor.rs`, tipos `WheelInstance` e `MassProperties` |

### Verificação e conclusão

- [x] Comparar posição mundial desenhada e calculada de sensores/rodas em várias poses do robô.
- [x] Testar sensores assimétricos e girados, rodas com raios diferentes e COM deslocado.
- [x] Validar soma de massas/inércia contra um caso analítico e testar override sem dupla contagem.
- [x] Verificar save/load e undo/redo sem perda de IDs, assets ou características físicas.

**Concluída quando:** modificar a montagem altera o runtime pela mesma geometria usada no preview.

### Entrega verificada em 22/09/2026

| Item | Implementação e evidência |
|---|---|
| 4.1 | `WheelInstance` com pose, tipo, motor, dimensões, inércia, material e pneu. Conversão para quatro rodas preserva geometria e inércia equivalente por lado. |
| 4.2 | Modos exclusivos medido/componentes; massa, COM XYZ e inércia por eixos paralelos. Referências repetidas são rejeitadas; caso analítico e override testados. |
| 4.3 | Alturas de COM, sensores e contatos persistidas; origem do corpo e transformação documentadas. Efeitos verticais não são ativados. |
| 4.4 | Sensor individual usa transformação compartilhada com o desenho. Preview da sessão usa montagem congelada; sensores assimétricos e poses giradas verificados. |
| 4.5 | Seleção por ID/canvas, inserir, mover, girar, duplicar, remover, alinhar aos eixos, grade de 1 mm, medidas e histórico completo de até 100 operações. |
| 4.6 | Preview mostra contatos, polígono de apoio, COM efetivo, áreas dos sensores e fans; painel explica efeitos ativos e metadados. |
| 4.7 | Validação de IDs, referências, massa, geometria e capacidade do solver. Configurações não suportadas podem ser editadas, mas não executadas silenciosamente. |
| 4.8 | Painel/preview extraídos para `src/app/robot_editor.rs`; montagem, massa, transformações, histórico e validação em `src/models/robot.rs`, sem dependência da GUI. |

Passaram **69 testes sem GUI e 70 com GUI**. Dez testes novos cobrem montagem, massa, poses, persistência e histórico; um teste adicional renderiza o editor/previews em egui sem abrir janela. Execução de 100 ms com física de 50 µs e controle de 1 ms completou 2.000 passos e gerou CSV, replay e snapshot da montagem. Não houve ensaio manual na janela nativa.

**Limite explícito:** o solver desta etapa ainda requer quatro rodas motrizes simétricas. Rodas com raios diferentes, apoios passivos/caster dinâmicos e contatos independentes são representados e validados, mas sua execução continua na etapa 6. Altura óptica, área e latência permanecem na etapa 8. Esses limites não são aproximados silenciosamente. Contrato completo: [Montagem física e editor](docs/etapa-4-montagem-e-editor.md).

## Etapa 5 — Completar a pista física, óptica e de corrida

**Objetivo:** fazer a pista influenciar sensores, contatos e regras de execução, além de ser desenhada. Resolver G11 e a parte geométrica de G10.

### Tarefas

- [x] **5.1 — Separar definição e representação de execução.** Preservar retas/arcos como fonte de verdade e construir um `TrackRuntime` imutável com geometria derivada. Reconstruir caches somente quando a definição mudar.
- [x] **5.2 — Criar camadas de superfície.** Representar material, atrito e refletância por região, separadamente da cor de exibição. Definir precedência para regiões sobrepostas e propriedades fora da área da pista.
- [x] **5.3 — Incorporar marcações ao sinal óptico.** Fazer largada/chegada, marcas laterais, falhas de linha e cruzamentos participarem da consulta de refletância. Reutilizar a mesma definição que o canvas desenha.
- [x] **5.4 — Preparar consultas por contato.** Expor material, atrito e, futuramente, altura/normal da superfície para cada roda. Eliminar a dependência de consultar toda a superfície apenas no centro do chassi.
- [x] **5.5 — Definir discretização e erro.** Separar qualidade do desenho, amostragem geométrica e campo óptico. Documentar resolução dos caches e testar sensibilidade em curvas apertadas e linhas estreitas; não mudar a física ao alterar zoom.
- [x] **5.6 — Implementar eventos de corrida.** Detectar início, checkpoints, volta, chegada e saída de área. Usar direção de passagem e estado da corrida para evitar contagem duplicada/reversa. Separar perda óptica de linha da invalidade geométrica do robô.
- [x] **5.7 — Evoluir o editor.** Adicionar edição de regiões/marcas, reordenação segura, snapping e undo/redo. Unificar a relação entre pose inicial da pista e pose do projeto, mostrando a opção efetivamente usada.
- [x] **5.8 — Validar e contextualizar regras.** Manter modos strict/warning/free, mas impedir configurações numericamente inválidas em todos eles. Associar edição/fonte aos perfis de regulamento antes de afirmar conformidade oficial.
- [x] **5.9 — Preparar relevo opcional.** Definir formato de altura/rugosidade e aviso de compatibilidade. Ativar efeitos verticais somente após existir solver apropriado; pistas planas continuam sendo a primeira entrega.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/rtsim_track.rs`: `TrackV2`, `TrackGeometry`, marcações e validadores | Fonte paramétrica, cache, regiões e eventos geométricos |
| `src/track.rs`: `TrackModel`, `VectorTrack` | Consultas ópticas/físicas com propriedades locais |
| `src/config.rs`: `TrackConfig`, `SurfaceProfile` | Novas camadas, persistência e resolução de assets |
| `src/ui.rs`: editor de pista, canvas e largada | Edição coerente com o runtime |
| `src/sim.rs`, `src/telemetry.rs` | Eventos de corrida, motivo de término e consulta por contato |
| Estruturas propostas | `src/track/{definition,runtime,persistence,events}.rs`, `src/app/track_editor.rs` |

### Verificação e conclusão

- [x] Detectar marca desenhada em uma varredura óptica de referência.
- [x] Simular duas rodas sobre materiais distintos e obter propriedades diferentes nos contatos.
- [x] Testar fechamento, cruzamento, retorno por uma marca e sequência inválida de checkpoints.
- [x] Demonstrar que o runtime reutiliza geometria e que zoom/FPS não altera suas consultas.

**Concluída quando:** a mesma pista alimenta editor, sensores, contato e eventos de corrida de forma consistente.

### Registro da entrega — 22/09/2026

| Item | Implementação e evidência |
|---|---|
| 5.1 | Runtime imutável compartilhado por `Arc`, cache por definição serializada, consultas sem reconstrução; teste de identidade e invalidação. |
| 5.2 | Regiões orientadas com material, atrito, refletância e RGB independentes; última região prevalece, exterior explícito. |
| 5.3 | Marcas automáticas/editadas e falhas consultadas pelos sensores e desenhadas pelo mesmo runtime; cruzamentos como união das faixas. |
| 5.4 | Consulta mundial por roda e limite de atrito local aplicado à física; teste com materiais distintos e resposta assimétrica. |
| 5.5 | Distância analítica a retas/arcos; cache geométrico de até 5 mm e erro de corda documentado. Testes com linha estreita, curva apertada e zoom. |
| 5.6 | Largada, checkpoints ordenados, voltas, chegada, retorno e saída; resumo de término e `.events.json`. Teste de término antecipado e tempo efetivo. |
| 5.7 | Editor de camadas/portais, reordenação por IDs estáveis, snapping e histórico de pista/pose. Fonte da pose inicial explícita e compartilhada entre GUI/CLI. |
| 5.8 | Validade numérica obrigatória em todos os modos; fonte/edição persistidas nos perfis. Ausência de referência gera aviso, sem alegar homologação. |
| 5.9 | Altura/rugosidade em formato persistente; aviso para metadados não nulos e rejeição de ativação sem solver vertical. |

**Verificação concluída:** 85 testes sem GUI (`cargo test --offline --no-default-features`) e 87 com GUI (`cargo test --offline`). Dezesseis testes de integração novos em `tests/stage5_track.rs` e teste de renderização do editor/canvas sem janela. Formatação geral, formatação explícita do arquivo incluído do editor e `git diff --check` aprovados. Compilação ainda emite avisos de código não utilizado; não houve teste manual da janela nativa.

[Exemplo](examples/stage5/projeto.rtsim) executado com física de 50 µs e controle de 1 ms: 60.000 passos, 3.001 amostras, largada registrada e término por duração de 3 s. CSV, replay, snapshot, metadados e eventos foram verificados em `target/`. O caso automatizado de corrida confirma término antecipado por chegada.

**Limites preservados:** forças ainda agregadas por lado; contato independente/aderência combinada são etapa 6. Altura/rugosidade não ativam dinâmica vertical. Óptica continua pontual até a etapa 8. Eventos estão em sidecar; índice de replay, worker e otimizações adicionais ficam na etapa 9. Ferramentas auxiliares do editor ainda podem calcular geometria própria; consultas físicas/ópticas usam o snapshot. Contrato completo: [Pista física, óptica e corrida](docs/etapa-5-pista-optica-e-corrida.md).

## Etapa 6 — Implementar níveis de fidelidade e contato por roda

**Objetivo:** permitir escolher a complexidade física sem alterar a estrutura do experimento, com modelos de contato adequados ao seguidor de linha.

### Tarefas

- [x] **6.1 — Criar um registro de modelos disponíveis.** Cada modelo declara parâmetros, estado interno, dependências e limitações. O carregamento deve rejeitar opções sem implementação ou combinações incompatíveis.
- [x] **6.2 — Implementar presets explícitos.** Ideal: cinemática/rolamento perfeito. Simplificado: corpo planar, Coulomb e perdas. Realista: modelos calibráveis avançados. Habilitar overrides por subsistema e salvar a expansão efetiva do preset.
- [x] **6.3 — Resolver contato individual.** Calcular velocidade no contato a partir da velocidade do corpo e da posição da roda, transformando para os eixos da roda. Somar forças e momentos de todos os contatos no corpo.
- [x] **6.4 — Introduzir leis força-slip.** Modelar força longitudinal e lateral a partir de slip e ângulo de deriva, com tratamento de velocidade próxima de zero. Implementar transição aderente/deslizante sem depender apenas do torque solicitado.
- [x] **6.5 — Implementar aderência combinada.** Acoplar tração/frenagem e curva em um orçamento de contato. Tratar normal zero sem divisão inválida e evitar permitir simultaneamente o máximo integral de força em ambos os eixos.
- [x] **6.6 — Evoluir a normal por apoio.** Incluir carga estática e transferência quase estática por aceleração/COM. Detectar perda de contato e equilíbrio impossível; não redistribuir cargas negativas arbitrariamente sem política documentada.
- [x] **6.7 — Acrescentar rolamento e rodas passivas.** Modelar perdas, caster e scrub de montagens com quatro rodas conforme o tipo selecionado. Preservar opção ideal para testes de controle.
- [x] **6.8 — Implementar refinamentos reduzidos opcionais.** Adicionar sensibilidade à carga, rigidez radial/tangencial e relaxação quando houver parâmetros utilizáveis. Se houver deformação vertical dinâmica, incluir estados e integração compatíveis; não tratar cálculo estático de normal como dinâmica vertical completa.
- [x] **6.9 — Expor estado e custo.** Registrar slip, forças, normal e regime de contato por roda. Mostrar modelos ativos no painel do experimento e medir seu custo por cenário.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/wheel.rs`: `TireModel`, `TireInput`, `WheelForces` | Forças longitudinais/laterais, momentos e estado transitório |
| `src/normal_force.rs` | Distribuição por contato e transferência de carga |
| `src/sim.rs` | Acúmulo de forças/momentos e estado individual das rodas |
| `src/config.rs`: pneus, chassi e drivetrain | Seleção de modelos e compatibilidade |
| `src/telemetry.rs`, `src/replay.rs`, `src/ui.rs` | Canais por roda, seleção e diagnóstico |
| Estruturas propostas | `src/models/{contact,chassis,fidelity}.rs` (pneu em `contact.rs`), presets em `examples/physics/` |

### Verificação e conclusão

- [x] Demonstrar diferenças previstas entre Ideal e Simplificado no mesmo cenário.
- [x] Testar tração/frenagem em curva, baixa velocidade, normal zero, reversão e contato assimétrico.
- [x] Verificar soma de forças/momentos, dissipação e transferência de carga contra casos de referência.
- [x] Mostrar que desacoplar um efeito opcional reproduz o modelo mais simples correspondente dentro da tolerância definida.

**Concluída quando:** os níveis selecionam comportamentos físicos reais e o contato por roda explica limites de aceleração, frenagem e curva. O preset Realista ainda depende da qualificação experimental da etapa 10.

### Registro da entrega — 22/09/2026

| Item | Implementação e evidência |
|---|---|
| 6.1 | Registro com parâmetros/estados/dependências/limitações; seleção tipada e rejeição de modelos, campos e combinações sem suporte. |
| 6.2 | Ideal cinemático, Simplificado Coulomb e Realista viscoelástico reduzido; overrides por contato/normal/rolamento/refinamentos; expansão salva no snapshot. Testes demonstram movimentos diferentes e redução ao Simplificado. |
| 6.3 | Estado angular por roda, transformação da velocidade local, forças longitudinais/laterais e momentos sobre o COM. Teste verifica os balanços de força e momento em curva. |
| 6.4 | Aderência/deslizamento por velocidade relativa; lei reduzida força–slip com relaxação opcional, slip/deriva regularizados em baixa velocidade. Casos de partida, reversão e dissipação. |
| 6.5 | Elipse conjunta de aderência por roda, limites locais de pneu/superfície e tratamento de normal zero. Teste impede usar máximos integrais nos dois eixos ao mesmo tempo. |
| 6.6 | Equilíbrio não negativo com força/momentos conservados, política explícita de mínima soma de cargas ao quadrado e transferência quase estática por aceleração/altura do COM. Apoio impossível encerra o run. Momento de downforce não é recortado. |
| 6.7 | Rolamento dissipativo implícito, rodas passivas, caster com alinhamento reduzido e scrub dos contatos fixos. Ideal suprime scrub como hipótese explícita de cinemática. |
| 6.8 | Sensibilidade à carga, resposta tangencial/relaxação e compressão radial quase estática opcionais. Efeitos desativáveis, energia de relaxação considerada e compressão fora do domínio rejeitada. Não foi introduzida dinâmica vertical. |
| 6.9 | Estado/forças/slip/normal/regime por roda no painel e `.contacts.csv` junto a CSV/replay, modelos nos metadados e parâmetros efetivos no snapshot. Três presets medidos em release. |

**Verificação concluída:** 102 testes sem GUI e 105 com GUI, incluindo sete testes diretos do novo solver, dez testes de integração da etapa e um teste adicional de renderização dos três presets sem janela. Passaram `cargo test --offline --no-default-features`, `cargo test --offline`, formatação geral e dos editores incluídos, e `git diff --check`. Permanecem avisos de código não utilizado. Não houve ensaio manual da janela nativa.

Execução final do exemplo Simplificado: 100 ms, física de 50 µs, controle de 1 ms, 2.000 passos e 101 amostras. CSV/replay, metadados e snapshots conferidos; cada arquivo acompanhante de contatos contém 404 registros (quatro rodas × 101 instantes), com IDs preservados e valores finitos.

**Custo medido:** mediana de três execuções release de 100 ms simulados, sem GUI/logs: Ideal 27,08× tempo real; Simplificado 0,40×; Realista reduzido 1,63×. O Coulomb rígido ainda fica abaixo de tempo real neste caso com atrito assimétrico; otimização do solver e separação cálculo/reprodução são prioridades da etapa 9. A precisão não foi reduzida para ocultar esse custo.

**Limites explícitos:** 3–8 apoios no plano; caster sem inércia do pivô/shimmy; compressão radial estática sem suspensão vertical; aceleração defasada em um tick na normal quase estática. Motores compartilhados usam divisão igual de torque e média de velocidade, sem transmissão rígida. Ideal não calcula forças/consumo dos motores. A referência agregada anterior permanece como opção identificada para comparação. Dados individuais acompanham o replay em arquivo separado; reprodução integrada desses canais é etapa 9. Realista exige calibração da etapa 10.

Contrato, equações, exemplos e medições: [Fidelidade e contatos por roda](docs/etapa-6-fidelidade-e-contatos.md).

## Etapa 7 — Evoluir motores, transmissão, alimentação e downforce

**Objetivo:** representar de forma acoplada a cadeia bateria → driver → motor → transmissão → roda, incluindo consumo e atuação de fans/sucção.

### Tarefas

- [x] **7.1 — Introduzir motor DC com estado elétrico.** Adicionar R, L, Ke, Kt e corrente assinada, além de inércia/perdas do rotor. Separar o motor simples e o elétrico, ambos com unidades e domínio de validade documentados.
- [x] **7.2 — Definir transmissão explícita.** Distinguir eixo do motor e eixo da roda, redução, eficiência e inércia refletida. Evitar aplicar novamente a redução a parâmetros de catálogo medidos na saída. Acrescentar backlash/elasticidade como opção posterior mensurável.
- [x] **7.3 — Separar driver do motor.** Modelar tensão média, resolução, deadband, brake/coast, perdas e corrente limite. Transformar modo de frenagem e latência em comandos explícitos, quando suportados.
- [x] **7.4 — Resolver o circuito de alimentação.** Determinar tensão e corrente coerentes entre consumidores e bateria. Definir estratégia de solução, tolerância/limite de iteração e política de falha. Incluir consumo de lógica/sensores quando habilitado.
- [x] **7.5 — Evoluir bateria e barramentos.** Permitir OCV×SoC medida, resistência variável e recuperação RC. Preparar barramentos separados, reguladores e perdas de fiação como extensões opcionais; conservar um modo de fonte ideal.
- [x] **7.6 — Modelar frenagem e proteção.** Distinguir energia dissipada de energia regenerada. Representar sobrecorrente/subtensão com estados e eventos; não supor que toda frenagem retorna energia à bateria.
- [x] **7.7 — Unificar modelos de fan/downforce.** Conectar o tipo selecionado às equações realmente executadas. Usar curvas coerentes de força/corrente/RPM por tensão e resposta dinâmica; respeitar limites configurados de PWM.
- [x] **7.8 — Evoluir sucção.** Introduzir pressão/vazamento reduzidos com dependência de folga/altura quando suportada pelo corpo. Preservar a aproximação simples por pressão×área. Relacionar força, consumo e posição de aplicação.
- [x] **7.9 — Preparar térmica e modelos especiais.** Definir estados térmicos lentos e parâmetros calibráveis. BLDC médio, FOC e PWM explícito são opcionais; chaveamento deve ter subpassos/eventos próprios, pois 50 µs não resolve bordas de um PWM de 20 kHz.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/motor.rs`: `MotorModel`, `DcMotorSimple`, `MotorOutput` | Motor com estado, dados elétricos e separação de driver/transmissão |
| `src/battery.rs`: `VoltageSagBattery`, `BatteryOutput` | Modelos selecionáveis e circuito coerente |
| `src/normal_force.rs`: `ConfiguredNormalForce` | Fan/sucção com seleção efetiva e acoplamento energético |
| `src/config.rs`, `RobotAssets/Motors`, `Drivers`, `Batteries`, `Fans` | Parâmetros, curvas e versões |
| Núcleo/scheduler, telemetria e replay | Estados elétricos, subpassos, potência, SoC e eventos |
| Estruturas implementadas | `src/models/{electrical,power}.rs`, `src/core/power_step.rs`, extensões em `motor.rs`, `contact.rs` e `normal_force.rs` |

### Verificação e conclusão

- [x] Comparar resposta de corrente de um motor DC com um caso analítico de primeira ordem.
- [x] Testar queda de tensão, corrente limite, brake/coast/reversão e destino da energia.
- [x] Testar degraus de fan/sucção, variação de tensão e influência da posição na normal.
- [x] Medir estabilidade/convergência com diferentes passos e verificar balanço de potência.

**Concluída quando:** torque, corrente, tensão, força adicional e energia são coerentes dentro das aproximações de cada modelo.

**Entrega verificada em 23/09/2026:** 117 testes sem GUI e 121 com GUI; 12 testes de integração novos, três de fan/sucção e um de renderização do editor elétrico. Exemplo release: 400 passos a 50 µs, controle a 1 ms, CSV/replay e registros de potência finitos. Benchmark curto: 0,10× tempo real; otimização fica na etapa 9.

**Limites explícitos:** uma roda motriz por motor; barramento único; regulação/dissipador ideais; transmissão rígida; fan com RPM equivalente; sucção com folga fixa; sem dinâmica vertical, BLDC/FOC ou chaveamento. Backlash, multibarramentos e modelos especiais estão preparados como extensões descritas, não anunciados como implementados. Contratos, equações, evidências e estruturas: [Motores, alimentação e downforce](docs/etapa-7-motores-alimentacao-e-downforce.md).

## Etapa 8 — Completar sensoriamento e lógica de controle

**Objetivo:** executar controladores com leituras equivalentes às que o robô real pode obter. Completar G01 e ampliar a fronteira de controle.

### Tarefas

- [x] **8.1 — Implementar pipeline óptico individual.** Transformar a pose de cada sensor, integrar refletância sobre sua área, aplicar resposta, ruído, filtro, ADC e latência. Preservar IDs, ordem dos canais e parâmetros por dispositivo.
- [x] **8.2 — Diferenciar saídas e modelos.** Implementar sensor analógico e digital com limiar/histerese quando configurados. Aplicar respostas linear, polinomial ou tabela apenas se implementadas; identificar tipos como ToF sem suporte em vez de simulá-los como linha.
- [x] **8.3 — Modelar aquisição e entrega.** Separar instante de amostragem e disponibilidade, com retenção, conversão, leitura sequencial/multiplexada e atraso configurável. Entregar idade e validade da leitura ao controlador.
- [x] **8.4 — Evoluir encoder e IMU.** Distinguir encoder no motor/roda, resolução efetiva e quadratura. Adicionar filtro/latência e, conforme necessidade, perdas de pulsos, acelerômetro, bias/drift e desalinhamento.
- [x] **8.5 — Organizar ruído determinístico.** Usar seeds independentes por dispositivo, documentando se parâmetros representam ruído por amostra ou densidade espectral. Desabilitar um sensor não deve alterar a sequência de ruído dos demais.
- [x] **8.6 — Ampliar entrada/saída do controlador.** Introduzir `SensorFrame`, `ControllerInput` e comandos por atuador com timestamp. Expor apenas leituras disponíveis ao robô; ground truth fica em modo de depuração explicitamente identificado.
- [x] **8.7 — Evoluir controladores internos.** Adicionar anti-windup, filtro de derivada, controle de velocidade das rodas e comportamento de perda/recuperação de linha. Garantir que limites do atuador sejam observáveis pela lógica quando aplicável.
- [x] **8.8 — Adicionar estimação e corrida.** Implementar odometria de encoder, fusão com gyro e reconhecimento de marcas. Depois, permitir mapa da primeira volta e perfil de velocidade subsequente, mantendo regras e observabilidade.
- [x] **8.9 — Criar adaptadores de lógica.** Implementar `ReplayController` para repetir comandos medidos e definir API para firmware nativo. Plugin em DLL é opcional até existir código real a integrar; especificar versão ABI, buffers, erros e ciclo de vida antes de carregar bibliotecas.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/sensor.rs`: `SensorModel`, `SimpleLineSensor`, `SensorOutput` | Pipeline por instância e formas de resposta |
| `src/encoder.rs`, `src/gyro.rs`, `src/rng.rs` | Tempo, filtros, ruído e modalidades físicas |
| `src/controller.rs`: `Controller`, `BuiltInPid`, `ControllerOutput` | Entrada observável completa e controle por atuador |
| Scheduler, configuração e editor | Agenda por dispositivo e parâmetros efetivos |
| Telemetria, replay e calibração | Raw/filtrado, aquisição/entrega, estado estimado e comandos |
| Estruturas implementadas | `src/models/sensing.rs`, `src/control/{mod,estimator,replay_controller,native}.rs`; PID em `src/controller.rs` e pipeline em `src/sensor.rs` |

### Verificação e conclusão

- [x] Comparar varreduras ópticas com respostas conhecidas para diferentes posições/áreas/ângulos.
- [x] Testar atraso de entrega, ordem de canais, amostra retida, saturação e limiar digital.
- [x] Verificar repetibilidade por dispositivo e que o controlador não usa leituras futuras ou ground truth indevido.
- [x] Demonstrar controle de velocidade, recuperação de linha e odometria em cenários específicos.

**Concluída quando:** a lógica pode ser testada sobre sensores e atuadores com os mesmos contratos temporais e observáveis esperados no robô real.

**Entrega verificada em 23/09/2026:** 135 testes sem GUI e 140 com GUI; 18 testes de integração novos e um de renderização do painel de aquisição. Exemplo release: 40.000 passos a 50 µs, controle a 1 ms, 2.001 observações e 8.004 registros ópticos sem entrega futura. Controle de velocidade chegou a 0,038488 m/s para alvo 0,04 m/s; replay de comandos reproduziu o ensaio verificado.

**Limites explícitos:** integração óptica por quadratura finita, sem óptica 3D; multiplexação por slots estáticos; encoder mede deslocamento entre amostras; IMU planar; fusão e reconhecimento de marcas reduzidos, separados das regras reais de corrida. Firmware nativo tem adaptador em processo e ABI C documentada, sem carregamento de DLL. Novos estados seguem em sidecars até a evolução do replay na etapa 9. Contratos, parâmetros e evidências: [Sensoriamento e controle observável](docs/etapa-8-sensoriamento-e-controle.md).

## Etapa 9 — Separar cálculo, visualização e experimentação

**Objetivo:** entregar cálculo antecipado, preview responsivo, replay navegável e execução em lote, medindo gargalos antes de mudanças de linguagem. Resolver G09/G10/G13.

### Tarefas

- [x] **9.1 — Implementar worker de simulação.** Executar o runner fora da thread gráfica, com mensagens de início, pausa, retomada, cancelamento e progresso. Garantir encerramento limpo ao fechar a aplicação e não criar inicialização automática no Windows.
- [x] **9.2 — Congelar cada execução.** Associar run ID à configuração resolvida. Alterações no editor exigem novo run ou intervenção explicitamente registrada; uma execução não pode consumir arquivos editados no meio do cálculo.
- [x] **9.3 — Separar canais científicos e visuais.** Publicar snapshots de preview em buffer limitado. Permitir descartar frames visuais atrasados, mas nunca passos físicos ou logs solicitados. Definir backpressure da gravação sem alterar o tempo simulado.
- [x] **9.4 — Implementar calcular e reproduzir.** Calcular primeiro e abrir o resultado no player. Acrescentar reprodução por tempo, velocidade, pausa, seek e interpolação visual de pose/yaw. Flags/eventos/comandos não são interpolados como grandezas contínuas.
- [x] **9.5 — Evoluir formato de replay.** Incluir configuração efetiva, versões, modelos, hashes, seeds, descrição de canais e motivo de término. Adicionar blocos, índice temporal, integridade e detecção explícita de arquivo incompleto; definir migração/leitura do v3.
- [x] **9.6 — Separar replay e checkpoint.** Replay reconstrói observáveis; checkpoint retoma a física e precisa dos estados de solver, RNG, controlador, filtros, motores, bateria e agenda. Implementar retomada somente após definir e testar esse contrato completo.
- [x] **9.7 — Otimizar o caminho quente.** Eliminar geometria reconstruída por passo, indexar consultas, pré-alocar buffers e evitar construir telemetria em toda integração. Resolver strings de modelo antes do loop e separar resolução visual da física.
- [x] **9.8 — Implementar batch e varreduras.** Substituir o comando planejado por execução de múltiplos experimentos/seeds, com resultados em diretórios distintos, limite de concorrência, cancelamento e resumo por run. Paralelizar experimentos independentes primeiro.
- [x] **9.9 — Criar benchmark representativo.** Cobrir CLI/sessão/worker, logs ligados/desligados, pistas complexas e números variados de sensores/contatos. Medir tempo total e do loop, memória, velocidade de gravação e responsividade.
- [x] **9.10 — Avaliar alternativas somente com evidência.** Experimentar otimizações de compilação, organização de dados e kernels específicos. Considerar C/DLL, SIMD/GPU ou processo separado apenas com perfil de custo e comparação de resultados; integração por ABI não implica aceleração.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/ui.rs`: simulação, replay, comparação e tune | Jobs assíncronos e player desacoplado |
| `src/sim.rs`, futuro `SimulationRunner` | Avanço em blocos, cancelamento e observação sem alocação desnecessária |
| `src/replay.rs`, `src/telemetry.rs` | Metadados, canais/blocos, leitura sob demanda e evolução de formato |
| `src/track.rs`, `src/rtsim_track.rs` | Cache e índice espacial compartilhados |
| `src/cli.rs`, `src/calibration.rs`, `Cargo.toml` | Batch, execução de jobs e perfil de benchmark |
| Estruturas propostas | `src/app/{jobs,replay_viewer}.rs`, `src/io/{replay,checkpoint}/`, `src/experiments/{batch,benchmark}.rs` |

### Verificação e conclusão

- [x] Rodar o mesmo experimento com diferentes FPS, pausas visuais e tamanhos de buffer; comparar resultados físicos.
- [x] Cancelar cálculo, gravação e batch sem corromper resultados concluídos ou deixar processos recorrentes.
- [x] Navegar em replay maior que a memória destinada ao viewer, sem truncamento silencioso.
- [x] Testar arquivo incompleto, versão incompatível e checkpoint restaurado contra execução contínua, se retomada for entregue.
- [x] Documentar mediana/dispersão dos benchmarks em máquina de referência, com meta inicial de pelo menos 1× a 50 µs no preset Simplificado.

**Concluída quando:** cálculo e apresentação são independentes, a UI permanece responsiva e o custo é conhecido para os cenários suportados.

### Entrega verificada — 23/09/2026

Concluídas 9.1–9.10 e as verificações acima. Worker com configuração congelada/run ID, preview limitado, pausa/passo/cancelamento, jobs auxiliares, calcular/reproduzir, replay v4 com índice/cache/integridade, batch e varreduras estão implementados. Checkpoint entregue somente em memória, com retomada testada e recusa explícita para firmware externo.

Validação: **149 testes sem GUI e 155 com GUI**, incluindo 12 testes de integração da etapa 9, equivalência do kernel de atrito, índice espacial e smoke test egui de simulação/replay. Batch de demonstração: oito runs completos de 2.000 passos, em diretórios separados. Formatação e verificação de diferenças sem erros.

Benchmark release a 50 µs: Simplificado original **1,87× sem logs / 1,59× com logs** no runner, CLI independente **1,86×**. Matriz de 126 tentativas: 111 completas, 15 com falha de convergência no caso de oito contatos sob controle fechado; oito contatos em linha reta concluíram. Alimentação elétrica e óptica densa continuam abaixo de tempo real. Não se reduziu precisão para ocultar falhas. Thin LTO teve ganho pequeno; permanece Rust com compilação padrão, sem DLL de física.

Limites explícitos: não há checkpoint persistido em disco; canais individuais completos permanecem nos sidecars, sem todos os gráficos históricos no player; orçamento do cache não inclui metadados; responsividade foi medida na API de preview e em smoke test sem janela, sem ensaio interativo prolongado. As falhas/condições de convergência conhecidas estão documentadas e exigem qualificação antes de uso nesses regimes.

Contratos e estruturas: [Cálculo, replay e experimentos](docs/etapa-9-calculo-replay-e-experimentos.md). Medições brutas, mediana/dispersão, falhas e alternativas: [Benchmarks da etapa 9](docs/etapa-9-benchmarks.md).

## Etapa 10 — Calibrar, validar e qualificar o preset Realista

**Objetivo:** demonstrar capacidade de previsão com medições reais e documentar limitações. Resolver G14; aumentar a fidelidade somente onde trouxer benefício medido.

### Tarefas

- [x] **10.1 — Definir protocolo experimental.** Para cada ensaio, registrar robô/componentes, instrumento, precisão, montagem, superfície, iluminação, temperatura, estado da bateria, unidade e taxa de aquisição. Identificar dados sintéticos separadamente dos dados medidos.
- [ ] **10.2 — Caracterizar subsistemas.** Medir motor/driver em vazio e sob carga, bateria com degraus de corrente, fan/sucção em diferentes tensões/folgas, sensor em varredura óptica e pneu sob diferentes cargas/superfícies. Aproveitar dados de fabricante apenas com condições de medição conhecidas.
- [x] **10.3 — Preparar sincronização dos logs.** Definir origem de tempo e coordenadas, estimar offset/atraso, tratar lacunas e amostragem irregular. Comparar sinais equivalentes, preservando diferenças entre instante de aquisição e de entrega.
- [x] **10.4 — Ampliar a identificação de parâmetros.** Permitir escolher parâmetros, bounds, objetivo e dados. Examinar sensibilidade/identificabilidade antes de ajustar muitos parâmetros; evitar que atrito e torque compensem erros estruturais do modelo.
- [ ] **10.5 — Separar ajuste e validação.** Reservar tensões, pistas, velocidades e condições não usadas na calibração. Comparar o preset avançado com o simples e registrar faixa válida e incerteza dos parâmetros.
- [x] **10.6 — Ampliar métricas.** Registrar trajetória, velocidade, yaw, sensores, tempo de volta, perda de linha, frenagem, corrente, energia e saturação. Informar cobertura de cada sinal; ausência de medição não equivale a erro zero.
- [x] **10.7 — Demonstrar convergência e robustez.** Comparar 100/50/25 µs mantendo taxas de controle/sensores fixas. Executar múltiplas seeds e perturbações de parâmetros, analisando estabilidade e dispersão dos resultados.
- [x] **10.8 — Decidir efeitos adicionais por evidência.** Avaliar deformação avançada, relaxação, temperatura, desgaste, vibração, arrasto, selo, relevo e pitch/roll. Implementar apenas o necessário para explicar erros relevantes; documentar os itens adiados e suas razões.
- [ ] **10.9 — Criar regressão reproduzível.** Organizar cenários e baselines por versão do modelo, com tolerâncias físicas definidas a partir da precisão experimental. Rodar testes do núcleo, build da GUI e verificações de formato na automação do repositório; benchmarks pesados podem ter execução separada e explícita.
- [ ] **10.10 — Publicar a qualificação do simulador.** Documentar modelos disponíveis, dados usados, limites, custo e instruções de configuração. Atualizar a arquitetura e este checklist; não chamar um modelo de validado apenas por compilar ou completar uma volta.

### Estruturas afetadas

| Estrutura atual | Alteração esperada |
|---|---|
| `src/calibration.rs`: importação, comparação, métricas e tune | Sincronização, sinais adicionais, bounds, objetivos e validação independente |
| `src/cli.rs`, `src/ui.rs` | Configuração e acompanhamento de experimentos/calibração |
| Modelos físicos, sensores e assets | Parâmetros medidos, proveniência e faixa de validade |
| Telemetria/replay | Sinais necessários e metadados do experimento |
| `examples/`, `docs/`, `README.md` | Cenários, relatórios e instruções alinhadas à implementação |
| Estruturas propostas | `src/experiments/{calibration,metrics}.rs`, `tests/scenarios/`, `datasets/` ou referências externas versionadas, workflow de CI conforme o provedor do repositório |

### Verificação e conclusão

- [ ] Definir e justificar tolerâncias antes de avaliar os dados reservados de validação.
- [ ] Demonstrar quais métricas melhoram ao passar de Simplificado para Realista e o aumento de custo correspondente.
- [x] Reproduzir resultados a partir da configuração, versão e dados registrados — verificado para os cenários sintéticos/numéricos; medições físicas continuam pendentes.
- [x] Documentar falhas conhecidas, condições sem validação e fenômenos opcionais adiados.

**Concluída quando:** há evidência experimental de precisão útil dentro de uma faixa declarada, sem perder as opções simples e rápidas. A obtenção de dados físicos depende de acesso ao robô e a instrumentos; software pronto não substitui essa evidência.

### Entrega de software e pendências — 23/09/2026

**A etapa 10 está parcialmente executada; o Realista não está qualificado experimentalmente.** Não foram encontrados logs de bancada com instrumentos/condições identificados. `real_log_demo.csv` permanece com proveniência desconhecida. Nenhuma medição física foi fabricada ou inferida do nome de um arquivo.

| Item | Evidência e limite |
|---|---|
| 10.1 concluído | Protocolo por subsistema, metadados de instrumento/precisão/condições/unidades, partição e critérios prévios definidos |
| 10.2 pendente de bancada | Faltam medições reais de motor/driver, bateria, fan/sucção, sensor e pneu; protocolo pronto para coletá-las |
| 10.3 concluído | CSV estrito, origem/offset e transformação explícitos, lacunas, retenção de ADC/flags e timestamps de aquisição/entrega; busca de offset por API testada em sinal excitado |
| 10.4 concluído | Seis escalas de parâmetros, bounds/objetivos/dados selecionáveis, triagem de excitação/colinearidade e grade limitada; sem ajuste usando o holdout |
| 10.5 parcial | Partições e grupos independentes implementados/testados; comparação entre presets registrada. Ainda faltam condições físicas reservadas, faixa válida e incerteza experimental dos parâmetros |
| 10.6 concluído | Trajetória, velocidade/yaw, ADC, corrente/tensão, energia, linha, saturação e eventos de volta/frenagem, com cobertura e `null` para ausência; definições reduzidas explícitas |
| 10.7 concluído no cenário documentado | 27 execuções de 100 ms, 100/50/25 µs, três seeds e ±5% no atrito; taxas lógicas/sensores preservadas e dispersão registrada |
| 10.8 concluído como decisão de escopo | Matriz de evidências necessárias e adiamentos publicada; nenhum fenômeno adicional implementado sem ganho físico demonstrado |
| 10.9 parcial | CI, testes, formatos e baselines sintéticos/numéricos versionados entregues. Baselines com tolerâncias derivadas da precisão de instrumentos ainda dependem da bancada |
| 10.10 parcial | Relatório preliminar, modelos/limites/custo/instruções e arquitetura atualizados. Qualificação física definitiva depende das medições e revisão dos dados reservados |

Subtarefas das entregas parciais:

- [x] **10.5 — Software:** separar calibração/validação, recusar conteúdo repetido/grupos sobrepostos e congelar parâmetros antes de avaliar holdout.
- [ ] **10.5 — Experimento:** obter condições físicas reservadas e estimar incerteza/faixa válida com repetições apropriadas.
- [x] **10.9 — Software:** adicionar CI, testes de núcleo/GUI/formato, regressão de identificação e baseline numérico congelado.
- [ ] **10.9 — Experimento:** registrar baselines físicos e tolerâncias justificadas pela instrumentação, antes da avaliação final.
- [x] **10.10 — Relatório preliminar:** publicar estado não qualificado, evidências de software, custos e limitações.
- [ ] **10.10 — Qualificação definitiva:** demonstrar precisão útil com medições físicas independentes, sem confundir execução correta com validade física.

Validação local: **165 testes sem GUI e 171 com GUI**, com 16 testes novos da etapa 10. Fixture sintética recuperou fator de torque 1,1 e passou no cenário reservado do próprio modelo gerador; isso verifica o processo de identificação, não a física real. Um baseline numérico congelado detecta deriva do modelo separadamente. A CI foi configurada, mas não foi disparada remotamente.

Revisão final em **24/09/2026**: relatório de robustez registra explicitamente a duração solicitada; os 16 testes da etapa 10 passaram novamente após essa correção. Links da documentação, arquivos JSON de evidência e formatação conferidos. Permanecem pendentes somente as partes experimentais indicadas acima, que exigem coleta e análise de medições físicas.

Contratos e evidências: [Calibração e qualificação](docs/etapa-10-calibracao-e-qualificacao.md). Coleta/decisões: [Protocolo experimental](docs/etapa-10-protocolo-experimental.md). Números versionados: [Resumo dos ensaios](docs/benchmarks/etapa-10-resumo.json).

## Mapa consolidado do impacto estrutural

| Camada | Arquivos atuais principais | Etapas com maior impacto |
|---|---|---|
| Entrada e build | `main.rs`, `cli.rs`, `Cargo.toml` | 1, 3, 9 |
| Persistência e definição | `config.rs`, `json.rs`, serialização em `ui.rs` | 3, 4, 5, 6 |
| Tempo e execução | `sim.rs` | 1, 2, 6, 7, 8, 9 |
| Mecânica e contato | `sim.rs`, `wheel.rs`, `normal_force.rs`, `math.rs` | 2, 4, 6, 7 |
| Elétrica/atuação | `motor.rs`, `battery.rs`, `normal_force.rs` | 2, 7 |
| Pista | `rtsim_track.rs`, `track.rs` | 5, 6, 9 |
| Percepção e controle | `sensor.rs`, `encoder.rs`, `gyro.rs`, `rng.rs`, `controller.rs` | 1, 4, 8 |
| Apresentação | `ui.rs` | 1, 3, 4, 5, 6, 8, 9 |
| Resultados | `telemetry.rs`, `replay.rs`, `calibration.rs` | 1, 6, 7, 8, 9, 10 |
| Conteúdo e validação | Assets, exemplos, docs e testes | Todas, conforme o recurso alterado |

## Critério comum para encerrar uma tarefa

Uma entrega deve registrar: o que mudou, quais estruturas foram afetadas, se houve migração ou alteração de resultado físico, como foi verificada e quais limitações permanecem. Usar cenários pequenos para verificar cada fenômeno e cenários completos para validar a integração.

| Campo de acompanhamento | Preenchimento esperado |
|---|---|
| ID | Exemplo: `1.3` ou `7.8` |
| Estado | Pendente / em andamento / concluída / bloqueada / opcional adiada |
| Evidência | Teste, comando, cenário ou relatório reproduzível |
| Compatibilidade | Schema/API/formato alterado e migração correspondente |
| Limitação | Fenômeno não coberto ou condição ainda não validada |

Ao concluir cada etapa, atualizar [a arquitetura](docs/arquitetura-do-projeto.md) para que a distinção entre implementado e proposto continue correta. O objetivo de conclusão do projeto é um simulador configurável, validado e utilizável; não a implementação indiscriminada de todos os efeitos opcionais.
