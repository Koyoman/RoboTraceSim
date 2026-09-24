# Etapa 1 — Executor único e contrato temporal

Implementação das tarefas 1.1–1.8 de [tasks.md](../tasks.md). O núcleo continua com os modelos físicos anteriores; esta etapa corrige a execução e a observação desses modelos.

## Estrutura entregue

- `src/lib.rs` expõe configuração, simulação, calibração, replay e telemetria para outros executores e testes. `main.rs` apenas escolhe CLI ou GUI. Implementações de dispositivos e o scheduler permanecem internos.
- `src/core/clock.rs` mantém tempo inteiro, duração e limite de execução; `src/core/scheduler.rs` define vencimentos e valida períodos.
- `SimulationCore`, em `src/sim.rs`, possui o estado físico, dispositivos, controlador, RNGs e relógio. `SimulationSession` é um alias da mesma API para compatibilidade com o viewer, não um segundo executor.
- `run_simulation` cuida de arquivos e resumo; `run_simulation_samples` coleta em memória para calibração. Ambos avançam o mesmo core e usam `should_log()`.
- `eframe` e `rfd` são dependências opcionais da feature `gui`. O build sem essa feature não compila dialogs nem o toolkit gráfico.

Os módulos físicos permanecem em seus caminhos atuais. Criar `core/state.rs` ou mover tudo para `models/` não é necessário para consolidar o dono do estado e fica para refatorações posteriores.

## Contrato temporal

Em `t=0`, sensores de linha, encoder e gyro são amostrados exatamente uma vez, nessa ordem. O controlador é executado uma vez com essas leituras. A inicialização mantém o período nominal como `dt` do controlador, preservando a convenção do PID existente.

Cada `step()` integra o intervalo com os comandos retidos, avança o relógio e processa aquisições vencidas antes do controlador. Assim, ao observar `t=1000 µs`, já estão disponíveis as leituras e o comando produzidos nesse instante. Leituras sem atualização naquele tick são retidas. Os modelos atuais têm latência zero; filas de entrega/atuação com latência configurável pertencem à etapa 8.

`sample()` não altera estado, RNG ou relógio. Sua pose representa o estado em `t`; sensores/comandos são os disponíveis após os eventos de `t`. Forças, torques, tensões dos motores e demais saídas físicas correspondem ao intervalo anterior. Em `t=0`, normal e bateria são inicializadas; as outras saídas físicas começam em zero. A amostra não implica novo cálculo de forças.

### Validação e término

- Períodos devem ser inteiros positivos em microssegundos. Campos fracionários, strings, valores não finitos e valores fora da faixa inteira segura do parser são rejeitados.
- Controle, sensor, encoder, IMU e log devem ter períodos múltiplos do passo físico. O período visual só precisa ser positivo; `16667 µs` continua válido com física de `50 µs`.
- Duração deve representar microssegundos inteiros e ser divisível pelo passo físico efetivo. A política é rejeitar, sem truncamento ou arredondamento de um tick. A conversão de segundos tolera somente erro de representação de ponto flutuante.
- Duração zero é válida: uma amostra em `t=0`, zero integrações e nenhuma evolução ao tentar avançar.
- O log contém a amostra inicial, os vencimentos regulares e a amostra terminal, sem duplicar quando o fim coincide com o log. Por exemplo: duração de `2150 µs`, física de `50 µs` e log de `1000 µs` produzem timestamps `0, 1000, 2000, 2150`, com **43 integrações e 4 amostras**.
- `step()` retorna `false` no final. `advance_steps(0)` não avança; blocos maiores que o restante param no término. `advance_until` rejeita destino anterior, fora da grade ou além do fim sem modificar o estado.

## Configuração efetiva e arquivos

`effective_config()` e `RunSummary.effective_config` registram duração, todos os períodos, seeds efetivas de linha/gyro e override do passo físico. A seed de linha registrada é a realmente utilizada pelo adaptador atual, não o valor legado eventualmente ignorado no JSON; corrigir a modelagem de sensores permanece pendente.

Para cada CSV/replay gerado, é gravado um arquivo acompanhante:

```text
resultado.csv
resultado.csv.metadata.json
resultado.rtlog
resultado.rtlog.metadata.json
```

O schema `rtsim-run-time-v1` registra versão do aplicativo, contrato de amostragem e política de término. O replay binário v3 permanece compatível com seu leitor. Esses metadados não são um checkpoint nem um snapshot completo dos assets/modelos: isso fica para a etapa 9. O benchmark registra a configuração no resumo, mas não cria arquivos de log/metadados.

O comando `run` agora aceita `--physics-dt-us`, como o benchmark. O override é validado antes da criação dos arquivos de saída e aparece nos metadados. Exemplo, usando um diretório de saída já existente:

```powershell
cargo run --release --no-default-features -- run examples/basic/projeto.rtsim --headless --duration 10s --physics-dt-us 50 --csv target/run.csv --replay target/run.rtlog
```

## Verificação

`tests/stage1_execution.rs` cobre a API pública e invoca o executável real da CLI:

- 20 integrações de 50 µs por intervalo de controle de 1 ms, contando separadamente a inicialização.
- Igualdade de todas as amostras de replay entre executor headless, sessão da GUI e calibração; comparação binária dos CSVs, replays e metadados entre API e CLI.
- Duração zero, amostra terminal fora do período regular e ausência de duplicação no fim.
- Diferentes tamanhos de blocos, observações repetidas, destinos inválidos e avanço após término.
- Rejeição de períodos/durações inválidos e preservação da independência da taxa visual.

O teste em `sim::timing_tests` compara as leituras iniciais com o primeiro resultado dos RNGs dos dispositivos, detectando a antiga dupla amostragem, e verifica a sintaxe dos metadados.

Comandos de verificação:

```powershell
cargo test --offline --no-default-features
cargo test --offline
cargo check --offline
cargo fmt --all -- --check
```

Resultado em 21/09/2026: **33 testes passaram em cada configuração**, com e sem a feature `gui` (25 unitários e 8 de integração). A compilação e a verificação de formatação também passaram. Permanecem avisos de código não utilizado; a interface gráfica não foi validada interativamente nesta etapa.

## Compatibilidade e limites

A correção da dupla amostragem e da fase de observação muda resultados novos em relação à versão anterior; não foi preservado o erro temporal para manter os antigos números. Replays antigos continuam legíveis. Projetos que dependiam de truncamento de períodos/duração agora recebem erro explícito.

A UI ainda executa os jobs sincronicamente. O indicador de sobreposição com a linha passou a ser calculado somente quando consultado, evitando levar a reconstrução geométrica por tick ao novo caminho headless. Cache/índice espacial completos continuam pendentes. Não foram adicionados modelos físicos avançados, latências configuráveis, worker, serviços do Windows ou monitor automático.
