# Etapa 9 — Cálculo, replay e experimentos

Implementação de 23/09/2026. O objetivo continua sendo um simulador planar de robô seguidor de linha, com física e lógica em relógios separados e fidelidade configurável. Esta etapa muda a execução e o custo, sem prometer validação experimental dos modelos da etapa 10.

## Execução e interface

`SimulationCore` continua sendo o único dono do estado. `experiments/jobs.rs` fornece `SimulationWorker`, que recebe uma cópia da configuração, atribui um `run_id` e executa `run_controlled` numa thread própria. O identificador acompanha os metadados e o resumo. O snapshot resolvido contém robô, pista, modelos, conexões, seeds e fingerprints das fontes. Depois da inicialização o núcleo não consulta arquivos de configuração durante a integração.

A interface oferece nova execução, iniciar/retomar, pausar, um passo, cancelar e calcular/reproduzir. Cada cálculo da GUI grava em um novo diretório `target/runs/`. Alterações no editor não modificam a cópia usada pelo worker; a interface solicita uma nova execução. O fechamento solicita cancelamento e aguarda o encerramento. Não há serviço, agendamento, startup do Windows ou PowerShell em segundo plano.

Pausa e cancelamento são cooperativos, nas fronteiras dos passos. A pausa usa variável de condição, sem loop consumindo CPU. A resposta pode aguardar o passo atual ou uma operação de disco. Não é uma garantia de prazo máximo sob travamento do sistema de arquivos ou de firmware externo.

O preview publica aproximadamente a cada 16 ms de tempo de parede, além das fronteiras de pausa/término. Seu buffer tem capacidade de 1 a 256 snapshots; a GUI usa 1. O consumidor recebe o mais recente e contabiliza os descartes. Sensores, estimativa, contatos e resumo elétrico vêm do estado calculado. Nenhum descarte visual elimina integração, evento lógico ou observação científica solicitada.

CSV, replay e sidecars são gravados sincronamente pelo runner. Esse é o backpressure: disco lento reduz a velocidade de execução de parede, sem alterar o passo físico, avançar relógios por FPS ou perder registros. O cancelamento grava uma observação terminal, mesmo fora do período regular de log, e finaliza o replay com motivo `cancelled`. Erro de escrita não produz um rodapé de sucesso.

Abertura de replay, exportação, importação, comparação e ajuste de parâmetros também usam jobs próprios. A GUI acompanha conclusão/erro e permite cancelar a operação. Exportações e arquivos dos jobs da GUI são publicados por arquivo temporário e renomeação após sucesso; cancelamento não substitui um arquivo completo por um parcial. Relatórios múltiplos são publicados individualmente, não como transação de diretório.

## Calcular e reproduzir

O botão **Calcular e reproduzir** retoma o worker e abre o resultado ao concluir. O player navega por tempo, permite pausa, velocidade de 0,01× a 100× e seek. A pista é reconstruída a partir da configuração embutida, não do projeto atualmente editado.

Somente `x`, `y` e yaw são interpolados; yaw percorre o menor arco. Leituras, PWM, flags e demais canais ficam retidos na amostra anterior. O timestamp da amostra retornada continua identificando essa aquisição; a posição do controle de tempo é o instante visual. A reprodução nunca chama o integrador. Em v3, sem cenário embutido, o viewer apresenta os dados disponíveis sem inventar configuração.

## Replay v4 e leitura v3

`src/io/replay_v4.rs`, incluído por `replay.rs`, implementa o formato little-endian:

| Parte | Conteúdo |
|---|---|
| Cabeçalho | `RTSRPL04`, versão u16=4, quantidade de sensores u16, 44 campos base, comprimento e checksum dos metadados |
| Metadados | JSON UTF-8, até 16 MiB: configuração efetiva, snapshot resolvido/versionado, canais com nome/tipo/unidade e sufixos dos sidecars |
| Bloco | `BLK4`, quantidade de registros, checksum e até 64 amostras |
| Amostra | Payload v3 mais dois contadores i64 exatos de encoder; ADCs u32 em ordem dos sensores congelados |
| Índice | `IDX4`, intervalos temporais/offsets/contagens por bloco, total de amostras e motivo de término |
| Trailer | Offset e checksum do índice, marcador `ENDRPL04` |

Os fingerprints/checksums usam FNV-1a 64: detectam corrupção acidental, não autenticam o arquivo. A abertura valida versão, metadados, rodapé, índice, contagens e limites. Cada bloco é verificado quando acessado, incluindo timestamps crescentes e concordância com o índice. Portanto a abertura não é uma varredura integral dos dados; exportar/percorrer tudo verifica todos os blocos. Sem rodapé, inclusive interrupção forçada do processo, o arquivo é declarado incompleto.

O índice permanece em disco e o leitor mantém somente um bloco. Seu cache é pré-alocado para 64 registros e respeita o orçamento informado. Metadados e configuração reconstruída são despesas separadas desse orçamento; não se anuncia um limite total de RAM igual ao cache. O escritor mantém um índice compacto em memória, proporcional ao número de blocos. O antigo carregamento integral retorna erro ao exceder o limite de amostras, sem truncamento silencioso.

O leitor aceita v3 de tamanho consistente e permite exportá-lo. Como v3 não possui rodapé/checksum, seu término é identificado como `legacy-v3-end-unverifiable`: um corte exatamente entre registros não pode ser distinguido de um arquivo completo. Novas execuções escrevem somente v4. Não existe obrigação de preservar resultados históricos. Não se inventam metadados, precisão de encoder acima de 2^53 ou configuração ausente ao ler v3.

Os observáveis agregados estão no binário; os canais individuais completos permanecem nos arquivos `.contacts.csv`, `.power.csv`, `.sensors.jsonl` e `.commands.csv`, além de eventos JSON. Os sufixos são anexados ao nome completo do resultado, por exemplo `result.rtlog.sensors.jsonl`. Arquivos de contato/potência dependem dos modelos ativos. A GUI mostra diagnósticos individuais no preview; o player atual não fornece gráficos históricos de todos os sidecars. Manter o conjunto de arquivos é necessário para análise completa.

## Checkpoint: contrato e limite

`SimulationCore::checkpoint()` cria um checkpoint opaco, **somente em memória e no mesmo processo/build**. `restore()` cria um núcleo independente. Não existe salvar/abrir checkpoint em disco, retomada após encerrar o aplicativo ou inferência de estado a partir de replay.

O snapshot clona relógio/agenda, estado mecânico, contatos e memória do pneu/caster, configuração mecânica resolvida, circuito elétrico, bateria, fans/normal, RNGs individuais, filtros, filas de aquisição/entrega, encoder/IMU, última velocidade de IMU, controlador/PID/estimador/perfil, comandos pendentes/manuais/repetidos, eventos de corrida, saídas e diagnóstico/falha. A pista imutável pode compartilhar `Arc`; estados mutáveis são independentes. A inicialização não é repetida e nenhum RNG é sorteado na restauração.

Firmware nativo externo é recusado porque o trait não fornece snapshot de seu estado. Os testes com aquisição sequencial, potência e contatos comparam continuação/restauração passo a passo, incluindo observáveis e estados internos expostos. Persistência futura exige versionamento completo, validação de compatibilidade e testes equivalentes; o arquivo de replay não serve como checkpoint.

## Otimizações implementadas

- A geometria analítica já congelada agora recebe uma árvore de caixas para consultas de linha, regiões e marcas. Consultas de sobreposição preservam a prioridade da última região; distâncias finais continuam analíticas. A árvore é construída uma vez.
- O subproblema convexo de atrito passa de 48 bisseções fixas para Newton salvaguardado em coordenadas do disco unitário, com bracket e fallback. Um teste compara 299 problemas contra o método anterior, além da regressão física das etapas 6 e 7.
- Linhas de Jacobiano usam arrays de tamanho máximo 11, adequados aos 3–8 contatos suportados; coleções usam capacidade conhecida. Ainda há alocações em estados temporários dos solvers e logs. Não se afirma um loop totalmente sem alocações.
- A configuração mecânica com inércia refletida do rotor é resolvida uma vez, em vez de clonar o robô e recalculá-la a cada passo elétrico.
- Seleção de contato e normal é resolvida em tipos antes do loop. `ControllerInput` é montado apenas no tick lógico. Nos ticks sem aquisição/entrega óptica, as leituras existentes recebem somente atualização de idade, preservando filas e consumo de RNG.
- Telemetria agregada é construída apenas quando há gravação ou preview; execução sem logs não monta uma amostra a cada integração.

O [relatório de desempenho](etapa-9-benchmarks.md) registra cenários, mediana/dispersão e limites. C/DLL não foi adicionada: o kernel Rust otimizado já cumpre a meta do exemplo Simplificado. O circuito acoplado e óptica muito densa continuam candidatos a perfil mais fino e paralelização de experimentos. SIMD deve ser avaliado para consultas ópticas em lote; GPU só se houver lote suficiente para amortizar transferência; processo separado só se necessário para isolamento de firmware/falhas. Uma ABI C isoladamente não acelera o mesmo algoritmo.

## Batch e varreduras

Exemplo de oito experimentos: [manifesto](../examples/batch/sweep.json).

```powershell
cargo run --release --offline --no-default-features -- batch examples/batch/sweep.json --out target/sweep-001 --jobs 2 --cancel-file target/cancel-sweep.flag
```

O diretório de saída deve ser novo; o pai deve existir. Cada entrada tem `project`, resolvido relativo ao manifesto, e arrays opcionais `seeds`, `physics_dt_us`, `presets`, `base_pwm`. A expansão é o produto cartesiano, limitado a 10.000 runs e 1–32 threads. Padrões: seed 1371 e demais valores do projeto. O seed mestre gera seeds independentes por ID de sensor e por subsistema; seeds efetivos ficam no snapshot. Os presets são `ideal`, `simplified`, `realistic`; combinações incompatíveis são reportadas como falhas, sem substituição silenciosa.

Cada run recebe diretório `run-00000`, ID próprio, CSV, replay e sidecars. `summary.json` por run e no diretório raiz registra índice, ID, término, passos e erro; passos desconhecidos após falha são `null`. O cancelamento da API usa `RunControl::cancel()`; pela CLI, criar o arquivo indicado em `--cancel-file` cancela cooperativamente (consulta a cada 50 ms durante o batch). Runs em curso finalizam como cancelados; pendentes recebem `not_started_cancelled`. Não há monitor após o término. Runs concluídos permanecem intactos. A CLI retorna erro se algum experimento falhar.

## Verificação

Validação final: **149 testes sem GUI e 155 com GUI**, formatação aprovada e batch de demonstração com oito runs completos.

`tests/stage9_execution.rs` cobre equivalência do worker com runner, pausa/passo/retomada, capacidades 1/8, configuração independente, cancelamento durante gravação, fechamento pausado, retomada por checkpoint, recusa de firmware, seek/interpolação/cache, encoder i64 exato, arquivo truncado/corrompido/versão inválida, leitura v3, reconstrução de configuração, batch e publicação atômica cancelada. Testes unitários cobrem equivalência do kernel e consultas espaciais; teste egui sem janela exercita os painéis de simulação e replay.

A qualificação não substitui um ensaio interativo prolongado de GUI nem validação contra o robô real. Custo de RAM medido é o pico do processo, não instrumentação por alocação; responsividade medida é a API de preview e não FPS real de uma janela. A etapa 10 permanece responsável por dados físicos, identificação de parâmetros e tolerâncias de validação.
