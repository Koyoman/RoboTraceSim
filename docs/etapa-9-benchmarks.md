# Etapa 9 — Benchmarks e decisão de implementação

Medição local em 23/09/2026, Rust 1.96.0, Windows x86_64, CPU AMD64 Family 25 Model 33 Stepping 2 (12 processadores lógicos). Perfil release padrão, física 50 µs e duração simulada 100 ms, três repetições por combinação. Repetições curtas sofrem influência de escalonamento, antivírus, cache e criação de arquivos; a faixa mínimo–máximo acompanha a mediana. Não é garantia de prazo em tempo real nem validação da física.

Fontes preservadas: [medições](benchmarks/etapa-9-medicoes.csv), [falhas](benchmarks/etapa-9-falhas.jsonl), [máquina/definições](benchmarks/etapa-9-maquina.txt), [compilação](benchmarks/etapa-9-compilacao.csv).

## Reprodução

```powershell
cargo build --release --offline --no-default-features --bin robotrace-sim --example stage9_benchmark
target/release/examples/stage9_benchmark.exe target/benchmark-novo 3 100000
```

O diretório deve ser novo. Rodar sem outros builds/benchmarks concorrentes. O exemplo Rust usa a biblioteca para sessão/runner/worker e inicia o executável para medições CLI reais. No Windows o processo filho é criado sem janela. Nada é instalado ou agendado.

A matriz tem 126 tentativas: 111 concluídas e 15 falhas de convergência documentadas. Falhas vão para `failures.jsonl`; não recebem um fator de tempo real calculado como se tivessem terminado. Cada execução mantém seus arquivos científicos no diretório de medição, fora do Git.

## Cenários

| Nome | Configuração |
|---|---|
| reference4 | Projeto básico, quatro sensores originais, física de referência agregada |
| sensors16 | Projeto básico, 16 instâncias da primeira configuração óptica em espaçamento lateral de 1 mm |
| complex64 | 64 instâncias, pista poligonal de 4096 pontos, índice espacial ativo |
| simplified4 | Exemplo Simplificado original, quatro sensores e quatro contatos |
| simplified8 | Mesmo controlador/sensores, oito contatos por duplicação dos apoios com deslocamento de 5 mm |
| simplified8-straight | Oito contatos, ganhos PID zerados e PWM constante 0,1 para medir movimento sem correção de direção |
| realistic4 | Exemplo Realista reduzido original, quatro contatos |
| power4 | Exemplo elétrico acoplado original, quatro sensores/contatos |
| cli-reference / cli-simplified | Executável em processo próprio, projetos originais sem alteração de sensores |

Nos casos em biblioteca, término por corrida/saída foi desligado para completar a duração. Variações de sensores/contatos podem alterar a trajetória e o trabalho do solver; diferenças entre cenários não são atribuições isoladas de custo por sensor. A comparação de modos dentro de cada cenário usa exatamente a mesma configuração.

## Resultados completos

Fator = tempo simulado / tempo do loop. Total inclui inicialização, metadados, encerramento e, em CLI, startup do processo. O loop de runner/worker inclui gravação e flush científicos; sessão avança o núcleo sem escritor. Logs ligados geram CSV, replay e sidecars disponíveis. Valores de total/loop são medianas em milissegundos.

| Cenário | Modo | Logs | Total ms | Loop ms | Fator mediano | Mín–máx |
|---|---|---|---:|---:|---:|---:|
| reference4 | session | não | 17.31 | 13.61 | 7.35× | 7.31–7.40× |
| reference4 | runner | não | 17.38 | 13.73 | 7.29× | 7.27–7.47× |
| reference4 | worker | não | 18.29 | 13.56 | 7.37× | 7.37–7.40× |
| reference4 | runner | sim | 33.79 | 23.44 | 4.27× | 4.02–4.57× |
| reference4 | worker | sim | 35.35 | 23.82 | 4.20× | 3.73–4.28× |
| sensors16 | session | não | 56.43 | 49.15 | 2.03× | 2.03–2.07× |
| sensors16 | runner | não | 54.95 | 47.48 | 2.11× | 1.55–2.12× |
| sensors16 | worker | não | 56.31 | 47.64 | 2.10× | 2.06–2.10× |
| sensors16 | runner | sim | 86.63 | 68.60 | 1.46× | 1.45–1.47× |
| sensors16 | worker | sim | 86.62 | 69.05 | 1.45× | 1.45–1.49× |
| complex64 | session | não | 877.10 | 735.57 | 0.14× | 0.13–0.14× |
| complex64 | runner | não | 877.93 | 739.71 | 0.14× | 0.13–0.14× |
| complex64 | worker | não | 867.68 | 728.34 | 0.14× | 0.14–0.14× |
| complex64 | runner | sim | 1075.76 | 801.37 | 0.12× | 0.12–0.13× |
| complex64 | worker | sim | 1094.97 | 799.15 | 0.13× | 0.12–0.13× |
| simplified4 | session | não | 56.13 | 53.01 | 1.89× | 1.89–1.89× |
| simplified4 | runner | não | 56.44 | 53.43 | 1.87× | 1.86–1.91× |
| simplified4 | worker | não | 57.44 | 52.61 | 1.90× | 1.90–1.92× |
| simplified4 | runner | sim | 74.63 | 62.84 | 1.59× | 1.59–1.62× |
| simplified4 | worker | sim | 92.90 | 79.63 | 1.26× | 1.22–1.59× |
| realistic4 | session | não | 24.11 | 20.81 | 4.81× | 4.73–4.81× |
| realistic4 | runner | não | 24.00 | 20.83 | 4.80× | 4.70–4.82× |
| realistic4 | worker | não | 25.87 | 21.14 | 4.73× | 4.67–4.75× |
| realistic4 | runner | sim | 41.87 | 31.21 | 3.20× | 2.98–3.30× |
| realistic4 | worker | sim | 45.94 | 32.53 | 3.07× | 2.13–3.20× |
| power4 | session | não | 307.30 | 303.33 | 0.33× | 0.32–0.33× |
| power4 | runner | não | 301.01 | 297.67 | 0.34× | 0.33–0.34× |
| power4 | worker | não | 304.14 | 298.84 | 0.33× | 0.33–0.34× |
| power4 | runner | sim | 346.29 | 312.75 | 0.32× | 0.28–0.32× |
| power4 | worker | sim | 331.99 | 315.70 | 0.32× | 0.32–0.32× |
| simplified8-straight | session | não | 45.74 | 41.82 | 2.39× | 2.35–2.40× |
| simplified8-straight | runner | não | 46.72 | 42.88 | 2.33× | 2.15–2.35× |
| simplified8-straight | worker | não | 47.16 | 42.51 | 2.35× | 2.33–2.36× |
| simplified8-straight | runner | sim | 72.32 | 58.33 | 1.71× | 1.37–1.89× |
| simplified8-straight | worker | sim | 73.16 | 60.01 | 1.67× | 1.28–1.87× |
| cli-reference | cli | não | 47.72 | 13.95 | 7.17× | 4.11–7.32× |
| cli-simplified | cli | não | 78.28 | 53.73 | 1.86× | 1.85–1.88× |

O **Simplificado original atingiu 1,87× no runner sem logs e 1,59× com logs**; a CLI independente mediu 1,86×. A meta inicial de 1× a 50 µs foi atendida nesse cenário. Alimentação elétrica acoplada (~0,34× sem logs) e óptica densa em pista complexa (~0,14×) continuam abaixo de tempo real. Cálculo antecipado e reprodução desacoplada permitem visualizar esses resultados em velocidade normal depois do cálculo.

### Convergência e falhas preservadas

`simplified8` interrompeu em 9.600 µs por não convergência do solver, de forma consistente nas 15 combinações/repetições de sessão/runner/worker. Oito contatos em linha reta concluíram. Não houve aumento artificial de tolerância para fazer o benchmark passar. Contatos redundantes sob correção agressiva de direção precisam de investigação adicional antes de uso validado nesse regime.

No ensaio inicial de batch, uma combinação de seed 1371/PWM 0,4 com preset Simplificado também interrompeu em 35.850 µs, mantendo os demais runs completos. O manifesto de demonstração usa PWM 0,1/0,2 e concluiu oito runs de 2.000 passos. Falhas são resultados reportáveis; contagem de passos desconhecida em um run com erro é `null`, não zero presumido. Replay de um run interrompido por erro fica explicitamente incompleto.

## Memória, gravação e responsividade

O pico acumulado de RAM do processo de medição foi **10.62 MiB**. Esse high-water mark inclui cenários anteriores; não mede RAM incremental por run. A medição de RAM da CLI filha não foi instrumentada (zero no CSV significa indisponível).

A maior chamada de consulta do preview nos runs concluídos durou **15.4 µs**; o primeiro preview chegou em até **305.3 ms**, incluindo inicialização. A thread de interface pode continuar trabalhando enquanto aguarda. Estes números medem a API, não FPS renderizado ou tempo de cancelamento sob falha de disco.

| Cenário (runner com logs) | Bytes gerados, mediana | Vazão efetiva MiB/s, mediana |
|---|---:|---:|
| reference4 | 393159 | 11.10 |
| sensors16 | 910180 | 10.02 |
| complex64 | 3489588 | 3.09 |
| simplified4 | 585907 | 7.49 |
| realistic4 | 583850 | 13.30 |
| power4 | 802818 | 2.21 |
| simplified8-straight | 784530 | 10.35 |

A vazão é bytes totais / tempo total (cálculo + serialização + disco), não benchmark isolado da unidade de armazenamento. O arquivo bruto contém os tempos por repetição. Testes navegam 4.097 registros em arquivo maior que o cache de 128 KiB, validam alocação limitada e rejeição de truncamento. Metadados são contabilizados separadamente do cache.

## Evidência para otimizações e alternativas

Antes de trocar o kernel de atrito, uma execução local do Simplificado original mediu 0,47×; após Newton salvaguardado, 1,79× (pares exploratórios de uma execução, não estatística de três repetições). O resultado final acima confirma a meta com mediana/dispersão. A mudança resolve a mesma minimização convexa; testes comparam contra a bisseção anterior e verificam regressões físicas.

Comparação separada, três execuções CLI de 100 ms, sem logs:

| Build | Cenário | Mediana | Mín–máx |
|---|---|---:|---:|
| release | simplified | 1.82× | 1.81–1.83× |
| release | power | 0.32× | 0.31–0.33× |
| thin_lto_cgu1 | simplified | 1.86× | 1.82–1.89× |
| thin_lto_cgu1 | power | 0.34× | 0.28–0.34× |

Thin LTO e uma unidade de geração de código foram testados em diretório de build separado com `CARGO_PROFILE_RELEASE_LTO=thin` e `CARGO_PROFILE_RELEASE_CODEGEN_UNITS=1`. O ganho mediano do Simplificado foi pequeno (~2,3%), com sobreposição das faixas; o padrão de compilação foi mantido. A organização dos dados, resolução prévia da configuração e algoritmo trouxeram o ganho relevante.

Não foi criado kernel C/DLL, SIMD ou GPU para comparação; não há evidência de que uma mudança de linguagem resolveria os custos restantes. A decisão nesta etapa é continuar em Rust e paralelizar experimentos independentes. Antes de qualquer migração, perfilar circuito/óptica separadamente, comparar o mesmo workload e verificar equivalência numérica. Processo separado pode melhorar isolamento de firmware, com custo de comunicação; não é pressuposto de aceleração.
