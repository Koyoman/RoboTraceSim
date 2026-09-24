# Etapa 10 — Calibração e qualificação

## Estado atual: não qualificado experimentalmente

Em 23/09/2026, foram entregues protocolo, ferramentas de estudo, testes numéricos e automação local reproduzível. **Não há dados reais de bancada com proveniência no repositório.** O preset Realista continua sendo uma aproximação planar configurável, sem faixa de precisão física certificada. `real_log_demo.csv` é um exemplo de proveniência desconhecida; não foi reclassificado como medição física.

A conclusão integral da etapa depende de caracterização de motor/driver, bateria, fan/sucção, sensores e pneus; reserva de condições físicas independentes; incertezas e tolerâncias baseadas nos instrumentos; comparação experimental entre presets. O checklist separa essas pendências das entregas de software.

## Estruturas e ferramentas

| Estrutura | Entrega |
|---|---|
| `src/experiments/metrics.rs` | CSV científico estrito, alinhamento temporal/espacial, busca de offset, interpolação com limite de lacuna, métricas/ausência/cobertura e eventos derivados |
| `src/experiments/calibration.rs` | Manifesto `rtsim-study-v1`, proveniência/partições, parâmetros limitados, triagem de identificabilidade, busca em grade e avaliação posterior dos dados reservados |
| `src/experiments/robustness.rs` | 100/50/25 µs, três seeds e perturbação de ±5% no atrito longitudinal, snapshots e dispersão |
| `src/cli.rs` | Comandos `qualify` e `robustness` |
| `src/ui.rs` | Painel de estudo versionado e robustez numérica, jobs canceláveis fora da UI; robustez congela a configuração editada |
| `examples/stage10_fixture.rs` | Gerador explicitamente sintético para regressão, sem arquivo fingindo medição real |
| `tests/scenarios/stage10-{synthetic,numerical}-v1.json` | Baselines de software: parâmetro conhecido e estado final numérico congelado, com tolerâncias independentes de precisão física |
| `scripts/check-qualification.py` | Verificação do baseline sem aceitar qualificação física de dados sintéticos |
| `.github/workflows/validation.yml` | Testes headless/GUI, formatação, fixture sintética e benchmark pesado apenas por acionamento explícito |

O [protocolo de bancada](etapa-10-protocolo-experimental.md) define medições necessárias, incertezas, partição e decisões sobre efeitos opcionais. A [pasta de dados](../datasets/README.md) define rastreabilidade. Os arquivos de estudo e resultados são JSON; não houve mudança dos modelos físicos ou do replay v4 nesta etapa.

## Executar a regressão sintética

Na raiz do repositório, usando um diretório de fixture ainda inexistente:

```powershell
cargo run --release --offline --no-default-features --example stage10_fixture -- target/stage10-fixture
cargo run --release --offline --no-default-features -- qualify target/stage10-fixture/study.json target/stage10-qualification.json
python scripts/check-qualification.py target/stage10-qualification.json
cargo run --release --offline --no-default-features -- robustness examples/physics/simplified.rtsim target/stage10-robustness.json 100ms
```

`qualify` recebe manifesto e destino do relatório. Uma execução completa pode ter `validation_criteria_failed` ou `identifiability_screen_failed`; esses são resultados científicos reportados, não falha de leitura do programa. CI precisa verificar conteúdo/critério do relatório, como faz o script fornecido, em vez de confiar apenas no exit code. Erros de formato/execução impedindo o estudo retornam erro. `robustness` registra as falhas de cada combinação e continua, sem inventar custo ou erro RMS para execuções que falharam.

Na interface, abrir as ferramentas de calibração, informar manifesto/relatório e executar o estudo. O botão de robustez usa 100 ms e a cópia da configuração atual. O cancelamento usa o job da etapa 9; relatórios são publicados atomicamente somente após a conclusão. Importação/compare/tune antigos permanecem identificados como ferramentas legadas; a qualificação científica usa o estudo versionado, não o score antigo isolado.

## Manifesto e contrato de identificação

O gerador fornece um manifesto executável completo. Campos obrigatórios:

- `schema: rtsim-study-v1`, `preset`, `duration_us` e `tolerance_basis`.
- `parameters`: nome, `min`, `max`, `steps`. Bounds são escalas positivas dos valores originais; 2–21 pontos por parâmetro, até três parâmetros e 256 combinações. Lista vazia avalia uma configuração sem ajuste.
- `objectives`: `signal`, unidade SI reconhecida, `scale`, `weight`, `max_rms` e `min_coverage`. Unidades incompatíveis são recusadas, nunca convertidas silenciosamente. Objetivos são sinais amostrados equivalentes; eventos derivados são relatados separadamente.
- `datasets`: ID, `kind` (`synthetic`, `measured`, `unknown`), `split`, `condition_group`, projeto, CSV, `origin_us`, `offset_us`, `max_gap_us`, `time_basis: delivery` e `protocol` completo. Caminhos são relativos ao manifesto; pode haver um projeto específico por condição. `frame` opcional fornece `x_m`, `y_m`, `yaw_rad` para a transformação de coordenadas.

Parâmetros implementados: `motor_torque_scale` para o motor simples; `motor_resistance_scale` para powertrain elétrico; `battery_resistance_scale`, `mu_longitudinal_scale`, `mu_lateral_scale` e `rolling_resistance_scale`. Pneus são alterados por contato e a resistência da bateria inclui a curva correspondente quando presente. Não há ajuste arbitrário de todo campo JSON. Escalas não devem absorver discrepâncias de montagem, atraso ou estrutura física.

A triagem compara resíduos normalizados nos extremos de cada bound e mede excitação e colinearidade entre pares. Sem excitação suficiente ou com cosseno absoluto maior que 0,98, não inicia a busca. Isso é uma triagem por diferenças finitas, **não prova de identificabilidade global**, particularmente em modelos não lineares ou com três parâmetros. Bounds muito amplos podem levar a falha do solver; corrigir a definição do ensaio em vez de remover proteções.

O objetivo soma erros quadráticos normalizados por escala, peso e número de amostras por sinal, usando apenas calibração. O melhor candidato é congelado antes da avaliação independente; arquivos com conteúdo repetido ou grupos presentes nos dois conjuntos são recusados. O offset já declarado não é reajustado no holdout. `estimate_offset` existe na API para sinais excitados de calibração; busca limitada, cobertura mínima e recusa de sinal constante evitam uma estimativa arbitrária.

O relatório registra candidatos/falhas, sensibilidade, parâmetros escolhidos, ótimos que tocam bounds, fingerprints dos dados e do manifesto, protocolo e snapshots completos por avaliação. Fingerprints FNV-1a detectam mudança de conteúdo; não são autenticação criptográfica. O relatório declara **incerteza dos parâmetros não estimada**: a grade e seus bounds não são intervalos de confiança. A etapa 10.5 só se completa quando repetições físicas e análise apropriada de incerteza estiverem disponíveis.

O preset escolhido e o Simplificado são comparados com os mesmos parâmetros ajustados. Isso mede mudança do modelo mantendo parâmetros fixos; não é uma competição entre dois modelos ajustados independentemente. Para essa comparação adicional, executar estudos separados com a mesma partição pré-definida e publicar os dois processos de seleção.

## Métricas e dados ausentes

RMS, máximo absoluto, média absoluta, número observado, número alinhado e cobertura por amostras acompanham cada sinal. Cobertura usa todas as linhas do conjunto medido como denominador; canais de taxas diferentes devem declarar a cobertura esperada. Campo ausente retorna `null`, nunca um erro igual a zero.

Diagnósticos incluem erro de trajetória XY, velocidade, yaw/yaw rate, linha, tensão/corrente e ADC por canal presente nos dados. ADC inválido ou ainda indisponível não é usado; ADC/PWM/flags são retidos, não interpolados. Interpolação contínua não atravessa lacunas maiores que `max_gap_us`, nem extrapola.

Energia integra potência `V×I` por trapézios somente em intervalos cobertos. Perda de linha e saturação informam duração e duração coberta. `saturated` na simulação significa limitação de corrente do driver elétrico; não equivale a slip do pneu ou PWM máximo e fica ausente sem powertrain. Volta usa eventos/coluna `lap_time_s`; ausência de volta válida não vira volta de duração zero.

Frenagem requer marcador explícito `brake_active`, posição XY e velocidade contínuas; mede caminho até `|vx_body_m_s| ≤ 0,01`, sem atravessar lacunas ou liberação do freio. O ensaio precisa marcar o início verdadeiro do comando. Uma coluna de distância medida pode ser fornecida explicitamente. São definições reduzidas a documentar ao comparar com um procedimento físico; não foram deduzidas frenagens de um simples PWM zero.

## Evidências obtidas

Verificação local: **165 testes sem GUI e 171 com GUI**, incluindo 16 testes novos da etapa 10.

O [resumo versionado dos ensaios](benchmarks/etapa-10-resumo.json) preserva os números e identifica todos os dados como sintéticos/numéricos. Relatórios completos com snapshots são gerados sob `target/` pelos comandos acima.

1. **Fixture sintética:** torque de geração 1,1; busca [0,9; 1,0; 1,1] recuperou 1,1. O conjunto reservado usa PWM 0,25, enquanto a calibração usa PWM 0,15. No próprio modelo gerador, velocidade teve RMS zero com 21/21 amostras. O Simplificado, com os mesmos parâmetros, teve RMS de aproximadamente 0,00235 m/s. Esse resultado é esperado ao usar o modelo gerador como referência; não prova superioridade física do Realista.
2. **Robustez numérica:** 27 execuções de 100 ms do exemplo Simplificado, passos 100/50/25 µs, seeds 1371–1373 e fatores de atrito 0,95/1/1,05. Todas concluíram no ensaio registrado. Controle permaneceu a 1 ms, sensores/log nos períodos originais. A referência de 25 µs não é ground truth físico.
3. **Refinamento:** pior RMS em X em relação a 25 µs caiu de cerca de 8,90 µm a 100 µs para 2,91 µm a 50 µs. O relatório mantém também Y, yaw, velocidade, corrente e dispersão, sem generalizar a convergência a todos os regimes/modelos.
4. **Regressão:** além da fixture que testa identificação no próprio modelo gerador, um estado final numérico congelado em `tests/scenarios/stage10-numerical-v1.json` detecta mudanças da dinâmica; sua tolerância de 1e-9 é de reprodutibilidade numérica, não precisão experimental. Os testes cobrem timestamps/NaN/duplicatas, lacunas, yaw, ADC retido/futuro, transformação, cobertura nula, energia/frenagem, offset conhecido, vazamento entre conjuntos, invariância do ajuste a alterações do holdout, parâmetro sem excitação e preservação das taxas lógicas durante refinamento.

Custos da etapa 9 continuam disponíveis em [benchmarks de desempenho](etapa-9-benchmarks.md). Os tempos de qualificação no resumo são de execuções curtas e não uma nova campanha de benchmark com dispersão. A CI foi adicionada ao repositório e suas verificações foram executadas localmente; nenhuma execução remota foi disparada nesta tarefa.

## O que falta para qualificar o Realista

- Coletar os ensaios de subsistemas e integração definidos no protocolo, com instrumentos identificados e arquivos brutos.
- Justificar tolerâncias físicas antes de avaliar o conjunto reservado; manter tensões, pistas, velocidades e condições fora do ajuste.
- Estimar incerteza/identificabilidade com repetições suficientes e validar a faixa de operação, incluindo limites de convergência.
- Demonstrar ganho físico útil por custo e publicar a qualificação definitiva. A falha conhecida de oito contatos sob controle fechado da etapa 9 continua documentada, não foi ocultada por ajustes de tolerância.

Até lá, `physically_qualified` permanece falso. Mesmo dados declarados medidos que atendam os critérios produzem `measured_criteria_passed_review_required`; não há certificação automática por completar simulação. A [decisão sobre efeitos adicionais](etapa-10-protocolo-experimental.md) adia complexidade sem evidência, preservando opções simples e rápidas.
