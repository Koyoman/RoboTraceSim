# Etapa 7 — Motores, alimentação e downforce

Implementada e verificada em 23/09/2026. Este contrato atualiza a análise histórica de arquitetura. O objetivo continua sendo um robô seguidor de linha configurável, com relógios independentes e fidelidade escolhida pelo usuário.

## Configuração e domínio

`robot.powertrain` habilita `coupled-dc-v1`. Os parâmetros são editáveis no painel “Motores e alimentação acoplados”, persistidos no robô e incorporados à configuração congelada de cada execução. `examples/power/projeto.rtsim` demonstra física de 50 µs e controle de 1 ms. Sem esse bloco, o modelo de referência permanece identificado como `staggered-simple-v1`.

O circuito acoplado exige contato individual não ideal e exatamente uma roda motriz por motor esquerdo/direito. Os demais apoios podem ser passivos. Essa restrição permite refletir a inércia do rotor corretamente sem inventar um acoplamento rígido entre várias rodas. A cinemática ideal e montagens incompatíveis são recusadas. Não existe obrigação de compatibilidade histórica.

## Motor e transmissão — 7.1 e 7.2

Cada motor possui corrente assinada, velocidade do rotor e temperatura. Unidades SI: R em Ω, L em H, Ke em V·s/rad, Kt em N·m/A, J em kg·m² e perda viscosa em N·m·s/rad. A configuração exige Ke=Kt em SI para preservar passividade. `dc_simple` requer L=0 e resolve a corrente algébrica; `dc_electrical` requer L>0 e usa Euler implícito:

`i_n = (L/dt i_anterior + V_aplicada − Ke ω_n) / (R + R_ponte + L/dt)`.

Velocidade, corrente e contato são resolvidos conjuntamente. `parameter_shaft=motor` usa redução g: ω_motor=g ω_roda e inércia adicional J_motor g², aplicada uma única vez. Parâmetros medidos na saída usam `parameter_shaft=output` e g=1. Não se deve fornecer simultaneamente a inércia refletida na roda e novamente no rotor.

A transmissão rígida aplica eficiência η na tração e 1/η no torque de frenagem. Na passagem por velocidade zero, a solução busca a faixa entre os dois ramos por bisseção, evitando alternância sem convergência. A tolerância de potência nesse ponto é 1e-10 W. Perdas viscosas, de transmissão e elétricas são contabilizadas separadamente.

Backlash/elasticidade ficam como extensão mensurável: um modelo futuro precisará de ângulos e velocidades separados nos dois eixos, folga em rad, rigidez em N·m/rad, amortecimento e energia elástica. Não foram adicionados parâmetros inativos que aparentem executar esses efeitos.

## Driver e comandos — 7.3

O driver fornece tensão média com resolução PWM, deadband, queda fixa e resistência da ponte. Comandos `ActuatorCommand` têm timestamp, PWM independente e modo Drive/Brake/Coast por motor. A latência precisa ser múltipla do passo físico; os comandos são retidos entre entregas. A API permite comandos manuais ou retorno ao controlador interno.

Brake curto-circuita o motor no modelo médio; Coast extingue a corrente indutiva sem impor corrente instantaneamente nula. Corrente limitada modifica o torque do mesmo passo. O orçamento da bateria é conservador: desconta fan/auxiliares e divide o restante entre os dois motores.

A regulação de corrente e o dissipador são ideais: a tensão inferida do enrolamento pode exceder a tensão do barramento durante descarga indutiva. Não há limite de avalanche, modelo de diodos, proteção de sobretensão ou capacidade térmica do resistor de descarga. `applied_pwm` é uma tensão terminal normalizada e limitada, não uma reconstrução das bordas de chaveamento; o comando solicitado permanece em `PowerState.applied`. Essa aproximação exige qualificação antes de prever estresse elétrico real.

## Circuito e bateria — 7.4 e 7.5

O barramento único resolve simultaneamente consumo dos motores, fan/sucção e potência auxiliar de lógica/sensores. O regulador auxiliar tem eficiência estática; perdas de fiação são resistivas. A fonte pode ser ideal ou Thevenin com OCV×SoC, R×SoC e um ramo RC de polarização. Curvas são interpoladas linearmente, com extremos em SoC 0 e 1. Sem curvas são usados os valores básicos da bateria.

`SoC_n = SoC_anterior − I dt / (capacidade_mAh × 3,6)`;
`Vp_n = (Vp_anterior + dt/τ Rp I)/(1+dt/τ)`;
`V_bus = OCV(SoC_n) − R(SoC_n) I − Vp_n`.

A fonte ideal mantém tensão nominal e SoC. O circuito busca o ramo de maior tensão com 32 sondagens descendentes e bisseção, até a tolerância configurada (padrão 1e-7 V, máximo padrão 80 iterações). Cada avaliação recomeça dos mesmos estados; não avança relógio nem ruído. A busca interna da transmissão/queda do driver tem limite de 80 avaliações. Falta de convergência produz erro explícito. Colapso de tensão produz evento de proteção.

O log distingue potência da fonte, dissipação resistiva, armazenamento RC (`C=τ/Rp`) e dissipação numérica de Euler implícito. O resíduo do balanço da bateria acompanha o resíduo do circuito, em vez de esconder armazenamento como perda.

Barramentos separados e reguladores dinâmicos continuam extensões: precisarão de estados e balanços próprios por nó, limites de conversão e solução conjunta. Chaves de configuração desconhecidas são rejeitadas, inclusive tentativas de ativar barramentos ainda inexistentes.

## Energia e proteção — 7.6

Potência negativa do motor somente retorna ao barramento quando regeneração está habilitada, existe margem de SoC e o modo permite. O limite de carga é dividido entre motores; o excedente é dissipado. Brake não presume recuperação. O log separa energia recuperada no barramento, retorno líquido à fonte após outros consumidores e energia dissipada no driver. O motor registra energia magnética, calor no cobre/ponte, perdas mecânicas e dissipação numérica `L (Δi)²/(2 dt)`.

Subtensão/colapso, sobrecorrente da bateria, limite de SoC e temperatura máxima encerram a execução no último estado válido. Eventos ficam latched, sem avançar um passo inválido. Entradas em limitação de corrente também geram eventos. O arquivo `.power.events.json` registra motivo e tempo; `.power.csv` possui uma linha por motor e amostra, inclusive terminal. Valores gerais repetidos nas duas linhas não devem ser somados duas vezes.

## Fan e sucção — 7.7 e 7.8

A seleção de modelo passa a determinar as equações executadas. Fans respeitam PWM mínimo/máximo e nominal de tensão/RPM. Linear usa curva linear; o tipo histórico Exponential corresponde à lei quadrática reduzida, não a uma exponencial matemática. LookupTable usa tabela de força. Curvas Polynomial/Custom e controle ClosedLoop sem implementação são recusados. Measured exige dados e rejeita combinações ambíguas de tabela agregada e fans.

Força varia com (V/V_nominal)². O consumo reduzido segue V²·PWM³; os parâmetros de corrente precisam de calibração independente. RPM é equivalente, calculado por RPM_nominal·sqrt(F/F_max), e não um estado mecânico independente do rotor do fan. A resposta de primeira ordem usa integração exponencial exata sob entrada mantida. A força e o primeiro momento respeitam a posição de cada fan; o barramento recebe seu consumo no mesmo passo.

Sucção preserva F=pressão×área com resposta dinâmica e vazamento reduzido. O fator de folga é `1/(1+folga_m × coeficiente_vazamento_por_m)`, além do vazamento básico configurado. A folga é fixa porque o chassi ainda não simula movimento vertical. Pressão e leitura equivalente de cada fan são observáveis. Não há CFD, vedação deformável ou dependência dinâmica de altura; pressão, corrente e força não constituem um modelo termodinâmico completo da bomba.

## Térmica e extensões — 7.9

Cada motor integra temperatura por um corpo térmico com capacidade J/K, resistência K/W, ambiente e temperatura máxima. Sob calor constante a integração é exponencial exata. A resistência elétrica varia linearmente com temperatura. Cobre, ponte, transmissão e mancal aquecem esse corpo agregado; dissipador de frenagem e bateria não possuem estados térmicos próprios.

Os parâmetros podem ser calibrados e persistidos. BLDC, FOC e PWM com bordas não estão implementados e não são tratados como DC silenciosamente. Chaveamento explícito exigirá agenda/subpassos próprios: 50 µs é um período inteiro de PWM de 20 kHz, insuficiente para resolver suas bordas.

## Estruturas e validação

| Arquivo | Responsabilidade |
|---|---|
| `src/models/electrical.rs` | Parâmetros, validação e equação DC |
| `src/models/power.rs` | Comandos, estados, fonte, RC e térmica |
| `src/core/power_step.rs` | Circuito acoplado ao executor mecânico |
| `src/motor.rs`, `src/models/contact.rs` | Torque afim, limites e perda viscosa |
| `src/normal_force.rs` | Fan/sucção efetivos e leituras |
| `src/config.rs`, `src/io/`, `src/models/robot.rs` | Persistência, capacidade e configuração congelada |
| `src/sim.rs`, `src/telemetry.rs` | Integração, proteção e registros |
| `src/app/robot_editor.rs`, `src/ui.rs` | Edição e inspeção |
| `tests/stage7_power.rs`, `examples/power/` | Casos verificáveis |

Validação final: 117 testes sem GUI e 121 com GUI. Inclui 12 testes de integração da etapa 7, três novos testes de fan/sucção e renderização automatizada do painel elétrico sem janela. São cobertos RL analítico, RC/térmica, balanço de potência, limite/latência, coast/brake/reversão, regeneração, proteções, persistência, seleção inválida, convergência de 100/50/25 µs e mudança de sentido com fans. Formatação e diff também verificados. Não houve teste interativo da janela ou calibração experimental.

O exemplo release completou 20 ms: 400 passos, 21 amostras, CSV e replay, com 42 linhas elétricas finitas. Maior resíduo de potência da bateria nas amostras: 3,49e-7 W. O replay binário v3 mantém seus canais agregados; os estados elétricos completos ficam nos sidecars até a evolução da etapa 9.

Ensaio local único, release, sem gravação: 0,197962 s para 20 ms simulados, 2.021 passos/s, aproximadamente 0,10× tempo real. É um diagnóstico curto, não um benchmark representativo. A busca elétrica chama repetidamente o solver mecânico e clona estados/configuração; otimizar esse caminho e executar cálculo fora da UI é prioridade da etapa 9. Migrar para DLL C sem perfil não elimina esse custo; Rust já gera código nativo.

Comandos reproduzíveis:

```powershell
cargo test --offline --no-default-features
cargo test --offline
cargo run --release --offline --no-default-features -- run examples/power/projeto.rtsim --headless --duration 20ms --csv target/stage7-final.csv --replay target/stage7-final.rtlog
cargo run --release --offline --no-default-features -- benchmark examples/power/projeto.rtsim --duration 20ms
```
