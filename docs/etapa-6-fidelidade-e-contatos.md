# Etapa 6 — Fidelidade e contatos por roda

Implementação de 22/09/2026, correspondente à etapa 6 de [tasks.md](../tasks.md). O objetivo continua sendo um simulador configurável de robôs seguidores de linha, com física e controle em períodos separados. Os modelos descritos aqui são reduzidos; a qualificação contra medições reais pertence à etapa 10.

## Seleção e persistência

O bloco `physics` do robô seleciona um preset e permite substituir seus subsistemas. `FidelityConfig::preset` expande todos os defaults; o snapshot e o salvamento do robô registram a seleção efetiva, incluindo overrides. `MODELS`, em `src/models/fidelity.rs`, descreve parâmetros, estados, dependências e limites dos modelos. Nomes desconhecidos, campos de física não suportados, parâmetros inválidos e combinações incompatíveis são rejeitados.

| Preset | Contato | Normal | Rolamento | Uso |
|---|---|---|---|---|
| `ideal` | Cinemática diferencial prescrita | Estática | Desligado | Referência de controle com rolamento longitudinal perfeito e scrub suprimido |
| `simplified` | Coulomb com elipse de aderência | Quase estática | Ligado | Corpo planar com forças, perdas e saturação por contato |
| `realistic` | Pneu viscoelástico reduzido (`brush`) | Quase estática | Ligado | Curvas força–slip regularizadas e parâmetros calibráveis |

O nome do preset indica a base escolhida; os campos `contact`, `normal` e `rolling` descrevem o comportamento efetivamente executado após overrides. O painel mostra ambos. Escolher `realistic` não certifica fidelidade experimental.

A ausência de `physics` seleciona explicitamente a referência anterior `planar-impulse-v2`, ainda disponível para comparação na interface. Esse caminho continua agregado por lado e não entrega contato individual. Os novos exemplos e os três presets usam `per-wheel-v1/<contact>` nos metadados. Não existe obrigação de preservar compatibilidade ou resultados históricos; a referência foi mantida como ferramenta de comparação numérica.

Exemplo de configuração com refinamentos:

```json
"physics": {
  "preset": "realistic",
  "contact": "brush",
  "normal": "quasi_static",
  "rolling": true,
  "longitudinal_stiffness": 20.0,
  "lateral_stiffness": 20.0,
  "relaxation_s": 0.001,
  "load_exponent": 0.15,
  "reference_load_n": 1.0,
  "radial_stiffness_n_m": 100000.0,
  "caster_trail_m": 0.01
}
```

Esses valores ilustram o formato; não são parâmetros medidos. Os coeficientes tangenciais têm unidade N·s/m. Os defaults dos refinamentos são zero (desligados), exceto carga de referência de 1 N e trail de 0,01 m. Os coeficientes longitudinal/lateral padrão são 20 N·s/m. A edição mantém definições incompletas para correção, mas execução e salvamento validam os modelos.

## Estado e equações dos contatos

Cada roda tem velocidade angular, ângulo acumulado, direção de caster quando aplicável, forças longitudinal/lateral, normal, slip, ângulo de deriva, torque de rolamento, compressão radial e regime. O executor conserva esses estados entre passos; observações não avançam a física.

Para um apoio na posição `r = posição_roda − COM`, a velocidade planar local é `v_contato = v_COM + omega_corpo × r`. A direção configurada da roda transforma essa velocidade para seus eixos longitudinal e lateral. O slip longitudinal em velocidade é `raio_efetivo × omega_roda − v_longitudinal`; o lateral usa a velocidade transversal do contato. O slip normalizado registrado divide a diferença pelo máximo das velocidades em módulo e pelo epsilon configurado, evitando singularidade na partida. O ângulo de deriva usa `atan2` com velocidade longitudinal regularizada.

O solver integra impulsos nos três graus de liberdade do corpo e em uma velocidade angular por roda. A reação do contato altera a roda e o corpo em sentidos coerentes. A força no referencial do corpo é a rotação da força local; o momento de yaw é `r_x × F_y − r_y × F_x`. Rodas dianteiras/traseiras fixas geram scrub lateral durante a curva, sem um amortecimento de yaw arbitrário.

O sistema inclui motores, contatos e rolamento no mesmo cálculo implícito. Um motor associado a várias rodas divide igualmente seu torque entre elas e observa a média de suas velocidades angulares: equivale a uma distribuição aberta ideal de torque, não a um eixo rígido que obrigue todas as rodas à mesma rotação. Transmissões mais completas são etapa 7. Os canais antigos de encoder continuam observando os dois eixos agregados dos motores; o registro por roda é a fonte dos estados individuais.

### Coulomb e pneu reduzido

Coulomb resolve aderência quando existe impulso suficiente para anular a velocidade relativa e deslizamento quando o limite é atingido. A força depende do movimento relativo e do sistema acoplado, não apenas do torque solicitado.

O modelo `brush` é uma aproximação viscoelástica linear saturada, não um modelo completo de carcaça nem Pacejka. Sem saturação, cada eixo segue `tau × dF/dt + F = C × velocidade_de_slip`. A discretização é implícita. Com `tau = 0`, a resposta é viscosa imediata; com `tau > 0`, há estado de força e rigidez tangencial equivalente `C/tau`. A energia elástica equivalente `tau × F²/(2C)` participa da verificação de passividade. Essa parametrização exige calibração na faixa de velocidade/carga de interesse.

A aderência é compartilhada: `(Fx/Lx)² + (Fy/Ly)² <= 1`. Cada limite usa a normal local e o menor coeficiente entre pneu e superfície, antes do fator opcional de sensibilidade à carga. Eixos com limite zero ficam sem força; normal zero produz regime `airborne`, sem divisão por normal. Não é possível usar simultaneamente os máximos integrais longitudinal e lateral.

O refinamento de carga multiplica os limites por `(N/N_ref)^(-expoente)` para N positivo. Expoente zero recupera a ausência de sensibilidade; o domínio permitido é [0, 1]. A compressão radial quase estática é `N/k_radial` e reduz o raio efetivo. Rigidez zero desliga o efeito; compressão acima de 10% do raio é rejeitada por sair do domínio desta aproximação. Não existem estados de salto, pitch, roll ou suspensão vertical.

### Rolamento e caster

A resistência de rolamento é um torque dissipativo limitado por `coeficiente × N × raio`, resolvido implicitamente para não inverter espontaneamente uma roda parada. Rodas passivas recebem reação do solo e inércia, sem torque motor.

O caster tem alinhamento cinemático amortecido por trail positivo: a velocidade transversal produz uma taxa de giro aproximadamente `v_transversal/trail`. O estado de direção altera os eixos do contato, e a roda possui rotação passiva. É um modelo reduzido sem inércia do pivô, atrito de giro detalhado ou shimmy. Esses efeitos não são simulados silenciosamente.

Os novos modelos aceitam de 3 a 8 apoios não colineares no plano, com raios, inércias, orientações e materiais individuais. Ideal exige rodas motrizes paralelas e uma posição lateral/raio por motor, além de bitola não nula. Seus apoios laterais não impõem scrub: a cinemática é a de um eixo diferencial virtual. Esse modelo prescreve velocidade, não prevê forças ou consumo dos motores; diagnósticos mecânicos zerados nesse modo não devem ser interpretados como balanço energético de um robô real. O downforce conserva seu modelo e consumo próprio.

## Normal e estabilidade do apoio

As forças verticais fornecem força total e primeiro momento de suas posições reais, antes de qualquer distribuição antiga pelos cantos. Isso inclui peso e downforce; uma força aplicada fora do polígono não tem sua posição limitada artificialmente ao retângulo.

O modelo estático encontra cargas não negativas que conservam `sum(N)`, `sum(xN)` e `sum(yN)`. Quando há mais de três apoios, seleciona a solução de menor soma de cargas ao quadrado entre os conjuntos ativos possíveis. É uma política explícita para o problema estaticamente indeterminado, não uma afirmação sobre a deformação real do chassi.

O modelo quase estático desloca o centro da resultante por `−massa × altura_COM × aceleração / força_total`. A aceleração vem do passo anterior e é transportada para o referencial atual. Essa defasagem é explícita; não resolve oscilações verticais ou dinâmica completa de tombamento. Cargas podem chegar a zero e retirar um apoio da solução. Se não existir distribuição não negativa que mantenha o equilíbrio, o run termina com diagnóstico de apoio impossível. O solver não recorta cargas negativas e redistribui o restante arbitrariamente.

## Integração, custo e diagnóstico

O cálculo usa iteração por blocos com projeção na elipse de aderência e integração implícita dos motores. A solução anterior é apenas uma estimativa inicial; o sistema do passo atual é resolvido novamente. Matrizes locais são calculadas uma vez por passo. O limite é 512 iterações, com convergência por alteração de impulso inferior a 1e-12 N·s (torques em N·m·s nos seus eixos). A projeção da elipse usa 48 iterações de bisseção. Falta de convergência, não finitos, escala numérica inválida, excesso de rotação por passo ou criação de energia interrompem a execução.

A verificação energética considera energia cinética das rodas/corpo, trabalho dos motores na convenção implícita e armazenamento do pneu com relaxação. A dissipação registrada inclui perda física e dissipação numérica do passo implícito. Não substitui a qualificação da cadeia elétrica da etapa 7.

O painel do experimento mostra modelos ativos, número de iterações e forças/slip/normal/regime por roda. CSV e replay recebem um arquivo adicional `<arquivo>.contacts.csv`, com IDs estáveis, nos mesmos instantes do registro principal, incluindo t=0 e o instante terminal. O replay binário v3 continua com os canais agregados; os canais individuais ficam no arquivo acompanhante. Carregamento integrado desses canais na reprodução e índices continuam na etapa 9.

### Medição local

Compilação release, sem GUI e sem gravação de logs; física 50 µs, controle 1 ms; 100 ms simulados (2.000 passos) para cada projeto em `examples/physics/`. Mediana de três execuções em 22/09/2026. São ensaios locais curtos, não garantias para outras máquinas ou pistas.

| Preset | Tempo de cálculo mediano | Passos/s | Simulado/real |
|---|---:|---:|---:|
| Ideal | 0,003693 s | 541.565 | 27,08× |
| Simplificado | 0,247070 s | 8.095 | 0,40× |
| Realista reduzido | 0,061528 s | 32.506 | 1,63× |

O caso Simplificado ainda fica abaixo de tempo real a 50 µs nesta pista com atrito assimétrico. O contato rígido exige mais iterações que o contato regularizado. O resultado orienta a etapa 9: aproveitar melhor a estrutura do sistema, reduzir alocações e separar cálculo da reprodução antes de considerar uma DLL C. Mudar de Rust nativo para C não elimina o custo das iterações. Não foi aplicada uma redução silenciosa de precisão para melhorar o número do benchmark.

## Verificação e arquivos

Sete testes do solver verificam balanço de forças/momentos em curva, aderência combinada, normal zero, dissipação/rolamento, reversão, apoio passivo/caster, sensibilidade à carga e compressão radial. Dez testes de integração verificam presets distintos, expansão/salvamento, overrides, rejeição de modelos, normal estática/quase estática, carga externa fora do apoio, convergência temporal, geometria individual e registro de contatos junto ao replay. Um teste adicional renderiza o editor com os três presets sem abrir janela.

O teste de redução desliga o contato avançado por override e compara a execução inteira com Simplificado, incluindo telemetria e forças por roda. O teste de convergência compara passos de 100, 50 e 25 µs com comando controlado; ruído do sensor não altera o comando nesse cenário. Os testes históricos permanecem como referências do solver agregado.

| Estrutura | Alteração |
|---|---|
| `src/models/fidelity.rs` | Registro, presets, overrides e validação |
| `src/models/contact.rs` | Estado por roda, solver, pneu reduzido, caster e rolamento |
| `src/models/chassis.rs` | Equilíbrio dos apoios e transferência quase estática |
| `src/models/robot.rs` | Capacidade de geometria por modelo e montagem resolvida |
| `src/config.rs`, `src/io/` | Carregamento, expansão e persistência da seleção |
| `src/normal_force.rs` | Primeiro momento das cargas verticais sem recorte |
| `src/sim.rs` | Integração no núcleo, estado, falhas e identificação do motor físico |
| `src/telemetry.rs` | Registro de estados/forças por contato |
| `src/app/robot_editor.rs`, `src/ui.rs` | Seleção, edição e inspeção |
| `tests/stage6_fidelity.rs`, `examples/physics/` | Casos de referência e projetos executáveis |

Comandos de validação: `cargo test --offline --no-default-features`, `cargo test --offline`, `cargo fmt --all -- --check` e formatação explícita do editor incluído. Resultados finais registrados em [tasks.md](../tasks.md). Não houve teste manual da janela nem calibração experimental.
