# Etapa 2 — Dinâmica física básica

Entregas 2.1–2.8 de [tasks.md](../tasks.md), verificadas em 21/09/2026. O modelo passa a ser identificado por `physics_model: "planar-impulse-v2"` nos metadados dos runs. Os resultados mudam; parâmetros de PID e calibrações anteriores precisam ser reavaliados.

## Referenciais e sinais — 2.1

- Mundo: plano XY em metros; Z sai do plano. Yaw positivo é anti-horário, em radianos.
- Corpo: X aponta para a frente, Y para a esquerda. A pose é a origem geométrica no centro do retângulo de apoio. Sensores e pontos editados continuam relativos a essa origem.
- `center_of_mass_m` é o deslocamento do COM em relação à origem. `vx_body_m_s` e `vy_body_m_s` representam a velocidade **do COM**, expressa nos eixos do corpo. A inércia do chassi é a de yaw em torno do COM, sem as inércias de rotação das rodas.
- O runtime desta etapa agrega cada lado da transmissão em uma roda equivalente, em `(0, +track_width/2)` e `(0, -track_width/2)`. `wheel_inertia_kg_m2` é a inércia equivalente **por lado**, referida ao eixo da roda; não inclui automaticamente a inércia do rotor. O nome legado `DifferentialDrive4Wheel` não implica quatro contatos independentes.
- Velocidade angular positiva da roda corresponde a avanço. PWM, corrente do enrolamento e torque positivos acionam o movimento para a frente; sinais negativos representam o sentido reverso.
- Torque positivo de yaw surge quando a força direita supera a esquerda. Frenagem tem torque contrário ao movimento. Corrente da bateria é consumo não negativo; não há crédito de regeneração.
- `motor_torque_left/right_nm` da telemetria mantém o significado legado de torque **na roda**, após redução. `motor_voltage_*` é a tensão comandada média, antes da limitação de corrente; não representa a forma de onda do driver.

## Integração e forças — 2.2 e 2.3

`src/core/integrator.rs` separa a solução de impulsos da atualização da pose. O tick calcula normais, atuação e limites, resolve velocidades e reações, verifica energia, e só então atualiza pose e ângulos das rodas.

O momento linear é transportado no referencial do mundo. Para uma orientação `theta`, a transformação é `v_world = R(theta) * v_body`. A posição do COM usa a média das velocidades do início e do fim; a orientação usa a média das velocidades de yaw. A posição da origem geométrica é recuperada subtraindo `R(theta_novo) * COM`. Finalmente, a velocidade mundial é expressa na nova orientação do corpo.

Isso preserva uma trajetória mundial retilínea em movimento livre mesmo quando o corpo gira. É equivalente a incluir os termos de transporte `dvx/dt = Fx/m + omega*vy` e `dvy/dt = Fy/m - omega*vx`, sem aplicar explicitamente esses termos pelo Euler anterior.

Os eixos dos contatos ficam congelados durante cada tick: a solução de contato é de primeira ordem no tempo, embora a quadratura da pose use médias. Não se promete integração global de segunda ordem. A força lateral passa a suprir a mudança de direção do momento linear em curvas.

Foi removido o amortecimento de yaw fixo `0.00008`, sem parâmetro nem origem física. Nesta etapa, perdas vêm do motor/frenagem, do contato e do coeficiente explícito de resistência ao rolamento. Arrasto aerodinâmico e resistências adicionais continuam no plano de fidelidade; não foram substituídos por novos amortecimentos arbitrários.

## Rodas, aderência e repouso — 2.4 e 2.7

O vetor de velocidades tem cinco componentes: `vx, vy, omega_z, omega_left, omega_right`. A matriz de massa contém `m, m, Iz, Jwheel, Jwheel`.

Cada contato longitudinal tem velocidade relativa `r*omega_wheel - (vx - y_contact*omega_z)`. O impulso de contato atua na roda e no chassi com reações opostas, incluindo seu braço em relação ao COM. O contato lateral agregado, no centro do eixo geométrico, considera `vy - COM.x*omega_z`.

Uma solução quadrática com limites resolve conjuntamente:

1. Dois motores com resposta elétrica quase estática implícita.
2. Dois contatos longitudinais, limitados a `mu_long * normal_lado * dt`.
3. Um contato lateral, limitado a `mu_lat * normal_total * dt`.
4. Dois torques dissipativos de rolamento, limitados a `rolling_resistance * normal_lado * raio * dt`.

O algoritmo de conjunto ativo resolve sistemas de até 7 incógnitas, sem alocação de matrizes no heap. Tem limite de 64 iterações e retorna erro se não convergir ou encontrar sistema singular/não finito. O resíduo de sinal para liberar uma restrição é `1e-9` nas unidades da respectiva equação; não há denominadores artificiais para tornar uma configuração inválida executável.

Em aderência reta e simétrica, a aceleração inclui a massa equivalente `m + 2*Jwheel/r²`. Na rotação diferencial, inclui a inércia `Iz + 2*Jwheel*(half_track/r)²`. Em escorregamento, o impulso é limitado; a roda continua evoluindo por sua inércia. A antiga atribuição direta `omega = velocidade/raio` foi removida.

O rolamento é uma aproximação por torque de Coulomb, com resistência estática até o limite configurado. Pode impedir a partida com torque muito baixo; não representa uma lei de deformação do pneu. Ao parar, a solução escolhe o impulso necessário dentro do limite, evitando ultrapassar zero por subtração de uma força fixa.

Os nomes legados `SlipRatioWheel` e `CoulombFrictionWheel` passam a usar essa mesma base rígida de contato. `slip` é agora a diferença cinemática normalizada por `max(|r*omega|, |v_contato|, slip_velocity_epsilon)`, sem o antigo excesso artificial de demanda. A separação de modelos reais de pneus pertence à etapa 6. Os limites longitudinal e lateral são independentes, formando uma aproximação retangular; não existe ainda elipse/círculo de atrito combinado.

## Motor e alimentação — 2.5 e 2.6

`MotorConfig.nominal_voltage_v` fixa a tensão à qual se referem os dados de catálogo. É carregada, editada e salva na GUI. Os dois assets N20 e os dois exemplos receberam explicitamente `7.4 V`; arquivos legados sem o campo adotam esse mesmo default, que deve ser revisado se o motor tiver outra tensão nominal.

Para o modelo simples:

```text
omega0 = no_load_rpm * 2*pi/60
R = nominal_voltage / stall_current
Ke = nominal_voltage / omega0
Kt = stall_torque / stall_current
Vcmd = PWM_quantizado * max(Vbateria - queda_driver, 0)
I = (Vcmd - Ke * gear_ratio * omega_roda) / R
torque_roda = Kt * gear_ratio * eficiencia * I
```

A equação do torque é avaliada implicitamente nas velocidades resolvidas junto ao contato. Isso evita instabilidade de frenagem por integrar explicitamente um motor rígido acoplado a uma roda de inércia pequena. A saturação usa a corrente permitida; não limita arbitrariamente todo torque ao valor de stall nominal. Sob tensão menor, partida e velocidade sem carga diminuem.

Os dados precisam ter `Kt <= Ke` e eficiência em `(0, 1]`. `Kt > Ke` criaria ganho energético neste modelo simplificado e é rejeitado. `Kt < Ke` representa perdas agregadas; não é um motor eletromagnético ideal nem identifica fisicamente cada perda. Indutância, perdas detalhadas, temperatura, comutação e eficiência reversa continuam na etapa 7.

PWM zero respeita o modo do driver: `coast` desacopla o motor, `brake` aplica a resposta de curto-circuito. A corrente do enrolamento é assinada. O consumo reservado no barramento usa `max(I*PWM, 0)`: energia retornada não recarrega a bateria, sendo tratada como dissipada pelo conjunto motor/driver simplificado.

O limite de corrente agora modifica a atuação:

- Downforce tem prioridade. Se exceder o orçamento, uma busca limitada reduz seu PWM, avançando o modelo a partir do mesmo estado anterior. Apenas o candidato escolhido é confirmado.
- Cada motor reserva metade da corrente restante. O limite de enrolamento considera PWM e o limite próprio do driver. A parcela não usada de um motor não é redistribuída ao outro; é uma política conservadora explícita.
- Limite zero corta consumo/acionamento alimentado. A carga restante da bateria também limita a corrente possível durante o tick. Frenagem passiva continua disponível.
- A bateria recebe a soma realmente usada; deixou de esconder sobrecarga apenas truncando a corrente registrada.

O acoplamento elétrico é **defasado em um tick**: motores e downforce usam a tensão terminal anterior; depois, a bateria atualiza SOC e queda `I*R` para o próximo tick. Isso admite erro transitório de energia na conexão bateria/carga em mudanças bruscas e deve ser avaliado pela convergência do passo. Não equivale a resolver o circuito instantâneo completo, previsto na etapa 7. A força residual de um ventilador desacelerando pode persistir após o corte, conforme seu estado de primeira ordem.

## Diagnóstico e falhas — 2.8

Antes do run são validados massa, inércias, dimensões, atrito, perdas, parâmetros do motor/bateria/driver e valores físicos de downforce usados pelo executor. COM fora do retângulo de apoio é rejeitado, pois tombamento não foi implementado. Isso não substitui a validação de todos os schemas e componentes da etapa 3.

`SimulationCore::diagnostics()` expõe energia cinética, trabalho dos motores, energia elétrica consumida, dissipação total, força lateral, torques de rolamento e iterações do solver. São diagnósticos do último intervalo, separados do formato de replay v3; não são automaticamente uma série adicional no CSV/replay.

O trabalho usa a convenção do integrador implícito: `Wmotor = dt * soma(torque * omega_final)`. O resíduo `Kfinal - Kinicial - Wmotor` deve ser não positivo, dentro de `1e-8 * (1 + Kinicial + |Wmotor|)` joules. A dissipação reportada inclui perdas de contato **e dissipação numérica**, inclusive `0.5 * delta_vᵀ M delta_v`; não deve ser interpretada inteira como calor físico do pneu.

O runtime verifica comandos, observações, sistema de contato, velocidades, energia e estado não finitos; também interrompe se a rotação superar `0.25 rad/tick`, indicando passo inadequado à aproximação de direções congeladas. Não foram introduzidos cortes arbitrários de velocidade para esconder divergência.

`try_step()` retorna erro com instante e subsistema. Falhas físicas preservam o último estado cinemático válido e interrompem a sessão; não há retomada de uma sessão falha. Falhas de observação após a integração são identificadas no timestamp do evento. CLI e calibração propagam o erro; a GUI para e mostra a mensagem. A API compatível `step()` retorna `false` em falha, distinguível pelo método `failure()`.

## Evidências e tolerâncias

Os testes estão em `src/core/physics_tests.rs`, `src/physics_tests.rs` e na suíte de execução da etapa 1. Os dois testes do modelo de força antigo foram substituídos por cenários do solver realmente usado.

| Cenário | Verificação |
|---|---|
| Movimento livre, corpo girando, COM deslocado | Trajetória mundial e transformação de velocidade; tolerância `1e-10` |
| Força mundial e torque conhecidos | Solução analítica de posição, orientação e velocidades; `1e-10` |
| Aderência e rotação diferencial | Massa/inércia equivalentes das rodas; velocidade `1e-10`, força `1e-7` |
| Rodas inicialmente patinando | Força limitada, reação gradual e energia não crescente |
| Rolamento, coast e brake | Sem ganho de energia e sem reversão espontânea, tolerância de repouso `1e-9` a `1e-8` |
| Comando reverso | Cruzamento de zero permitido por atuação, corrente assinada e balanço limitado |
| Tensão e corrente | Menor tensão reduz torque; limite da bateria reduz aceleração; downforce respeita orçamento; corte com limite zero |
| Curva a 1 m/s e 2 rad/s | Força centrípeta próxima de 2 N (`1e-4 N`); erro de posição a 50 µs abaixo de 11 µm em 0,1 s |
| Reta, passos 100/50/25 µs | Erro decrescente contra 5 µs, sem ajustar controlador; a 50 µs: posição `< 10 µm` e velocidade `< 1e-4 m/s` |
| Falhas | Entradas inválidas, NaN/inf, divergência angular, timestamp, parada persistente e ausência de commit de tick inválido |
| Consumidores | CLI, sessão e calibração continuam produzindo os mesmos resultados |

No cenário de reta, os erros de posição contra 5 µs foram aproximadamente `0,296 / 0,136 / 0,060 µm` para passos de `100 / 50 / 25 µs`. São verificações numéricas específicas, não uma medida de precisão perante um robô real.

Benchmarks locais em release, sem gravação de logs: o exemplo de sucção calculou 10 s com física de 50 µs em cerca de **0,364 s** (200.000 integrações; **27,5×** tempo real). O exemplo de pista geométrica completou 10 s a 500 µs em cerca de **0,443 s**. São medições de uma execução neste computador, não garantia de desempenho de outras pistas/modelos.

Comandos de validação:

Resultado: **48 testes aprovados** em cada configuração, com e sem GUI (40 unitários e 8 de integração); formatação aprovada. Existem avisos de código não utilizado. A interface foi compilada, sem avaliação interativa.

```powershell
cargo test --offline --no-default-features
cargo test --offline
cargo fmt --all -- --check
cargo run --offline --release --no-default-features -- benchmark examples/basic/projeto_suction.rtsim --duration 10s --physics-dt-us 50
```

## Limites e próximas etapas

Esta entrega corrige a base plana agregada; não encerra G02–G05 em suas extensões avançadas. Faltam contatos por roda com geometria completa, atrito combinado, transferência dinâmica de carga, deformação, arrasto, circuitos e regeneração completos, parâmetros identificados e validação experimental. A UI permanece síncrona. O benchmark atual não indica necessidade de DLL C para esta base; novas otimizações continuam condicionadas a medições.

## Referências consultadas

A formulação local usa impulsos e restrições de atrito, abordagem descrita na [documentação de computação do MuJoCo](https://mujoco.readthedocs.io/en/latest/computation/). Este projeto não incorpora MuJoCo nem reproduz seu solver.

A referência nominal dos dados de motor e suas limitações seguem a distinção entre grandezas de catálogo e modelo discutida pela [maxon: dados de motor e simulação](https://support.maxongroup.com/hc/en-us/articles/360013761160-Motor-data-and-simulation). As equações, aproximações e testes acima definem a implementação específica deste simulador.
