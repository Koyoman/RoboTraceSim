# Etapa 5 — Pista física, óptica e de corrida

Implementada em 22/09/2026. Contrato da etapa 5 de [tasks.md](../tasks.md), mantendo o objetivo de um simulador configurável de robôs seguidores de linha. Não há compromisso de compatibilidade com configurações ou resultados históricos.

## Definição e execução

`TrackConfig` guarda a definição editável. Retas e arcos de `TrackV2` são a fonte geométrica; `TrackEnvironment` acrescenta regiões, marcas e portais de corrida. `TrackRuntime::try_new` valida e congela essa definição em dados compartilhados por `Arc`. Clonar o runtime reutiliza os dados. Sensores e contatos consultam esse snapshot sem reconstruir geometria. Alterações no editor não alteram uma sessão já iniciada.

`TrackRuntimeCache` compara a definição serializada e só reconstrói quando ela muda. A polilinha derivada não faz parte da chave paramétrica. O editor usa esse cache no desenho da pista; ferramentas auxiliares de validação e largada ainda podem calcular sua própria geometria. Não há índice espacial nesta entrega: consultas percorrem primitivas/regiões; otimização adicional depende de medição na etapa 9.

| Arquivo | Responsabilidade |
|---|---|
| `src/track/definition.rs` | Camadas, unidades, validação, pose inicial, histórico e recorte de polígonos |
| `src/track/runtime.rs` | Snapshot, primitivas analíticas, caches e consultas locais |
| `src/track/events.rs` | Estado da corrida, sequência e término |
| `src/track/persistence.rs` | JSON do ambiente da pista |
| `src/config.rs`, `src/rtsim_track.rs`, `src/io/` | Fonte paramétrica, perfis, resolução e persistência |
| `src/app/track_editor.rs`, `src/ui.rs` | Edição, canvas, diagnósticos e histórico |
| `src/sim.rs` | Consultas por roda, eventos e arquivos de resultado |

## Superfície e óptica

Regiões são retângulos orientados com ID, posição, dimensões, ângulo, material, coeficiente de atrito, refletância, cor RGB, altura e rugosidade. No JSON, comprimentos usam mm; internamente usam metros. Refletância está em [0, 1], atrito é finito e não negativo. Cor de exibição não determina a leitura do sensor.

A última região da lista que contém o ponto define o substrato. Fora do retângulo da mesa paramétrica prevalecem `outside_material`, `outside_mu` e `outside_reflectance`, mesmo que uma região ou marca se estenda além da mesa. Pistas de polilinha sem mesa têm domínio ilimitado.

A consulta óptica segue esta precedência: fora da mesa → última marca contendo o ponto → faixa da linha → substrato local. `paint` fornece sua refletância; `gap` restaura o substrato local, inclusive regiões sobrepostas. A faixa da linha é a união das primitivas, portanto cruzamentos também são detectados. Regiões alteram o substrato; a linha continua sobre elas até que uma falha a remova.

Marcas automáticas de largada/chegada e de entrada/saída de curva são retângulos ópticos de 10 × 40 mm, com afastamento lateral de 40 mm além da meia largura da linha. Essa é a convenção local implementada, sem alegação de homologação de regulamento. Marcas do usuário vêm depois das automáticas e têm prioridade. O canvas usa os mesmos retângulos do runtime; falhas recompõem a cor do substrato com recorte das regiões. Grade, eixos, setas de largada e portais amarelos de corrida são guias e não geram sinal óptico.

Cada sensor continua independente: sua posição local é transformada pela pose do robô e consultada separadamente. A leitura ainda é pontual; integração sobre área, altura óptica e latência ficam na etapa 8.

## Contatos e relevo

Em cada passo físico, as quatro posições das rodas consultam seu atrito local. Os limites `min(mu_pneu, mu_superficie) × normal` são somados por lado para alimentar o solver atual; a interface permite inspecionar material e atrito por roda. Isso elimina a consulta única no centro do chassi, mas não implementa ainda forças independentes e aderência longitudinal/lateral combinada por contato. Essas equações pertencem à etapa 6.

`height_mm` e `roughness_mm` são metadados por região. Valores não nulos produzem aviso; `relief_enabled=true` é rejeitado porque não existe solver vertical. A normal consultada é [0, 0, 1]. Guardar altura não simula deformação, suspensão ou inclinação.

## Resolução e erro

O campo óptico usa distância analítica a retas e arcos, inclusive arcos horários e linhas menores que o espaçamento de desenho. Zoom e FPS não entram nessas equações. A polilinha geométrica usa espaçamento de arco de até 5 mm; a sobreposição geométrica da área de validade do robô usa esse cache.

Para um arco de raio r e intervalo de comprimento ds, o desvio máximo da corda é `r × (1 − cos(ds / (2r)))`, aproximadamente `ds² / (8r)` para pequenos intervalos angulares. Com r = 100 mm e ds = 5 mm, o desvio é aproximadamente 0,03125 mm. Esse erro pertence à representação geométrica amostrada, não à consulta óptica analítica. Curvas muito apertadas podem apresentar diferença visual/geometria amostrada maior e exigem avaliação conforme a tolerância pretendida.

Raio e comprimento devem ser positivos; o módulo do ângulo de arco fica em (0, 360] graus. O cache aceita no máximo 100 mil segmentos e comprimento total de 5 milhões de mm para limitar alocação. Esses limites são de implementação, não normas de competição.

## Corrida e pose inicial

`start_source` escolhe explicitamente `project` ou `track`. A mesma função resolve a pose usada pela GUI, pelo executor e pelo snapshot. Uma fonte de pista sem largada resolvível é rejeitada; polilinhas usam a pose do projeto. O editor apresenta a fonte e a pose efetiva, e o histórico inclui a pose do projeto.

A corrida é opcional. Quando ativa, exige um portal de largada e um de chegada; checkpoints seguem a ordem da lista. Portais possuem centro, meia largura e direção normal de travessia. Sem portais personalizados, largada/chegada são derivadas das marcações, com direção `travel_heading_deg + 180°`, acompanhando a pose inicial existente que aponta para START. Portais personalizados permitem escolher outro sentido explicitamente.

O movimento da origem do robô entre dois passos é intersectado com os portais. Vários portais cruzados no mesmo passo são processados na ordem de passagem. Eventos recebem o instante final do passo físico, sem interpolação temporal abaixo do tick. Tocar o plano e repetir observações não duplica eventos; passagem reversa gera diagnóstico sem avançar a sequência. Uma volta exige largada, checkpoints em ordem e chegada. Depois de uma chegada não terminal, é necessário cruzar novamente a largada para armar a próxima volta; oscilar sobre a chegada não conta outra volta. Sequência inválida impede completar aquela volta.

Entrada/saída da mesa considera cantos do chassi e das rodas. `stop_on_exit` permite encerrar a execução, inclusive quando o robô começa fora. Perda de leitura óptica, sobreposição geométrica da linha e saída da mesa são estados distintos; uma falha pintada não invalida por si só a geometria nem encerra a corrida.

O resumo informa motivo de término e tempo realmente simulado. CSV e replay recebem um sidecar `.events.json` (`rtsim-race-events-v1`) com eventos, voltas e término. O snapshot conserva a duração configurada, enquanto o resumo registra a duração efetiva. Eventos não foram inseridos no replay binário v3; índice, checkpoint e reprodução integrada de eventos ficam para a etapa 9.

## Editor, persistência e regras

O painel permite criar, editar, excluir e reordenar regiões, marcas e portais, além de reordenar segmentos preservando seus IDs. Snapping usa a grade configurada. Undo/redo mantém até 100 operações da pista e da pose do projeto. Referências inválidas a segmentos são detectadas antes da execução; uma edição pode permanecer incompleta até ser corrigida.

O bloco `environment` tem schema próprio `rtsim-track-environment-v1`; ausência do bloco recebe os defaults documentados no código. JSON inválido, não finitos, dimensões inválidas, IDs duplicados e referências de largada inválidas são rejeitados. As restrições numéricas valem em strict, warning e free. O modo strict bloqueia violações de regras; warning/free não dispensam a validade numérica e a capacidade do solver.

`rules.source` e `rules.edition` acompanham pista e perfis de superfície/regulamento. Campos vazios geram aviso de ausência de referência. Preenchê-los registra procedência declarada pelo usuário; não comprova conformidade oficial. Não foi feita certificação externa nem validação experimental dos valores.

## Exemplo e validação

[Projeto de ensaio](../examples/stage5/projeto.rtsim) usa física de 50 µs, controle de 1 ms, uma região de menor atrito, marca lateral, falha e três portais em uma reta. Reutiliza o robô de `examples/basic`; não é um circuito homologado nem um controlador ajustado para completar a prova em três segundos.

```powershell
cargo run --offline --no-default-features -- run examples/stage5/projeto.rtsim --headless --csv target/stage5-demo.csv --replay target/stage5-demo.rtlog
```

O ensaio de três segundos completou 60.000 passos e 3.001 amostras, registrando largada em 2.357.850 µs e término por duração. CSV, replay, metadados, snapshot e eventos foram produzidos. O teste de término antecipado usa outro cenário controlado e verifica chegada, tempo efetivo e persistência do evento final.

A suíte inclui 16 testes em `tests/stage5_track.rs`: sobreposição de materiais, consultas por roda, marcas/falhas, cruzamentos, fechamento, arcos estreitos horários/anti-horários, cache imutável, regras inválidas, relevo, pose, persistência/histórico, sequência e retorno por portais, saída e término antecipado. O teste da GUI renderiza painel/canvas em três níveis de zoom sem abrir janela e verifica a reutilização do runtime. Totais finais e comandos estão registrados em [tasks.md](../tasks.md). Não houve ensaio manual da janela nativa.
