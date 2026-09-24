# Etapa 4 — Montagem física e editor de robô

Implementada em 22/09/2026. O aplicativo permanece 0.6.0; o formato de robô passa a `rtsim-robot-v8`. Não há obrigação de compatibilidade com versões anteriores.

## Fonte de verdade

`src/models/robot.rs` contém `RobotAssembly`, `WheelInstance`, `MassElement`, `MassProperties`, transformações, validação e histórico de edição. Não depende da GUI. `src/app/robot_editor.rs` contém o painel e o preview extraídos de `ui.rs`; a aplicação conserva navegação, arquivos e sessão.

`RobotConfig.assembly` é a montagem editável. Na resolução do experimento, o adaptador valida essa montagem, calcula massa/COM/inércia, extrai a geometria admitida pelo solver e congela os valores efetivos. Alterações posteriores no editor não mudam uma execução existente.

Os campos agregados `drivetrain` são uma representação de adaptação para o núcleo atual; **quando há montagem explícita, as rodas são a autoridade**. O pneu global é um modelo de catálogo: deve ser aplicado às rodas pelo botão do editor. Isso evita que uma alteração de catálogo sobrescreva instâncias continuamente. O snapshot registra tanto a montagem quanto os valores efetivamente consumidos pelo núcleo.

## Rodas, apoios e motores

Cada roda/apoio tem ID, posição XY, altura do contato, ângulo, raio, largura, inércia própria, material, pneu e tipo `Driven`, `Passive` ou `Caster`. Rodas motrizes devem referenciar `motor:left` ou `motor:right`; apoios passivos/caster não referenciam motor. IDs vazios, repetidos ou conflitantes com outros componentes são rejeitados.

A conversão do drivetrain anterior cria quatro instâncias em um retângulo centrado na origem. Duas rodas de cada lado compartilham o motor daquele lado. Como o solver antigo tem uma inércia equivalente por lado, cada roda recebe metade dessa inércia: a soma preserva a dinâmica anterior da montagem equivalente.

O solver `planar-impulse-v2` ainda exige quatro rodas motrizes alinhadas, retângulo centrado, raios/larguras/pneus iguais e somas de inércia iguais entre os lados. Essas condições são verificadas antes de iniciar. Montagens com raios diferentes, caster, posição não retangular ou contato elevado podem ser editadas e salvas, mas a execução é recusada com motivo explícito. Não se faz média silenciosa de características incompatíveis.

Isso cumpre a estrutura e a validação da montagem da etapa 4; contatos independentes, rodas assimétricas em movimento, caster dinâmico, transferência de carga e atrito combinado continuam na etapa 6. Os estados de rotação continuam agregados por lado enquanto esse solver estiver ativo.

## Massa, centro de massa e inércia

Há dois modos **exclusivos**:

- **Measured:** os valores globais de massa, COM XY e inércia do painel Chassis são o override medido. A altura medida do COM fica na montagem. A lista de componentes não é somada ao total.
- **Components:** somente a lista de massas participa do cálculo; os valores globais não são adicionados. Cada elemento tem massa, posição, altura e inércia própria de yaw. Pode referenciar um componente conhecido ou representar uma peça estrutural independente.

Para massas associadas a rodas, sensores e fans, XY acompanha a montagem correspondente. Z dos sensores acompanha a altura do sensor; Z das rodas é altura de contato mais raio. Fans usam a altura definida no elemento de massa. Bateria, motores, chassi e peças independentes usam a posição de massa editada. Cada referência pode aparecer somente uma vez; referências inexistentes ou repetidas geram erro.

O cálculo usa:

- `M = soma(m_i)`;
- `COM = soma(m_i * posição_i) / M`, incluindo Z;
- `Iz = soma(Iz_i + m_i * distância_xy_ao_COM²)`.

Z não entra no teorema dos eixos paralelos para o eixo vertical. Massa total e inércia final precisam ser positivas e finitas. Massas individuais podem ser zero para representar componentes ainda não pesados.

Na conversão, a massa antiga inteira é atribuída ao chassi e os componentes adicionais começam com massa zero. O usuário deve redistribuir essa massa antes de preencher os demais componentes: o programa impede soma dos dois modos e referência duplicada, mas não consegue inferir se uma massa informada como chassi já inclui a bateria real.

## Referenciais e altura

A origem do desenho é a mesma origem do corpo utilizada na simulação: centro do retângulo de apoio da montagem suportada. X aponta para a frente, Y para a esquerda e Z para cima. A pose do robô é a posição/orientação dessa origem na pista, não a posição do COM. O integrador faz o transporte para o COM ao calcular a dinâmica.

A transformação compartilhada é `posição_mundial = translação_do_robô + rotação_yaw * posição_local`. Rodas e sensores usam essa mesma transformação no preview e no runtime. A geometria do chassi fica centrada na origem; deslocar o COM não desloca o desenho do chassi.

Alturas de COM/sensores/apoios são persistidas. O runtime permanece planar: não calcula pitch, roll, deformação vertical ou variação óptica com altura. Altura não nula de contato torna a montagem incompatível com o solver atual. COM fora do polígono de apoio gera aviso no editor e impede execução porque tombamento não está implementado.

## Sensores e visualização

Cada sensor conserva ID, asset incorporado, posição, ângulo, altura, aquisição, resposta e RNG próprios. A leitura é pontual no centro transformado para a pista. A orientação altera o desenho da área de detecção, mas não introduz um efeito óptico inexistente na amostragem pontual. Área efetiva, altura óptica, latência e filtros pertencem à etapa 8.

O editor mostra rodas orientadas, pontos de contato, polígono de apoio, COM calculado/medido, sensores e suas áreas e fans. O simulador visual desenha a montagem congelada da sessão na pose calculada, incluindo as posições individuais dos sensores. O player de replays antigos ainda usa a representação reduzida; reconstrução completa a partir do snapshot pertence à etapa 9.

## Operações no editor

No painel **Montagem física**, é possível selecionar por ID, inserir roda/apoio, sensor ou fan, duplicar e remover. Duplicar cria ID novo e preserva as características e eventual massa associada; remover pelo painel de montagem remove também essa massa. Os painéis existentes de assets continuam disponíveis.

O canvas permite selecionar/mover com botão esquerdo, deslocar a câmera com botão direito/meio e usar scroll para zoom. A seleção é destacada. Coordenadas em mm, rotação, alinhamento a X/Y zero, ajuste à grade de 1 mm e distância à origem estão disponíveis no painel. Fans são axissimétricos no modelo atual e não têm rotação editável.

**Desfazer/Refazer** guarda definições completas, incluindo IDs e assets, com limite de 100 operações. Movimentos contínuos são agrupados enquanto o ponteiro está pressionado; edição textual é concluída ao deixar o foco. Uma nova alteração após desfazer elimina o ramo de refazer. Carregar/criar outro robô reinicia o histórico. Histórico é da sessão, não é serializado.

## Persistência e exemplos

O schema v8 acrescenta `assembly` e `sensors[].height_mm`; a montagem tem `wheels`, `mass_mode`, `measured_com_height_mm` e `masses`. Arquivos anteriores conhecidos sem montagem usam conversão explícita do drivetrain. O salvamento sempre escreve v8 com a montagem expandida. Os exemplos `examples/basic/robot.json` e `robot_suction.json` foram atualizados.

Comprimentos são mm nos arquivos e metros no domínio; massas são g nos arquivos e kg no domínio; inércia da montagem é kg·m². A inércia antiga `wheel_inertia_g_cm2` é equivalente por lado, enquanto `assembly.wheels[].inertia_kg_m2` é por roda.

## Evidência e limites da verificação

- **69 testes sem GUI**, incluindo 10 testes novos de montagem e todos os testes anteriores.
- **70 testes com GUI**, incluindo renderização automatizada do painel, preview local e montagem na pista em contexto egui sem janela.
- Caso analítico com duas massas: 4 kg, COM `(20, 15, 35)` mm e inércia de yaw `0,0045 kg·m²`.
- Round-trip de rodas com propriedades distintas, alturas, massas, IDs e sensores; desfazer/refazer com assets completos.
- Testes de associação inválida, IDs duplicados, polígono degenerado, valores não finitos, COM fora do apoio e recusa de montagens não suportadas.
- Execução CLI de 100 ms, física de 50 µs e controle de 1 ms: **2.000 passos**, CSV/replay e snapshot com quatro rodas explícitas.

Não foi realizado ensaio manual de interação na janela nativa. Não há validação experimental de pneus, massa, motores ou óptica por esta etapa; os testes verificam contratos, geometria e consistência numérica.
