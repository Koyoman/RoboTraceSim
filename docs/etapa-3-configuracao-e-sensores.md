# Etapa 3 — Configuração, persistência e sensores independentes

> Registro da entrega da etapa 3. O formato atual evoluiu para v8 na [etapa 4](etapa-4-montagem-e-editor.md), com montagem física explícita. As referências a v7 abaixo descrevem aquela entrega.

Implementação das tarefas 3.1–3.8 em 22/09/2026, incorporando a decisão do usuário: **não há obrigação de compatibilidade com versões anteriores**. O objetivo é representar corretamente o robô atual e seu experimento. Conversões existentes facilitam atualizar exemplos; não garantem resultados históricos idênticos.

## Domínio e estrutura

| Estrutura | Responsabilidade |
|---|---|
| `ProjectConfig`, `RobotConfig`, `TrackConfig` | Definições editáveis, em unidades internas explícitas |
| `RobotSensorInstance`, `FanConfig` | Instâncias com IDs persistentes; parâmetros distintos são preservados |
| `io/models.rs` | Seletores canônicos e rejeição de nomes desconhecidos |
| `io/validation.rs` | Validação de documentos, parâmetros físicos, IDs e tabelas |
| `io/assets.rs` | Resolução relativa ao arquivo proprietário e geração de IDs de novas instâncias |
| `io/persistence.rs` | Escrita de projetos, robôs, pistas e todos os perfis, compartilhada pela GUI e API |
| `io/experiment.rs` | `ResolvedExperiment`, conexões, fingerprints, snapshot e exportação de pasta portável |
| `RobotState`, estados dos dispositivos | Estado dinâmico privado da execução, separado das definições |

Sensores e fans mantêm seu ID ao salvar/reabrir ou editar parâmetros. Duplicar um sensor ou adicionar um fan gera um novo ID. Componentes fixos da transmissão agregada têm IDs reservados, como `motor:left`, `wheel:right`, `battery` e `controller`. As conexões atuais são derivadas da montagem suportada; um editor de conexões arbitrárias pertence à evolução da montagem.

`NormalForceConfig.model` tornou-se um enum: deixou de existir o par concorrente `model`/`model_kind`. O runtime usa esse seletor único. Motor, pneu e modo do driver são resolvidos em seletores tipados antes do run; as strings editáveis de catálogo passam por validação. Seletores de modelos desconhecidos retornam erro. Configurações de resposta de sensor e áreas de detecção permanecem enums com dados específicos por variante.

## Cada sensor é uma instância física separada

Cada sensor possui `id`, nome, posição X/Y, ângulo, habilitação, asset com resposta própria e um bloco `acquisition` com resolução ADC, ruído e seed. Seu ponto mundial é:

```text
p_sensor_mundo = p_origem_robo + R(yaw_robo) * p_sensor_no_robo
```

O runtime consulta a refletância nesse ponto, aplica a resposta daquela instância, gera seu ruído com um RNG próprio e quantiza no seu ADC. Mover outro sensor, reordenar a lista ou desabilitar um vizinho não consome os números aleatórios dessa instância. Seeds iguais continuam possíveis por escolha do usuário e podem produzir ruídos correlacionados.

Não existe mais redução da montagem a uma barra uniforme. A posição de linha para o PID é calculada **depois**, como agregação das leituras individuais e suas posições laterais. `sensor_adc` segue a ordem dos sensores habilitados; `SimulationCore::sensor_ids()` e o snapshot identificam os canais. Ao comparar logs, é preciso manter o mapeamento de IDs/canais consistente.

Esta etapa antecipou a leitura **pontual** da tarefa 4.4. O ângulo próprio e a área de detecção são preservados/editáveis, mas ainda não alteram uma consulta feita no centro do sensor. Integração sobre a área, orientação óptica, altura e latência pertencem às etapas 4/8. Sensores de distância ou resposta `Custom` podem ser armazenados no editor, mas devem estar desabilitados neste runtime; a execução retorna erro explícito se estiverem ativos.

A GUI deixou de sobrescrever automaticamente os assets dos demais sensores com o primeiro, bem como seus flags de habilitação/visibilidade. Cada instância oferece edição de asset e aquisição. Também foram removidas a cópia automática do motor esquerdo sobre o direito e a sincronização permanente dos fans; parâmetros distintos sobrevivem à edição e à persistência.

## Schemas e conversões

| Formato | Versão atual |
|---|---|
| Aplicativo | `0.6.0`, obtida de `Cargo.toml` nas identificações da CLI/GUI/relatório |
| Projeto | `rtsim-project-v1` |
| Robô | `rtsim-robot-v7` |
| Pista paramétrica | `rtsim-track-v2` |
| Pista por polilinha | `rtsim-track-v1` |
| Perfis de componentes/superfície | Seus schemas `rtsim-*-profile-v1` |
| Configuração congelada | `rtsim-resolved-experiment-v1` |

Versões de arquivo desconhecidas são rejeitadas. O leitor ainda converte as estruturas conhecidas de robô v1–v6 para v7; isso não é um compromisso de suportar todos os arquivos antigos ou preservar suas trajetórias.

Um `line_sensor` antigo, quando não existe lista explícita, é convertido para N instâncias. O exemplo original gera 16 posições individuais, da esquerda para a direita, preservando largura, avanço, ganho, offset, ADC e amplitudes de ruído. A instância `i` recebe seed `seed_base + i`; isso substitui o RNG compartilhado e muda a sequência histórica. O arquivo salvo contém os sensores independentes, sem bloco de array global.

O antigo `downforce_model` era um segundo seletor não utilizado pelo runtime. Duplicações conhecidas de tipo compatível são descartadas na conversão em favor de `normal_force.model`, dos parâmetros físicos externos e dos fans. Tipos conflitantes são rejeitados. Parâmetros que existiam apenas nesse bloco redundante não são preservados: precisam ser configurados no modelo canônico. Modelos avançados antes anunciados sem implementação não passam a ser implementados por essa conversão.

Os dois exemplos de robô foram gravados como v7; o array legado original permanece apenas como fixture de teste. A antiga cópia do motor esquerdo para o direito no carregamento foi corrigida.

## Validação e JSON

O parser próprio foi corrigido, sem acrescentar dependências. Ele interpreta UTF-8, escapes de controle e pares substitutos Unicode, rejeita números não finitos, vírgulas finais, chaves duplicadas e sintaxe numérica inválida. A escrita escapa os controles e usa representação numérica sem o truncamento decimal fixo anterior.

A validação antecede conversões numéricas para inteiros e rejeita tipos incorretos, inteiros fracionários, resoluções ADC fora de 1–24 bits, IDs vazios/duplicados, dimensões físicas inválidas, tabelas desordenadas e modelos inexistentes. Períodos seguem o contrato da etapa 1; massa, inércia, motor, alimentação e contato seguem o contrato físico da etapa 2. Números inteiros persistidos ficam limitados à faixa exata do parser baseado em `f64` (`2^53 - 1`).

Erro impeditivo é retornado por `Result`. Avisos do experimento são separados, disponíveis em `ResolvedExperiment::warnings()` e no resumo de execução; a CLI os apresenta. A validação não equivale a uma calibração: parâmetros sem certificado experimental e a aproximação pontual dos sensores são identificados como limitações.

Salvar/reabrir preserva instâncias e os parâmetros serializados. Pequenas diferenças de representação binária podem ocorrer na conversão mm↔m; os testes usam tolerância de `1e-12` para esses valores. Saídas de log desabilitadas são salvas como `null`, em vez de reaparecerem como nomes padrão.

## Assets e portabilidade

- Projeto resolve robô/pista relativamente à pasta do `.rtsim`.
- Robô resolve referências de sensor relativamente à pasta do arquivo do robô.
- Não há tentativa adicional de encontrar o arquivo pelo diretório atual do processo, nem substituição por um sensor padrão quando um asset referenciado está ausente/inválido.
- A escrita do robô incorpora o asset efetivo de cada sensor. Quando `asset` está incorporado, ele é a definição autoritativa; `asset_path` serve como indicação da origem para o editor e não exige o arquivo externo.
- `save_project_bundle(config, destino)` salva projeto, robô e pista em uma pasta, com referências locais e assets incorporados. Remove dependência das referências externas de sensores e usa nomes locais para logs.

Os exemplos não contêm caminhos absolutos. A operação de salvar um bundle é API; não foi adicionada uma nova janela de empacotamento nesta etapa. O salvamento comum do projeto continua usando os caminhos escolhidos pelo usuário.

## Configuração congelada

Cada `SimulationCore` cria um `ResolvedExperiment` sem acesso mutável externo. Ele contém uma cópia completa da definição efetiva, modelos resolvidos, regras de pista com defaults expandidos, instâncias, conexões, avisos e fingerprints das fontes disponíveis. O core usa essa definição congelada; alterações posteriores no editor não modificam o run.

Overrides de passo e duração são aplicados **antes** da resolução. Para cada CSV/replay criado, o executor grava, além dos metadados temporais, `<arquivo>.experiment.json` com projeto, robô, pista, conexões e fontes. O snapshot registra os valores em memória; os fingerprints descrevem os arquivos existentes no momento da resolução, que podem diferir de uma edição ainda não salva. A definição incorporada é a autoridade sobre o que foi executado.

O fingerprint é **FNV-1a de 64 bits**, identificado pelo nome `fnv1a64`; serve para detectar mudanças, não para autenticação ou proteção contra colisões deliberadas. Fontes ausentes de definições criadas em memória ou assets incorporados geram aviso, não desaparecem do snapshot. A exigência de referência externa no carregamento continua estrita.

Este snapshot de configuração não é um checkpoint de estado dinâmico e não permite retomar um run intermediário; isso permanece na etapa 9.

## Unidades

| No catálogo/arquivo | No domínio/runtime |
|---|---|
| Massa em g | kg |
| Posições, largura, raio em mm | m |
| Torque em mN·m | N·m |
| Inércia de roda em g·cm² | kg·m², fator `1e-7` |
| RPM do motor | rad/s no cálculo, antes da redução |
| Ângulos de montagem em graus | Preservados em graus na definição; convertidos nas transformações |
| Yaw de `start_pose_m` | radianos |
| Períodos em µs | inteiros em µs |
| Tensão/corrente/força | V / A / N |

## Verificação

`tests/stage3_configuration.rs` verifica Unicode e sintaxe JSON, round-trip com motores e sensores distintos, IDs, aquisição, conversão das 16 instâncias, erros de modelo/schema/asset, curvas, configuração congelada, fingerprints, conexões e execução após mover uma pasta. Também verifica a leitura individual sob translação/rotação do robô e independência dos RNGs em reordenação/desabilitação.

As suítes das etapas 1 e 2 continuam verificando agenda, paridade CLI/sessão/calibração e física. A GUI é compilada; teste interativo do editor não faz parte dessa evidência.

Em 22/09/2026, passaram 59 testes: 40 unitários, 8 de execução temporal e 11 de configuração/sensores. As suítes passaram com e sem a feature GUI. A execução CLI de 2 ms produziu 40 integrações de 50 µs, controle a cada 1.000 µs, CSV, replay e sidecars de configuração com 16 sensores. A formatação e a verificação de whitespace também passaram.

```powershell
cargo test --offline --no-default-features
cargo test --offline
cargo fmt --all -- --check
```
