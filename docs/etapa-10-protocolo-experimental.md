# Etapa 10 — Protocolo experimental

Protocolo definido em 23/09/2026, antes de qualquer avaliação de dados físicos reservados. Não há medições de bancada disponíveis no repositório. Este documento define coleta e rastreabilidade; não apresenta resultados físicos inventados.

## Registro obrigatório por ensaio

| Campo | Registrar |
|---|---|
| Identificação | ID único de campanha/run, data, operador, robô, versão de firmware/configuração e montagem |
| Componentes | Motor, redução, driver, bateria, rodas/pneus, sensores e fans identificados por modelo e unidade física |
| Instrumentos | Fabricante/modelo/serial, resolução, precisão, unidade, calibração e incerteza; distinguir erro sistemático de repetibilidade |
| Montagem | Desenho/fotos referenciados, posição dos sensores/rodas/COM, massa/inércia, fixação e orientação dos eixos |
| Superfície | Material, condição, limpeza, textura, inclinação e regiões de atrito/reflectância |
| Ambiente | Iluminação e geometria da fonte, temperatura, umidade se relevante, estado do pneu e altura óptica |
| Alimentação | Tensão em repouso/sob carga, SOC estimado e seu método, temperatura da bateria, cabos e contatos |
| Aquisição | Sinal/unidade/taxa por canal, resolução temporal, filtros, anti-aliasing, triggers, timestamps de aquisição e entrega |
| Proveniência | `measured`, `synthetic` ou `unknown`; fornecedor com referência e condições completas quando aplicável |
| Integridade | Arquivo bruto imutável, hash, transformações/unidades, registros descartados e justificativas |
| Partição | `calibration` ou `validation`, grupo de condições e critério de reserva definidos antes do ajuste |

Na interface de estudo, `protocol` exige os campos textuais `robot_components`, `instrument`, `instrument_accuracy`, `mounting`, `surface`, `lighting`, `temperature`, `battery_state`, `units`, `acquisition_rate`, `date` e `operator`. Eles devem referenciar a ficha detalhada; passar pela validação de formato não autentica uma medição.

## Bancada por subsistema — medições pendentes

| Subsistema | Entradas controladas e condições | Observações necessárias | Parâmetros/checagens |
|---|---|---|---|
| Motor + driver | Tensão/PWM, temperatura, operação em vazio e cargas conhecidas, sentidos e brake/coast | Corrente, tensão nos terminais, RPM, torque/carga, temperatura, tempo de resposta | R, L quando identificável, Ke/Kt, perdas, eficiência, queda do driver e limite de corrente |
| Transmissão | Carga/sentido/velocidade, montagem e lubrificação | RPM em ambos os eixos, torque, reversão | Redução, eficiência, atrito e eventual folga; não atribuir folga a pneu |
| Bateria + cabos | SOC/temperatura, degraus conhecidos de corrente e repouso | Tensão e corrente sincronizadas, duração, temperatura | OCV×SOC, resistência, RC, recuperação e carga integrada |
| Fan/sucção | Tensão/PWM, superfície, folga, geometria do selo e altura | Força/normal, corrente, RPM/pressão quando disponíveis, resposta transitória | Curvas de força/consumo, dinâmica e influência de folga |
| Óptica | Posição/ângulo/altura, materiais de fundo/linha, iluminação e velocidade de varredura | ADC bruto/digital, referência óptica, posição, aquisição/entrega | Área efetiva, resposta, ruído, saturação, filtro, limiar/histerese e latência |
| Pneu/contato | Normal conhecida, superfície, velocidade, slip e direção da força | Forças longitudinal/lateral, torque, velocidade do corpo/roda, temperatura, deformação observável | Atrito, aderência combinada, rigidez/relaxação, rolamento e dependência da carga |
| Robô completo | Pistas/velocidades/tensões reservadas, pose inicial e controlador fixo | Trajetória externa, velocidade/yaw, sensores, corrente/tensão e eventos | Predição fora do ajuste, frenagem, perda de linha, volta e energia |

Instrumentação de bancada deve respeitar limites nominais de motor, driver, bateria e carga. Não usar o simulador ainda não qualificado para estabelecer esses limites. Dados de fabricante só entram como medidos quando o método e as condições são identificados; valores típicos sem condições são hipóteses de configuração.

Repetir ensaios suficientes para estimar dispersão e deriva; determinar o número com base no ruído observado, sem declarar que três seeds simuladas representam três repetições físicas. Registrar também ensaios falhos, aquecimento e saturação.

## Sincronização e equivalência

1. Usar origem explícita: `t_sim = t_log - origin_us + offset_us`. Não deduzir silenciosamente a origem a partir da primeira linha.
2. Preferir trigger comum. Quando necessário, estimar offset em sinal excitado **somente na calibração**, com bounds conhecidos e cobertura mínima; congelar a regra para os dados reservados.
3. Guardar timestamps de aquisição e disponibilidade. ADC entregue ao controlador não é o mesmo sinal que reflectância no instante atual. A ferramenta de estudo compara entrega; dados de aquisição exigem conversão explícita, sem apagar timestamps originais.
4. Definir transformação rígida XY/yaw para a pista, em metros/radianos. Velocidade `vx_body_m_s` permanece no referencial do corpo. Conversões de escala/unidades são pré-processamento documentado, não um ajuste oculto.
5. CSV científico começa por `t_us` inteiro. Aceita amostragem irregular e campos vazios; rejeita NaN/infinito, duplicatas, ordem temporal invertida e linhas inconsistentes. Não ordena nem elimina dados silenciosamente.
6. `max_gap_us` limita interpolação. Grandezas contínuas usam interpolação linear e yaw usa menor arco; ADC, PWM, flags e contadores são retidos. Não extrapolar além dos dados. Cobertura por amostras e por tempo são indicadores diferentes.

Os canais ópticos podem incluir `sensor_00_acquired_us`, `sensor_00_available_us` e `sensor_00_valid`; os timestamps são transformados juntos. Leituras marcadas inválidas ou disponíveis no futuro não participam da comparação. Sem esses campos, o responsável pelo ensaio precisa garantir que `t_us` representa uma leitura já entregue. Mapear os índices ADC aos IDs/posições do snapshot do robô.

## Tolerâncias anteriores à validação

Antes de abrir o conjunto reservado, registrar, por sinal: faixa operacional, incerteza do instrumento, erro de sincronização/registro espacial, repetibilidade, tolerância de modelo aceitável e cobertura mínima. Combinar incertezas somente sob hipóteses justificadas (por exemplo independência); não somar números sem justificar a correlação. Documentar se a tolerância é RMS, máximo, viés ou probabilidade de cobertura.

O manifesto congela `unit`, `scale`, `weight`, `max_rms`, `min_coverage` e `tolerance_basis`; seu hash é gravado no relatório. `scale` normaliza o objetivo, não é automaticamente desvio padrão. Uma aprovação numérica não certifica que a tolerância foi cientificamente bem escolhida. **Não há tolerâncias físicas numéricas definidas aqui**, porque a precisão dos instrumentos ainda é desconhecida.

Separar campanhas/grupos de tensão, pista, velocidade e condições ambientais. Não copiar o mesmo log com outro nome: conteúdo repetido e grupos sobrepostos entre calibração/validação são recusados. O programa detecta essas formas de vazamento, mas não prova independência física de dados declarados por terceiros.

## Efeitos adicionais: decisão atual

| Efeito | Evidência necessária antes de ampliar o modelo | Decisão atual |
|---|---|---|
| Deformação avançada/relaxação | Força×slip e transientes sob várias normais | Manter aproximações reduzidas; aguardar curvas de bancada |
| Temperatura/desgaste do pneu | Séries repetidas, temperatura e estado do material | Adiar novos estados até separar deriva de erro de montagem |
| Vibração/shimmy | Medição temporal/IMU adequada e resposta do apoio | Adiar dinâmica extra; não inferir de erro de PID |
| Arrasto | Ensaio de coast-down com perdas mecânicas caracterizadas | Manter termos existentes; identificar coeficientes antes de enriquecer |
| Selo/folga da sucção | Pressão/força×folga/superfície e transientes | Manter folga configurada; adiar acoplamento vertical |
| Relevo, pitch/roll e suspensão | Perfil de pista e movimento vertical medidos | Manter aproximação planar e declarar limite |
| Circuito detalhado/BLDC | Tensões/correntes com banda suficiente para os fenômenos pretendidos | Manter modelo médio disponível; não adicionar chaveamento sem ganho demonstrado |

Nenhum efeito novo foi introduzido para reduzir erro contra dados sintéticos gerados pelo próprio modelo. A decisão pode ser revisada quando resíduos físicos repetíveis ultrapassarem tolerâncias pré-definidas, e o efeito adicional melhorar dados reservados com custo aceitável.
