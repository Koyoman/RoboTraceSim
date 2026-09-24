# Dados de qualificação

Não há medições físicas qualificadas neste repositório. `examples/basic/real_log_demo.csv` tem **proveniência desconhecida**: seu nome não comprova coleta real. Assets existentes são parâmetros de configuração, não certificados de bancada.

Separar cada campanha por robô, data e condição. Preservar CSV original imutável, protocolo, conversões documentadas, configuração resolvida, condições de calibração/validação e hashes do relatório. Não reutilizar ensaios reservados para escolher bounds, offsets, modelos ou tolerâncias.

O protocolo obrigatório está em [protocolo experimental](../docs/etapa-10-protocolo-experimental.md). A estrutura executável é demonstrada por `examples/stage10_fixture.rs`; ela gera **somente dados sintéticos**, sob `target/`, identificados no manifesto e nos relatórios.

Para dados reais, preencher o protocolo com instrumento/precisão/montagem/condições medidos, converter explicitamente para SI e construir o manifesto `rtsim-study-v1`. Não trocar `kind` por `measured` em uma fixture sintética. Campos em branco permanecem ausentes; não preencher lacunas com zero.
