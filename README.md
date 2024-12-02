# Projeto de Controle Fuzzy-PD para Drones

## Descrição do Projeto
Este projeto visa implementar um sistema de controle Fuzzy-PD para estabilizar o movimento de descida de um drone no modo S-Mode. Utilizamos a lógica fuzzy para ajustar os parâmetros proporcional e derivativo, garantindo maior precisão e estabilidade durante a operação. A comunicação entre os componentes é realizada via protocolo MQTT, e a interface de controle foi desenvolvida no Node-RED.

## Contexto do Problema
A estabilidade de voo é fundamental para o desempenho de drones, especialmente em missões autônomas. Problemas como instabilidade e interferências podem comprometer o controle e a precisão de medições. Este projeto propõe uma solução que combina controle fuzzy com ajustes dinâmicos dos motores para resolver esses desafios.

## Arquitetura do Projeto
- **Protocolo de Comunicação**: MQTT para troca de mensagens entre os componentes.
- **Controle Fuzzy-PD**: Implementado com as bibliotecas `skfuzzy` e `skfuzzy.control`.
- **Interface Visual**: Node-RED, proporcionando uma interface intuitiva para monitoramento e controle.

## Funcionalidades Implementadas
1. **Definição de Variáveis Fuzzy**:
   - Universos: erro, delta erro, e potência do motor.
   - Funções de pertinência para entradas e saídas.
2. **Criação de Regras**:
   - Base de regras fuzzy para ajustar a potência dos motores de acordo com o erro e delta erro.
3. **Simulação do Controle**:
   - Controle de altitude com ajuste automático de potência.
4. **Comunicação via MQTT**:
   - Tópicos para comandos de destino, movimentos e feedback de posição.

## Execução do Projeto
1. Clone o repositório:
   ```bash
   git clone https://github.com/bsmvictor/C213-ControleFuzzyPD
   ```
2. Execute o programa principal:
   ```bash
   python main.py
   ```
3. Configure a interface Node-RED
   ```bash
   Importe NodeRed_flow.json
   ```

## Tecnologias Utilizadas
- **Python**: Lógica fuzzy com as bibliotecas `skfuzzy` e `numpy`.
- **Node-RED**: Criação da interface de controle.
- **MQTT**: Protocolo para comunicação entre componentes.

## Contribuidores
- Victor Boaventura Souza Muniz
- Yves Antonio Guimarães Ribeiro
