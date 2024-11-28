import re
import itertools
import numpy as np
import pandas as pd
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
from tabulate import tabulate
import time

# Configurações do Broker MQTT
BROKER = "broker.hivemq.com"
PORT = 1883

# Configurações iniciais
ciano = '\033[96m'  # Cor azul-ciano

# Função para definir os universos e funções de pertinência
def define_variables():
    # Universos
    error = ctrl.Antecedent(np.arange(0, 1000.1, 0.1), 'error')
    delta_error = ctrl.Antecedent(np.arange(-1000, 1000.5, 0.5), 'delta_error')
    motor_power = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'motor_power')

    # Funções de pertinência
    error['Z'] = fuzz.trimf(error.universe, [0, 0, 5])
    error['P'] = fuzz.trimf(error.universe, [0, 15, 25])
    error['M'] = fuzz.trimf(error.universe, [15, 45, 75])
    error['G'] = fuzz.trimf(error.universe, [50, 125, 350])
    error['MG'] = fuzz.trimf(error.universe, [125, 770, 1000])

    delta_error['MN'] = fuzz.trapmf(delta_error.universe, [-1000, -1000, -3, -1.5])
    delta_error['N'] = fuzz.trimf(delta_error.universe, [-3, -1.5, 0])
    delta_error['Z'] = fuzz.trimf(delta_error.universe, [-1.5, 0, 1.5])
    delta_error['P'] = fuzz.trimf(delta_error.universe, [0, 1.5, 3])
    delta_error['MP'] = fuzz.trapmf(delta_error.universe, [1.5, 3, 1000, 1000])

    motor_power['MP'] = fuzz.trapmf(motor_power.universe, [0, 0, 0.03, 0.18])
    motor_power['P'] = fuzz.trimf(motor_power.universe, [0.08, 0.18, 0.4])
    motor_power['M'] = fuzz.trimf(motor_power.universe, [0.25, 0.4, 0.55])
    motor_power['G'] = fuzz.trimf(motor_power.universe, [0.4, 0.62, 0.8])
    motor_power['MG'] = fuzz.trapmf(motor_power.universe, [0.62, 0.88, 1, 1])

    return error, delta_error, motor_power


# Função para exibir as funções de pertinência
# def show_fuzzyfication(error, delta_error, motor_power):
    # error.view()
    # plt.axline((0, 0.25), (1000, 0.25), color='black', linestyle='--')
    # plt.xlim(0, 400)
    #
    # delta_error.view()
    # plt.axline((0, 0.4), (1000, 0.4), color='black', linestyle='--')
    # plt.xlim(-250, 250)
    #
    # motor_power.view()
    # plt.axline((0, 0.4), (1000, 0.4), color='black', linestyle='--')
    # plt.show()


# Função para criar as regras de controle
def create_rules(error, delta_error, motor_power):
    power_result = [
        'MP', 'P', 'M', 'G', 'MG',
        'MP', 'P', 'M', 'M', 'M',
        'P', 'P', 'M', 'M', 'G',
        'P', 'M', 'M', 'G', 'G',
        'M', 'M', 'G', 'G', 'MG',
    ]

    BaseRules = [
        ctrl.Rule(error[Error] & delta_error[Delta_error], motor_power[Motor_power])
        for (Error, Delta_error), Motor_power in
        zip(itertools.product(error.terms.keys(), delta_error.terms.keys()), power_result)
    ]

    return BaseRules


# Função para exibir a tabela de regras
def show_table(error, delta_error, motor_power, BaseRules):
    table = []
    for Erro in error.terms:
        for Delta_erro in delta_error.terms:
            for regra in BaseRules:
                antecedente = str(regra).split('IF ')[1].split(' THEN')[0].replace('AND ', '')
                consequente = str(regra).split('IF ')[1].split(' THEN')[1].split('AND ')[0]

                classificacoes = re.findall(r'\[(.*?)\]', (antecedente + consequente))
                if Erro == classificacoes[0] and Delta_erro == classificacoes[1]:
                    table.append([classificacoes[0], classificacoes[1], classificacoes[2]])
                    break

    df = pd.DataFrame(table, columns=[error.label, delta_error.label, motor_power.label])
    pivotTable = pd.DataFrame(df.pivot(index=delta_error.label, columns=error.label, values=motor_power.label).reindex(
        index=delta_error.terms, columns=error.terms))
    pivotTable.index.name = f'{ciano}{"E"}\033[0m'
    print(tabulate(pivotTable, headers='keys', tablefmt='fancy_grid', stralign='center', showindex='always'))


# Função para simular o controle
def control_simulation(baseRules):
    setpoint = 1
    current_position = 1000
    t = 0
    dt = 1

    ErrorControl = ctrl.ControlSystemSimulation(ctrl.ControlSystem(baseRules))

    positions = [current_position]
    errors = [abs(setpoint - current_position)]

    client = mqtt.Client()
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    while True:
        try:
            if current_position < setpoint:
                U_max = 6
            else:
                U_max = 4

            current_error = abs(setpoint - current_position)
            errors.append(current_error)

            if current_error < 10:
                FA = 0.984825
            elif current_error < 25:
                FA = 0.994
            else:
                FA = 0.996

            current_delta_error = (errors[-1] - errors[-2])

            ErrorControl.input['error'] = current_error
            ErrorControl.input['delta_error'] = current_delta_error
            ErrorControl.compute()

            P_Motor = ErrorControl.output['motor_power']

            if current_error > 5:
                P_H13 = P_Motor
                P_H24 = P_Motor
            else:
                P_H13 = 0.353
                P_H24 = 0.353

            d_t = FA * current_position * 1.01398 + 0.5 * (U_max * P_H13 + U_max * P_H24)

            if current_position < setpoint:
                current_position = d_t
            else:
                delta_movement = d_t - current_position
                current_position = current_position - delta_movement

            positions.append(current_position)
            t += 1

            # Publicar no MQTT
            # payload = f"Tempo: {t}, Deslocamento: {current_position:.2f}"
            position_payload = f"\n{current_position:.2f}"
            client.publish("drone/deslocamento", position_payload)

            motor_payload = f"\n{P_H13*100:.2f}"
            client.publish("drone/potencia", motor_payload)

            error_payload = f"\n{current_error:.2f}"
            client.publish("drone/erro", error_payload)

            print(f"Publicado: {position_payload}")

            time.sleep(0.2)


        except Exception as e:
            print(f"Erro no loop principal: {e}")

    # Gráfico do deslocamento
    # plt.plot(range(len(positions)), positions, label='Deslocamento (m)', color='blue')
    # plt.axhline(setpoint, color='red', linestyle='--', label='Setpoint')
    # plt.xlabel('Tempo (s)')
    # plt.ylabel('Deslocamento (m)')
    # plt.title('Deslocamento do Drone ao Longo do Tempo')
    # plt.grid()
    # plt.legend()
    # plt.show()


# Função principal
def main():
    erro, delta_erro, potencia_motor = define_variables()
    # show_fuzzyfication(erro, delta_erro, potencia_motor)
    base_rules = create_rules(erro, delta_erro, potencia_motor)
    show_table(erro, delta_erro, potencia_motor, base_rules)

    control_simulation(base_rules)


if __name__ == "__main__":
    main()