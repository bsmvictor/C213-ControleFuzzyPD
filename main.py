import re
import itertools
import numpy as np
import pandas as pd
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import paho.mqtt.client as mqtt
from tabulate import tabulate
import time

free_move = False
is_going_home = False
is_moving = False
current_position = 0
setpoint = None
origin = None
destination = None
errors = []
direction = None

# Configurações do Broker MQTT
BROKER = "test.mosquitto.org"
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


# Função para criar as regras de controle
def create_rules(error, delta_error, motor_power):
    power_result = [
        'MP', 'P', 'M', 'G', 'MG',
        'MP', 'P', 'M', 'M', 'M',
        'P', 'P', 'M', 'M', 'G',
        'P', 'M', 'M', 'G', 'G',
        'M', 'M', 'G', 'G', 'MG',
    ]

    base_rules = [
        ctrl.Rule(error[Error] & delta_error[Delta_error], motor_power[Motor_power])
        for (Error, Delta_error), Motor_power in
        zip(itertools.product(error.terms.keys(), delta_error.terms.keys()), power_result)
    ]

    return base_rules


# Função para exibir a tabela de regras
def show_table(error, delta_error, motor_power, base_rules):
    table = []
    for Erro in error.terms:
        for Delta_erro in delta_error.terms:
            for regra in base_rules:
                antecedente = str(regra).split('IF ')[1].split(' THEN')[0].replace('AND ', '')
                consequente = str(regra).split('IF ')[1].split(' THEN')[1].split('AND ')[0]

                classificacoes = re.findall(r'\[(.*?)]', (antecedente + consequente))
                if Erro == classificacoes[0] and Delta_erro == classificacoes[1]:
                    table.append([classificacoes[0], classificacoes[1], classificacoes[2]])
                    break

    df = pd.DataFrame(table, columns=[error.label, delta_error.label, motor_power.label])
    pivot_table = pd.DataFrame(df.pivot(index=delta_error.label, columns=error.label, values=motor_power.label).reindex(
        index=delta_error.terms, columns=error.terms))
    pivot_table.index.name = f'{ciano}{"E"}\033[0m'
    print(tabulate(pivot_table, headers='keys', tablefmt='fancy_grid', stralign='center', showindex='always'))


def on_message(client, userdata, msg):
    global is_going_home
    global setpoint
    global current_position
    global free_move
    global destination
    global errors
    global direction

    try:
        payload = msg.payload.decode('utf-8')
        if msg.topic == "drone/rth":
            setpoint = 1
            is_going_home = False

        if msg.topic == "drone/destino":
            destination = int(payload)

            if destination > 1000:
                destination = 1000
            elif destination <= 0:
                destination = 1

            print(destination)

        if msg.topic == "drone/can_move":
            setpoint = destination

        if msg.topic == "drone/joystick":
            direction = payload
            print(payload)

        if msg.topic == "drone/free_movement":
            if payload == "true":
                free_move = True
                errors = [abs(setpoint - current_position)]
            else:
                free_move = False
            print(payload)

    except Exception as e:
        print(f"Erro ao processar mensagem: {Exception}")


# Função para simular o controle
def control_simulation(base_rules):
    global current_position
    global setpoint
    global errors
    global direction

    t = 0
    p_h13 = 0
    error_control = ctrl.ControlSystemSimulation(ctrl.ControlSystem(base_rules))

    positions = [current_position]

    client = mqtt.Client()
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    client.subscribe("drone/rth")
    client.subscribe("drone/origem")
    client.subscribe("drone/joystick")

    client.subscribe("drone/deslocamento")
    client.subscribe("drone/potencia")
    client.subscribe("drone/erro")
    client.subscribe("drone/can_move")
    client.subscribe("drone/free_movement")

    while True:
        try:
            if setpoint is not None and not free_move:
                if current_position < setpoint:
                    u_max = 6
                else:
                    u_max = 4

                current_error = abs(setpoint - current_position)
                errors.append(current_error)

                if current_error < 10:
                    fa = 0.9849
                elif current_error < 25:
                    fa = 0.994
                else:
                    fa = 0.996

                current_delta_error = (errors[-1] - errors[-2])

                error_control.input['error'] = current_error
                error_control.input['delta_error'] = current_delta_error
                error_control.compute()

                p_motor = error_control.output['motor_power']

                if current_error > 5:
                    p_h13 = p_motor
                    p_h24 = p_motor
                else:
                    p_h13 = 0.37
                    p_h24 = 0.37

                d_t = fa * current_position * 1.01398 + 0.5 * (u_max * p_h13 + u_max * p_h24)

                if current_position < setpoint:
                    current_position = d_t
                else:
                    delta_movement = d_t - current_position
                    current_position = current_position - delta_movement

                positions.append(current_position)
                t += 1

                time.sleep(0.1)

                error_payload = f"\n{current_error:.2f}m"
                client.publish("drone/erro", error_payload)

            if free_move:
                if direction == "up":
                    if current_position >= 1000:
                        current_position = 1000
                    else:
                        current_position += 1

                elif direction == "down":
                    if current_position <= 1:
                        current_position = 1
                    else:
                        current_position -= 1

                elif direction == "null":
                    current_position = current_position

                p_h13 = 0.37

            position_payload = f"\n{current_position:.2f}"
            client.publish("drone/deslocamento", position_payload)

            motor_payload = f"\n{p_h13 * 100:.2f}"
            client.publish("drone/potencia", motor_payload)

            time.sleep(0.1)

        except Exception as e:
            print(f"Erro no loop principal: {e}")

# Função principal
def main():
    erro, delta_erro, potencia_motor = define_variables()
    # show_fuzzyfication(erro, delta_erro, potencia_motor)
    base_rules = create_rules(erro, delta_erro, potencia_motor)
    show_table(erro, delta_erro, potencia_motor, base_rules)

    client = mqtt.Client()
    client.on_message = on_message
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    global current_position
    global setpoint

    client.subscribe("drone/rth")
    client.subscribe("drone/origem")
    client.subscribe("drone/destino")
    client.subscribe("drone/deslocamento")
    client.subscribe("drone/potencia")
    client.subscribe("drone/erro")
    client.subscribe("drone/can_move")
    client.subscribe("drone/free_movement")
    client.subscribe("drone/joystick")

    while True:
        control_simulation(base_rules)


if __name__ == "__main__":
    main()
