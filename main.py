import time
import paho.mqtt.client as mqtt
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import numpy as np

# Configurações do Broker MQTT
BROKER = "localhost"
PORT = 1883
TOPIC_PUBLISH = "drone/deslocamento"

# Definir universos fuzzy
error = ctrl.Antecedent(np.arange(0, 1000.1, 0.1), 'error')
delta_error = ctrl.Antecedent(np.arange(-1000, 1000.5, 0.5), 'delta_error')
motor_power = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'motor_power')

# Definir funções de pertinência
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

# Regras fuzzy (25 regras originais)
rules = []
motor_power_results = [
    'MP', 'P', 'M', 'G', 'MG',
    'MP', 'P', 'M', 'M', 'M',
    'P', 'P', 'M', 'M', 'G',
    'P', 'M', 'M', 'G', 'G',
    'M', 'M', 'G', 'G', 'MG',
]

for i, (error_term, delta_error_term) in enumerate(
        [(e, d) for e in error.terms.keys() for d in delta_error.terms.keys()]):
    rules.append(ctrl.Rule(
        error[error_term] & delta_error[delta_error_term],
        motor_power[motor_power_results[i]]
    ))

# Sistema de controle fuzzy
control_system = ctrl.ControlSystem(rules)
fuzzy_simulation = ctrl.ControlSystemSimulation(control_system)

# Inicializa o cliente MQTT
client = mqtt.Client()
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()

# Variáveis do sistema
tempo = 0
current_position = 300
setpoint = 1000

# Loop principal
while True:
    try:
        # Calcular erro
        current_error = abs(setpoint - current_position)

        # Calcular delta do erro
        delta_error_value = current_error - (abs(setpoint - current_position) if tempo > 0 else 0)

        # Inserir entradas fuzzy
        fuzzy_simulation.input['error'] = current_error
        fuzzy_simulation.input['delta_error'] = delta_error_value

        # Computar sistema fuzzy
        fuzzy_simulation.compute()

        # Obter potência do motor
        P_Motor = fuzzy_simulation.output.get('motor_power', 0)  # Garantir valor padrão caso não exista

        # Calcular deslocamento
        deslocamento = P_Motor * 10
        current_position += deslocamento
        tempo += 1

        # Publicar no MQTT
        payload = f"Tempo: {tempo}, Deslocamento: {current_position:.2f}"
        client.publish(TOPIC_PUBLISH, payload)
        print(f"Publicado: {payload}")

        # Aguardar próximo loop
        time.sleep(1)

    except Exception as e:
        print(f"Erro no loop principal: {e}")
