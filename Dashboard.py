import asyncio
import websockets
import json
from paho.mqtt.client import Client

# Configurações do MQTT
BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC_SUBSCRIBE = "drone/deslocamento"

# Fila para mensagens WebSocket
message_queue = asyncio.Queue()

# Lista para armazenar conexões WebSocket ativas
websocket_clients = []

# Inicialize o loop principal
main_event_loop = asyncio.get_event_loop()

# Callback para conexão MQTT
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Conectado ao broker!")
        client.subscribe(TOPIC_SUBSCRIBE)
    else:
        print(f"Erro na conexão: {rc}")

# Callback para mensagens MQTT
def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        print(f"Mensagem recebida: {payload}")

        # Processar dados e colocar na fila usando o loop principal
        data = {
            "tempo": float(payload.split(",")[0].split(":")[1].strip()),
            "deslocamento": float(payload.split(",")[1].split(":")[1].strip()),
        }
        asyncio.run_coroutine_threadsafe(message_queue.put(data), main_event_loop)
    except Exception as e:
        print(f"Erro ao processar mensagem: {e}")

# Função para enviar dados para os WebSockets conectados
async def send_to_websockets():
    while True:
        data = await message_queue.get()
        if websocket_clients:
            message = json.dumps(data)
            print(f"Enviando dados para WebSocket: {message}")
            for ws in websocket_clients:
                try:
                    await ws.send(message)
                except websockets.exceptions.ConnectionClosed:
                    websocket_clients.remove(ws)

# Servidor WebSocket
async def websocket_handler(websocket, path):
    websocket_clients.append(websocket)
    try:
        async for message in websocket:
            pass  # Não espera mensagens do cliente
    finally:
        websocket_clients.remove(websocket)

async def start_websocket_server():
    server = await websockets.serve(websocket_handler, "localhost", 8766)
    print("Servidor WebSocket iniciado na porta 8766")
    await asyncio.Future()  # Mantém o servidor rodando

# Inicialização do cliente MQTT
mqtt_client = Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(BROKER, PORT, keepalive=60)
mqtt_client.loop_start()

# Inicialização do servidor WebSocket
if __name__ == "__main__":
    try:
        # Use o loop principal para rodar o servidor WebSocket e enviar mensagens
        main_event_loop.create_task(send_to_websockets())
        main_event_loop.run_until_complete(start_websocket_server())
    except KeyboardInterrupt:
        print("Encerrando...")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
