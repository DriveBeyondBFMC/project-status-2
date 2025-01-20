import time
import serial
import threading
import socket
import json


serial_port = '/dev/ttyACM0'
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)
broker = 'broker.emqx.io'
port = 1883
topic = "bfmc"
client_id = "vehicle"
message =""
steer_param = 0
raw_posA = 0
raw_posB = 0

SERVER_HOST = '127.0.0.1' 
SERVER_PORT = 65432         
BUFFER_SIZE = 1024          
yaw_value = 0

commands = {
    '0': '#kl:30;;\r\n',
    '1': '#battery:0;;\r\n',
    '2': '#instant:0;;\r\n',
    '3': '#imu:0;;\r\n',
    '4': '#resourceMonitor:0;;\r\n',
    '5': '#speed:{0};;\r\n',
    '6': '#steer:{0};;\r\n',
    '7': '#vcd:200;0;121;\r\n',
}

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    client = mqtt_client.Client(client_id)

    client.on_connect = on_connect
    client.connect(broker, port)
    return client


FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60


aruco1Host = "192.168.0.10"
aruco2Host = "192.168.0.9"
aruco3Host = "192.168.0.2"
port = 5000
port_1 = 5001
port_2 = 5002

aruco1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
aruco2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
aruco3 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
aruco1.connect((aruco1Host, port))
aruco2.connect((aruco2Host, port_1))
aruco3.connect((aruco3Host, port_2))

connector = [aruco3, aruco2, aruco1]
connectorOutput = ["None" for _ in range(len(connector))]
def subscribe():
    global raw_posA, raw_posB

    buffer = ""
    while True:
        ...
        markers = [3, 2, 1]
    
        for marker, message in zip(markers, connectorOutput):

            if message == "None":
                
                continue
            if message != "None":
                coordinates = message.split("|")
                raw_posA = float(coordinates[0])
                raw_posB = float(coordinates[1])
                print(marker)
                print(f"raw_posA {raw_posA:.4f}, raw_posB {raw_posB:.4f}")
                break
        time.sleep(.05)
        
def subscribeTest(arucoDetector, index):
    global connectorOutput

    buffer = ""
    while True:
        data = arucoDetector.recv(1024).decode("utf-8")
        buffer += data

        message = ""
        while "\n" in buffer:
            message, buffer = buffer.split("\n", 1)

        connectorOutput[index] = message
    

def send_command(command):
    ser.write(command.encode())

def convert_yaw_value(yaw_value):
    i = 0 
    current_value = int(yaw_value)  
    previous_value = None 
    if previous_value == 32 and current_value == -32:
        i += 1  
    elif previous_value == -32 and current_value == 32:
        i -= 1  
    previous_value = current_value
    convert = (60 * i) + yaw_value 
    return convert

def socket_client():
    global yaw_value, raw_posA, raw_posB, steer_param

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            client_socket.connect((SERVER_HOST, SERVER_PORT))
            print(f"Connected to {SERVER_HOST}:{SERVER_PORT}")
            while True:
                yaw_value = convert_yaw_value(yaw_value)
                message = f"{yaw_value}|{raw_posA}|{raw_posB}"
                print("mess", message)
                if message.lower() == 'exit':
                    break
                client_socket.sendall(message.encode('utf-8'))
                response = client_socket.recv(1024).decode('utf-8')
                try:
                    values = response.split('|')
                    value_steer = values[0].strip()  
                    value_speed = values[1].strip()  

                    try:
                        steer_param = float(value_steer)
                        speed_param = float(value_speed)
                    except ValueError:
                        print(f"Error: Value '{value_steer}' is not a valid number.")

                    send_command(commands['6'].format(steer_param))
                    send_command(commands['5'].format(speed_param))
                except ValueError:
                    print(f"Error: Server response is not a valid number: {response}")
                
                time.sleep(0.1)
        except Exception as e:
            print(f"Error connect to server: {e}")


def read_response():
    global yaw_value
    while True:
        if ser.in_waiting > 0:  
            response = ser.readline().decode('utf-8').strip()
            if response:
                if response.startswith("@imu:"):
                    values = response.replace("@imu:", "").split(";")
                    if len(values) > 2: 
                        yaw_value = float(values[2])


if __name__ == "__main__":
    print("File1: Starting communication...")
    send_command(commands['0'])
    send_command(commands['1'])
    send_command(commands['2'])
    send_command(commands['4'])
    speed_param = 100

    thread1 = threading.Thread(target=read_response, daemon=True)
    thread2 = threading.Thread(target=socket_client, daemon=True)
    thread3 = threading.Thread(target=subscribe, args = (), daemon=True) 
    threads = [threading.Thread(target = subscribeTest, args = (aruco, index)) for index, aruco in enumerate(connector)]
    thread1.start()
    thread2.start()
    thread3.start()
    for thread in threads:
        thread.start()

    try:
        while True:
            time.sleep(1) 
    except KeyboardInterrupt:
        print("File1: Communication stopped.")
        thread1.join()
        thread2.join()
        thread3.join()
        for thread in threads:
            thread.join()
        ser.close()  