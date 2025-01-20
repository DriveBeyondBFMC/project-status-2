import serial
import time
import sys
import termios
import tty
import threading
import socket


serial_port = '/dev/ttyACM0'
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate, timeout=1)

speed_param = 0
steer_param = 0

# CMD list format
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

def send_command(command):
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")

def read_response():
    if ser.in_waiting > 0:  # Kiểm tra xem có dữ liệu trong buffer hay không
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(response)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())  # Chuyển sang chế độ raw
        ch = sys.stdin.read(1)  # Đọc một ký tự từ bàn phím
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Khôi phục cấu hình ban đầu
    return ch

def main():
    global speed_param, steer_param

    host = "localhost"
    port = 12345

    receiverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    receiverSocket.connect((host, port))
    
    # In ra menu lựa chọn
    print("****************************")
    print("0: Enable full specification")
    print("1: Disable battery")
    print("2: Disable instant")
    print("3: Disable imu")
    print("4: Disable resource monitor")
    print("5: Control speed")
    print("6: Control steering")
    print("****************************")

    buffer = ""
    try:
        while True:
            data = receiverSocket.recv(1024).decode("utf-8")    
            buffer += data

            while "\n" in buffer:
                message, buffer = buffer.split("\n", 1)
            message = float(message)
            angle = 90 - message 
            
            steer_param = (angle / 30) * 230
            if steer_param > 230: steer_param = 230
            if steer_param < -230: steer_param = -230
            print(f"Angle: {angle:.2f}| Steering: {steer_param:.4f}")
            speed_param = 170
            send_command(commands['6'].format(steer_param))
            send_command(commands['5'].format(speed_param))
    except Exception as e:
        print(f"Error: {e}")
    finally:
        receiverSocket.close()

if __name__ == "__main__":
    serial_thread = threading.Thread(target=read_response)
    serial_thread.daemon = True
    serial_thread.start()

    main()

    ser.close()
