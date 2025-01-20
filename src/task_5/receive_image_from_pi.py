import socket
import struct
import pickle
import cv2
from datetime import datetime

# Connect to the Raspberry Pi
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.0.10', 5000))  
data = b""
payload_size = struct.calcsize(">L")

try:
    cnt = 0
    while True:
        while len(data) < payload_size:
            data += client_socket.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        # Receive video frame data
        while len(data) < msg_size:
            data += client_socket.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        cv2.imshow("Video Feed", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            cv2.imwrite(f"calib_pi/frame_{cnt}.jpg", frame)
            print(f"save frame id: {cnt}")
            cnt += 1

        if key == ord('q'):
            break
finally:
    client_socket.close()
    cv2.destroyAllWindows()
