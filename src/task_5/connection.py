import socket
import select
import cv2
import numpy as np
import pickle
import struct

class Connector():
    def __init__(self, hostname, port):

        self.senderSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.senderSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.senderSocket.settimeout(1)
        self.senderSocket.bind((hostname, port))
        self.senderSocket.listen(1)
        print("Waiting for client connection...")
        self.clientConn, self.clientAddr = None, None
        
    def sendMessage(self, message):
        
        readable, _, _ = select.select([self.senderSocket], [], [], 0)
        if readable:  # If the socket is ready
            self.clientConn, self.clientAddr = self.senderSocket.accept()
            print(f"New connection at: {self.clientAddr}")

        if self.clientConn:
            try:
                if isinstance(message, str):
                    try:
                        self.clientConn.sendall(message.encode("utf-8"))
                    except (BlockingIOError, BrokenPipeError):
                        self.clientConn = None
                
                if isinstance(message, np.ndarray) or isinstance(message, cv2.Mat) or isinstance(message, list):
                    _, frame_encoded = cv2.imencode(".jpg", message)
                    frame_data = pickle.dumps(frame_encoded)
                    self.clientConn.sendall(struct.pack(">L", len(frame_data)) + frame_data)

            except (BrokenPipeError, ConnectionResetError) as e:
                print(f"Client disconnected: {e}")
                self.clientConn.close()
                self.clientConn = None
                print("Waiting for new client connection...")