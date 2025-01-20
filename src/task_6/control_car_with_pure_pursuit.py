import numpy as np
import math
import matplotlib.pyplot as plt
import time
import threading
import sys
import math
import socket
from scipy.interpolate import CubicSpline


condition = threading.Condition()
k = 0.1 
Lfc = 0.27
Kp = 1.0 
dt = 0.1 
WB = 0.27
speed_param = 0
steer_param = 0
x = 0
y = 0
cx = []
cy = []
yaw_raw = 0
posA_value = -0.1
posB_value = 0.27
show_animation = True

message = ""
target_value = 0
HOST = '127.0.0.1'  
PORT = 65432        
conn = None 
server_socket = None  
data_lock = threading.Lock()
terminate_flag = False



def handle_data():
    global yaw_raw, posA_value, posB_value, steer_param, speed_param, server_socket, conn
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))  
        server_socket.listen()            
        print(f"The server is listening {HOST}:{PORT}...")

        conn, addr = server_socket.accept()
        with conn:
            print(f"Connected with {addr}")
            while True:
                with condition:
                    data = conn.recv(1024).decode('utf-8')  
                    if not data:
                        print("Client is not connected.")
                        break
                    try:
                        values = data.split('|')
                        yaw_raw = -float(values[0])   
                        
                        posA_value = float(values[1]) 
                        posB_value = float(values[2]) 
                        response = f"{steer_param}|{speed_param}"
                        if response.lower() == 'exit':
                            break
                        conn.sendall(response.encode('utf-8'))
                    except (ValueError, IndexError) as e:
                        print(f"Data handle error: {e}")
                        conn.sendall("error".encode('utf-8'))  

                    condition.notify()  
                    condition.wait()
                    time.sleep(0.05)

class State:
    def __init__(self, x=posA_value, y=posB_value, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw_raw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        global yaw_raw, posA_value, posB_value
        self.x = posA_value
        self.y = posB_value

        degrees = float(yaw_raw)
        radians = degrees * (math.pi / 180)
        self.yaw = radians

        self.v = 0.09
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        if self.old_nearest_point_index is None:
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def scale_value(x, input_min, input_max, output_min, output_max):
    return output_min + (x - input_min) * (output_max - output_min) / (input_max - input_min)


def pure_pursuit_steer_control(state, trajectory, pind):
    global steer_param
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    degrees_ = delta * (180 / math.pi)
    steer_param = -(degrees_ / 30) * 230

    if steer_param > 230: steer_param = 230
    if steer_param < -230: steer_param = -230
    return delta, ind


def open_file():
    global cx, cy
    with open("coordinates.txt", "r") as file:
        lines = file.readlines()
        cx = []
        cy = []
        for i, line in enumerate(lines):
            if line.startswith("Array_x:"):
                cx_line = lines[i + 1].strip()
                cx = list(map(float, cx_line.split(", ")))  
            elif line.startswith("Array_y:"):
                cy_line = lines[i + 1].strip()
                cy = list(map(float, cy_line.split(", ")))  


def main():
    global posA_value, posB_value, speed_param, cx, cy
    
    x1, y1 = cx[0], cy[0]  
    x2, y2 = cx[-1], cy[-1] 

    distance_ = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    cubic_spline = CubicSpline(range(len(cx)), np.array([cx, cy]).T, axis=0)
    t_spline = np.linspace(0, len(cx) - 1, 300)
    x_spline, y_spline = cubic_spline(t_spline).T   

    distances = np.sqrt(np.diff(x_spline)**2 + np.diff(y_spline)**2)
    arc_lengths = np.concatenate(([0], np.cumsum(distances)))

    total_length = arc_lengths[-1]
    t_even = np.linspace(0, total_length, int(distance_/0.020))
    x_even = np.interp(t_even, arc_lengths, x_spline)
    y_even = np.interp(t_even, arc_lengths, y_spline)
    cx = (x_even)
    cy = (y_even)


    target_speed = 100  

    T = 100.0 
    time = 0.0

    state = State(x=posA_value, y=posB_value, yaw=0.0, v=0.0)
    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while lastIndex > target_ind:
        with condition:
            ai = proportional_control(target_speed, state.v)
            di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)
            state.update(ai, di)  
            time += dt
            states.append(time, state)
            condition.notify()  
            condition.wait()
            
        if show_animation:  
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(x_even, y_even, 'go', label="Evenly Spaced Points")
            plt.plot(x_spline, y_spline, '-y', label="Cubic Spline")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)
            print("Start")
            speed_param = 110

    print("Go to Goal!")
    
    speed_param = 0
    steer_param = 0  
    response = f"{steer_param}|{speed_param}"  
    with condition:
        condition.notify()  
    try:
        conn.sendall(response.encode('utf-8')) 
    except Exception as e:
        print(f"Lỗi khi gửi dữ liệu cuối: {e}")
    conn.close()
    server_socket.close()
    sys.exit(0)

    assert lastIndex >= target_ind, "Cannot goal"


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    open_file()
    serial_thread = threading.Thread(target=handle_data)
    serial_thread.daemon = True 
    serial_thread.start()
    main()