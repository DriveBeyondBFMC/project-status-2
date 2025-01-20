# relative_pose.py
import cv2
import numpy as np
import math
from collections import defaultdict, deque

def buildTransformationMatrix(rvec, tvec):
    rvec = np.array(rvec).reshape(-1)
    tvec = np.array(tvec).reshape(-1)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[0,3], T[1,3], T[2,3] = tvec[0], tvec[1], tvec[2]
    return T

def getTranslationVector(T):
    return (T[0,3], T[1,3], T[2,3])

def getDistance(vec3):
    x, y, z = vec3
    return math.sqrt(x*x + y*y + z*z)

def moving_average(new_value, buffer, window_size = 10):
    buffer.append(new_value)
    while len(buffer) > window_size:
        buffer.popleft()
    return np.mean(buffer, axis = 0)

class RelativePoseEstimator:

    def __init__(
        self, 
        real_dist_04: float = 0, 
        window_size: int = 10,
        offset: np.ndarray = np.array([0.0, 0.0, 0.0], dtype = np.float32)
    ):
        self.real_dist_04 = real_dist_04
        self.scale_factor = None
        self.window_size  = window_size
        self.positions = {}  # positions[marker_id] = deque()
        self.offset = offset

    def __init_marker_buffer(self, marker_id):
        if marker_id not in self.positions:
            self.positions[marker_id] = deque(maxlen = self.window_size)

    def compute_scale_factor(self, T_c0, T_c4):
        T_0c = np.linalg.inv(T_c0)
        T_04 = T_0c @ T_c4
        x_04, y_04, z_04 = getTranslationVector(T_04)
        dist_cam_04 = getDistance((x_04, y_04, z_04))
        if dist_cam_04 > 1e-6:
            self.scale_factor = self.real_dist_04 / dist_cam_04
        else:
            self.scale_factor = None

    def compute_relative_pose(self, T_c0, T_cX, marker_id):
        if self.scale_factor is None:
            return None
        T_0c = np.linalg.inv(T_c0)
        T_0X = T_0c @ T_cX
        x_0X, y_0X, z_0X = getTranslationVector(T_0X)
        x_real = x_0X * self.scale_factor
        y_real = y_0X * self.scale_factor
        z_real = z_0X * self.scale_factor

        self.__init_marker_buffer(marker_id)
        new_pos = np.array([x_real, y_real, z_real], dtype = np.float32)
        smoothed_pos = moving_average(new_pos, self.positions[marker_id], window_size = self.window_size)
        return smoothed_pos + self.offset
