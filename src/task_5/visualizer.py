# visualizer.py
import cv2
import numpy as np
from typing import List
from marker_manager import Marker, MarkerManager
import logging

class Visualizer:

    @staticmethod
    def draw_markers(img: np.ndarray, markers: List[Marker]):
        for marker in markers:
            corners = marker.corners.astype(int)
            cv2.polylines(img, [corners], True, (0,255,0), 2)
            cx, cy = int(marker.center[0]), int(marker.center[1])
            cv2.circle(img, (cx, cy), 4, (0,0,255), -1)
            cv2.putText(img, f"ID:{marker.id}", 
                        (cx - 50, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

    @staticmethod
    def draw_3d_distance(img: np.ndarray, manager: MarkerManager) -> None:
        moving_marker = manager.moving_marker
        if moving_marker is None:
            return
        for fixed_id, distance in manager.relative_distances.items():
            pt_m = (int(moving_marker.center[0]), int(moving_marker.center[1]))
            pt_f = (int(manager.fixed_markers[fixed_id].center[0]), int(manager.fixed_markers[fixed_id].center[1]))
            cv2.line(img, pt_m, pt_f, (0,255,255), 2)

            mid_x = (pt_m[0] + pt_f[0]) // 2
            mid_y = (pt_m[1] + pt_f[1]) // 2
            cv2.putText(img, f"{distance:.3f}m",
                        (mid_x, mid_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,0,255),
                        2)

    @staticmethod
    def draw_relative_info(img: np.ndarray, scale_factor: float=None):

        if scale_factor is not None:
            cv2.putText(img,
                f"Scale= {scale_factor:.3f}",
                (30,60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,0,255), 2
            )
