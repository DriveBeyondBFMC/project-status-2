import cv2
import numpy as np
import logging
from collections import defaultdict

class ArucoDetector:
    def __init__(
        self, 
        camera_calibration,
        marker_length = 0.05, 
        max_lost = 5,  
        dictionary_name="DICT_4X4_50"
    ):

        self.camera_calibration = camera_calibration
        self.marker_length = marker_length
        self.max_lost = max_lost  

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))

        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    def detect_markers(self, gray_image):

        detected_markers = []
        logging.info("Starting marker detection...")

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_image, 
            self.aruco_dict, 
            parameters=self.parameters
        )

        if ids is not None:
            ids = ids.flatten()
            for i, marker_id in enumerate(ids):
                corner = corners[i].reshape((4, 2))
                detected_markers.append({
                    'id': int(marker_id),
                    'corners': corner,
                    'dict': "DICT_4X4_50"
                })
                logging.info(f"Detected marker ID {marker_id}")

        logging.info(f"Total markers detected: {len(detected_markers)}")
        return detected_markers

    def estimate_pose(self, markers):
        poses = []
        if len(markers) == 0:
            return poses

        corners = [m['corners'] for m in markers]
        corners = np.array(corners).reshape(-1, 4, 2)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_calibration.camera_matrix,
            self.camera_calibration.dist_coeffs
        )

        for i, m in enumerate(markers):
            poses.append({
                'id': m['id'],
                'dict': m['dict'],
                'corners': m['corners'],
                'rvec': rvecs[i],
                'tvec': tvecs[i]
            })

        logging.info(f"Estimated poses for {len(poses)} markers.")
        return poses

   