import cv2
import numpy as np
import logging

from aruco_detector import ArucoDetector
from marker_manager import Marker, MarkerManager
from visualizer import Visualizer
from picamera2 import Picamera2
from connection import Connector 

from relative_pose import (
    RelativePoseEstimator,
    buildTransformationMatrix,
    getDistance
)

class CONFIG:
    MARKER_LENGTH = 0.1 # 10 cm
    VEHICLE_ID = 3 # vehicle marker id
    REAL_DIST = 0.464 # real distance between root marker and reference marker
    WINDOW_SIZE = 10 # window size for moving average           
    REFERENCE_ID = 1 # reference marker id
    ROOT_ID = 0 # root marker id
    OFFSET = np.array([0.0, 0.0, 0.0], dtype = np.float32) # offset for the local root with global root marker

def apply_gamma_correction(image, gamma = 1.2):
    look_up_table = np.array([
        ((i / 255.0) ** (1.0 / gamma)) * 255 for i in range(256)
    ]).astype("uint8")
    return cv2.LUT(image, look_up_table)


class CameraCalibration:
    def __init__(self, camera_matrix, dist_coeffs):

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs


def main():

    picam = Picamera2()
    config = picam.create_preview_configuration(
        main = {"size": (640, 480), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()

    dataSender = Connector("0.0.0.0", 5002)

    logging.basicConfig(
        level = logging.INFO,
        format = '%(asctime)s - %(levelname)s - %(message)s'
    )

    # Load calibration data
    data = np.load('calib_results.npz')
    K = data['K']
    dist = data['dist']

    calibration = CameraCalibration(K, dist)

    detector = ArucoDetector(
        calibration, 
        marker_length = CONFIG.MARKER_LENGTH
    )  

    manager = MarkerManager(
        fixed_ids = [CONFIG.ROOT_ID, CONFIG.REFERENCE_ID], 
        move_id = CONFIG.VEHICLE_ID,
    )

    rel_pose = RelativePoseEstimator(
        real_dist_04 = CONFIG.REAL_DIST, 
        window_size = CONFIG.WINDOW_SIZE,
        offset = CONFIG.OFFSET
    )

    try:

        while True:
            frame = picam.capture_array()

            gamma_corrected = apply_gamma_correction(frame, gamma = 0.1)
            gray = cv2.cvtColor(gamma_corrected, cv2.COLOR_RGB2GRAY)

            detected_markers = detector.detect_markers(gray)
            pose_markers = detector.estimate_pose(detected_markers)

            marker_objs = []

            for pm in pose_markers:
                marker_id = pm['id']
                corners = pm['corners']
                cx = np.mean(corners[:,0])
                cy = np.mean(corners[:,1])

                position_3d = pm['tvec'].flatten()

                marker_obj = Marker(
                    marker_id = marker_id,
                    corners = corners,
                    center = (cx, cy),
                    position_3d = position_3d
                )
                marker_objs.append(marker_obj)

            manager.update_markers(marker_objs)

            root_marker = next((pm for pm in pose_markers if pm['id'] == CONFIG.ROOT_ID), None)
            Reference_marker = next((pm for pm in pose_markers if pm['id'] == CONFIG.REFERENCE_ID), None)
            vehicle_marker = next((pm for pm in pose_markers if pm['id'] == CONFIG.VEHICLE_ID), None)

            if root_marker and Reference_marker:
                T_c0 = buildTransformationMatrix(root_marker['rvec'], root_marker['tvec'])
                T_c4 = buildTransformationMatrix(Reference_marker['rvec'], Reference_marker['tvec'])
                rel_pose.compute_scale_factor(T_c0, T_c4)


            if rel_pose.scale_factor and root_marker and vehicle_marker:
                T_c0 = buildTransformationMatrix(root_marker['rvec'], root_marker['tvec'])
                T_cX = buildTransformationMatrix(vehicle_marker['rvec'], vehicle_marker['tvec'])
                pos_f = rel_pose.compute_relative_pose(T_c0, T_cX, CONFIG.VEHICLE_ID)
                
                x_f, y_f, _ = pos_f
                cx_ = np.mean(vehicle_marker['corners'][:,0])
                cy_ = np.mean(vehicle_marker['corners'][:,1])

                cv2.putText(
                    frame,
                    f"[{CONFIG.VEHICLE_ID}]({x_f:.2f},{y_f:.2f})",
                    (int(cx_), int(cy_)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    2
                )

                message = f"{x_f}|{y_f}\n"
                print(f"Coordinate: {x_f:.2f} - {y_f:.2f}")
        
                dataSender.sendMessage(message)
                print("[SEND]", message)
            else:
                print("[SEND] None")
                dataSender.sendMessage("None\n")
                

            # Visualizer.draw_relative_info(frame, rel_pose.scale_factor)

            # Visualizer.draw_markers(frame, marker_objs)

            # Visualizer.draw_3d_distance(frame, manager)


            # cv2.imshow("Relative Pose", frame)
            # dataSender.sendMessage(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        logging.error(f"An error occurred: {e}")

    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
