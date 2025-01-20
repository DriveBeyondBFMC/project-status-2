import cv2
import numpy as np
import glob

CHECKERBOARD = (9, 6)
SQUARE_SIZE = 0.038  

objpoints = []  # 3D
imgpoints = []  # 2D

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

images = glob.glob('calib_pi/*.jpg')

for fname in images:
    img = cv2.imread(fname)    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(refined_corners)
        print(f"number of finded checkerboard: {len(imgpoints)}")
        cv2.drawChessboardCorners(img, CHECKERBOARD, refined_corners, ret)
        cv2.imshow('Checkerboard', img)
        cv2.waitKey(500)
    else:
        print(f"No checkboard in frame {fname}")

cv2.destroyAllWindows()

# Calibrate camera
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Intrinsic Matrix (K):\n", K)
print("Distortion Coefficients:\n", dist)

np.savez("calib_pi.npz", K=K, dist=dist, rvecs=rvecs, tvecs=tvecs)
