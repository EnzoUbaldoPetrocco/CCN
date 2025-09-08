import cv2
import numpy as np
import glob

# ==== CONFIGURATION ====
CHECKERBOARD = (9, 6)  # Number of inner corners per row and column
SQUARE_SIZE = 0.025  # in meters, update if you use another unit

# Paths to your checkerboard images
images = glob.glob("./*.png")  # Folder with calibration images
print(images)
# ==== PREPARE OBJECT POINTS ====
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # scale to real world units

objpoints = []  # 3D real-world points
imgpoints = []  # 2D image points

# ==== FIND CORNERS ====
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    print(f"Processing {fname}")
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)

        # Optionally show the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow("Corners", img)
        cv2.waitKey(100)
    else:
        print("Checkerboard not found in", fname)

cv2.destroyAllWindows()

# ==== CALIBRATE CAMERA ====
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# ==== RESULTS ====
print("\n=== CAMERA MATRIX ===")
print(camera_matrix)

print("\n=== DISTORTION COEFFICIENTS ===")
print(dist_coeffs.ravel())

print("\n=== REPROJECTION ERROR ===")
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print("Total error: {}".format(mean_error/len(objpoints)))