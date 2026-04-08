import cv2
import numpy as np
import glob
import os

# ================= SETTINGS =================
CHECKERBOARD = (5, 6) # Number of inside corners (Actual checkerboard 6 x 7)
square_size = 0.024  # meters

# ================= PREP OBJECT POINTS =================
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0],
                       0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

# images = sorted(glob.glob('screenshots/*.png'))

# images = sorted(glob.glob(os.path.join('../../screenshots', '*.png')))
images = sorted(glob.glob('/home/bihan/ros2_ws/screenshots/*.png'))

# ================= FIND CORNERS =================
# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

#     if ret:
#         objpoints.append(objp)
#         imgpoints.append(corners)

# ================= CAMERA CALIBRATION =================
# ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints, imgpoints, gray.shape[::-1], None, None)

# ================= FIND CORNERS =================
img_shape = None  # will store image size

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        if img_shape is None:
            img_shape = gray.shape[::-1]  # (width, height)
    else:
        print(f"Checkerboard NOT detected in image: {fname}")

# check if we found any corners
if img_shape is None:
    raise RuntimeError("No checkerboard corners detected in any images!")

# ================= CAMERA CALIBRATION =================
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None)

print("\nCamera Matrix:\n", K)
print("\nDistortion:\n", dist)

# ================= LOAD OR DEFINE POSES =================
# if os.path.exists("ee_poses.txt"):
#     print("\nLoading poses from ee_poses.txt")
#     poses = np.loadtxt("ee_poses.txt")

# else:
#     print("\nee_poses.txt not found → using default poses")

#     # 20 example poses: [x y z qx qy qz qw]
#     poses = np.array([
#         [-26.12, -301.15, 75.78, 0.046, -3.137, 0.009],
#         [-239.72, -304.00, 6.41, 0.008, -3.550, 0.190],
#         [203.54, -300.04, -14.84, 0.409, -2.527, -0.044],
#         [0.53, -328.68, -86.95, 0.198, -3.151, -0.077],
#         [-152.67, -308.82, -92.09, 0.207, 2.662, -0.081],
#         [-281.14, -301.93, -130.30, 0.230, -3.702, 0.208],
#         [229.72, -325.02, -90.22, 0.020, -2.548, 0.014],
#         [286.46, -292.69, -80.06, 0.334, -2.519, 0.070],
#         [-5.05, -330.71, -139.62, 0.079, -3.126, -0.123],
#         [-144.04, -289.09, -128.19, 0.160, 2.780, -0.006],
#         [-193.20, -276.07, -160.00, 0.130, 2.451, -0.124],
#         [-240.52, -274.70, -202.13, 0.053, 2.515, -0.125],
#         [148.64, -302.07, -151.01, 0.299, -2.653, -0.004],
#         [216.33, -295.25, -153.30, 0.355, -2.360, 0.076],
#         [233.69, -312.39, -188.42, 0.215, -2.441, -0.042],
#         [-57.34, -246.66, 66.66, 0.243, 2.913, -0.137],
#         [-256.03, -212.53, -22.89, 0.726, 2.555, 0.024],
#         [264.23, -215.05, 1.99, 0.135, -2.715, 0.485],
#         [-259.06, -317.02, -239.74, 0.411, 2.233, 0.242],
#         [282.86, -287.06, -218.50, 0.364, -2.244, 0.051]
#     ])

# ================= LOAD OR DEFINE POSES =================
if os.path.exists("ee_poses.txt"):
    print("\nLoading poses from ee_poses.txt")
    # ee_poses.txt should have columns: x y z rx ry rz
    raw_poses = np.loadtxt("ee_poses.txt")

    poses = []
    for p in raw_poses:
        x, y, z, rx, ry, rz = p
        rvec = np.array([rx, ry, rz])
        R, _ = cv2.Rodrigues(rvec)  # rotation matrix
        # convert rotation matrix to quaternion
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4*qw)
        qy = (R[0,2] - R[2,0]) / (4*qw)
        qz = (R[1,0] - R[0,1]) / (4*qw)
        poses.append([x, y, z, qx, qy, qz, qw])
    poses = np.array(poses)

else:
    print("\nee_poses.txt not found → using default example poses")
    # Example poses with rotation vectors (rx, ry, rz)
    raw_poses = np.array([
        [-26.12, -301.15, 75.78, 0.046, -3.137, 0.009],
        [-239.72, -304.00, 6.41, 0.008, -3.550, 0.190],
        [203.54, -300.04, -14.84, 0.409, -2.527, -0.044],
        [0.53, -328.68, -86.95, 0.198, -3.151, -0.077],
        [-152.67, -308.82, -92.09, 0.207, 2.662, -0.081],
        [-281.14, -301.93, -130.30, 0.230, -3.702, 0.208],
        [229.72, -325.02, -90.22, 0.020, -2.548, 0.014],
        [286.46, -292.69, -80.06, 0.334, -2.519, 0.070],
        [-5.05, -330.71, -139.62, 0.079, -3.126, -0.123],
        [-144.04, -289.09, -128.19, 0.160, 2.780, -0.006],
        [-193.20, -276.07, -160.00, 0.130, 2.451, -0.124],
        [-240.52, -274.70, -202.13, 0.053, 2.515, -0.125],
        [148.64, -302.07, -151.01, 0.299, -2.653, -0.004],
        [216.33, -295.25, -153.30, 0.355, -2.360, 0.076],
        [233.69, -312.39, -188.42, 0.215, -2.441, -0.042],
        [-57.34, -246.66, 66.66, 0.243, 2.913, -0.137],
        [-256.03, -212.53, -22.89, 0.726, 2.555, 0.024],
        [264.23, -215.05, 1.99, 0.135, -2.715, 0.485],
        [-259.06, -317.02, -239.74, 0.411, 2.233, 0.242],
        [282.86, -287.06, -218.50, 0.364, -2.244, 0.051]
    ])

    poses = []
    for p in raw_poses:
        x, y, z, rx, ry, rz = p
        rvec = np.array([rx, ry, rz])
        R, _ = cv2.Rodrigues(rvec)
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4*qw)
        qy = (R[0,2] - R[2,0]) / (4*qw)
        qz = (R[1,0] - R[0,1]) / (4*qw)
        poses.append([x, y, z, qx, qy, qz, qw])
    poses = np.array(poses)

# ================= QUATERNION → ROTATION =================
def quat_to_rot(qx, qy, qz, qw):
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R

# ================= BUILD GRIPPER→BASE =================
# R_gripper2base = []
# t_gripper2base = []

# for p in poses:
#     x, y, z, qx, qy, qz, qw = p

#     R = quat_to_rot(qx, qy, qz, qw)
#     t = np.array([x, y, z]).reshape(3,1)

#     # IMPORTANT: invert base→ee → ee→base
#     R_inv = R.T
#     t_inv = -R_inv @ t

#     R_gripper2base.append(R_inv)
#     t_gripper2base.append(t_inv.flatten())

R_gripper2base = []
t_gripper2base = []

for p in poses:
    x, y, z, qx, qy, qz, qw = p
    R = quat_to_rot(qx, qy, qz, qw)
    t = np.array([x, y, z]).reshape(3,1)
    R_inv = R.T
    t_inv = -R_inv @ t
    R_gripper2base.append(R_inv)
    t_gripper2base.append(t_inv.flatten())

# ================= TARGET (CHECKERBOARD) =================
R_target2cam = []
t_target2cam = []

for i in range(len(rvecs)):
    R, _ = cv2.Rodrigues(rvecs[i])
    t = tvecs[i].reshape(3)
    R_target2cam.append(R)
    t_target2cam.append(t)

# ================= TRIM GRIPPER POSES TO MATCH IMAGES =================
if len(poses) > len(R_target2cam):
    print(f"Trimming gripper poses from {len(poses)} → {len(R_target2cam)} to match detected images")
    poses = poses[:len(R_target2cam)]

# ================= BUILD GRIPPER→BASE =================
R_gripper2base = []
t_gripper2base = []

for p in poses:
    x, y, z, qx, qy, qz, qw = p
    R = quat_to_rot(qx, qy, qz, qw)
    t = np.array([x, y, z]).reshape(3,1)
    R_inv = R.T
    t_inv = -R_inv @ t
    R_gripper2base.append(R_inv)
    t_gripper2base.append(t_inv.flatten())

# ================= HAND-EYE CALIBRATION =================
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base,
    t_gripper2base,
    R_target2cam,
    t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# # ================= TARGET (CHECKERBOARD) =================
# R_target2cam = []
# t_target2cam = []

# for i in range(len(rvecs)):
#     R, _ = cv2.Rodrigues(rvecs[i])
#     t = tvecs[i].reshape(3)

#     R_target2cam.append(R)
#     t_target2cam.append(t)

# # ================= HAND-EYE CALIBRATION =================
# R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
#     R_gripper2base,
#     t_gripper2base,
#     R_target2cam,
#     t_target2cam,
#     method=cv2.CALIB_HAND_EYE_TSAI
# )

print("\n=== RESULT ===")
print("R_cam2gripper:\n", R_cam2gripper)
print("t_cam2gripper:\n", t_cam2gripper)