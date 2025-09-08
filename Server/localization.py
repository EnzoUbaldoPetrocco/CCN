import cv2
import numpy as np
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R

def convert_pose_to_2d(pose_dict):
    """
    Extract planar x, y, and theta (yaw) from full 6DoF camera pose.
    
    Args:
        pose_dict (dict): Dictionary with keys 'camera_position' and 'camera_orientation_euler'.
    
    Returns:
        dict: {'x': float, 'y': float, 'theta': float}
    """
    cam_pos = pose_dict["camera_position"]  # [x, y, z]
    euler = pose_dict["camera_orientation_euler"]  # [roll, pitch, yaw]

    return {
        "x": float(cam_pos[0]),       # X position
        "y": float(cam_pos[1]),       # Y position
        "theta": float(euler[2])      # Yaw angle in radians
    }


def invert_pose(rvec, tvec):
    """Inverts a pose (marker-to-camera → camera-to-marker)."""
    R_marker_to_cam, _ = cv2.Rodrigues(rvec)
    R_cam_to_marker = R_marker_to_cam.T
    t_cam_to_marker = -R_cam_to_marker @ tvec.reshape(3, 1)
    rvec_inv, _ = cv2.Rodrigues(R_cam_to_marker)
    return rvec_inv, t_cam_to_marker

def estimate_robot_pose_from_markers(
    image, camera_matrix, dist_coeffs, marker_length,
    known_marker_positions, T_cam_in_robot
):
    """
    Estimate robot pose in world frame using ArUco markers and known camera-to-robot transform.

    Parameters:
        image (np.ndarray): BGR image.
        camera_matrix (np.ndarray): Intrinsic matrix.
        dist_coeffs (np.ndarray): Distortion coefficients.
        marker_length (float): Side length of markers (in meters).
        known_marker_positions (dict): Mapping of marker_id -> (x, y, z).
        T_cam_in_robot (np.ndarray): 4x4 transform matrix (camera → robot)

    Returns:
        dict with keys 'robot_position' and 'theta' (yaw in degrees), or None.
    """
    assert T_cam_in_robot.shape == (4, 4)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


    if ids is None or len(ids) == 0:
        return None

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
    if rvecs is None or len(rvecs) == 0:
        return None

    robot_positions = []
    robot_rotations = []

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id not in known_marker_positions:
            continue
        
        print(f"Marker id: {marker_id}")
        # Marker pose w.r.t. camera
        rvec_marker_to_cam = rvecs[i][0]
        tvec_marker_to_cam = tvecs[i][0].reshape(3, 1)
        print(f"\tr vec: {rvec_marker_to_cam}")
        print(f"\tt vec: {np.transpose(tvec_marker_to_cam)}")
        R_marker_to_cam, _ = cv2.Rodrigues(rvec_marker_to_cam)

        T_marker_to_cam = np.eye(4)
        T_marker_to_cam[:3, :3] = R_marker_to_cam
        T_marker_to_cam[:3, 3:] = tvec_marker_to_cam

        T_cam_to_marker = np.linalg.inv(T_marker_to_cam)

        # Marker world position (assume identity rotation)
        marker_world_pos = np.array(known_marker_positions[marker_id]).reshape(3, 1)
        T_marker_to_world = np.eye(4)
        T_marker_to_world[:3, 3:] = marker_world_pos

        # Camera pose in world
        T_cam_to_world = T_marker_to_world @ T_cam_to_marker

        # Robot pose in world
        T_robot_to_world = T_cam_to_world @ np.linalg.inv(T_cam_in_robot)

        robot_pos_world = T_robot_to_world[:3, 3].flatten()
        R_robot_to_world = T_robot_to_world[:3, :3]

        robot_positions.append(robot_pos_world)
        robot_rotations.append(R_robot_to_world)

    if not robot_positions:
        return None

    avg_position = np.mean(robot_positions, axis=0)

    try:
        thetas = [np.arctan2(R[1, 0], R[0, 0]) for R in robot_rotations]
        avg_theta = np.mean(thetas)
        avg_theta_deg = np.degrees(avg_theta)
    except Exception as e:
        print(f"ERROR in averaging rotations: {e}")
        avg_theta_deg = None

    return {
        "robot_position": avg_position,
        "theta": avg_theta_deg,
    }
