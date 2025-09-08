from flask import Flask, request, jsonify, send_from_directory
import json
import base64
import time
import os
import cv2
from rrt_real import Node, Human, RRT
from cv2 import imread
from math import floor
import io
import matplotlib.pyplot as plt
import numpy as np
from localization import estimate_robot_pose_from_markers, convert_pose_to_2d
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R
import re
import glob
import math
from conversation_manager import ConversationManager

app = Flask(__name__)
debug = True  # Set to False in production

UPLOAD_FOLDER = "./uploaded_maps"
VISUALIZATION_FOLDER = "./visualizations"
ROBOT_PHOTOS_FOLDER = "./robot_photos"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
os.makedirs(VISUALIZATION_FOLDER, exist_ok=True)
os.makedirs(ROBOT_PHOTOS_FOLDER, exist_ok=True)

manager = ConversationManager()

# RRT Algorithm
max_iterations = 5000
step_size = 6
tolerance = 1
# Original Points: [0.46       0.73333333 1.00666667 1.28       1.55333333 1.82666667  2.1]
# Scaled Points:   [ 9.2        14.66666667 20.13333333 25.6        31.06666667 36.53333333  42. ]

# Original Points: [0.46  0.668 0.876 1.084 1.292 1.5  ]
# Scaled Points: [ 9.2  13.36 17.52 21.68 25.84 30.  ]
# proxemics_radius = 9.2
manager.proxemics = 9.2
radius = 12  # to find neighbours in RRT
back = False  # this bool is needed to know if the robot is going back


# Information
goal_node = Human(80, 80, radius=0.5, theta=0.0)
start_node = Human(
    40, 45, radius=0.5, theta=0.0
)  # Robot is considered a human because it has also an orientation
robot_node = copy.copy(start_node)

humans = [Human(75, 48.0, radius=manager.proxemics, theta=0.0)]

# Compute scaling
real_limits = [[0, 3], [0, 2.95]]  # x axis and y axis limits - ~robot dimension
# map_limits = [[0, 124], [0, 124]]
map_limits = [[0, 124], [0, 124]]  # x axis and y axis limits - map dimension


# Map metadata
# Try to load map_metadata from latest file, otherwise use default values


def load_latest_map_metadata():
    latest_metadata_path = os.path.join(UPLOAD_FOLDER, "map_metadata_latest.json")
    if os.path.exists(latest_metadata_path):
        try:
            with open(latest_metadata_path, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load map_metadata_latest.json: {e}")
    # fallback default
    return {
        "x": 40,
        "y": 40,
        "theta": 0,
        "scale": 0.05,
    }


map_metadata = load_latest_map_metadata()
mapShift = Node(64, 64)

# Next navigation pose, but I need to define it like if it was a Human because I need a theta
next_node = Human(0, 0, 0, theta=0)

# DEPTH_CAMERA
depth_camera_info = None

# CAMERA INFO
camera_info = None
marker_length = 0.1375
camera_matrix = np.array(
    [
        [301.19737843, 0.0, 294.19149046],
        [0.0, 300.16100611, 251.79375696],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)
dist_coeffs = np.array([0.06398157, -0.8037002, -0.00686162, -0.00332916, 1.40681795])
reprojection_error = 0.16175387922329082
known_marker_positions = {
    0: [-0.6, 1.5, 1.6],
    1: [0.9, 1.5, 1.6],
    2: [1.5, 0.3, 1.6],
    3: [0.9, -1.5, 1.6],
    4: [-0.6, -1.5, 1.6],
    5: [1.5, 0.3, 0.5],
    6: [1.5, -1.2, 0.5],
    7: [0.0, -1.5, 0.4],
}  # they are related to the world frame (center of room)

# Define the reference points and scale
world_ref_x = map_metadata["x"]  # -3.0836718
world_ref_y = map_metadata["y"]
map_ref_x = 0
map_ref_y = 124
scale_x = map_metadata["scale"]  # 2.05 / abs(goal_node.x - start_node.x)
scale_y = map_metadata["scale"]  # 2. / abs(goal_node.y - start_node.y)
scale_world_to_map_x = (
    1 / scale_x
)  # map_metadata["scale"]  # 1 map unit = 0.05 world units, so 1 world unit = 20 map units
scale_map_to_world_x = (
    scale_x  # map_metadata["scale"]      # 1 map unit = 0.05 world units
)
scale_world_to_map_y = (
    1 / scale_y
)  # map_metadata["scale"]  # 1 map unit = 0.05 world units, so 1 world unit = 20 map units
scale_map_to_world_y = (
    scale_y  # map_metadata["scale"]      # 1 map unit = 0.05 world units
)
scale_world_to_map = (scale_world_to_map_x + scale_world_to_map_y) / 2
scale_map_to_world = (scale_map_to_world_x + scale_map_to_world_y) / 2
print(
    f"scale_world_to_map: {scale_world_to_map}, scale_map_to_world: {scale_map_to_world}"
)


def transformRadius2WorldCoordinates(value):
    return value * scale_map_to_world


def transformRadius2MapCoordinates(value):
    return value * scale_world_to_map


def transformNode2MapCoordinates(node: Node | Human) -> Node | Human:
    """
    Converts a Node from world (meters) to image (pixels), accounting for origin and Y flip.
    Returns a Node in image coordinates.
    """
    x = node.x
    y = - node.y
    #y = -node.y
    # Scaling
    x = x * scale_world_to_map_x
    y = y * scale_world_to_map_y
    # Transform to image coordinates
    # x = -x
    # Assign changes to the node
    node_copy = copy.copy(node)
    node_copy.x = x
    node_copy.y = y
    # Apply shift from the center of the map to the center of the image
    node_copy = node_copy + mapShift
    
    # node_copy.y = 124 - node_copy.y  # Flip the y coordinate to match image coordinates

    if isinstance(node, Human):
        # if human, flip the theta to match the image coordinate system
        node_copy.theta = -node_copy.theta
    # print(f"Node {node} transformed in {node_copy} in map coordinates")
    return node_copy


def transformNode2WorldCoordinates(node: Node | Human) -> Node | Human:
    """
    Converts a Node from image (pixels) to world (meters), accounting for origin and Y flip.
    Returns a Node in world coordinates.
    """
    node_copy = copy.copy(node)
    # print(f"Node_copy {node_copy} ")
    # node_copy.y = 124 - node.y  # Flip the y coordinate to match world coordinates

    node_copy = node_copy - mapShift  # Shift back to center of the map
    # print(f"Node_copy after shift {node_copy} ")
    # print(f"Node {node} shifted to {node_copy} in map coordinates")
    # Flip the y coordinate to match world coordinates
    # node_copy.x = - node_copy.x
    # print(f"Node {node_copy} flipped in x coordinate to match world coordinates")
    # Scaling
    node_copy.x = node_copy.x * scale_map_to_world_x
    node_copy.y = node_copy.y * scale_map_to_world_y
    # print(f"Node {node_copy} scaled to world coordinates")

    node_copy.y = -node_copy.y  # Flip x coordinate to match world coordinates
    # print(f"Node_copy right before is instance{node_copy} ")

    if isinstance(node, Human):
        node_copy.theta = (
            -node_copy.theta
        )  # Flip theta to match world coordinate system
    # print(f"Node {node} transformed in {node_copy} in world coordinates\n\n")
    return node_copy


def parse_transform_string_to_matrix(transform_str):
    """
    Parses a TransformTime string and returns a 4x4 transformation matrix.

    Parameters:
        transform_str (str): A string in the specified format.

    Returns:
        np.ndarray: A 4x4 transformation matrix.
    """
    # Extract quaternion and translation using regex
    quat_match = re.search(
        r"Quaternion\{ x=([^,]+), y=([^,]+), z=([^,]+), w=([^\}]+) \}", transform_str
    )
    trans_match = re.search(
        r"Vector3\{ x=([^,]+), y=([^,]+), z=([^\}]+) \}", transform_str
    )

    if not quat_match or not trans_match:
        raise ValueError("Could not parse the transform string")

    # Parse values
    qx, qy, qz, qw = map(float, quat_match.groups())
    tx, ty, tz = map(float, trans_match.groups())

    # Convert quaternion to rotation matrix
    rot = R.from_quat([qx, qy, qz, qw]).as_matrix()

    # Assemble 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [tx, ty, tz]

    return T


@app.route("/robot/position", methods=["GET", "POST"])
def robot_position():
    global robot_node
    TAG = "\n/robot/position "
    if request.method == "POST":
        try:
            data = request.json
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta")

            if x is None or y is None or theta is None:
                print("/robot/position" + "Missing x, y, or theta in request")
                return jsonify({"error": "Missing x, y, or theta in request"}), 400

            robot_node = transformNode2MapCoordinates(
                Human(x, y, robot_node.radius, theta)
            )
            
            #print(TAG + f"According to client robot is in x:{x}, y:{y}, theta:{theta}")
            """print(
                TAG
                + f"According to server robot is in x:{robot_node.x}, y:{robot_node.y}, theta:{getattr(robot_node, 'theta', 0)}"
            )"""
            return jsonify({"message": "Robot position updated successfully"}), 200
        except Exception as e:
            print(TAG + f"Exception occurred: {str(e)}")
            return jsonify({"error": f"Exception occurred: {str(e)}"}), 500
    elif request.method == "GET":
        rNode = transformNode2WorldCoordinates(robot_node)
        """print(
            TAG
            + f"Server knows Robot is in x:{robot_node.x}, y:{robot_node.y}, theta:{robot_node.theta}"
        )
        print(
            TAG
            + f"Server will send to client Robot is in x:{rNode.x }, y:{rNode.y }, theta:{rNode.theta}"
        )"""
        return (
            jsonify(
                {
                    "x": rNode.x,
                    "y": rNode.y,
                    "theta": rNode.theta,
                }
            ),
            200,
        )


@app.route("/start/position", methods=["GET", "POST"])
def start_position():
    global start_node
    TAG = "\n/start/position "
    if request.method == "POST":
        try:
            data = request.json
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta")

            if x is None or y is None or theta is None:
                print(TAG + "Missing x, y, or theta in request")
                return jsonify({"error": "Missing x, y, or theta in request"}), 400

            start_node = transformNode2MapCoordinates(
                Human(x, y, start_node.radius, theta)
            )
            #print(TAG + f"According to client start is in x:{x}, y:{y}, theta:{theta}")
            #print(
            #    TAG
            #    + f"According to server start is in x:{ start_node.x}, y:{ start_node.y}, theta:{getattr(start_node, 'theta', 0)}"
            #)
            return jsonify({"message": "Start position updated successfully"}), 200
        except Exception as e:
            print(TAG + f"Exception occurred: {str(e)}")
            return jsonify({"error": f"Exception occurred: {str(e)}"}), 500
    elif request.method == "GET":
        sNode = transformNode2WorldCoordinates(start_node)
        #print(
        #    TAG
        #    + f"Server knows start of Robot is in x:{start_node.x }, y:{start_node.y }, theta:{start_node.theta}"
        #)
        #print(
            #TAG
            #+ f"Server will send to client Robot is in x:{sNode.x }, y:{sNode.y }, theta:{getattr(sNode, 'theta', 0)}"
        #)
        return (
            jsonify(
                {
                    "x": sNode.x,
                    "y": sNode.y,
                    "theta": getattr(sNode, "theta", 0),
                }
            ),
            200,
        )


@app.route("/robot/position_using_camera", methods=["POST"])
def robot_position_using_camera():
    global robot_node
    TAG = "\n/robot/position_using_camera "
    if request.method == "POST":
        try:
            data = request.json
            dtime = data["time"]
            #print(f"Type of image data: {type(data["image"])}")
            image_bytes = base64.b64decode(data["image"])
            np_arr = np.frombuffer(image_bytes, np.uint8)

            # Decode to image
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR_RGB)

            if img is None:
                print(TAG + "image field is empty")
                return jsonify({"message": TAG + "image field is empty"}), 400

            cameraTransform = data["cameraTransform"]
            if cameraTransform is None:
                print(TAG + "camera transform is empty")
                return jsonify({"message": TAG + "camera transform is empty"}), 400

            #print(
                #TAG
                #+ f"Camera Transform is:  {cameraTransform},\n{parse_transform_string_to_matrix(cameraTransform)}"
            #)

            pose = estimate_robot_pose_from_markers(
                img,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                marker_length=marker_length,
                known_marker_positions=known_marker_positions,
                T_cam_in_robot=parse_transform_string_to_matrix(cameraTransform),
            )
            if pose is None:
                print(TAG + "pose is empty")
                return jsonify({"message": TAG + "pose is empty"}), 400

            if pose["theta"] == None:
                print(TAG + "theta is empy")
                x, y = pose["camera_position"][0], pose["camera_position"][1]
                theta = 0
                return (
                    jsonify(
                        {"x": x, "y": y, "theta": theta, "message": "theta is empty"}
                    ),
                    200,
                )

            x, y, theta = (
                pose["robot_position"][0],
                pose["robot_position"][1],
                pose["theta"],
            )
            #print(
                #TAG
                #+ f"According to QrCode estimation, Robot is in x:{x}, y:{y}, theta:{theta}"
            #)
            return (
                jsonify({"x": x, "y": y, "theta": theta}),
                200,
            )
        except Exception as e:
            print(f"{TAG}: ERROR:  pose estimation failed: {str(e)}")
            return (
                jsonify(
                    {"message": f"{TAG}: ERROR:  pose estimation failed: {str(e)}"}
                ),
                400,
            )


@app.route("/goal/position", methods=["GET", "POST"])
def goal_position():
    global goal_node
    TAG = "\n/goal/position "
    if request.method == "POST":
        try:
            data = request.json
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta")

            if x is None or y is None or theta is None:
                print(TAG + "Missing x, y, or theta in request")
                return jsonify({"error": "Missing x, y, or theta in request"}), 400

            goal_node = transformNode2MapCoordinates(Human(x, y, 0, theta))
            #print(TAG + f"According to client goal is in x:{x}, y:{y}, theta:{theta}")
            
            return jsonify({"message": "Robot position updated successfully"}), 200
        except Exception as e:
            #print("/robot/position" + f"Exception occurred: {str(e)}")
            return jsonify({"error": f"Exception occurred: {str(e)}"}), 500
    elif request.method == "GET":
        return_goal_node = transformNode2WorldCoordinates(goal_node)
        
        return (
            jsonify(
                {
                    "x": return_goal_node.x,
                    "y": return_goal_node.y,
                    "theta": getattr(return_goal_node, "theta", 0),
                }
            ),
            200,
        )


@app.route("/map-upload", methods=["POST"])
def map_image_and_binary_map():
    TAG = "\n/map-upload"
    if "metadata" not in request.form or "map_file" not in request.files:
        print(
            TAG
            + f"Missing 'metadata' or 'map_file' in request: form:{request.form} and files:{request.files}"
        )
        return (
            jsonify(
                {
                    "error": f"Missing 'metadata' or 'map_file' in request: form:{request.form} and files:{request.files}"
                }
            ),
            400,
        )

    try:
        metadata_json = request.form["metadata"]
        metadata = json.loads(metadata_json)

        x = metadata.get("x")
        y = metadata.get("y")
        theta = metadata.get("theta")
        scale = metadata.get("scale")
        image_base64 = metadata.get("image_base64")

        map_metadata.update({"x": x, "y": y, "theta": theta, "scale": scale})

        #print(f"Received metadata: x={x}, y={y}, theta={theta}, scale={scale}")

        timestamp = str(int(time.time()))

        if image_base64:
            image_bytes = base64.b64decode(image_base64)
            image_filename = f"map_image_{timestamp}.png"

            with open(os.path.join(UPLOAD_FOLDER, image_filename), "wb") as img_file:
                img_file.write(image_bytes)
            #print(f"Saved map preview image: {image_filename}")

        map_file = request.files["map_file"]
        map_filename = f"map_buffer_{timestamp}.bin"
        map_path = os.path.join(UPLOAD_FOLDER, map_filename)

        map_file.save(map_path)
        json_map_metadata = os.path.join(
            UPLOAD_FOLDER, f"map_metadata_{timestamp}.json"
        )

        with open(json_map_metadata, "w") as meta_file:
            json.dump(map_metadata, meta_file)
        json_map_metadata = os.path.join(UPLOAD_FOLDER, f"map_metadata_latest.json")
        with open(json_map_metadata, "w") as meta_file:
            json.dump(map_metadata, meta_file)
        #print(TAG + f" Saved raw map buffer: {map_filename}")

        return (
            jsonify(
                {
                    "message": "Map and image received successfully",
                    "map_file": map_filename,
                }
            ),
            200,
        )

    except Exception as e:
        print("/map-upload" + f"Exception occurred: {str(e)}")
        return jsonify({"error": f"Exception occurred: {str(e)}"}), 500


@app.route("/map-download", methods=["GET"])
def latest_map_file():
    TAG = "\n/map-download"
    try:
        # List all .bin files in the upload folder
        bin_files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith(".bin")]
        if not bin_files:
            return jsonify({"error": TAG + " No map files found"}), 404

        # Find the latest file by timestamp in filename
        latest_file = max(bin_files, key=lambda f: int(f.split("_")[-1].split(".")[0]))
        return send_from_directory(UPLOAD_FOLDER, latest_file, as_attachment=True)
    except Exception as e:
        print(TAG + f"Exception occurred: {str(e)}")
        return jsonify({"error": TAG + f" Exception occurred: {str(e)}"}), 500


def init_navigation_path(TAG, destination_node):
    global map_metadata
    global robot_node
    global goal_node
    # In a real application, this would be generated based on the map and navigation logic
    if map_metadata == None:
        latest_map_metadata = os.path.join(UPLOAD_FOLDER, "map_metadata_latest.json")
        map_metadata = json.loads(open(latest_map_metadata, "r").read())
    bin_files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith(".png")]
    if not bin_files:
        print(TAG + "No map files found")
        return jsonify({"error": "No map files found"}), 404
    # Find the latest file by timestamp in filename
    latest_file = max(bin_files, key=lambda f: int(f.split("_")[-1].split(".")[0]))
    map_img = imread(os.path.join(UPLOAD_FOLDER, latest_file))
    #print(f"Using map file: {latest_file}")
    if map_img is None:
        print(TAG + "Failed to read the map image")
        return jsonify({"error": "Failed to read the map image"}), 500

    x_bounds = (0, map_img.shape[1])
    y_bounds = (0, map_img.shape[0])

    if debug:
        #print(TAG + f"Using map file: {latest_file}")
        #print(TAG + f"Map bounds: x={x_bounds}, y={y_bounds}")
        print(TAG + f"Robot Node: {robot_node}")
        print(TAG + f"Goal Node: {destination_node}")
        #print(TAG + f"Humans: {[human for human in humans]}")
        #print(TAG + f"Map Metadata: {map_metadata}")
    
    for human in humans:
        human.radius = manager.proxemics

    rrt = RRT(
        start_node=robot_node,
        goal_node=destination_node,
        x_bounds=x_bounds,
        y_bounds=y_bounds,
        humans=humans,
        max_iterations=max_iterations,
        step_size=step_size,
        radius=radius,
        tolerance=tolerance,
        map_data=latest_file,
        map_resolution=map_metadata["scale"],
        map_origin=[map_metadata["x"], map_metadata["y"]],
        obstacles=humans,
        debug=debug,
    )
    return rrt


@app.route("/navigation/path", methods=["GET"])
def navigation_path():
    global back
    TAG = "\n/navigation/path "
    try:
        destination_node = goal_node
        value = request.headers.get("back", "false").lower()
        back = value == "true"
        if back:
            destination_node = start_node

        rrt = init_navigation_path(TAG, destination_node)
        if debug:
            nodes, optimal_path, smooth_path, fig = rrt.run_complete_and_get_plot(
                robot_node=robot_node
            )
        else:
            nodes, optimal_path, smooth_path = rrt.run_complete()

        nodes = [transformNode2WorldCoordinates(node) for node in nodes]
        optimal_path = [transformNode2WorldCoordinates(node) for node in optimal_path]

        #print(TAG + f"\nsmooth_path before conversion is {smooth_path}\n")
        smooth_path = [transformNode2WorldCoordinates(node) for node in smooth_path]
        #print(TAG + f"\nsmooth_path is {smooth_path}\n")

        if debug:
            fig.savefig(
                f"{start_node.x}_{start_node.y}_to_{goal_node.x}_{goal_node.y}_prox={manager.proxemics}_back={back}.png"
            )

        # Return the actual path from run_complete
        return (
            jsonify(
                {
                    "nodes": [{"x": n.x, "y": n.y} for n in nodes],
                    "optimal_path": (
                        [{"x": n.x, "y": n.y} for n in optimal_path]
                        if optimal_path
                        else []
                    ),
                    "smooth_path": (
                        [{"x": n.x, "y": n.y} for n in smooth_path]
                        if smooth_path
                        else []
                    ),
                }
            ),
            200,
        )

    except Exception as e:
        print(TAG + f" Exception occurred: {str(e)}")
        return jsonify({"error": f" Exception occurred: {str(e)}"}), 500


@app.route("/navigation/path_with_plot", methods=["GET"])
def navigation_path_with_plot():
    global back
    TAG = "\n/navigation/path_with_plot "
    try:
        destination_node = goal_node
        value = request.headers.get("back", "false").lower()
        back = value == "true"
        if back:
            destination_node = start_node
        rrt = init_navigation_path(TAG, destination_node)

        nodes, optimal_path, smooth_path, fig = rrt.run_complete_and_get_plot(
            robot_node=robot_node
        )

        nodes = [transformNode2WorldCoordinates(node) for node in nodes]
        optimal_path = [transformNode2WorldCoordinates(node) for node in optimal_path]
        # print(TAG + f"smooth_path before conversion is {smooth_path}")
        smooth_path = [transformNode2WorldCoordinates(node) for node in smooth_path]

        # print(TAG + f"smooth_path is {smooth_path}")

        buf = io.BytesIO()
        fig.savefig(buf, format="png")
        buf.seek(0)
        img_base64 = base64.b64encode(buf.read()).decode("utf-8")
        fig.savefig(
            f"{start_node.x}_{start_node.y}_to_{goal_node.x}_{goal_node.y}_prox={manager.proxemics}_back={back}.png"
        )

        if not nodes or len(nodes) < 2:
            print(TAG + "No path found")
            return jsonify({"error": "No path found"}), 404

        # Return the actual path from run_complete
        return (
            jsonify(
                {
                    "nodes": [{"x": n.x, "y": n.y} for n in nodes],
                    "optimal_path": (
                        [{"x": n.x, "y": n.y} for n in optimal_path]
                        if optimal_path
                        else []
                    ),
                    "smooth_path": (
                        [{"x": n.x, "y": n.y} for n in smooth_path]
                        if smooth_path
                        else []
                    ),
                    "figure": img_base64,
                    "figure_format": "png",
                }
            ),
            200,
        )

    except Exception as e:
        print(TAG + f"Exception occurred: {str(e)}")
        return jsonify({"error": f"Exception occurred: {str(e)}"}), 500


@app.route("/visualize/robot_position", methods=["GET"])
def visualize_robot_position_in_map():
    TAG = "\n/visualize/robot_position "
    try:
        # Find latest map image
        bin_files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith(".png")]
        if not bin_files:
            return jsonify({"error": "No map image found"}), 404
        latest_file = max(bin_files, key=lambda f: int(f.split("_")[-1].split(".")[0]))
        map_img_path = os.path.join(UPLOAD_FOLDER, latest_file)
        map_img = cv2.imread(map_img_path)
        if map_img is None:
            return jsonify({"error": "Failed to read map image"}), 500

        # Draw robot position
        x = int(robot_node.x)
        y = int(robot_node.y)
        theta = getattr(robot_node, "theta", 0)
        #print(
        #    TAG
        #    + f"Robot position in map is x:{x}, y:{y}, theta:{theta} according to server information"
        #)
        img_viz = map_img.copy()
        # Draw robot as a circle
        cv2.circle(img_viz, (x, y), 8, (0, 0, 255), -1)
        # Draw orientation as an arrow
        arrow_length = 20
        end_x = int(x + arrow_length * np.cos(theta))
        end_y = int(y + arrow_length * np.sin(theta))
        cv2.arrowedLine(img_viz, (x, y), (end_x, end_y), (255, 0, 0), 2, tipLength=0.4)

        # Save visualization
        viz_path = os.path.join(VISUALIZATION_FOLDER, "robot_position_viz.png")
        cv2.imwrite(viz_path, img_viz)

        return send_from_directory(
            VISUALIZATION_FOLDER, "robot_position_viz.png", as_attachment=False
        )
    except Exception as e:
        print(TAG + " Exception occurred:", str(e))
        return jsonify({"error": f"Exception occurred: {str(e)}"}), 500


@app.route("/visualize/map_position", methods=["GET"])
def visualize_map_position_in_map():
    TAG = "\n/visualize/map_position "
    try:
        # Find latest map image
        bin_files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith(".png")]
        if not bin_files:
            return jsonify({"error": "No map image found"}), 404
        latest_file = max(bin_files, key=lambda f: int(f.split("_")[-1].split(".")[0]))
        map_img_path = os.path.join(UPLOAD_FOLDER, latest_file)
        map_img = cv2.imread(map_img_path)
        if map_img is None:
            return jsonify({"error": "Failed to read map image"}), 500

        # Draw map origin position
        x = int(mapShift.x)
        y = int(mapShift.y)
        theta = map_metadata.get("theta", 0)
        img_viz = map_img.copy()
        # Draw map origin as a green circle
        cv2.circle(img_viz, (x, y), 8, (0, 255, 0), -1)
        # Draw orientation as an arrow
        arrow_length = 20
        end_x = int(x + arrow_length * np.cos(theta))
        end_y = int(y + arrow_length * np.sin(theta))
        cv2.arrowedLine(
            img_viz, (x, y), (end_x, end_y), (0, 255, 255), 2, tipLength=0.4
        )

        # Save visualization
        viz_path = os.path.join(VISUALIZATION_FOLDER, "map_position_viz.png")
        cv2.imwrite(viz_path, img_viz)

        return send_from_directory(
            VISUALIZATION_FOLDER, "map_position_viz.png", as_attachment=False
        )
    except Exception as e:
        print(TAG + " Exception occurred:", str(e))
        return jsonify({"error": f"Exception occurred: {str(e)}"}), 500


@app.route("/navigation/go_to", methods=["GET", "POST"])
def nav_go_to():
    TAG = "\n/navigation/go_to"
    global next_node
    if request.method == "POST":
        try:
            data = request.json
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta")

            if x is None or y is None or theta is None:
                print(TAG + "Missing x, y, or theta in request")
                return jsonify({"error": "Missing x, y, or theta in request"}), 400

            next_node = transformNode2MapCoordinates(
                Human(x, y, next_node.radius, theta)
            )

            return jsonify({"message": "Robot position updated successfully"}), 200
        except Exception as e:
            print(TAG + f"Exception occurred: {str(e)}")
            return jsonify({"error": f"Exception occurred: {str(e)}"}), 500
    if request.method == "GET":
        next_node_w = transformNode2WorldCoordinates(next_node)
        return (
            jsonify(
                {
                    "x": next_node_w.x,
                    "y": next_node_w.y,
                    "theta": next_node_w.theta,
                }
            ),
            200,
        )


@app.route("/camera", methods=["POST"])
def camera():
    global camera_info
    TAG = "\n/camera "
    if request.method == "POST":
        try:
            data = request.json
            dtime = data["time"]
            #print(f"Type of image data: {type(data["image"])}")
            image_bytes = base64.b64decode(data["image"])
            np_arr = np.frombuffer(image_bytes, np.uint8)

            # Decode to image
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR_RGB)

            if image is None or dtime is None:
                print(TAG + "Missing image or time in request")
                return jsonify({"error": "Missing image or time in request"}), 400

            camera_info = {"image": image, "time": dtime}

            if debug:
                # I want to print and save camera info (maybe also visualize them)
                pass
                # print(TAG + "image is " + str(image))
                # print(TAG + "time is " + str(dtime))
                # fig = plt.figure()
                # plt.imshow(image)
                # file_name = os.path.join(ROBOT_PHOTOS_FOLDER, str(dtime) + ".png")
                # fig.savefig(file_name)

            return jsonify({"message": "depth_camera info updated successfully"}), 200

        except Exception as e:
            print(TAG + f"Exception occurred: {str(e)}")
            return jsonify({"error": f"Exception occurred: {str(e)}"}), 500


########################################################################
######################## CONVERSATION ENDPOINTS ########################
# Global conversation manager instance
# For multi-user use, switch to a session/token-based system
manager = ConversationManager()


@app.route("/respond", methods=["POST"])
def respond():
    """
    Endpoint to get a response from the conversation manager based on user input.
    Expects a JSON payload with a "user_input" field.
    """
    TAG = "\n/respond "
    try:
        start = time.time()
        data = request.get_json()
        user_input = data.get("user_input")
        reply = manager.respond(user_input)
        print(TAG + f"User input: {user_input}")
        print(TAG + f"Reply from manager: {reply}")
        print(TAG + f"Current phase: {manager.phase}")
        print(TAG + f"Current language: {manager.language}")
        print(TAG + f"Current language style: {manager.language_style}")
        print(TAG + f"Respond took: {time.time()-start}s")
    except Exception as e:
        print(TAG + f"Exception occurred: {str(e)}")
        return jsonify({"error": f"Exception occurred: {str(e)}"}), 500
    return jsonify({"reply": reply})


@app.route("/phase", methods=["POST", "GET"])
def phase():
    """
    Endpoint to get or set the current phase of the conversation manager.
    - GET: Returns the current phase.
    """
    TAG = "\n/phase "
    if request.method == "POST":
        data = request.get_json()
        new_phase = data.get("phase")
        try:
            print(TAG + f"Changing phase to {new_phase}")
            manager.set_phase(new_phase)
            return jsonify({"status": "success", "phase": new_phase})
        except AssertionError:
            return jsonify({"error": "Invalid phase"}), 400
    if request.method == "GET":
        return jsonify({"phase": manager.phase})


@app.route("/language", methods=["POST", "GET"])
def language():
    """
    Endpoint to get or set the language for the conversation manager.
    - GET: Returns the current language.
    - POST: Accepts a JSON payload with a "text" field to set the language
    """
    TAG = "\n/language "
    if request.method == "POST":
        data = request.get_json()
        lang_code = data.get(
            "language"
        )  # Expecting 0, 1, or 2 for Italian, English, or German
        try:
            print(TAG + f"Changing language to {lang_code}")
            manager.change_language(int(lang_code))
            return jsonify({"status": "success", "language": manager.language})
        except Exception as e:
            return jsonify({"error": "Invalid language code: " + e}), 400
    if request.method == "GET":
        return jsonify({"language": manager.language})


@app.route("/language_style", methods=["POST", "GET"])
def language_style():
    """
    Endpoint to get or set the language style for the conversation manager.
    - GET: Returns the current language style.
    - POST: Accepts a JSON payload with a "text" field to set the language style
    """
    TAG = "\n/language_style "
    if request.method == "POST":
        data = request.get_json()
        style_text = data.get("language_style")
        if not style_text:
            return jsonify({"error": "Missing text"}), 400
        print(TAG + f"Changing language style to {style_text}")
        manager.interpret_change_language_style(style_text)
        return jsonify({"status": "success", "style": manager.language_style})
    if request.method == "GET":
        return jsonify({"language_style": manager.language_style})


@app.route("/proxemics", methods=["POST", "GET"])
def proxemics():
    """
    Endpoint to get or set the proxemics radius for the conversation manager.
    - GET: Returns the current proxemics radius.
    - POST: Accepts a JSON payload with a "text" field to set the proxemics radius.
    The text should be useful for understanding if the user wants to change the proxemics.
    """
    global manager
    TAG = "\n/proxemics "
    if request.method == "POST":
        data = request.get_json()
        text = data.get("text")
        try:
            manager.interpret_change_proxemics(text)
            print(TAG + f"Proxemics radius set to {manager.proxemics}")
            return jsonify({"status": "success", "proxemics": manager.proxemics})
        except Exception as e:
            return jsonify({"error": f"Error while changing the proxemics: {e}"}), 400
    if request.method == "GET":
        return jsonify({"proxemics": manager.proxemics})


@app.route("/status", methods=["GET"])
def status():
    """
    Returns the current status of the conversation manager, including model, language, phase, proxemics, and style.
    This is useful for debugging and monitoring the state of the conversation.
    """
    TAG = "\n/status "
    st = {
        "model": manager.model,
        "language": manager.language,
        "phase": manager.phase,
        "proxemics": manager.proxemics,
        "style": manager.language_style,
    }
    print(TAG + f"Status: {st}")
    return jsonify(st)


@app.route("/reset", methods=["POST"])
def reset():
    """
    Resets the conversation manager to its initial state.
    This is useful for starting a new conversation or clearing the current state.
    """
    global manager
    TAG = "\n/reset "
    manager = ConversationManager()
    manager.set_paradigm("baseline", "English")    
    print(TAG + "Conversation manager reset")
    return jsonify({"status": "success", "message": "Conversation manager reset"})


@app.route("/paradigm", methods=["POST"])
def paradigm():
    """
    Sets the conversation manager to a new paradigm.
    """
    global manager
    TAG = "\n/paradigm "
    paradigm = request.json.get("paradigm")
    culture = request.json.get("culture", "English")  # Default to "default"
    manager.set_paradigm(paradigm, culture)
    print(
        TAG
        + "Conversation manager set to new paradigm: "
        + paradigm
        + " with culture: "
        + culture
    )

    return jsonify(
        {
            "status": "success",
            "message": "Conversation manager set to new paradigm: "
            + paradigm
            + "; with culture: "
            + culture,
        }
    )


@app.route("/is_proxemic_changed", methods=["GET"])
def is_proxemic_changed():
    """
    If the proxemic radius is changed, it returns a bool as True and the calls the navigation path
    """
    global manager
    TAG = "\n/is_proxemic_changed "
    if manager.proxemics == manager.prev_proxemics_radius:
        print(TAG + "Proxemics radius has not changed")
        return (
            jsonify(
                {
                    "changed": False,
                    "nodes": [],
                    "optimal_path": [],
                    "smooth_path": [],
                }
            ),
            200,
        )
    else:
        print(TAG + "Proxemics radius has changed")
        manager.prev_proxemics_radius = manager.proxemics
        destination_node = goal_node
        if back:
            destination_node = start_node
        rrt = init_navigation_path(TAG, destination_node)
        nodes, optimal_path, smooth_path = rrt.run_complete()
        nodes = [transformNode2WorldCoordinates(node) for node in nodes]
        optimal_path = [transformNode2WorldCoordinates(node) for node in optimal_path]
        smooth_path = [transformNode2WorldCoordinates(node) for node in smooth_path]
        # Return the actual path from run_complete
        return (
            jsonify(
                {
                    "changed": True,
                    "nodes": [{"x": n.x, "y": n.y} for n in nodes],
                    "optimal_path": (
                        [{"x": n.x, "y": n.y} for n in optimal_path]
                        if optimal_path
                        else []
                    ),
                    "smooth_path": (
                        [{"x": n.x, "y": n.y} for n in smooth_path]
                        if smooth_path
                        else []
                    ),
                }
            ),
            200,
        )


@app.route("/set_culture ", methods=["POST"])
def set_culture():
    global manager
    TAG = "\n/set_culture"
    if request.method == "POST":
        data = request.get_json()
        culture = data.get("culture")
        try:
            manager.change_culture(culture)
            print(TAG + f"Culture changed to {culture}")
            return jsonify({"status": "success"})
        except Exception as e:
            return jsonify({"error": f"Error while changing the proxemics: {e}"}), 400

########################################################################

app.run(host="0.0.0.0", port=5000, debug=debug)
