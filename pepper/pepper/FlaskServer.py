#!/usr/bin/env python3.10
import base64
import time
import rclpy
from rclpy.node import Node
from flask import Flask, make_response
from flask import request
from std_msgs.msg import String
from PIL import Image as PImage

from robot_interfaces.srv import Inference
from sensor_msgs.msg import Image

from matplotlib import pyplot as plt
import asyncio
import os
from flask_cors import CORS
from flask import jsonify
import socket
import numpy as np
from werkzeug.utils import secure_filename
from PIL import Image
import json

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]

#Initialization parameters:
img_culture = "Unknown"
sim_with_models = False
cultural_info = None # Cultural info can be 0,1,2 
CC = False

# Mock models (Replace with actual model loading logic)
if sim_with_models:
        # Select the appropriate model based on cultural information
        if not CC:
            model = None #tf.keras.models.load_model('simple_model.h5')
        else:
            model = None #tf.keras.models.load_model('culturally_aware_model.h5')
            if cultural_info!=None:
                discriminator_model = None #tf.keras.models.load_model('discriminator.h5')
                
else:
    model = None


import json

def append_to_json_file(file_path, new_data):
    try:
        # Step 1: Read the existing JSON data
        with open(file_path, 'r') as file:
            data = json.load(file)  # Load existing data
        
        # Step 2: Ensure it's a list (or appropriate structure)
        if isinstance(data, list):
            data.append(new_data)  # Append the new dictionary
        else:
            raise ValueError("The JSON root must be a list to append data.")
        
        # Step 3: Write the updated JSON data back to the file
        with open(file_path, 'w') as file:
            json.dump(data, file, indent=4)
        
        print("Data appended successfully!")
    
    except FileNotFoundError:
        # If the file does not exist, create it with the new data
        with open(file_path, 'w') as file:
            json.dump([new_data], file, indent=4)
        print("File not found. Created a new JSON file with the provided data.")
    
    except json.JSONDecodeError:
        print("Error: The file is not a valid JSON.")
    
    except Exception as e:
        print(f"An error occurred: {e}")


# Movement Command received from the Keyboard Node via ROS
current_command = 'stop'

app = Flask(__name__)

UPLOAD_FOLDER = 'uploads'
res_file = 'results.json'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg', 'gif'}
# Ensure the upload folder exists
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/move', methods=['PUT'])
def move():
    global current_command
    direction = request.args.get('direction')
    current_command = direction
    # You could log or process this command further if needed
    print(f"Direction received: {direction}")
    return jsonify({"status": "Command received", "direction": current_command})

@app.route('/predict', methods=['POST'])
def predict():
    t_start = time.time()
    if 'image' not in request.files:
        return jsonify({"error": "No image file uploaded"}), 400

    image = request.files['image']
    print(image)
    if image.filename == '' or not allowed_file(image.filename):
        return jsonify({"error": "Invalid file type"}), 400

    filename = secure_filename(image.filename)

    # Save the file
    try:
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        image.save(filepath)
    except FileNotFoundError:
        # If the directory doesn't exist, handle it here
        print(f"Error: The file {filepath} does not exist.")
        
        # You can create the directory and the file if needed
        if not os.path.exists(app.config['UPLOAD_FOLDER']):
            print(f"Creating directory: {app.config['UPLOAD_FOLDER']}")
            os.makedirs(app.config['UPLOAD_FOLDER'])

        image.save(filepath)

    image = np.asarray(image) 
    print(image)
        
    # Return the prediction and the command
    if sim_with_models:
        prediction = model.predict(image)
        if CC:
            if cultural_info!=None:
                res = {
                "prediction": prediction[cultural_info],
                "current_command": current_command
                }
                t_stop = time.time()
                deltaT = t_stop - t_start
                line = {
                    "prediction": prediction[cultural_info],
                    "probabilities": list(prediction),
                    "cultural_info" : cultural_info,
                    "filepath": filepath,
                    "label":None,
                    "deltaT": deltaT
                }
            else:
                cultural_probs = discriminator_model.predict(image)
                res = {
                "prediction": np.dot(prediction, cultural_probs),
                "current_command": current_command
                }
                t_stop = time.time()
                deltaT = t_stop - t_start
                line = {
                    "prediction": np.dot(prediction, cultural_probs),
                    "probabilities": list(prediction),
                    "cultural_info" : list(cultural_probs),
                    "filepath": filepath,
                    "label":None,
                    "deltaT": deltaT
                }
        else:
            res = {
            "prediction": prediction,
            "current_command": current_command
            }
            t_stop = time.time()
            deltaT = t_stop - t_start
            line = {
                "prediction": prediction,
                "probability": int(prediction),
                "filepath": filepath,
                "label":None,
                "deltaT": deltaT
            }
    else:
        prediction = np.random.random()
        res = {
        "prediction": prediction,
        "current_command": current_command
        }
        t_stop = time.time()
        deltaT = t_stop - t_start
        line = {
            "prediction": prediction,
            "probability": int(prediction),
            "filepath": filepath,
            "label":None,
            "deltaT": deltaT
        }
    
    append_to_json_file(res_file, line)

    return jsonify(res)





def main(args=None): 
    app.run(host='0.0.0.0', port=5000)


if __name__ == '__main__':
    main()
