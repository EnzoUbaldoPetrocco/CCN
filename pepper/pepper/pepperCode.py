# Import necessary libraries
import time
import json
import threading
import qi  # Import NAOqi's SDK
import requests  # To handle HTTP requests for image uploading
import argparse
import sys
import numpy as np
import cv2
from io import BytesIO
from matplotlib import pyplot as plt
#import vision_definitions https://github.com/ahornung/nao_camera/blob/master/src/nao_camera/vision_definitions.py
# overflow https://stackoverflow.com/questions/77987028/how-can-i-connect-to-pepper-naoqi-2-9-via-libqi-python

# Robot's IP and port
ROBOT_IP = "10.186.13.37"
PORT = 9503 
SERVER_IP = "10.186.13.39"

x_shift = 0.2
theta_shift = 0.5
username = "nao"
password = "venere"

class Authenticator:

    def __init__(self, username, password):
        self.username = username
        self.password = password

    # This method is expected by libqi and must return a dictionary containing
    # login information with the keys 'user' and 'token'.
    def initialAuthData(self):
        return {'user': self.username, 'token': self.password}

class AuthenticatorFactory:

    def __init__(self, username, password):
        self.username = username
        self.password = password

    # This method is expected by libqi and must return an object with at least
    # the `initialAuthData` method.
    def newAuthenticator(self):
        return Authenticator(self.username, self.password)

class ImageUploader:
    def upload_image(self, target_url, encoded_image, counter):
        # Assuming encoded_image is a bytearray or bytes object

        boundary = "----WebKitFormBoundary" + str(int(time.time() * 1000))
        file_name = "image_{}.jpg".format(counter)

        # Set up the headers
        headers = {
            "Content-Type": "multipart/form-data; boundary={}".format(boundary)
        }

        # Convert the NumPy array to a JPEG image in memory
        _, buffer = cv2.imencode('.jpg', encoded_image)
        image_bytes = BytesIO(buffer)

        files = {"image": (file_name, image_bytes, "image/jpeg")}

        # Make the POST request
        try:
            response = requests.post(target_url, files=files)
            response.raise_for_status()  # Raise an error if the response is not successful
            return json.loads(response.text)  # Parse the JSON response
        except Exception as e:
            print("Error uploading image:", e)
            return None

class MainActivity:
    def __init__(self, ip, port):
        # Connect to the robot fails at app.start() => RuntimeError: disconnected
        self.app = qi.Application(sys.argv, url=f"tcps://{ip}:{port}")
        logins = (username, password)
        factory = AuthenticatorFactory(*logins)
        self.app.session.setClientAuthenticatorFactory(factory)
        self.app.start()

        print("started")
        self.counter = 0
        self.target_url = f"http://{SERVER_IP}:5000/predict"  # Flask server endpoint
        self.current_command = None
        self.prediction = 0.0
        self.ip = ip
        self.port = port
        self.motion_proxy = None
        self.camera_proxy = None
        self.nameId = None

    def initialize_proxies(self):
        """
        Initialize the proxies for motion and camera.
        """
        try:
            self.motion_proxy = self.app.session.service("ALMotion")
            self.video_service = self.app.session.service("ALVideoDevice")

            # Register a Generic Video Module
            resolution = 0 # 160*120 
            colorSpace = 11 #vision_definitions.kYUVColorSpace
            fps = 20
            camera_index = 0
            subs = self.video_service.getSubscribers()
            for sub in subs:
                self.video_service.unsubscribe(sub)
                
            self.nameId = self.video_service.subscribeCamera("CCN", camera_index,  resolution, colorSpace, fps)
            print(self.nameId)

            #self.camera_proxy = self.app.session.service("ALPhotoCapture") #ALProxy("ALPhotoCapture", self.ip, self.port)
            print("Proxies initialized successfully.")
        except Exception as e:
            print("Error initializing proxies:", e)

    def take_picture(self):
        """
        Takes a picture using the robot's camera and returns the image data.
        """
        try:
            # Set camera parameters (e.g., resolution and format)
            #self.camera_proxy.setResolution(2)  # 2 = 640x480
            #self.camera_proxy.setPictureFormat("jpg")

            # Capture and save the image to a temporary file
            image_path = "~/enzo/images/image_{}.jpg".format(self.counter)
            #self.camera_proxy.takePicture("/home/nao/", "image_{}".format(self.counter))
            img = self.video_service.getImageRemote(self.nameId)
            #print(img)
            shape = img[0:3]
            img = list(img[6])
            img = np.reshape(img, (shape))

            return  img
        except Exception as e:
            print("Error taking picture:", e)
            return None

    def on_robot_focus_gained(self):
        """
        The main loop that takes pictures and uploads them periodically.
        """
        img_uploader = ImageUploader()

        def send_info_to_server():
            while True:
                # Take a picture
                image_data = self.take_picture()
                
                try:
                    # Upload the image and get the response
                    json_response = img_uploader.upload_image(self.target_url, image_data, self.counter)
                    if json_response:
                        if "prediction" in json_response:
                            self.prediction = json_response["prediction"]
                            print("Prediction:", self.prediction)

                        if "current_command" in json_response:
                            self.current_command = json_response["current_command"]
                            print("Current Command:", self.current_command)
                            if self.current_command=="stop": 
                                # TODO: MoveToward()
                                x     = 0.0
                                y     = 0.0
                                theta = 0.0
                            elif self.current_command=="front": 
                                # TODO: MoveToward()
                                x     = x_shift
                                y     = 0.0
                                theta = 0.0
                                
                            elif self.current_command=="back": 
                                # TODO: MoveToward()
                                x     = -x_shift
                                y     = 0.0
                                theta = 0.0
                            elif self.current_command=="left": 
                                # TODO: MoveToward()
                                x     = 0.0
                                y     = 0.0
                                theta = theta_shift
                            elif self.current_command=="right": 
                                # TODO: MoveToward()
                                x     = 0.0
                                y     = 0.0
                                theta = -theta_shift
                            self.motion_proxy.moveToward(x, y, theta)
                            time.sleep(1)
                            self.motion_proxy.stopMove()

                    self.counter += 1

                except Exception as e:
                    print("Error in upload or response processing:", e)

                time.sleep(3)  # Wait for 3 seconds before taking the next picture

        # Start the thread for the task
        thread = threading.Thread(target=send_info_to_server)
        thread.start()

    def on_robot_focus_lost(self):
        """
        Called when the robot focus is lost.
        """
        print("Robot focus lost.")

    def on_robot_focus_refused(self, reason):
        """
        Called when the robot focus is refused.
        """
        print("Robot focus refused:", reason)


def main():
    # Initialize and start the main activity
    main_activity = MainActivity(ROBOT_IP, PORT)
    main_activity.initialize_proxies()
    main_activity.on_robot_focus_gained()
    
if __name__ == "__main__":
    main()
