#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
import requests
import os
from pathlib import Path
import random
import time
from matplotlib import pyplot as plt
from functools import wraps
import json
import socket
#ip = "130.251.218.63"
ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]



url = "http://" + ip + ":5000"
#url = "http://130.251.13.139:5000"

random.seed(time.time())

frequency = 3

import requests

def upload_image(target_url, file_path):
    try:
        # Open the image file in binary mode
        with open(file_path, 'rb') as file:
            # Prepare the files payload for multipart form data
            files = {'image': (str(file_path).split('/')[-1], file, 'image/jpeg')}
            
            # Make the POST request
            res = requests.post(target_url, files=files)

            # Print response details
            print("Response Code:", res.status_code)
            print("Response Text:", res.text)
            return res

    except Exception as e:
        print(f"An error occurred: {e}")



class RobotSimulator(Node):
    def __init__(self):
        super().__init__("RobotSimulator")
        self.init_ds()
        
        self.timer = self.create_timer(frequency, self.send_random_image)

    def init_ds(self):
        rt = "/home/enzo/Desktop/FINALDS"
        
        carpet_paths = [
            rt + "/carpets_stretched/indian/200/RGB",
            rt + "/carpets_stretched/japanese/200/RGB",
            rt + "/carpets_stretched/scandinavian/200/RGB",
        ]

        self.init_spec_dataset(carpet_paths)

    def init_spec_dataset(self, paths):
        self.dataset = []
        for j, path in enumerate(paths):
            labels = self.get_labels(path)
            imgs_per_culture = []
            for i, label in enumerate(labels):
                images = self.get_images_paths(path + "/" + label)
                X = []
                for k in range(len(images)):
                    X.append([images[k], [j, i]])
                # print(f"Culture is {j}, label is {i}")
                # plt.imshow(images[0])
                # plt.show()
                imgs_per_culture.append(X)
                del images
                del X
            self.dataset.append(imgs_per_culture)

        self.carpet_ds = self.dataset

        self.dataset = None

    def get_labels(self, path):
        """
        get_labels returns a list of the labels in a directory

        :param path: directory in which search of the labels
        :return list of labels
        """
        dir_list = []
        for file in os.listdir(path):
            d = os.path.join(path, file)
            if os.path.isdir(d):
                d = d.split("\\")
                if len(d) == 1:
                    d = d[0].split("/")
                d = d[-1]
                dir_list.append(d)
        return dir_list

    def get_images_paths(self, path, n=1000, rescale=False):
        """
        get_images returns min(n, #images contained in a directory)

        :param path: directory in which search for images
        :param n: maximum number of images

        :return list of paths
        """
        types = ("*.png", "*.jpg", "*.jpeg")
        paths = []
        for typ in types:
            paths.extend(Path(path).glob(typ))
        paths = paths[0 : min(len(paths), n)]
        return paths

    def send_random_image(self):
        file_path = self.rnd_get_image()
        target_url = f"http://{ip}:5000/predict"  # Flask server endpoint
        
        res = upload_image(target_url, file_path)
        if res.status_code==200:
            print(res)
        else:
            print(f"ERROR: {res.status_code} ")

        print(json.loads(res.text))

    def rnd_get_image(self):
        img, label =  self.carpet_ds[random.randint(0, len(self.carpet_ds)-1)][random.randint(0, len(self.carpet_ds[0])-1)][random.randint(0, len(self.carpet_ds[0][0])-1)]
        return img
    
    

def main(args=None):
    rclpy.init(args=args)
    robot = RobotSimulator()
    
    rclpy.spin(robot)
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
