#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import String
import requests
import socket

# Flask Server Configuration
ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]

FLASK_SERVER_URL = f"http://{ip}:5000"
FLASK_SERVER_URL += "/move"

AVAILABLE_KEYS = ['up', 'down', 'left', 'right', 'w', 's', 'a', 'd']

class KeyboardCommands(Node):
    def __init__(self):
        super().__init__("KeyboardCommands")
        self.publisher_ = self.create_publisher(String, 'keyboard_commands', 10)

    def send_command_to_flask(self,direction):
        try:
            response = requests.put(f"{FLASK_SERVER_URL}?direction={direction}", headers={'Content-Type': 'application/json'})
            self.get_logger().info(f"Sent direction {direction} to Flask server: {response.json()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send direction {direction} to Flask server: {e}")



    def on_press(self, key :keyboard.Key):
        try:
            if key.char in AVAILABLE_KEYS:
                print('Command key {0} pressed'.format(
                    key.char))                
                if key.char == 'w':
                    self.send_command_to_flask('front')
                elif key.char == 's':
                    self.send_command_to_flask('back')
                elif key.char == 'a':
                    self.send_command_to_flask('left')
                elif key.char == 'd':
                    self.send_command_to_flask('right')
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            self.send_command_to_flask('stop')

    def on_release(self,key:keyboard.Key):
        self.send_command_to_flask('stop')
        

def main(args=None):
    rclpy.init(args=args)
    kc = KeyboardCommands()

    # ...or, in a non-blocking fashion:
    listener = keyboard.Listener(
        on_press=kc.on_press,
        on_release=kc.on_release)
    listener.start()

    rclpy.spin(kc)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
