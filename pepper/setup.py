from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pepper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.world'))),
        (os.path.join('share', package_name, 'pepper_meshes/meshes/1.0'), glob(os.path.join('pepper_meshes/meshes/1.0', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rice',
    maintainer_email='enzopetrocco@hotmail.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
        "controller = pepper.Controller:main",
            "keyboardCommands = pepper.KeyboardCommands:main",
            "flaskServer = pepper.FlaskServer:main",
            "modelInference = pepper.ModelInference:main",
            "robotSimulator = pepper.RobotSimulator:main",
            'state_publisher = pepper.state_publisher:main'
        ],
    },
)
