# For running the docker file
```
docker run -p 6080:6080 --name ros-gui -it ros-humble-gui
```
# From VSCode gui
- Open in Browser
- Go to vnc.html or where you want

# ROS workspace
```
mkdir ros2_ws
```
```
cd ros2_ws
```
```
mkdir src
```
```
colcon build
```
```
source install/setup.bash
```

# Gazebo installation
```
RUN cd ign-gazebo
```

```
RUN mkdir build
```

```
RUN cd build
```

```
RUN cmake ../
```

```
RUN make
```

# ROS docker/laptop communication
For exchanging ROS messages between your laptop and the docker container you need to have the same ROS_DOMAIN_ID as environment variable and you need to be connected to the same LAN. 
By default we set the ROS_DOMAIN_ID to 42 (range is between 0 and 101).
```
export ROS_DOMAIN_ID=42
```

# ROS Naoqi Driver 2
Instruction for install naoqi driver 2 are already mapped in the dockerfile, but for completeness I will add them here
```
cd <ws>/src
```
```
git clone https://github.com/ros-naoqi/naoqi_driver2.git
```
```
vcs import < naoqi_driver2/dependencies.repos
```
```
cd <ws>
```
```
rosdep install --from-paths src --ignore-src --rosdistro <distro> -y
```
```
colcon build --symlink-install
```
## Avoid Interference with autonomous life
```
ssh nao@<robot_host>
```
```
qicli call ALAutonomousLife.setState disabled
```
```
qicli call ALMotion.wakeUp
```
## NAOqi 2.8 and lower
```
source /opt/ros/<distro>/setup.bash # or source <ws>/install/setup.bash if built from source
```
```
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> qi_listen_url:=tcp://0.0.0.0:0
```

## NAOqi 2.9 and higher
```
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> nao_username:=nao nao_password:=<robot_password> qi_listen_url:=tcp://0.0.0.0:0
```

## From a Docker Container
```
source /opt/ros/<distro>/setup.bash # or source <ws>/install/setup.bash if built from source
```
```
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> qi_listen_url:=tcp://0.0.0.0:56000
```

## On the robot or on the same machine
(For running directly from the robot or running a virtual robot)
```
ros2 launch naoqi_driver naoqi_driver.launch.py
```

## More on : [naoqi_driver2 Github repo](https://github.com/ros-naoqi/naoqi_driver2)
