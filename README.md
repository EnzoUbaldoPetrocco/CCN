# CCN
Application for Implementing Pepper Navigation
 
 Ros 1 Installing: [Noetic Guide for Ubuntu 22.04](https://gist.github.com/Meltwin/fe2c15a5d7e6a8795911907f627255e0), [Actually I used this](https://github.com/GNDeSouza/ROS-Noetic-and-Gazebo-in-Ubuntu-22.04)
 Pepper connection via NaOqi 2.9 and libqi-python: [StackOverflow](https://stackoverflow.com/questions/77987028/how-can-i-connect-to-pepper-naoqi-2-9-via-libqi-python), [GithubRepo](https://github.com/aldebaran/libqi-python/issues/22#issuecomment-1941618222)

Pepper Robot repository [Github](https://github.com/ros-naoqi/pepper_robot), [Controlling Nao](https://wiki.ros.org/nao/Tutorials/Getting-Started#Controlling_NAO)

[Pepper Virtual Github Repo](https://github.com/ros-naoqi/pepper_virtual/tree/master)

[Pepper Description](https://github.com/jrl-umi3218/pepper_description)



Navigation algorithms: [GMAPPING](https://wiki.ros.org/gmapping), [AMCL](https://wiki.ros.org/amcl)

[Pepper Navigation Repository](https://www.finnrietz.dev/linux/how-to-pepper-navigation/)

[Pepper Navigation Github Repository](https://github.com/qcr/pepper_navigation) but it says "ROS Packages available for Kinetic and Melodic".

[Naoqi Driver 2](https://github.com/ros-naoqi/naoqi_driver2)

[Pepper Chat](https://github.com/ilabsweden/pepperchat)

# Things that may actually work

[Working Pepper Robot](https://github.com/awesomebytes/pepper_robot)
[Working Pepper Virtual](https://github.com/awesomebytes/pepper_virtual)

## Changes from this amazing setup:
### in pepper_publisher.launch (in pepper_robot/pepper_description/launch directory), change type="state_publisher"->type="robot_state_publisher"
### type="state_publisher"->type="robot_state_publisher" in every launch file in pepper_virtual/pepper_gazebo_plugin/launch/ directory
### laser Publisher does not work
- laser_publisher ddynamic_reconfigure does not work --> changed for implementing dynamic_reconfigure
Put: 

```
generate_dynamic_reconfigure_options(
  cfg/IncrementConfig.cfg
)
```

in CMakeLists.txt.
Create an IncrementConfig.cfg file containing the following code:

```
#!/usr/bin/env python

from dynamic_reconfigure.parameter_generator_catkin import *

PACKAGE = "pepper_gazebo_plugin"  # Replace with your package name

gen = ParameterGenerator()

gen.add("angle_increment", double_t, 0, "Angle increment", 0.06, 0.05, 0.08)
gen.add("half_max_angle", double_t, 0, "Half maximum angle", 120.0, 115.0, 145.0)

exit(gen.generate(PACKAGE, "pepper_gazebo_plugin", "IncrementConfig"))
```

Instead of importing ddynamic_reconfigure -->

```
from dynamic_reconfigure.server import Server
from pepper_gazebo_plugin.cfg import IncrementConfig
```

Change from lines 271 to lines 286

```
self.angle_increment = radians(120.0 * 2.0) / 61.0  # Default value
        self.half_max_angle = 120.0  # Default value
        
        # Start the dynamic reconfigure server
        self.srv = Server(IncrementConfig, self.dyn_rec_callback)
        self.ts.registerCallback(self.scan_cb)
        rospy.loginfo("Ready to go.")

    def add_variables_to_self(self):
         
        # In case you want to create dynamic attributes based on the config
        self.__setattr__('angle_increment', self.angle_increment)
        self.__setattr__('half_max_angle', self.half_max_angle)
        return 
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        rospy.loginfo(f"Received reconfigure request: {config}")
        
        # Update the parameters based on the new configuration
        self.angle_increment = config['angle_increment']
        self.half_max_angle = config['half_max_angle']
        
        rospy.loginfo(f"Updated values: angle_increment={self.angle_increment}, half_max_angle={self.half_max_angle}")
        
        return config 
        rospy.loginfo("Received reconf call: " + str(config))
        # Update all variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config
```

modify:

```
print("dist, px, py: " + str(dist) +
                  " " + str(p[0])) + " " + str(p[1])
```

with:

```
print("dist, px, py: " + str(dist) +
                  " " + str(p[0]) + " " + str(p[1]))
```

replace:

```
print("tmp_angle: " + str(degrees(tmp_angle))) + " deg"
```

with:

```
print("tmp_angle: " + str(degrees(tmp_angle)) + " deg")
```

once compiled using catkin_make

go to /catkin_ws/devel/lib/python3/dist-packages/pepper_gazebo_plugin/cfg and make sure that
the name of the file is actually: IncrementConfig.py and that

```
from .IncrementConfig import *
```

is inside __init__.py, otherwise, add the line

### For launching both Rviz and Gazebo I have added a custom file named: "pgp_exp" 
The file contains also a reference to the launch file in pepper_description
```
<?xml version="1.0"?>
<launch>
  <arg name="launch_control_trajectory_all" default="true"/>
  <!-- Load the URDF Model -->
  <include file="$(find pepper_description)/launch/display.launch" />  

  <env name="GAZEBO_MODEL_PATH" value="$(find pepper_gazebo_plugin)/models:$(optenv GAZEBO_MODEL_PATH)"/>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find pepper_gazebo_plugin)/worlds/arena.world"/>
    <arg name="paused" value="false"/>
  </include>


  <!-- Call Pepper Robot Trajectory Controller -->

  <include file="$(find pepper_control)/launch/pepper_control_trajectory.launch" unless="$(arg launch_control_trajectory_all)"/>
  <include file="$(find pepper_control)/launch/pepper_control_trajectory_all.launch" if="$(arg launch_control_trajectory_all)"/>
<!--
  <include file="$(find pepper_control)/launch/pepper_control_position.launch"/>
-->

  <!--node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" /-->  
  <!-- Spawn a robot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
    args="-param robot_description -urdf -x -0.5 -y 1 -z 0.8 -model pepper_MP" />

  <!--node name="lower_arms" pkg="pepper_gazebo_plugin" type="arms_down.sh" output="screen" /-->
  
  <node name="laser_publisher" pkg="pepper_gazebo_plugin" type="laser_publisher2.py"/>

</launch>
```

### RVIZ
- Change "torso" -> "base_link" in pepper_robot/pepper_description/display.launch
- Always launch rviz before gazebo and then restart simulation if necessary

### Gazebo Controllers
- If you do not need navigation you can skip the wheels part

- uncomment (or add): 
```
    WheelFL_controller:
     type: effort_controllers/JointEffortController
     joint: WheelFL
    WheelFR_controller:
     type: effort_controllers/JointEffortController
     joint: WheelFR
    WheelB_controller:
     type: effort_controllers/JointEffortController
     joint: WheelB
```

Add these to the other controllers: 
```
/pepper/WheelB_controller /pepper/WheelFL_controller /pepper/WheelFR_controller
```

Add this in pepperGazebo.xacro
```
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <alwaysOn>true</alwaysOn>
    <legacyMode>false</legacyMode>
    <updateRate>100</updateRate>
    <robotNamespace>/</robotNamespace>
    <leftJoint>WheelFL</leftJoint>
    <rightJoint>WheelFR</rightJoint>
    <wheelSeparation>0.170</wheelSeparation>
    <wheelDiameter>0.140</wheelDiameter>
    <torque>36.9</torque>
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom_diffdrive</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <odometrySource>odom</odometrySource>
    <publishTf>1</publishTf>
    <rosDebugLevel>na</rosDebugLevel>
    <wheelAcceleration>0</wheelAcceleration>
    <wheelTorque>5</wheelTorque>
    <robotBaseFrame>base_link</robotBaseFrame>
    <publishWheelTF>false</publishWheelTF>
    <publishWheelJointState>false</publishWheelJointState>
  </plugin>
</gazebo>
```

Uncomment or add:
```
 <xacro:macro name="wheel_joints_transmissions" params="side">
                <transmission name="Wheel${side}_Transmission">
                        <type>transmission_interface/SimpleTransmission</type>
                        <joint name="Wheel${side}">
                                <hardwareInterface>EffortJointInterface</hardwareInterface>
                        </joint>
                        <actuator name="Wheel${side}_Motor">
                                <mechanicalReduction>${speed_red_type5B}</mechanicalReduction>
                        </actuator>
                </transmission>
        </xacro:macro>
        <xacro:wheel_joints_transmissions side="FL"/>
        <xacro:wheel_joints_transmissions side="B"/>
        <xacro:wheel_joints_transmissions side="FR"/> 
```