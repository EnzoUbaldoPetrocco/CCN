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
