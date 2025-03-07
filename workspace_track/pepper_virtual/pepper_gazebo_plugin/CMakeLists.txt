cmake_minimum_required(VERSION 2.8.3)
project(pepper_gazebo_plugin)

find_package(catkin REQUIRED COMPONENTS
rospy
  dynamic_reconfigure
  # other dependencies...
)
generate_dynamic_reconfigure_options(cfg/IncrementConfig.cfg)


catkin_package()


install(DIRECTORY launch worlds
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
