from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros
from launch.substitutions.command import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()


    robot_spawn = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('pepper_moveit_config'), 'launch', 'demo.launch']),
        launch_arguments={
            
        })  
    ld.add_action(robot_spawn)
    

    return ld