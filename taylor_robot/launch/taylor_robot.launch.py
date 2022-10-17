import os
import re
from sys import prefix

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():

    package_name='taylor_robot' 

    # GAZEBO WORLD
    world_description_package='lab_usig_world_description' 
    world_name='lab_28'
    world_path = PathJoinSubstitution(
        [FindPackageShare(world_description_package), "worlds", world_name]
    )

    #remappings
    remappings = [('/diff_cont/cmd_vel_unstamped', 'cmd_vel')]

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','taylor.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control'}.items()
                
    )

    robot_description=Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",      
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",      
        #remappings=[('/diff_cont/cmd_vel_unstamped', '/cmd_vel')],
        arguments=["diff_cont"]
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"]

    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        diff_drive_spawner,
        joint_broad_spawner,
    ])