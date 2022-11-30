import os
import subprocess
import time
import rclpy

from ament_index_python.packages import get_package_share_directory

from colorama import Fore
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    print(Fore.LIGHTRED_EX + r"""

▀█▀ ▄▀█ █▄█ █░░ █▀█ █▀█   █▀█ █▀█ █▄▄ █▀█ ▀█▀  
░█░ █▀█ ░█░ █▄▄ █▄█ █▀▄   █▀▄ █▄█ █▄█ █▄█ ░█░  

░░░░░░░▄█▄▄▄█▄
▄▀░░░░▄▌─▄─▄─▐▄░░░░▀▄
█▄▄█░░▀▌─▀─▀─▐▀░░█▄▄█
░▐▌░░░░▀▀███▀▀░░░░▐▌
████░▄█████████▄░████

    """)
    print(Fore.RESET) 


    namespace_arg = DeclareLaunchArgument('namespace', default_value='taylor')
    namespace = LaunchConfiguration('namespace')

    package_name = 'taylor_robot'

    ros2_control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','ros2_control.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'namespace': namespace}.items()
    )

    # Launch them all!
    return LaunchDescription([
        namespace_arg,
        ros2_control
    ])