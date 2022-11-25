import os
import subprocess
import time
import rclpy

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def wait_until_up(necessary_nodes):
    rclpy.init()
    
    # Decode because the subproces.check_output return a bytes object
    output = subprocess.check_output(["ros2", "topic", "list"]).decode("utf-8") 

    i = 0;
    dont_find_a_node = False

    while i < len(necessary_nodes):
        while output.find(necessary_nodes[i]) == -1:
            rclpy.logging.get_logger('Taylor Robot Launch').warn('Waiting for the ' + necessary_nodes[i] + ' node! ')
            output = subprocess.check_output(["ros2", "topic", "list"]).decode("utf-8") 
            dont_find_a_node = True
            time.sleep(1.0)
        
        # If a node was not up, so check again all the nodes
        if dont_find_a_node == True:
            i = 0
        else:
            i += 1
    
    rclpy.logging.get_logger('Taylor Robot Launch').info('All necessary nodes are up!')

def generate_launch_description():
    
    necessary_nodes = ['robot_description']

    wait_until_up(necessary_nodes)

    # Launch them all!
    return LaunchDescription([
    ])