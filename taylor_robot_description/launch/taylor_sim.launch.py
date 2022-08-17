import os
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

    package_name='taylor_robot_description' 

    # GAZEBO WORLD
    world_description_package='lab_usig_world_description' 
    world_name='lab_25'

    world_path = PathJoinSubstitution(
        [FindPackageShare(world_description_package), "worlds", world_name]
    )

    #rviz_config_path = PathJoinSubstitution(
    #    [FindPackageShare(package_name), "rviz", "my_bot.rviz"]
    #)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','taylor.launch.py'
                )]), launch_arguments={'use_sim_time': 'true',}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'taylor_robot'],
                        output='screen')

    #rviz = Node(package='rviz2', 
    #            executable='rviz2', 
    #            name="rviz2", 
    #            arguments=['-d', rviz_config_path],
    #            output='screen')


    # Launch them all!
    return LaunchDescription([
        rsp,
        ExecuteProcess( 
            cmd=['gazebo', '--verbose',  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-w', world_path], 
            output='screen'),
        spawn_entity,
        #rviz
    ])