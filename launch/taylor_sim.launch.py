import os
import rclpy

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name='taylor_robot' 

    debbug_mode = LaunchConfiguration('debbug_mode')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file')

    # gazebo verbosity on/off
    if debbug_mode:
        gazebo_verbosity = '--verbose'
    else:
        gazebo_verbosity = ''

    # GAZEBO WORLD
    world_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'description', 'gazebo', 'worlds', gazebo_world_file]
    )

    #remappings
    remappings = [('/diff_cont/cmd_vel_unstamped', 'cmd_vel')]

    #rviz_config_path = PathJoinSubstitution(
    #    [FindPackageShare(package_name), "rviz", "my_bot.rviz"]
    #)
    #

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','taylor.launch.py'
                )]), launch_arguments={'use_sim_time': 'true',}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'taylor_robot',
                        '-x','12','-y','10'],
                        output='screen')

    #rviz = Node(package='rviz2', 
    #            executable='rviz2', 
    #            name="rviz2", 
    #            arguments=['-d', rviz_config_path],
    #            output='screen')

    #remappings = [('/diff_cont/cmd_vel_unstamped', '/cmd_vel')]

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
        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument('debbug_mode', default_value='false'),
        DeclareLaunchArgument('gazebo_world_file', default_value='lab_world.sdf'),
        LogInfo(msg=['Launching ', LaunchConfiguration('robot_name'), ' robot!']),
        rsp,
        ExecuteProcess( 
            cmd=['gazebo', gazebo_verbosity,  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-w', world_path], 
            output='screen'),
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        #rviz
    ])