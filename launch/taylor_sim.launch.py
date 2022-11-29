import os
import rclpy

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name='taylor_robot' 

    # see https://answers.gazebosim.org//question/2596/gazebo-being-moody-sometimes-starting-sometimes-not/

    cool_path = os.path.join(get_package_share_directory(package_name), 'launch', 'cool_taylor.txt')
    taylor_ascii = open(cool_path, 'r').read()

    # Arguments
    debbug_mode_arg = DeclareLaunchArgument('debbug_mode', default_value='false')
    gazebo_world_file_arg = DeclareLaunchArgument('gazebo_world_file', default_value='lab_world.sdf')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='taylor')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='false')
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='true')

    # Arg vars
    debbug_mode = LaunchConfiguration('debbug_mode')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file')
    namespace = LaunchConfiguration('namespace')
    simulation = LaunchConfiguration('simulation') 
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # If launch with sim on
        #launch gazebo 
    # otherwise
        # don't launch gazebo

    # gazebo verbosity on/off
    if debbug_mode:
        gazebo_verbosity = '--verbose'
    else:
        gazebo_verbosity = ''

    

    robot_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name),'launch','taylor.launch.py')
        ]), 
        launch_arguments={'use_sim_time': simulation, 'use_ros2_control': use_ros2_control, 'namespace': namespace}.items()
    )

    # Gazebo
    gazebo_world_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'description', 'gazebo', 'worlds', gazebo_world_file]
    )

    gazebo = ExecuteProcess( 
        condition=IfCondition(simulation),
        cmd=['gazebo', gazebo_verbosity,  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-w', gazebo_world_path], 
        output='screen'
    )

    # ros2_control
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name),'launch','ros2_control.launch.py')
        ]), 
        launch_arguments={'namespace': namespace, 'sim_mode': simulation}.items()
    )

    # Spawn robot
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.

    robot_description_node_path = ['/',namespace, '/robot_description']

    spawn_entity = Node(
        condition=IfCondition(simulation),
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', robot_description_node_path,'-entity', 'taylor_robot','-x','12','-y','10', '-z', '2'],
        output='screen',
    )


    # see https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
    #delayed_spawn_entity = RegisterEventHandler(
    #    event_handler=OnProcessStart(
    #        target_action=gazebo,
    #        on_start=[
    #            LogInfo(msg=['Gazebo started, spawning ', namespace,'!']),
    #            spawn_entity
    #        ]
    #    )
    #)

    delayed_spawn_entity = TimerAction(period=20.0, actions=[spawn_entity])
    delayed_ros2_control = TimerAction(period=20.0, actions=[ros2_control])
    
    # Launch them all!
    return LaunchDescription([
        LogInfo(
            msg=taylor_ascii
        ),
        namespace_arg,
        debbug_mode_arg,
        simulation_arg,
        gazebo_world_file_arg,
        use_ros2_control_arg,
        robot_description_node,
        gazebo,
        delayed_spawn_entity,
        delayed_ros2_control
        #rviz
    ])