import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, GroupAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'taylor_robot' 

    # Arguments 
    namespace_arg = DeclareLaunchArgument('namespace')
    sim_mode_arg = DeclareLaunchArgument('sim_mode', default_value='false')

    # Arg vars
    namespace = LaunchConfiguration('namespace') 
    sim_mode = LaunchConfiguration('sim_mode')

    robot_description = Command(['ros2 param get --hide-type /', namespace,'/robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        OnExecutionComplete(
            target_action=controller_manager,
            on_completion=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        OnExecutionComplete(
            target_action=controller_manager,
            on_completion=[joint_broad_spawner],
        )
    )

    nodes_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
        ]
    )


    # Launch them all!
    return LaunchDescription([
        namespace_arg,
        sim_mode_arg,
        nodes_with_namespace
    ])