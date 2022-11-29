import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, GroupAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace') 

    package_name='taylor_robot' 

    robot_description = Command(['ros2 param get --hide-type /', namespace,'/robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    nodes_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner
        ]
    )


    # Launch them all!
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='taylor',
            description='Namespace'
        ),
        nodes_with_namespace
    ])