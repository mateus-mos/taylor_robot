import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    namespace = LaunchConfiguration('namespace')
    
    
    # Process the description file
    pkg_path = os.path.join(get_package_share_directory('taylor_robot'))
    xacro_file = os.path.join(pkg_path, 'description', 'taylor.robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=',
                                        use_ros2_control, ' sim_mode:=', use_sim_time])

    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_robot_state_publisher_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            node_robot_state_publisher,
        ]
    )

    # Launch!
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='taylor',
            description='Namespace'),

        node_robot_state_publisher_with_namespace
    ])