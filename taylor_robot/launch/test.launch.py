# Author: Addison Sears-Collins
# Date: September 2, 2021
# Description: Launch a basic mobile robot using the ROS 2 Navigation Stack
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    #Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')  
    #pkg_share = FindPackageShare(package='taylor_robot').find('taylor_robot')
    pkg_share = os.path.join(get_package_share_directory('taylor_robot'))
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'urdf', 'taylor.robot.urdf.xacro')
    
    robot_description_config = xacro.process_file(default_model_path)

    #default_model_path = os.path.join(pkg_share, 'urdf/taylor.robot.urdf.xacro')

    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    #   robot_name_in_urdf = 'taylor_robot'
    default_rviz_config_path = os.path.join(pkg_share,'rviz2', 'taylor_rviz2_config.rviz')

    amcl_config_path = os.path.join(get_package_share_directory(
        'taylor_robot'), 'config', 'taylor_amcl_config_file.yaml')

    world_description_package='lab_usig_world_description'
    world_name='lab_28'
    world_path = PathJoinSubstitution(
                [FindPackageShare(world_description_package), "worlds", world_name]
            )

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    static_map_path = os.path.join(pkg_share, 'rviz2_map', 'lab_usig_map_rviz2.yaml')
    map_file = os.path.join(get_package_share_directory(
        'taylor_robot'), 'rviz2_map', 'lab_usig_map_rviz2.yaml')
    #nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    
    #Launch configuration variables specific to simulation

    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    headless = LaunchConfiguration('headless')
    map_yaml_file = LaunchConfiguration('map')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    #params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam = LaunchConfiguration('slam')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    #remappings = [('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
                    #('/tf_static', 'tf_static')]
    
    #Declare the launch arguments  

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')
            
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use')
            
    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')
            
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file')
        
    #declare_params_file_cmd = DeclareLaunchArgument(
        #name='params_file',
        #default_value=nav2_params_path,
        #description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')
        
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    # Specify tzhe actions

    # Start Gazebo server
    #start_gazebo_server_cmd = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        #condition=IfCondition(use_simulator),
        #launch_arguments={'world': world}.items())

    # Start Gazebo client    
    #start_gazebo_client_cmd = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        #condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    #Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])

    #Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time}],
        #remappings=remappings,
        arguments=[default_model_path])

    #Launch Gazebo
    start_gazebo_cmd = ExecuteProcess( 
            cmd=['gazebo', '--verbose',  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-w', world_path], 
            output='screen')

    #Spawn entity
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'taylor_robot',
                        '-x','12',
                        '-y','10'],
                        output='screen'
                        )

    #Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])   

    #Launch map server
    start_map_server =         Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]
        )

    #Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            #'remmapings': remappings,
                            #'params_file': params_file,
                            'default_bt_xml_filename': default_bt_xml_filename,
                            'autostart': autostart}.items()
                            )

    #Launch Lifecycle manager nav2 node
    start_lifecycle_manager_nav2 = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    #Launch AMCL node
    
    start_amcl_nav2 = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_path]
        )
    
    #Create the launch description and populate
    ld = LaunchDescription()

    #Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_model_path_cmd)
    #ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    #ld.add_action(start_gazebo_server_cmd)
    #ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_localization_cmd)
    #ld.add_action(start_robot_state_publisher_cmd)
    #ld.add_action(start_gazebo_cmd)
    #ld.add_action(spawn_entity)
    ld.add_action(start_rviz_cmd)
    #ld.add_action(start_map_server)
    #ld.add_action(start_lifecycle_manager_nav2)
    ld.add_action(start_ros2_navigation_cmd)
    #ld.add_action(start_amcl_nav2)

    return ld