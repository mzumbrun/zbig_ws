import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_description = get_package_share_directory('zbig_description')
    pkg_mapping = get_package_share_directory('zbig_mapping')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_description)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Generate path to config file
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    nav2_navigation_launch_path = os.path.join(
        pkg_mapping,
        'launch',
        'navigation_launch.py'
    )

    navigation_params_path = os.path.join(
        pkg_mapping,
        'config',
        'navigation.yaml'
    )

    slam_toolbox_params_path = os.path.join(
        pkg_mapping,
        'config',
        'slam_toolbox.yaml'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_mapping, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    return LaunchDescription([

    rviz_launch_arg,
    rviz_config_arg,
    sim_time_arg,
    rviz_node,
   # interactive_marker_twist_server_node,
    slam_toolbox_launch,
    navigation_launch,

    ])