import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_mapping = get_package_share_directory('zbig_mapping')
    pkg_localization = get_package_share_directory('zbig_localization')

    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="floor2"
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
     )

     # Generate path to config file
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    # Path to the Slam Toolbox launch file
    nav2_localization_launch_path = os.path.join(
        pkg_localization,
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        pkg_mapping,
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        pkg_localization,
        'config',
        'amcl.yaml'
    )

    navigation_params_path = os.path.join(
        pkg_mapping,
        'config',
        'navigation.yaml'
    )

    map_file_path = PathJoinSubstitution([
        pkg_mapping,
        "maps",
        map_name,
        "map.yaml"
    ])

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_mapping, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    interactive_marker_twist_server_node = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[interactive_marker_config_file_path],
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': "True",
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': "True",
                'params_file': navigation_params_path,
        }.items()
    )

    return LaunchDescription([
            rviz_launch_arg,
            map_name_arg,
            rviz_config_arg,
            rviz_node,
            #interactive_marker_twist_server_node,
            localization_launch,
            navigation_launch,

 ])
