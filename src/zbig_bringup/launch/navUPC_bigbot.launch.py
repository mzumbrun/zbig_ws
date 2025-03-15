import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_description = get_package_share_directory('zbig_description')
    pkg_localization = get_package_share_directory('zbig_localization')
    pkg_mapping = get_package_share_directory('zbig_mapping')

    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument("use_slam",
        default_value="false"
    )

    navslam = LaunchConfiguration("navslam")
    navslam_arg = DeclareLaunchArgument("navslam",
        default_value="false"
    )
    
    use_map = LaunchConfiguration("use_map")
    use_map_arg = DeclareLaunchArgument("use_map",
        default_value="false"
    )

    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="floor2"
    )

    use_safety_stop = LaunchConfiguration("use_safety_stop")
    use_safety_stop_arg = DeclareLaunchArgument("use_safety_stop",
        default_value="false"
    )

    gz_bridge_params_path = os.path.join(
        pkg_description,
        'config',
        'gz_bridge.yaml'
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': False}
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
        }.items()
    )

    safety_stop = Node(
        package="zbig_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[
            {"use_sim_time": False,
             "warning_distance": 0.6,
             "danger_distance": 0.2,
             }],
        condition=IfCondition(use_safety_stop)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "slam.launch.py"
        ),
            launch_arguments={
            "use_sim_time": "False",
        }.items(),
        condition=IfCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                pkg_mapping,
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_slam)
    )

    nav_given_map = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "navigation_given_map.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
            "map_name": map_name,
        }.items(),
        condition=IfCondition(use_map)
    )

    nav_with_slam = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "navigation_with_slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
        }.items(),
        condition=IfCondition(navslam)
    )

    
    return LaunchDescription([
        use_slam_arg,
        navslam_arg,
        use_map_arg,
        map_name_arg,
        use_safety_stop_arg,
        gz_bridge_node,
        joystick,
        safety_stop,
        slam,
        rviz_slam,
        nav_given_map,
        nav_with_slam,

    ])