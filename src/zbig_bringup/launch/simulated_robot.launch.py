import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = get_package_share_directory("zbig_description")
    
    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument("use_slam",default_value="false"    )
    
    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument("map_name", default_value="provided" )
    
    world = LaunchConfiguration("world") 
    world_arg = DeclareLaunchArgument(name="world", default_value="home.world")
    
    x_arg = DeclareLaunchArgument('x', default_value='-2.8',)
    y_arg = DeclareLaunchArgument('y', default_value='1.5',)
    z_arg = DeclareLaunchArgument('z', default_value='0.1',)
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.57',)

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
        "world": world,
        "x": LaunchConfiguration('x'), 
        "y": LaunchConfiguration('y'), 
        "z": LaunchConfiguration('z'), 
        "yaw": LaunchConfiguration('yaw'), 
        }.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True",
        }.items(),
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
            {'use_sim_time': True}
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
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo,
        controller,
        gz_bridge_node,
        joystick,
        localization,
        slam,
        navigation,
        rviz,
    ])