import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_description = get_package_share_directory('zbig_description')
    pkg_localization = get_package_share_directory('zbig_localization')
    pkg_mapping = get_package_share_directory('zbig_mapping')
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    use_depth_cam = LaunchConfiguration("use_depth_cam")
    use_depth_cam_arg = DeclareLaunchArgument("use_depth_cam",default_value="false")

    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")

    use_navslam = LaunchConfiguration("use_navslam")
    use_navslam_arg = DeclareLaunchArgument("use_navslam", default_value="false")
    
    use_navmap = LaunchConfiguration("use_navmap")
    use_navmap_arg = DeclareLaunchArgument("use_navmap", default_value="false")
    
    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument("map_name", default_value="floor2" )
    
    world = LaunchConfiguration("world") 
    world_arg = DeclareLaunchArgument(name="world", default_value="empty.world")
    
    model = LaunchConfiguration("model")
    model_arg = DeclareLaunchArgument('model', default_value='bigbot.urdf.xacro',)

    use_big = LaunchConfiguration("use_big")
    use_big_arg = DeclareLaunchArgument('use_big', default_value='true',)

    x_arg = DeclareLaunchArgument('x', default_value='-1.',)
    y_arg = DeclareLaunchArgument('y', default_value='0',)
    z_arg = DeclareLaunchArgument('z', default_value='0.1',)
    yaw_arg = DeclareLaunchArgument('yaw', default_value='3.14',)
    
    use_safety_stop = LaunchConfiguration("use_safety_stop")
    use_safety_stop_arg = DeclareLaunchArgument("use_safety_stop", default_value="false" )
    
    use_ekf = LaunchConfiguration("use_ekf")
    use_ekf_arg = DeclareLaunchArgument("use_ekf", default_value="false" )

    sim_only_launch = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[      
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'robot_description': Command(['xacro', ' ', PathJoinSubstitution([pkg_description, "urdf", model])]),
                    'use_sim_time': True}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(pkg_localization, 'config', 'ekf.yaml'),
                    {'use_sim_time': True}
                ],
                condition=IfCondition(use_ekf),
            ),
            
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "bigbot",
                    "-topic", "robot_description",
                    "-x", LaunchConfiguration('x'), 
                    "-y", LaunchConfiguration('y'), 
                    "-z", LaunchConfiguration('z'), 
                    "-Y", LaunchConfiguration('yaw'), 
                ],
                output="screen",
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "controller_bigbot.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_big": use_big,
        }.items(),
        condition=IfCondition(use_sim_time)
    )
    
    gz_bridge_params_path = os.path.join(
        pkg_description,
        'config',
        'gz_bridge.yaml'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description,
            'launch',
            'world.launch.py'),
        ),
            launch_arguments={
            'world': world,
        }.items(),
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
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )

    joystick = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("zbig_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items()
    )

    safety_stop = Node(
        package="zbig_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[
            {"use_sim_time": True,
             "warning_distance": 0.56,
             "danger_distance": 0.28,
             }],
        condition=IfCondition(use_safety_stop)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "slam.launch.py"
        ),
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
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_slam)
    )

    nav_given_map = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "navigation_given_map.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_name": map_name,
            "use_depth_cam": use_depth_cam,
        }.items(),
        condition=IfCondition(use_navmap)
    )

    nav_with_slam = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "navigation_with_slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_navslam)
    )

    
    return LaunchDescription([
        use_sim_time_arg,
        use_depth_cam_arg,
        use_slam_arg,
        use_navslam_arg,
        use_navmap_arg,
        map_name_arg,
        model_arg,
        use_big_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_safety_stop_arg,
        use_ekf_arg,
        sim_only_launch,
        world_launch,
        controller,
        gz_bridge_node,
        world_arg,
        joystick,
        safety_stop,
        slam,
        rviz_slam,
        nav_given_map,
        nav_with_slam,

    ])