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
    
    world = LaunchConfiguration("world") 
    world_arg = DeclareLaunchArgument(
        name="world", 
        default_value="empty.world"
    )
    
    model = LaunchConfiguration("model")
    model_arg = DeclareLaunchArgument(
        'model', 
        default_value='bigbot.urdf.xacro',
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
    )
    
    z_arg = DeclareLaunchArgument(
        'z', default_value='0.1',
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
    )
    # robot model path
    urdf_file_path = PathJoinSubstitution([
        pkg_description,  
        "urdf",
        model
    ])

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

    # Spawn the URDF model using the /world/<world>/create service
    spawn_urdf_node = Node(
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
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': True}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_localization, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
             ]
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "controller_bigbot.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_sim_time": "True",
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items()
    )

    safety_stop = Node(
        package="zbig_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[{"use_sim_time": True}]
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
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )

    nav_given_map = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "navigation_given_map.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
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
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(navslam)
    )

    
    return LaunchDescription([
        use_slam_arg,
        navslam_arg,
        use_map_arg,
        map_name_arg,
        model_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        world_launch,
        spawn_urdf_node,
        gz_bridge_node,
        robot_state_publisher_node,
        ekf_node,
        world_arg,
        controller,
        joystick,
        safety_stop,
        slam,
        rviz_slam,
        nav_given_map,
        nav_with_slam,

    ])