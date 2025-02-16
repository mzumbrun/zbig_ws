import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_description = get_package_share_directory('zbig_description')
    pkg_localization = get_package_share_directory('zbig_localization')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_description)
    
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='bigbot.urdf.xacro',
        description='Name of the URDF description to load'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='2.5',
        description='x coordinate of spawned robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='1.5',
        description='y coordinate of spawned robot'
    )
    
    z_arg = DeclareLaunchArgument(
        'z', default_value='1.0',
        description='z coordinate of spawned robot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='yaw angle of spawned robot'
    )
    
    pitch_arg = DeclareLaunchArgument(
        'pitch', default_value='0.0',
        description='pitch angle of spawned robot'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # robot model path
    urdf_file_path = PathJoinSubstitution([
        pkg_description,  
        "urdf",
        LaunchConfiguration('model') 
    ])

    gz_bridge_params_path = os.path.join(
        pkg_description,
        'config',
        'gz_bridge.yaml'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Spawn the URDF model using the /world/<world_name>/create service
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
            "-P", LaunchConfiguration('pitch')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
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
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
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
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )

    return LaunchDescription([
            world_arg,
            model_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            pitch_arg,
            sim_time_arg,
            world_launch,
            spawn_urdf_node,
            gz_bridge_node,
            robot_state_publisher_node,
            ekf_node,
            
])