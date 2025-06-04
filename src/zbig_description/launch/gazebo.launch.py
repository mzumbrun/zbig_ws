import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    zbig_description = get_package_share_directory("zbig_description")

    # Declare the world file argument
    world_arg = DeclareLaunchArgument(
        'world', default_value='home.world',
        description='Name of the Gazebo world file to load'
    )
    
    x_arg = DeclareLaunchArgument('x', default_value='-3.',)
    y_arg = DeclareLaunchArgument('y', default_value='0',)
    z_arg = DeclareLaunchArgument('z', default_value='0.1',)
    yaw_arg = DeclareLaunchArgument('yaw', default_value='3.14',)

    # Get package directories
    pkg_description = get_package_share_directory('zbig_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Update GZ_SIM_RESOURCE_PATH with the path to worlds folder
    gazebo_models_path = os.path.join(pkg_description, 'worlds')
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                pkg_description,
                'worlds',
                LaunchConfiguration('world')
            ]),
            TextSubstitution(text=' -r -v -v1')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                zbig_description, "urdf", "cbot2.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    # world = LaunchConfiguration("world") 
    # world_arg = DeclareLaunchArgument(name="world", default_value="home.world",)

    # world_path = PathJoinSubstitution([
    #         zbig_description,
    #         "worlds",
    #         PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    #     ]
    # )

    # model_path = str(Path(zbig_description).parent.resolve())
    # model_path += pathsep + os.path.join(get_package_share_directory("zbig_description"), 'models')

    # gazebo_resource_path = SetEnvironmentVariable(
    #     "GZ_SIM_RESOURCE_PATH",
    #     model_path
    #     )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(zbig_description,
    #                 'launch',
    #                 'world.launch.py'),
    #             ),
    #                 launch_arguments={
    #                 'world': world,
    #             }.items(),
    #         ),

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bigbot",
                   "-x", LaunchConfiguration('x'), 
                   "-y", LaunchConfiguration('y'), 
                   "-z", LaunchConfiguration('z'), 
                   "-Y", LaunchConfiguration('yaw'), 
                   ],
        parameters=[
                    {'use_sim_time': True}
                ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        # gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
