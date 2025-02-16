import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    # Declare the world file argument
    world_arg = DeclareLaunchArgument(
        'world', default_value='home.world',
        description='Name of the Gazebo world file to load'
    )

    # Get package directories
    pkg_description = get_package_share_directory('zbig_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Update GZ_SIM_RESOURCE_PATH with the path to worlds folder
    gazebo_models_path = os.path.join(pkg_description, 'worlds')
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
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

    
    return LaunchDescription([
        world_arg,
        gazebo_launch,

    ])