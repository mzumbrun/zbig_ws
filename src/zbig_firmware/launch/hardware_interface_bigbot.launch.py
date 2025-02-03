import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
      
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_ros2_control_arg = DeclareLaunchArgument("use_ros2_control", default_value='true')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('zbig_description'))
    xacro_file = os.path.join(pkg_path,'urdf','bigbot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'is_sim' : 'false', 'use_ros2_control' : 'true'})
    robot_description = doc.toprettyxml(indent='  ')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("zbig_controller"),
                "config",
                "bigbot_controllers.yaml",
            ),
        ],
    )


    return LaunchDescription(
        [
            use_sim_time_arg,
            use_ros2_control_arg,
            robot_state_publisher_node,
           # delay_controller_manager,
           #model_arg,
            controller_manager,
 
        ]
    )