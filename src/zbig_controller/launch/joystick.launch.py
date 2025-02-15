from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    zbig_controller_pkg = get_package_share_directory('zbig_controller')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('zbig_controller'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper_node',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_in','/cmd_vel'),
                        ('/cmd_vel_out','/zbig_controller/cmd_vel')]
         )

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "zbig_controller/cmd_vel_unstamped", # this is class
          #  "cmd_vel_out": "zbig_controller/cmd_vel",
            "config_locks": os.path.join(zbig_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(zbig_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(zbig_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    
    twist_relay_node = Node(
        package="zbig_controller",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        twist_stamper,
        twist_mux_launch,
        twist_relay_node       
    ])