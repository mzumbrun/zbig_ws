import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_description = get_package_share_directory('zbig_description')

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True",)

    use_big = LaunchConfiguration("use_big")
    use_big_arg = DeclareLaunchArgument("use_big", default_value="True",)

    cbot_wheel_dims = os.path.join(pkg_description, 'config', 'dims_cbot.yaml')
    smallbot_wheel_dims = os.path.join(pkg_description, 'config', 'dims_smallbot.yaml')
  
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zbig_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )

    wheel_calculator_big = Node(
            package="zbig_controller",
            executable="wheel_calculator.py",
            parameters=[ {
                "use_sim_time": use_sim_time},
                cbot_wheel_dims,],
            condition=IfCondition(use_big),
        )
    
    wheel_calculator_small = Node(
            package="zbig_controller",
            executable="wheel_calculator.py",
            parameters=[ {
                "use_sim_time": use_sim_time},
                smallbot_wheel_dims,],
            condition=UnlessCondition(use_big),
        )


    return LaunchDescription(
        [
            use_sim_time_arg,
            use_big_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            wheel_calculator_big,
            wheel_calculator_small,
        ]
    )