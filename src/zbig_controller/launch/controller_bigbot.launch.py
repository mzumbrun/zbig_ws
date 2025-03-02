from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True",)
    
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.074",)
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.445",)
    wheel_radius_error_arg = DeclareLaunchArgument("wheel_radius_error", default_value="0.000",)
    wheel_separation_error_arg = DeclareLaunchArgument("wheel_separation_error", default_value="0.000",)
    
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

    def noisy_controller(context, *args, **kwargs):
        use_sim_time = LaunchConfiguration("use_sim_time")
        wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
        wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
        wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
        wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

        wheel_calculator = Node(
            package="zbig_controller",
            executable="wheel_calculator.py",
            parameters=[
                {"wheel_radius": wheel_radius + wheel_radius_error,
                "wheel_separation": wheel_separation + wheel_separation_error,
                "use_sim_time": use_sim_time}],
        )

        return [
            wheel_calculator,
        ]

    wheel_calculator_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            wheel_calculator_launch,
        ]
    )