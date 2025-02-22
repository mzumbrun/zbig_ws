import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_firmware"),
            "launch",
            "hardware_interface_bigbot.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
            "use_ros2_control": "True"
        }.items(),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_composition",
            parameters=[os.path.join(
                get_package_share_directory("zbig_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "controller_bigbot.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_sim_time": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
        
    imu_driver_node = Node(
        package="zbig_firmware",
        executable="mpu6050_driver.py"
    )

    safety_stop = Node(
        package="zbig_utils",
        executable="safety_stop.py",
        output="screen",
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zbig_localization"),
            "launch",
            "global_localization.launch.py"
        ),
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
    
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
     #   laser_driver,
        controller,
     #   joystick,
        imu_driver_node,
     #   safety_stop,
     #   localization,
     #   slam
    ])