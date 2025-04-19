import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_bringup = get_package_share_directory('zbig_bringup')
    pkg_controller = get_package_share_directory('zbig_controller')
    pkg_firmware = get_package_share_directory('zbig_firmware')
    pkg_localization = get_package_share_directory('zbig_localization')
    pkg_mapping = get_package_share_directory('zbig_mapping')
    pkg_mpu_6050 = get_package_share_directory('ros2_mpu6050')

    param_file = LaunchConfiguration('param_file')
    params_arg = DeclareLaunchArgument('param_file',
                                        default_value=os.path.join(pkg_mpu_6050, 'config', 'params.yaml'),
                                        description='Path to the ROS2 parameter file')
    
    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false" )
    
    use_ekf = LaunchConfiguration("use_ekf")
    use_ekf_arg = DeclareLaunchArgument("use_ekf", default_value="false" )
    
    use_ros_mpu = LaunchConfiguration("use_ros_mpu")
    use_ros_mpu_arg = DeclareLaunchArgument("use_ros_mpu", default_value="false" )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            pkg_firmware,
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
                pkg_bringup,
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            pkg_controller,
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            pkg_controller,
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
        
    imu_driver_node = Node(
        package="zbig_firmware",
        executable="mpu6050_driver.py",
        condition=UnlessCondition(use_ros_mpu)
    )
    
    mpu6050_sensor = Node(
        package='ros2_mpu6050',
        executable='ros2_mpu6050',
        name='mpu6050_sensor',
        output="screen",
        emulate_tty=True,
        parameters=[param_file],
        condition=IfCondition(use_ros_mpu)
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_localization, 'config', 'ekf.yaml'),
            {'use_sim_time': False}
             ],
        condition=IfCondition(use_ekf)
    )

    safety_stop = Node(
        package="zbig_utils",
        executable="safety_stop.py",
        output="screen",
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            pkg_localization,
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        params_arg,
        use_slam_arg,
        use_ekf_arg,
        use_ros_mpu_arg,
        hardware_interface,
     #   laser_driver,
        controller,
     #   joystick,
        imu_driver_node,
        mpu6050_sensor,
        ekf_node,
     #   safety_stop,
     #   localization,
     #   slam
    ])