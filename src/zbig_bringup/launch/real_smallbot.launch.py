import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_controller = get_package_share_directory('zsmall_controller')
    pkg_firmware = get_package_share_directory('zsmall_firmware')
    pkg_localization = get_package_share_directory('zsmall_localization')   

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            pkg_firmware,
            "launch",
            "hardware_interface_smallbot.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
            "use_ros2_control": "True"
        }.items(),
    )

   
    controller = IncludeLaunchDescription(
        os.path.join(
            pkg_controller,
            "launch",
            "controller_smallbot.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_sim_time": "False",
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
        package="zsmall_firmware",
        executable="mpu6050_driver.py"
    )
    
        
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_localization, 'config', 'ekf.yaml'),
            {'use_sim_time': False}
             ]
    )

    safety_stop = Node(
        package="zsmall_utils",
        executable="safety_stop.py",
        output="screen",
    )
    
    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_composition",
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'inverted': False,
                'scan_mode': 'Standard'
            }],
            output="screen"
    ) 
   
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("zsmall_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("zsmall_remote"),
                "launch",
                "remote_interface.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    return LaunchDescription([
        hardware_interface,
        controller,
     #   joystick,
        imu_driver_node,
        laser_driver,
      #  ekf_node,
      #  safety_stop,
     #   moveit,
     #   remote_interface,
    ])