import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    config_prefix = get_package_share_directory("zbig_mapping")

    name = LaunchConfiguration("name").perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(config_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": name,
                "params_file": params_file,
                "parent_frame": LaunchConfiguration("parent_frame"),
                "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                "cam_roll": LaunchConfiguration("cam_roll"),
                "cam_pitch": LaunchConfiguration("cam_pitch"),
                "cam_yaw": LaunchConfiguration("cam_yaw"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "imu_from_descr": LaunchConfiguration("imu_from_descr"),
            }.items(),
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyziNode",
                    name="point_cloud_xyzi",
                    remappings=[
                        ("depth/image_rect", name + "/stereo/image_raw"),
                        ("intensity/image_rect", name + "/right/image_rect"),
                        ("intensity/camera_info", name + "/stereo/camera_info"),
                        ("points", name + "/points"),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():

    config_prefix = get_package_share_directory("zbig_mapping")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="depth_camera"),
        DeclareLaunchArgument("parent_frame", default_value="base_link"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.190338"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.108"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("imu_from_descr", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(config_prefix, "config", "pcl.yaml"),
        ),
        DeclareLaunchArgument("use_rviz", default_value="False"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
