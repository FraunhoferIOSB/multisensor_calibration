#***********************************************************************
#
# Copyright (c) Fraunhofer Institute of Optronics,
# System Technologies and Image Exploitation IOSB
#
#***********************************************************************
#***********************************************************************
#
# Headless camera-LiDAR calibration: calibration and guidance nodes composed
# in one container, without the Qt GUI. Control it via services or a remote UI.
#
#***********************************************************************

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"

APP_NAME = "extrinsic_camera_lidar_calibration"


def generate_launch_description():
    this_pkg_dir = get_package_share_directory("multisensor_calibration")

    launch_args = [
        DeclareLaunchArgument("namespace", default_value="calibration"),
        DeclareLaunchArgument(
            "target_config_file",
            default_value=PathJoinSubstitution([this_pkg_dir, "cfg/TargetWithCirclesAndAruco.yaml"]),
        ),
        DeclareLaunchArgument(
            "robot_ws_path",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("HOME"), "multisensor_calibration/example"]
            ),
        ),
        DeclareLaunchArgument("camera_sensor_name", default_value="camera"),
        DeclareLaunchArgument("camera_image_topic", default_value="/camera/image_color"),
        DeclareLaunchArgument("camera_info_topic", default_value=""),
        DeclareLaunchArgument("image_state", default_value="DISTORTED"),
        DeclareLaunchArgument("is_stereo_camera", default_value="False"),
        DeclareLaunchArgument("right_camera_sensor_name", default_value=""),
        DeclareLaunchArgument("right_camera_info_topic", default_value=""),
        DeclareLaunchArgument("lidar_sensor_name", default_value="lidar"),
        DeclareLaunchArgument("lidar_cloud_topic", default_value="/lidar/cloud"),
        DeclareLaunchArgument("base_frame_id", default_value=""),
        DeclareLaunchArgument("use_initial_guess", default_value="True"),
        DeclareLaunchArgument("save_observations", default_value="True"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
    ]

    common_params = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    calibration_params = {
        "target_config_file": LaunchConfiguration("target_config_file"),
        "robot_ws_path": LaunchConfiguration("robot_ws_path"),
        "camera_sensor_name": LaunchConfiguration("camera_sensor_name"),
        "camera_image_topic": LaunchConfiguration("camera_image_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "image_state": LaunchConfiguration("image_state"),
        "is_stereo_camera": LaunchConfiguration("is_stereo_camera"),
        "right_camera_sensor_name": LaunchConfiguration("right_camera_sensor_name"),
        "right_camera_info_topic": LaunchConfiguration("right_camera_info_topic"),
        "lidar_sensor_name": LaunchConfiguration("lidar_sensor_name"),
        "lidar_cloud_topic": LaunchConfiguration("lidar_cloud_topic"),
        "base_frame_id": LaunchConfiguration("base_frame_id"),
        "use_initial_guess": LaunchConfiguration("use_initial_guess"),
        "save_observations": LaunchConfiguration("save_observations"),
    }

    container = ComposableNodeContainer(
        name=APP_NAME + "_container",
        namespace=LaunchConfiguration("namespace"),
        package="rclcpp_components",
        executable="component_container_isolated",
        arguments=["--use_multi_threaded_executor"],
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package="multisensor_calibration",
                plugin="multisensor_calibration::ExtrinsicCameraLidarCalibration",
                name=APP_NAME,
                namespace=LaunchConfiguration("namespace"),
                parameters=[common_params, calibration_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="multisensor_calibration",
                plugin="multisensor_calibration::GuidedCameraLidarTargetPlacementNode",
                name=APP_NAME + "_guidance",
                namespace=LaunchConfiguration("namespace"),
                parameters=[common_params, {"calibrator_node_name": APP_NAME}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription(launch_args + [container])
