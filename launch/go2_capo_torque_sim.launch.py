"""Direct MuJoCo IMU/encoder/motor torque input; no Unitree foot sensors."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("fusion_estimator"), "config", "sim_torque.yaml"
    )
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="fusion_estimator",
            executable="fusion_estimator_node",
            name="fusion_estimator_node",
            output="screen",
            additional_env={"CAPO_PARAMS_FILE": config},
            parameters=[config],
        ),
    ])
