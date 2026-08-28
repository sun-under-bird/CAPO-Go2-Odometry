import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """联合启动当前 Go2 driver、LowState 适配节点和 CAPO 里程计。"""
    driver_launch_path = "/root/stereo/src/go2_driver/launch/driver.launch.py"
    capo_launch_path = os.path.join(
        get_package_share_directory("fusion_estimator"),
        "launch",
        "go2_capo.launch.py",
    )

    driver_publish_odom_argument = DeclareLaunchArgument(
        "driver_publish_odom",
        default_value="true",
        description="是否启动会发布 base_footprint 到 base_link TF 的 Go2 driver",
    )

    return LaunchDescription([
        driver_publish_odom_argument,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch_path),
            launch_arguments={
                "publish_odom": LaunchConfiguration("driver_publish_odom"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(capo_launch_path),
        ),
    ])
