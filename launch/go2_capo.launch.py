from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """同时启动 Go2 LowState 适配节点和 CAPO 融合估计节点。"""
    return LaunchDescription([
        Node(
            package="fusion_estimator",
            executable="go2_lowstate_adapter_node",
            name="go2_lowstate_adapter_node",
            output="screen",
        ),
        Node(
            package="fusion_estimator",
            executable="fusion_estimator_node",
            name="fusion_estimator_node",
            output="screen",
        ),
    ])
