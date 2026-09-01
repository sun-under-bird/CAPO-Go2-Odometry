from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """MuJoCo 仿真验证专用启动文件（阶段十三/十六/十七新增）。

    在原 go2_capo.launch.py（adapter + fusion estimator，真机/仿真通用）的基础上，
    额外启动仿真专用组件：
      - ground_truth_node:   将 MuJoCo /sportmodestate 转成 /ground_truth/odom
      - capo_evaluator_node: 实时对比 /SMX/Odom_2D 与 Ground Truth 并输出误差指标
    真机链路请继续使用 go2_capo.launch.py，不受本文件影响。
    """
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
        Node(
            package="fusion_estimator",
            executable="ground_truth_node",
            name="ground_truth_node",
            output="screen",
        ),
        Node(
            package="fusion_estimator",
            executable="capo_evaluator_node",
            name="capo_evaluator_node",
            output="screen",
            parameters=[{
                # 仿真隔离：与 MuJoCo 使用相同的 domain id（见 jihua.md 第七阶段）
                "capo_topic": "SMX/Odom_2D",
                "gt_topic": "ground_truth/odom",
                "report_period_s": 5.0,
                "csv_path": "/tmp/capo_eval.csv",
            }],
        ),
    ])
