#!/bin/bash
# run_capo_sim.sh —— 启动 CAPO + Ground Truth + 评估（jihua.md 阶段十三/十六/十七）
#
# 前置条件：
#   1. MuJoCo 仿真已由 scripts/run_mujoco_sim.sh 在另一终端启动
#   2. unitree_ros2 (cyclonedds_ws) 与 capo_ws 均已编译
#
# 环境隔离（jihua.md 第七阶段）：
#   ROS_DOMAIN_ID=1 与仿真一致；CycloneDDS 强制走 lo 网卡，避免碰真机网口。
set -e

source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/capo_ws/install/setup.bash"

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "启动 CAPO 仿真验证链路（adapter + fusion + ground truth + evaluator）"
exec ros2 launch fusion_estimator go2_capo_sim.launch.py
