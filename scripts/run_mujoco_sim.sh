#!/bin/bash
# run_mujoco_sim.sh —— 启动 Unitree MuJoCo Go2 仿真（jihua.md 阶段六/七）
#
# 用法：
#   ./run_mujoco_sim.sh [场景xml]     # 默认 scene.xml，可传 scene_terrain.xml
#
# 关键点：
#   1. LD_LIBRARY_PATH 必须把 /opt/unitree_robotics/lib 放在最前，
#      否则会加载到 ROS Humble 的 libddsc(0.10.5) 与 SDK2 自带的 0.10.2 混用，
#      触发 dds_writecdr_impl_common 断言崩溃（已在本机实测复现）。
#   2. DDS 参数（domain_id=1、interface=lo）来自 simulate/config.yaml，无需额外设置。
set -e

SCENE="${1:-scene.xml}"
SIM_DIR="$HOME/unitree_mujoco/simulate/build"

export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}"

echo "启动 MuJoCo 仿真：robot=go2 scene=${SCENE}"
exec "$SIM_DIR/unitree_mujoco" -r go2 -s "$SCENE"
