# CAPO-Go2-Odometry

CAPO 足式里程计 ROS 2 Humble 适配版本，包含 Unitree Go2 流程及 Marsdog V1.7 仿真适配。

本仓库基于原项目
[ShineMinxing/CAPO-LeggedRobotOdometry](https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry)
修改，保留运行所需的纯 C++ 融合核心，新增 `/lowstate` 适配节点、Go2 启动文件和
`odom -> base_footprint` TF 广播。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Unitree `unitree_go/msg/LowState`
- 当前路径：`/root/CAPO-LeggedRobotOdometry`
- 当前编译工作空间：`/root/capo_ws`

## 接口

输入：

- `/lowstate`：`unitree_go/msg/LowState`

输出：

- `SMX/Odom`：3D `nav_msgs/msg/Odometry`
- `SMX/Odom_2D`：2D `nav_msgs/msg/Odometry`
- `/tf`：`odom -> base_footprint`

Go2 driver 继续广播 `base_footprint -> base_link`。

## 无足端传感器的 MuJoCo 仿真

`sim-torque-reproduction` 分支提供直接使用 IMU、关节位置/速度和实际电机力矩的
MuJoCo 仿真桥，不需要 `unitree_go` 或 Go2 足端传感器。CAPO 数学核心沿用上游。

```bash
scripts/setup_torque_sim.sh
scripts/run_torque_sim.sh --scenario straight --duration 30 --viewer --realtime
```

完整配置、上游实现依据和本机实测结果见 [关节力矩仿真复现说明](TORQUE_SIM_REPRODUCTION.md)。

## Marsdog V1.7 仿真适配

支持 12 个腿轴和 2 个活动腰轴，使用关节实际执行器力矩和 IMU，在独立构建工作区运行：

```bash
scripts/setup_marsdog_sim.sh
scripts/run_marsdog_sim.sh --scenario straight --duration 30 --viewer --realtime
```

本机运行条件、话题、仿真假设与 30 秒结果见 [Marsdog 适配说明](MARSDOG_SIM_ADAPTATION.md)。

当前默认使用新实测 URDF 与原 SDK 策略，已修正机身角速度、球形脚滚动和预测时间间隔，并对支撑脚观测加权。新模型的十米方形位置 RMSE 从 0.538 m 降到 0.150 m，见 [优化结果与复现](MARSDOG_OPTIMIZATION.md)。

## 编译

```bash
source /opt/ros/humble/setup.bash
source /root/unitree_ros2/cyclonedds_ws/install/setup.bash

colcon build \
  --base-paths /root/CAPO-LeggedRobotOdometry \
  --build-base /root/capo_ws/build \
  --install-base /root/capo_ws/install \
  --packages-select fusion_estimator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 启动

只启动 CAPO 和 `/lowstate` 适配器：

```bash
source /opt/ros/humble/setup.bash
source /root/unitree_ros2/cyclonedds_ws/install/setup.bash
source /root/capo_ws/install/setup.bash
ros2 launch fusion_estimator go2_capo.launch.py
```

与当前 Go2 driver 一起启动：

```bash
source /opt/ros/humble/setup.bash
source /root/unitree_ros2/cyclonedds_ws/install/setup.bash
source /root/stereo/install/setup.bash
source /root/capo_ws/install/setup.bash
ros2 launch fusion_estimator go2_capo_with_driver.launch.py
```

## 验证

```bash
ros2 topic hz /SMX/Odom_2D
ros2 run tf2_ros tf2_echo odom base_footprint
```

算法原理、论文和原始平台实现请参考上游项目。Marsdog 真机输入与精度尚未验证。
