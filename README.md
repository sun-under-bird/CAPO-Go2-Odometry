# CAPO-Go2-Odometry

面向当前 Unitree Go2 的 CAPO 足式里程计 ROS 2 Humble 适配版本。

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

算法原理、论文和原始平台实现请参考上游项目。本仓库只维护当前 Go2 运行适配。
