# Go2 CAPO 里程计仿真复现：IMU、编码器和关节力矩

复现日期：2026-11-07（Asia/Shanghai）。本机已经编译并运行成功，包含无界面评测和 MuJoCo 可视化窗口。

## 1. 本次采用的版本和数据链路

- 仓库：`sun-under-bird/CAPO-Go2-Odometry`，基于 `main` 的 `ca12af3`，本地工作分支 `sim-torque-reproduction`。
- CAPO 数学核心：`FusionEstimator/` 与上游 `ShineMinxing/CAPO-LeggedRobotOdometry` 的 `f41893227d62229d44fdbde8db66d68bcf8c7855` 一致，未修改数学核心。
- 官方 Go2 模型：`unitreerobotics/unitree_mujoco` 的 `4134cb5dc7ff1ba7f484deda48b5274b58694519`，下载到 `.runtime/go2/`，按 Git blob SHA-1 校验下载文件。
- Python 依赖：MuJoCo 3.3.6、ONNX Runtime 1.23.2、NumPy 1.26.4。
- 行走策略：仓库自带 `sim_patch/models/go2_policy.onnx`。观测、动作滤波、默认关节角和 PD 参数与 `sim_patch/example_cpp/policy_runner.cpp` 对应，策略运行在 50 Hz。

```text
MuJoCo Go2（物理步长 0.005 s）
  ├─ imu_quat / imu_gyro / imu_acc ──→ /SMX/Go2IMU
  ├─ jointpos / jointvel / jointactuatorfrc ──→ /SMX/Go2Joint
  ├─ 仿真时间 ──→ /clock
  └─ 基座位姿/速度真值 ──→ /sim/ground_truth ──→ 评估和 CSV

/SMX/Go2IMU + /SMX/Go2Joint ──→ CAPO 原始 C++ 核心
  ├─ /SMX/Odom（3D）
  ├─ /SMX/Odom_2D（2D）
  ├─ /SMX/ContactEstimate（等效力、支撑判断和接触概率）
  └─ /tf（odom → base_footprint）
```

CAPO 和策略不订阅真值话题。策略只使用 IMU、关节位置/速度和自身历史动作。
估计器不读取 `foot_force`、`foot_force_est`、MuJoCo 接触标志或接触约束力。
物理引擎中的足地碰撞正常参与动力学计算。

本方案实现了原 `/home/sunbird/odom.md` 所描述的“仿真桥直接接入 CAPO”链路，输入的接触信息改为实际关节力矩。
原文档中的 `unilab:capo-sim` 镜像和外部仿真工程并未随这个仓库提供，因此本次使用本机 ROS 2 Humble 和独立 Python 环境。

## 2. 关节力矩为什么可以替代足端传感器

上游已经提供了基于力矩的接触判断，相关实现：

- [上游 README](https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/blob/f41893227d62229d44fdbde8db66d68bcf8c7855/README.md)：核心输入是 IMU 和关节电机数据。
- [上游 Sensor_Legs.cpp](https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/blob/f41893227d62229d44fdbde8db66d68bcf8c7855/FusionEstimator/Sensor_Legs.cpp)：`Joint2HipFoot()` 构建腿部雅可比并估算足端等效力。
- [上游配置](https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry/blob/f41893227d62229d44fdbde8db66d68bcf8c7855/config.yaml)：`contact_sensor_threshold` 可设为 `0.0`，关闭足端传感器数值转换。

对三自由度腿，代码计算的等效力形式是：

```text
f_eq = (J Jᵀ)⁻¹ J τ
```

这里 `J` 为腿部位置雅可比，`τ` 为 hip/thigh/calf 的实际电机力矩，单位 N·m。
CAPO 当前符号约定下，支撑腿的等效竖直力通常为负，满足 `f_eq.z < foot_force_threshold` 时判为支撑。
随后用支撑足的运动学位置和速度约束里程计。

配置文件为 `config/sim_torque.yaml`：

```yaml
contact_sensor_threshold: 0.0
foot_force_threshold: -20.0
```

不要把控制器的目标位置、未执行的期望力矩或 `100/0` 接触标志作为本模式的电机力矩。
本桥读取 MuJoCo 的 `jointactuatorfrc`，其中包含实际执行的 PD 力矩及执行器限幅效果。
在 Go2 真机上，对应需要读取实际反馈的 `motor_state[].tau_est`，当前原有 LowState 适配器仍是足端传感器模式，不能直接混用。

阈值依赖机器人、负载和运动。在本模型的静止站立阶段，前腿等效 Fz 约 −36 N，后腿约 −28 N。
上游常见的 −30 N 阈值会漏掉这里的后腿，默认改为 −20 N，以使静止时四条腿均被识别为支撑腿。
该值不是通用 Go2 标定结果。

## 3. 本机安装位置

```text
源码：       /home/sunbird/CAPO-Go2-Odometry
ROS 工作区： /home/sunbird/capo_torque_ws
Python 环境：/home/sunbird/capo_sim_venv
模型缓存：   /home/sunbird/CAPO-Go2-Odometry/.runtime/go2
结果目录：   /home/sunbird/CAPO-Go2-Odometry/.runtime/results
```

本机已经完成安装，可以直接执行下一节。
如果重建环境，使用：

```bash
cd /home/sunbird/CAPO-Go2-Odometry
scripts/setup_torque_sim.sh
```

脚本需要已有 ROS 2 Humble、colcon、Python pip 和相关 ROS 消息/TF 开发包。
Python 包安装在独立环境中，虚拟环境通过 `--system-site-packages` 访问系统 ROS Python 包。
没有安装 `python3-venv` 时，脚本使用 `virtualenv` 创建环境。
下载较慢时，可以在运行安装脚本前设置：

```bash
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
```

可以通过 `CAPO_SIM_WS` 和 `CAPO_SIM_VENV` 指定工作区与 Python 环境位置。

## 4. 启动和查看数据

带 MuJoCo 窗口的直行：

```bash
cd /home/sunbird/CAPO-Go2-Odometry
scripts/run_torque_sim.sh --scenario straight --duration 30 --viewer --realtime
```

先站立和策略预热 6 秒，然后按 0.4 m/s 的目标命令直行 30 秒，命令有 3 秒淡入。
策略实际速度由闭环动力学决定。测试结束后窗口和本次 CAPO 进程自动退出。
`Ctrl+C` 可以提前结束。

运行静止、直行和转向三个评测场景：

```bash
scripts/run_torque_sim.sh --scenario all --duration 30
```

无 `--realtime` 时，仿真尽可能快速运行；CAPO 仍按每 5 ms 仿真时间更新一次。
`ros2 topic hz` 测的是墙上时间频率，快速模式下会高于 200 Hz。
需要检查墙上时间频率时使用 `--realtime`。

更多运行示例：

```bash
scripts/run_torque_sim.sh --scenario stand --duration 30 --viewer --realtime
scripts/run_torque_sim.sh --scenario turn --duration 30 --wz 0.4 --viewer --realtime
scripts/run_torque_sim.sh --scenario curve --duration 30 --vx 0.4 --wz 0.2 --viewer --realtime
scripts/run_torque_sim.sh --scenario straight --duration 30 --foot-threshold -30
```

脚本默认使用 `ROS_DOMAIN_ID=71`、`ROS_LOCALHOST_ONLY=1`。同一个 domain 中同时只运行一组仿真。
如需换 domain，启动前设置 `CAPO_SIM_DOMAIN_ID`，查看话题时同步更改 `ROS_DOMAIN_ID`。

在另一个终端查看运行中的数据：

```bash
source /opt/ros/humble/setup.bash
source /home/sunbird/capo_torque_ws/install/setup.bash
export ROS_DOMAIN_ID=71
export ROS_LOCALHOST_ONLY=1

ros2 topic echo /SMX/Odom_2D --field pose.pose.position
ros2 topic echo /SMX/Odom_2D --field twist.twist.linear
ros2 topic echo /SMX/Odom
ros2 topic echo /SMX/ContactEstimate
ros2 topic echo /sim/ground_truth
ros2 run tf2_ros tf2_echo odom base_footprint
```

`/SMX/ContactEstimate` 是 12 维数组，腿序均为 FL、FR、RL、RR：

| 范围 | 数据 |
|---|---|
| `0..3` | CAPO 估算的足端等效 Fz，单位 N |
| `4..7` | 根据 Fz 阈值判定的支撑状态，0 或 1 |
| `8..11` | CAPO 原始接触概率 |

`/SMX/Go2Joint` 沿用 48 维布局，q 为 `0..15`，dq 为 `16..31`，τ 为 `32..47`。
每腿 hip/thigh/calf 的三个力矩都按 N·m 写入，wheel 槽为 0。
桥在 `layout.dim[0].label` 中携带 `imu_stamp_ns=...`，CAPO 匹配同时间戳的 IMU 后再更新，避免不同话题到达顺序导致错配。

## 5. 结果和评估方法

每个场景都有：

- `samples.csv`：逐帧估计、真值、速度、等效力、接触概率、实际关节力矩和 CAPO 高度。
- `summary.json`：误差、完成状态、更新频率和输入来源。
- `comparison.png`：轨迹、位置误差、等效力和偏航误差。
- `config.yaml`：该次运行实际使用的配置。
- `capo.log`、`ros_logs/`：CAPO 启动和 ROS 日志。

本机最终评测保存在 `.runtime/results/final-default/`。
每个场景预热 6 秒，之后评估 30 秒；各接收 7200 帧三维和二维里程计，其中评估阶段约 6000 帧。
每一帧里程计都按精确传感器时间戳与真值配对，所有场景完整运行，仿真时间更新频率为 200 Hz。

真值起点为 `(1.0, -0.5)`，初始偏航为 `0.7 rad`，与 CAPO 的平面位置原点不同。
评估只在预热结束时对齐一次初始 SE(2) 位姿，不做轨迹尺度拟合或后续重新对齐。
二维位置比较使用对齐后的真值；高度直接比较基座绝对高度；偏航比较相对起始角度的变化。

默认 −20 N 阈值的最终结果：

| 场景 | 真值运动 | 终点平面误差 | 平面位置 RMSE | 高度 RMSE | 偏航 RMSE |
|---|---|---|---|---|---|
| 静止 30 s | 实际缓慢位移约 1.31 mm | 小于 0.01 mm | 小于 0.01 mm | 6.29 mm | 接近 0 |
| 直行 30 s | 行程 9.54 m | 0.283 m（行程的 2.97%） | 0.159 m | 6.48 mm | 0.136° |
| 转向 30 s | 转过 577.60°，基座伴随小幅平移 | 0.0446 m | 0.0343 m | 12.84 mm | 0.0424° |

转向场景 CAPO 累计偏航变化为 577.61°。直行场景的目标速度为 0.4 m/s，实际行程受速度淡入与策略跟踪误差影响。
静止误差很小来自理想、确定性的仿真，不代表真机的可达到精度；静止场景不使用百分比作为主要指标。

保留的 20 秒阈值比较结果：

| 等效力阈值 | 静止时识别的支撑腿 | 直行终点误差 / 真值行程 | 转向终点平面误差 |
|---|---|---|---|
| −30 N | 仅前两腿 | 0.152 m / 6.15 m（2.48%） | 9.77 mm |
| −20 N | 四腿 | 0.185 m / 6.15 m（3.00%） | 5.11 mm |

对应 `.runtime/results/flat-minus30/` 和 `.runtime/results/flat-minus20/`。
默认阈值根据静止支撑腿的等效力选择。不同阈值对直行与转向的影响不同。

## 6. 改动和限制

本次增加了可选 `unitree_go` 构建、只启动融合器的 `go2_capo_torque_sim.launch.py`、独立力矩配置和仿真桥。
ROS 包装层使用传感器时间戳发布里程计，配对同一物理步的输入，并将世界系速度转换到 Odometry 的子坐标系。
原有 `go2_capo_sim.launch.py` 是 `/lowstate`、真值和评估节点的另一套流程，本次启动脚本使用新增的力矩 launch。

官方默认场景包含跨栏和阶梯，评测时移除了这些障碍，只使用平地。
本次没有验证阶梯、斜坡、外载荷或真机带噪声反馈。
关节力矩包含腿部惯性、重力和摩擦的影响，CAPO 的雅可比等效力估计不能完全分离这些分量。
快速摆腿、腿部接近奇异姿态、打滑或扭矩估计偏差可能导致接触误判，需要结合数据调整阈值。
