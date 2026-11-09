# Marsdog V1.7 CAPO 仿真适配

更新：2026-11-09（Asia/Shanghai）。

已打通 Marsdog 14 轴仿真、传感器输入、CAPO C++ 融合、ROS 2 里程计和真值评估。质量、IMU 外参、关节输出力矩和传感器时序先采用仿真模型提供的值；本版用于平地仿真开发，尚未接入或验证真机。

## 本机运行

源码目录为 `/home/sunbird/CAPO-Go2-Odometry`，新增构建工作区为 `/home/sunbird/capo_marsdog_ws`，Python 使用已有 `/home/sunbird/capo_sim_venv`。原 Go2 的 `/home/sunbird/capo_torque_ws` 安装目录保留。

本机已完成编译。启动直行并显示 MuJoCo 窗口：

```bash
cd /home/sunbird/CAPO-Go2-Odometry
scripts/run_marsdog_sim.sh --scenario straight --duration 30 --viewer --realtime
```

默认预热 4 秒，随后执行 30 秒评测。`--realtime` 按实际时间运行；无此参数时尽快运行。按 Ctrl+C 或关闭窗口会结束本次仿真并保存已采集结果。

一次运行静止、直行、原地转向：

```bash
scripts/run_marsdog_sim.sh --scenario all --duration 30
```

单独转向：

```bash
scripts/run_marsdog_sim.sh --scenario turn --duration 30 --wz 0.3 --viewer --realtime
```

`--vx` 默认 0.25 m/s，支持 −0.3～0.4；`--wz` 默认 0.3 rad/s，范围 ±0.6。SDK 目录默认 `/home/sunbird/Marsdog_V1.7_Roll2100_SLAM_SDK_v0.2.0`，可用 `--sdk` 指定。`--output` 指定新的结果根目录，已有同名场景目录会被拒绝，防止覆盖结果。

源码修改后编译：

```bash
scripts/setup_marsdog_sim.sh
```

依赖：ROS 2 Humble、colcon、C++17，以及 Python 的 mujoco、onnxruntime、numpy、PyYAML、matplotlib 和 rclpy。本机实测 Python 3.10.12 / MuJoCo 3.3.6 / ONNX Runtime 1.23.2。SDK 声明的 MuJoCo 固定版本为 3.11.0；本次结果仅对应已记录的本机版本。交付的策略运行器支持 CPython 3.10、3.12。

## 方形测试与 ROS 2 输出

### 边长 10 米的方形测试

```bash
scripts/run_marsdog_sim.sh --scenario square --side-length 10 --vx 0.25
```

方形使用独立的仿真路点控制器，以 MuJoCo 位置判断沿边进度和转弯点，IMU 航向用于跟踪方向。底层 ONNX 策略和 CAPO 输入不加入位置真值；全程不重置里程计。路点控制器在四条边完成并停稳后结束，`--duration` 不限制该场景，可用 `--square-timeout` 设置最长仿真时间。

2026-11-09 已完成 10 米方形测试：四边拐角间距约 9.989～9.994 米，实际累计行程 41.503 米，持续 217.46 秒；全程位置 RMSE 为 0.723 米、最大误差 1.099 米，最终与真实终点相差 3.21 厘米。各边距离低估约 7.67%～7.88%，不能用较小的闭环误差代表全程精度。

[方形完整报告](.runtime/marsdog/20261109-140158-854924/square/square_report.md) · [轨迹对比图](.runtime/marsdog/20261109-140158-854924/square/square_report.png)

可从结果目录重新生成报告：

```bash
/home/sunbird/capo_sim_venv/bin/python -B integration/report_square_run.py .runtime/marsdog/20261109-140158-854924/square
```

### 订阅输出

另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/sunbird/capo_marsdog_ws/install/setup.bash
export ROS_DOMAIN_ID=72
export ROS_LOCALHOST_ONLY=1
ros2 topic echo --once /SMX/Odom_2D
```

| 话题 | 类型 | 内容 |
|---|---|---|
| `/marsdog/joint_states` | `sensor_msgs/JointState` | 带名称、时间戳的 14 轴 q/dq/实际执行器力矩 |
| `/marsdog/imu` | `sensor_msgs/Imu` | 含重力响应的加速度计、角速度、wxyz 姿态 |
| `/SMX/Odom` | `nav_msgs/Odometry` | 3D 里程计，`odom → base_link` |
| `/SMX/Odom_2D` | `nav_msgs/Odometry` | 2D 里程计，`odom → base_footprint` |
| `/SMX/ContactEstimate` | `std_msgs/Float64MultiArray` | 各脚等效 Fz、阈值接触判定、接触概率 |
| `/sim/ground_truth` | `nav_msgs/Odometry` | 仿真基座真值，仅用于评估 |
| `/clock` | `rosgraph_msgs/Clock` | 仿真时钟 |
| `/tf` | TF | `odom → base_footprint` |

本版未发布 `base_footprint → base_link` 和完整机器人关节 TF 树。3D 位置直接读取 `/SMX/Odom`；后续可接 robot_state_publisher 完善可视化。

一个 domain 同时运行一组仿真。可用 `CAPO_MARSDOG_DOMAIN_ID` 更改默认 72；订阅端同步设置 `ROS_DOMAIN_ID`。这里通过 ROS 2 直接发布传感器数据，运行本流程不依赖交付 SDK 的原生 DDS 读取器或缺失的 `libddsc.so.0`。

## 已实现的适配

- 前腿运动链包含腰 pitch、腰 yaw、三个腿关节及足端；后腿直接从 `base_link` 出发。每个前腿链有 7 个变换节点，后腿有 5 个。
- 全部活动关节参与足端速度雅可比；每只脚的力估计只使用本腿三个电机的力矩。腰部力矩不重复分配给两条前腿。
- `JointState` 按名称映射，并检查完整性、重复名称与非有限值。缺项不会静默补零。
- C++ 端仍保留原有 16 槽内部布局：12 个腿轴沿用每腿前三槽，腰 yaw 放槽 3，腰 pitch 放槽 7；这些槽在 Marsdog 中不表示轮子。
- 每个 2 ms 物理步产生一组 IMU/关节数据；CAPO 匹配时间戳，仿真桥收到同时间戳的 3D、2D 和接触输出后再推进。策略周期 20 ms，即 50 Hz。
- IMU 输入已采用机身轴向；Marsdog 配置清除原 Go2 的固定 1.7° 修正，使用模型重力 9.81 m/s²，并按角速度及其差分补偿 IMU 平移外参引起的切向/向心加速度。
- 使用模型质量 8.31 kg，关闭旧的 40 kg 下限估重，以及依赖未标定力矩的加速度/姿态加速度观测。位置、速度和 IMU 融合继续工作。
- 点足采用仿真器的球心和 22 mm 半径，在平地世界竖直方向取接触位置。该几何假设需要在斜坡、非球形脚垫或真机足底模型中重新处理。

使用交付的原 ONNX 与策略运行器，只读加载 SDK 文件。`integration/marsdog_model.py` 从实际编译后的仿真模型提取静态关节原点、轴向、足端几何、质量和 IMU 位置，写入每次运行的 `config.yaml`。因此本次不会混用新版机械 URDF 与旧训练 MJCF。

估计器读取的力矩为 MuJoCo `qfrc_actuator`，不包含接触约束力；底层 ONNX 策略和 CAPO 估计器都不读取足端接触标志、约束力、基座位置或基座线速度真值。方形场景的上层路点控制器使用仿真位置导航。理想 IMU 的姿态是本次仿真传感器输入。

默认 Fz 接触阈值为 −8 N。静止时等效 Fz 约为前腿 −20 N、后腿 −14 N；阈值只是当前仿真初值。动态惯性、腿部自重和打滑仍可能引起接触误判，未完成完整动力学补偿。

2026-11-09 对方形和直行日志完成离线诊断，确认接触选择显著影响速度低估，并发现基座角速度漏项及球形脚位置约束不一致。详情见 [误差诊断与优化顺序](MARSDOG_DIAGNOSIS.md)。这些是诊断结果，尚未完成算法修正后的轨迹验收。

## 已完成的验证

1. 80 组随机姿态、关节角和速度，对比 C++ 足端位置/速度与 MuJoCo 独立雅可比；用已知力生成关节力矩验证力求解。包含运动腰部及大幅变化的腰部力矩，确认共享腰部不会污染独立足力求解。
2. 单独腰部运动的有限差分验证：前脚速度非零、后脚速度为零；非法运动链会被拒绝。
3. Marsdog 静止、直行、转向各 30 秒，均完成，单场景含 4 秒预热和 17,000 组时间戳匹配输出。
4. 在新编译核心上运行原 Go2 直行流程：5 秒评测完成，终点误差约 2.90 cm，行程约 1.046 m。

运动学测试命令：

```bash
/home/sunbird/capo_sim_venv/bin/python -B tests/test_marsdog_kinematics.py -v
```

Marsdog 30 秒结果：

| 场景 | 真值行程 | 终点平面误差 | 平面位置 RMSE | 相对偏航 RMSE | 高度 RMSE |
|---|---:|---:|---:|---:|---:|
| 静止指令 | 0.0698 m | 0.75 mm | 0.37 mm | 0.0028° | 1.10 mm |
| 直行指令 | 6.390 m | 45.79 cm，行程的 7.17% | 26.59 cm | 0.0100° | 4.62 mm |
| 转向指令 | 1.199 m | 4.22 cm | 2.68 cm | 0.0304° | 6.15 mm |

“静止指令”下模型仍有微小滑动；表中的误差是相对于其真实运动的估计误差，不是机器人本身完全不动。“直行”表示发送前进指令，策略没有绝对航向锁定，实际轨迹可以弯曲。初始位姿只对齐一次，不做尺度拟合或后续轨迹校正。当前直行误差仍需后续改进，不能作为真机精度承诺。

本次完整结果目录：

```text
/home/sunbird/CAPO-Go2-Odometry/.runtime/marsdog/20261109-123414-658303/
```

每个场景包含：

- `config.yaml`：实际使用的运动链、质量、外参及参数。
- `provenance.json`：SDK 路径、模型/策略摘要、MuJoCo 版本和仿真假设。
- `inputs.csv`：全部 14 轴、IMU 与时间戳输入。
- `samples.csv`：逐帧里程计、真值及接触估计。
- `summary.json`：完成状态、误差统计与腰部实际角度范围。
- `comparison.png`：轨迹、误差、足力与偏航对比图。
- `capo.log`：C++ 节点运行日志。

## 真机接入前仍需替换/确认

模型和外参先用训练仿真版本；目前不读取历史 CSV 中有争议的腰 pitch 零位，也没有擅自裁剪/反转这些数据。真机接入前需统一新 URDF、腰部零位/方向及前脚坐标变换，再补力矩标定和实测 IMU 参数。

本版同步模式针对同时间戳的仿真输入。真机 LowState 的各轴与各类 IMU 是异步收包，后续需要实现基于时间戳、有效位和数据年龄的缓存/插值适配器，不能直接把不同时间的数据盖成相同时间戳。噪声下的角加速度差分还需要滤波和延迟评估。

尚未实现真机 DDS 订阅适配、坡地/台阶验证、完整动力学足力补偿或精度验收。当前可以继续在这条已运行的链路上做接触估计、噪声与时延测试。
