# 任务目标

在 Ubuntu 22.04 + ROS2 Humble 环境中完成以下链路：

```text
Unitree MuJoCo Go2
        ↓
Unitree SDK2 / DDS
        ↓
/lowstate
        ↓
CAPO-Go2-Odometry
        ↓
/SMX/Odom
/SMX/Odom_2D
        ↓
Ground Truth 对比
```

当前 CAPO 仓库：

```text
https://github.com/sun-under-bird/CAPO-Go2-Odometry.git
```

原作者仓库仅用于参考：

```text
https://github.com/ShineMinxing/CAPO-LeggedRobotOdometry.git
```

不要将原作者仓库作为主要仿真运行版本。

要求：

1. 保持当前已经在 Go2 真机成功运行的 CAPO 核心和 Go2 adapter 尽可能不变。
2. 优先修改仿真侧，使其提供符合真实 Go2 `/lowstate` 语义的数据。
3. 仿真环境使用 Unitree 官方 `unitree_mujoco`。
4. ROS2 使用 Humble。
5. 仿真 DDS 使用 `ROS_DOMAIN_ID=1`。
6. 仿真 DDS 网络接口使用 `lo`。
7. 最终必须能够获得 CAPO 里程计并和 MuJoCo Ground Truth 做对比。
8. 所有修改都必须有明确注释，不要删除现有实机功能。

---

# 第一阶段：检查当前系统环境

先检查：

```bash
lsb_release -a
uname -a
free -h
nproc
```

确认：

```text
Ubuntu 22.04
RAM >= 8 GB
CPU >= 4 核
```

推荐：

```text
RAM 12~16 GB
CPU 6~8 核
```

检查 ROS2：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

这里：

```text
source /opt/ros/humble/setup.bash
```

代表：

> 加载 ROS2 Humble 的环境变量。

检查 CycloneDDS：

```bash
ros2 pkg list | grep cyclonedds
```

这个 ROS2 命令代表：

> 列出已经安装的 ROS2 包，并检查是否存在 CycloneDDS 相关包。

---

# 第二阶段：准备目录结构

统一使用：

```text
~/unitree_sdk2
~/unitree_ros2
~/unitree_mujoco
~/CAPO-Go2-Odometry
```

如果目录已经存在，不要重复 clone，先检查 Git 状态：

```bash
git status
git remote -v
git branch
```

不要破坏已有修改。

---

# 第三阶段：安装 Unitree SDK2

如果没有：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
make -j$(nproc)
sudo make install
```

确认：

```bash
ls /opt/unitree_robotics
```

如果已经正确安装，则跳过。

---

# 第四阶段：安装 Unitree ROS2

如果没有 `unitree_ros2`：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

检查仓库 README，根据 Ubuntu 22.04 + ROS2 Humble 的官方方法完成 CycloneDDS 依赖编译。

完成后需要存在类似：

```text
~/unitree_ros2/cyclonedds_ws/install/setup.bash
```

检查：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
```

然后：

```bash
ros2 interface show unitree_go/msg/LowState
```

这个 ROS2 命令代表：

> 查看 `unitree_go/msg/LowState` 消息结构。

确认里面至少存在：

```text
imu_state
motor_state
foot_force
foot_force_est
```

---

# 第五阶段：安装 Unitree MuJoCo

Clone：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

如果已经存在，则：

```bash
cd ~/unitree_mujoco
git status
git pull --ff-only
```

安装需要的基础依赖：

```bash
sudo apt update
sudo apt install -y \
    git \
    cmake \
    build-essential \
    libyaml-cpp-dev \
    libspdlog-dev \
    libboost-all-dev \
    libglfw3-dev
```

按照 `unitree_mujoco` 当前 README 指定的 MuJoCo 版本安装。

不要自行猜版本。

如果 README 要求类似：

```text
~/.mujoco/mujoco-x.x.x
```

则严格使用官方要求的版本。

然后进入：

```bash
cd ~/unitree_mujoco/simulate
```

按照官方方式建立 MuJoCo 链接并编译：

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

如果 CMake 报：

```text
Unitree SDK2 not found
MuJoCo not found
GLFW not found
```

逐项修复，不要绕过依赖。

---

# 第六阶段：先单独运行 MuJoCo Go2

运行：

```bash
cd ~/unitree_mujoco/simulate/build
./unitree_mujoco -r go2 -s scene_terrain.xml
```

其中：

```text
-r go2
```

表示：

> 选择 Go2 机器人。

```text
-s scene_terrain.xml
```

表示：

> 加载 terrain 仿真场景。

如果虚拟机 OpenGL 有问题：

```text
GLFW error
OpenGL context error
libGL error
```

先检查 VMware 是否开启：

```text
Accelerate 3D Graphics
```

如果 GUI 仍然不可用，再研究 MuJoCo headless 模式。

不要因为 GUI 错误就认为物理仿真一定不能运行。

---

# 第七阶段：配置仿真 DDS

仿真和真机必须隔离。

仿真使用：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

这里：

```text
ROS_DOMAIN_ID=1
```

表示：

> ROS2/DDS 使用编号为 1 的通信域，与真机默认 domain 0 隔离。

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

表示：

> ROS2 使用 CycloneDDS 作为 DDS 中间件。

DDS 网络接口使用：

```text
lo
```

不要使用 Go2 真机网口。

优先使用 Unitree 官方提供的：

```text
setup_local.sh
```

如果官方脚本可用，就调用官方脚本，不要自己重复写 CycloneDDS 配置。

---

# 第八阶段：确认 MuJoCo LowState DDS 是否存在

在 MuJoCo 已运行情况下，打开另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

执行：

```bash
ros2 topic list
```

这个 ROS2 命令代表：

> 列出当前 DDS domain 中所有 ROS2 话题。

检查是否存在：

```text
/lowstate
```

如果不存在：

不要马上修改 CAPO。

先确认 Unitree MuJoCo 使用的是：

```text
rt/lowstate
```

还是已经通过 `unitree_ros2` 转成：

```text
/lowstate
```

明确 DDS → ROS2 的桥接关系。

最终必须让：

```bash
ros2 topic echo /lowstate
```

能够打印：

```text
unitree_go/msg/LowState
```

这个 ROS2 命令代表：

> 实时查看 `/lowstate` 中的 Go2 低层状态数据。

---

# 第九阶段：重点检查 LowState 内容

检查：

```text
imu_state.quaternion
imu_state.gyroscope
imu_state.accelerometer

motor_state[0..11].q
motor_state[0..11].dq
motor_state[0..11].tau_est

foot_force[0..3]
foot_force_est[0..3]
```

重点记录：

```text
机器人静止时数值
机器人走动时数值
脚落地时数值
脚离地时数值
```

检查频率：

```bash
ros2 topic hz /lowstate
```

这个 ROS2 命令代表：

> 统计 `/lowstate` 的实际发布频率。

---

# 第十阶段：检查当前 CAPO adapter

仓库：

```text
~/CAPO-Go2-Odometry
```

重点检查：

```text
go2_lowstate_adapter_node.cpp
fusion_estimator_node.cpp
config.yaml
launch/go2_capo.launch.py
```

确认当前 adapter 数据映射仍然是：

```text
Unitree motor order
FR FL RR RL

↓

CAPO leg order
FL FR RL RR
```

不要改变已经在真机验证成功的关节映射。

当前模式应该是：

```text
motor_state.q
    ↓
CAPO q

motor_state.dq
    ↓
CAPO dq

foot_force
    ↓
CAPO calf tau slot
```

不要未经验证改成 `tau_est` 模式。

---

# 第十一阶段：判断 MuJoCo foot_force 能不能直接用

运行机器人运动后观察：

```bash
ros2 topic echo /lowstate
```

检查：

```text
foot_force
```

分两种情况。

## 情况 A

如果：

```text
foot_force 随脚落地/抬起明显变化
```

则：

```text
直接使用
```

不修改 CAPO adapter。

继续下一阶段。

---

## 情况 B

如果：

```text
foot_force:
- 0
- 0
- 0
- 0
```

一直不变：

则不要修改 CAPO core。

应该修改：

```text
unitree_mujoco simulation bridge
```

增加：

```text
MuJoCo foot contact force
        ↓
LowState.foot_force[4]
```

要求：

1. 正确识别：

   * FR foot
   * FL foot
   * RR foot
   * RL foot

2. 从 MuJoCo contact 中取得足端和地面的接触力。

3. 最低要求得到：

```text
normal contact force
```

4. 映射到：

```text
LowState.foot_force[4]
```

5. 保持 Unitree 的脚顺序。

6. 接触时明显 > 0。

7. 离地时接近 0。

8. 不修改：

```text
CAPO FusionEstimator/
```

---

# 第十二阶段：编译 CAPO-Go2-Odometry

使用：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
```

建立单独工作空间，例如：

```bash
mkdir -p ~/capo_ws
```

根据当前仓库 CMakeLists 编译。

如果仓库本身就是独立 ROS2 package，可继续沿用已有的：

```bash
colcon build \
    --base-paths ~/CAPO-Go2-Odometry \
    --build-base ~/capo_ws/build \
    --install-base ~/capo_ws/install \
    --packages-select fusion_estimator \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

这里：

```text
colcon build
```

代表：

> 编译 ROS2 工作空间中的 package。

```text
--packages-select fusion_estimator
```

代表：

> 只编译 `fusion_estimator` 包。

---

# 第十三阶段：启动 CAPO

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/capo_ws/install/setup.bash

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

运行：

```bash
ros2 launch fusion_estimator go2_capo.launch.py
```

这个 ROS2 命令代表：

> 同时启动 Go2 LowState adapter 和 CAPO fusion estimator。

---

# 第十四阶段：验证 CAPO 输出

检查：

```bash
ros2 topic list | grep SMX
```

应该至少看到：

```text
/SMX/Odom
/SMX/Odom_2D
```

检查频率：

```bash
ros2 topic hz /SMX/Odom_2D
```

这个 ROS2 命令代表：

> 查看 CAPO 2D odometry 的实际发布频率。

检查数据：

```bash
ros2 topic echo /SMX/Odom_2D
```

这个 ROS2 命令代表：

> 查看 CAPO 当前估计的位置、姿态和速度。

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

这个 ROS2 命令代表：

> 实时查看 `odom → base_footprint` 的位置和姿态变换。

---

# 第十五阶段：处理仿真时间

这是必须检查的部分。

当前 `go2_lowstate_adapter_node.cpp` 使用：

```cpp
create_wall_timer(...)
```

并固定：

```text
publish_rate_hz = 200 Hz
```

需要检查 MuJoCo：

```text
simulation time
vs
wall time
```

是否基本 1:1。

如果 MuJoCo 运行：

```text
Real Time Factor ≈ 1.0
```

第一版可以暂时保持现状。

如果 MuJoCo 明显不是 1x：

```text
0.5x
2x
暂停
快进
```

则需要设计仿真专用时间模式：

```text
use_sim_time = true
```

以及使用仿真时间戳，而不是纯 wall timer。

要求：

不要破坏真机模式。

建议增加参数：

```yaml
simulation_mode: false
```

真机：

```yaml
simulation_mode: false
```

仿真：

```yaml
simulation_mode: true
```

仿真模式下尽量由：

```text
LowState 到达事件
```

或者：

```text
simulation clock
```

驱动 CAPO，而不是 wall clock。

第一版不需要过度设计，但必须记录这个风险。

---

# 第十六阶段：增加 Ground Truth

仿真验证必须有真值。

从 MuJoCo 获取 Go2 base：

```text
position
orientation
linear velocity
angular velocity
```

发布 ROS2 Ground Truth：

```text
/ground_truth/odom
```

消息：

```text
nav_msgs/msg/Odometry
```

frame：

```text
world
```

child：

```text
base_footprint
```

如果 MuJoCo bridge 已经有 frame position / velocity，优先直接使用现成值，不重复计算。

---

# 第十七阶段：建立 CAPO vs Ground Truth 对比

最终形成：

```text
                    MuJoCo
                       │
              ┌────────┴────────┐
              ↓                 ↓
         /lowstate      /ground_truth/odom
              ↓
            CAPO
              ↓
        /SMX/Odom_2D
              │
              └──────────┐
                         ↓
                    evaluator
```

至少计算：

```text
x error
y error
z error
yaw error

position RMSE
velocity RMSE
ATE
RPE
```

第一版可以先实现：

```text
终点位置误差
累计距离
相对位置误差百分比
```

后续再增加完整 evo 风格评估。

---

# 第十八阶段：设计测试场景

至少做以下测试。

## Test 1：静止

```text
30~60 秒站立
```

检查：

```text
position drift
yaw drift
z drift
```

---

## Test 2：直线

```text
5 m
10 m
```

检查：

```text
x/y error
velocity error
```

---

## Test 3：原地旋转

```text
90°
180°
360°
```

检查：

```text
yaw drift
position fake motion
```

---

## Test 4：矩形

```text
5 m × 5 m
```

检查闭环误差。

---

## Test 5：坡道

利用：

```text
scene_terrain.xml
```

测试：

```text
z
roll
pitch
```

---

## Test 6：楼梯

如果 MuJoCo 有现成楼梯场景，测试：

```text
height estimation
contact switching
```

---

# 第十九阶段：记录 ROS bag

测试时记录：

```bash
ros2 bag record \
    /lowstate \
    /SMX/Odom \
    /SMX/Odom_2D \
    /ground_truth/odom \
    /tf \
    /tf_static
```

这个 ROS2 命令代表：

> 将指定 ROS2 话题完整记录成 rosbag，方便离线分析。

---

# 第二十阶段：最终交付内容

完成后输出以下内容：

## 1. 环境

```text
Ubuntu version
ROS2 version
MuJoCo version
unitree_mujoco commit
CAPO-Go2-Odometry commit
```

## 2. 是否成功

明确回答：

```text
MuJoCo Go2：成功 / 失败
/lowstate：成功 / 失败
foot_force：有效 / 无效
CAPO：成功 / 失败
Ground Truth：成功 / 失败
```

## 3. 修改文件

逐个列出：

```text
path
修改原因
修改内容
```

## 4. 完整启动顺序

整理最终命令，例如：

```text
Terminal 1
MuJoCo

Terminal 2
ROS2 / Unitree DDS

Terminal 3
CAPO

Terminal 4
验证 / rosbag
```

## 5. 风险

重点列出：

```text
foot_force 仿真真实性
IMU 是否无噪声
joint q/dq 是否过于理想
时间戳
Real Time Factor
contact force scale
sim-to-real gap
```

---

# 非常重要的限制

不要：

```text
修改 CAPO 数学核心来迁就仿真
```

不要：

```text
仿真用原作者版本
真机用 CAPO-Go2-Odometry
```

不要：

```text
直接把 Ground Truth 当 CAPO 输入
```

不要：

```text
没有确认 foot_force 就开始评价 CAPO 精度
```

不要：

```text
因为仿真 IMU 完美，就认为真机也应该有同样精度
```

---

# 最终目标

最终必须做到：

```text
                         CAPO-Go2-Odometry
                                ↑
                       同一套运行代码
                                ↑
                   ┌────────────┴────────────┐
                   │                         │
               Simulation                  Real
                   │                         │
             simulated LowState        Go2 LowState
                   │                         │
                   ↓                         ↓
             CAPO Odometry             CAPO Odometry
                   │
                   ↓
           MuJoCo Ground Truth
                   │
                   ↓
             Quantitative Error
```

优先完成最小闭环：

```text
MuJoCo Go2
→ /lowstate
→ foot_force 有效
→ CAPO
→ /SMX/Odom_2D
→ Ground Truth
→ 误差比较
```

如果某一步失败，先解决当前层，不要跳到下一层。

