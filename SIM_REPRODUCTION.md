# CAPO-Go2-Odometry MuJoCo 仿真链路复现指南

本文档描述在**一台全新的 Ubuntu 22.04 电脑**上从零复现完整仿真链路：

```text
MuJoCo Go2 仿真 → DDS /lowstate（含 foot_force）→ CAPO 里程计
    → /SMX/Odom、/SMX/Odom_2D → Ground Truth → 定量误差对比
```

验证结果（本机实测，供对照）：

```text
crawl 基线：平地直线 16.91 m / CAPO 15.97 m，相对误差 3.64%，yaw RMSE 0.004 rad
RL 步态（NP3O）：直线 vx=0.5 误差 4.68%；原地转 643° 转角比 1.000；
                 矩形闭环 25 m 终点误差 0.36%；8° 坡道全速翻越（终点误差 ~4%）
```

> 另见 `SIM_FINAL_REPORT.md`（交付报告：全部测试结果、修改清单、风险分析）
> 与 `RL_GAIT_PLAN.md`（RL 步态路线的执行记录、三大根因、运行规范）。

---

## 0. 前提条件

```text
Ubuntu 22.04 LTS（本机为 22.04.3，VMware 虚拟机可用，无需 GPU 直通）
ROS2 Humble（桌面版或基础版均可）
磁盘 ≥ 15 GB，内存 ≥ 4 GB
可访问 GitHub（克隆 4 个仓库 + 下载 MuJoCo 发布包）
```

涉及 4 个代码库：

| 仓库 | 用途 | 参考版本 |
|---|---|---|
| [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | DDS 通信库，装到 `/opt/unitree_robotics` | 最新（自带 CycloneDDS 0.10.2） |
| [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) | 物理引擎 3.3.6 发布包（unitree_mujoco 指定版本） | **必须 3.3.6** |
| [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | Go2 仿真器 | commit `4134cb5` |
| **本仓库** CAPO-Go2-Odometry | CAPO 里程计 + 全部适配代码 + **RL 策略模型与导出脚本** | `main` 最新（全部修改已提交） |

RL 策略来源（可选，仅重新导出模型时需要）：[zeonsunlightyu/LocomotionWithNP3O](https://github.com/zeonsunlightyu/LocomotionWithNP3O)——
本仓库 `sim_patch/models/go2_policy.onnx` 已含导出好的模型（948 KB），直接用即可。

---

## 1. 系统依赖

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential \
  libglfw3-dev libyaml-cpp-dev libfmt-dev \
  libboost-program-options-dev \
  python3-colcon-common-extensions python3-pip \
  ros-humble-rmw-cyclonedds-cpp
```

（`ros-humble-rmw-cyclonedds-cpp` 若随 Humble 桌面版已装可跳过；确认 `ls /opt/ros/humble/setup.bash` 存在。）

---

## 2. 安装 unitree_sdk2（→ /opt/unitree_robotics）

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make install        # 安装到 /opt/unitree_robotics/{include,lib}
```

验证：

```bash
ls /opt/unitree_robotics/lib/cmake          # 应有 unitree_sdk2 的 cmake 配置
ls /opt/unitree_robotics/lib/libddsc.so.0   # SDK2 自带 CycloneDDS 0.10.2
```

---

## 3. 安装 MuJoCo 3.3.6（→ ~/.mujoco）

```bash
mkdir -p ~/.mujoco && cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/mujoco-3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz
tar xzf mujoco-3.3.6-linux-x86_64.tar.gz
# 得到 ~/.mujoco/mujoco-3.3.6/，内含 include/ lib/ simulate/ 等
```

> 版本必须是 3.3.6（unitree_mujoco 的模型与 API 按此版本适配）。

---

## 4. 编译 unitree_mujoco + 应用仿真侧修改

### 4.1 克隆并编译

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco
git checkout 4134cb5    # 本机验证所用 commit

# 将 MuJoCo 软链进 simulate（CMakeLists 以相对路径 mujoco/ 引用）
ln -s ~/.mujoco/mujoco-3.3.6 simulate/mujoco

# 编译仿真器
cd simulate
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
# 产出 simulate/build/unitree_mujoco
```

### 4.2 应用桥接层修改（foot_force + 线程安全）

官方 unitree_mujoco 有两个问题，必须修改后才能支撑 CAPO 验证：**(a)** LowState 不含 foot_force；**(b)** 桥接线程直接遍历 `mjData->contact` 与物理线程无锁并发，行走等高接触频率场景下偶发段错误。

**推荐做法：直接用本仓库 `sim_patch/` 下验证过的文件覆盖**（随仓库迁移，无需手工改代码）：

```bash
cp ~/CAPO-Go2-Odometry/sim_patch/unitree_sdk2_bridge.h  ~/unitree_mujoco/simulate/src/
cp ~/CAPO-Go2-Odometry/sim_patch/main.cc                ~/unitree_mujoco/simulate/src/
cp ~/CAPO-Go2-Odometry/sim_patch/scene_terrain.xml      ~/unitree_mujoco/unitree_robots/go2/
cp ~/CAPO-Go2-Odometry/sim_patch/go2_scene/scene.xml    ~/unitree_mujoco/unitree_robots/go2/
cp ~/CAPO-Go2-Odometry/sim_patch/example_cpp/gait_go2.cpp      ~/unitree_mujoco/example/cpp/
cp ~/CAPO-Go2-Odometry/sim_patch/example_cpp/policy_runner.cpp ~/unitree_mujoco/example/cpp/
cp ~/CAPO-Go2-Odometry/sim_patch/example_cpp/CMakeLists.txt    ~/unitree_mujoco/example/cpp/

# 重新编译仿真器与步态控制器
cd ~/unitree_mujoco/simulate/build && make -j$(nproc)
cd ~/unitree_mujoco/example/cpp && mkdir -p build && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
```

> `go2_scene/scene.xml` 是**平地版**（官方原版 scene.xml 含 x=1.2/1.6 挡板和
> x=2.3~3.4 递增台阶——RL 行走会爬上 1.8 m 台阶再摔落，干扰直线测试，已删除）。
> 场景选择规则：`./unitree_mujoco -r go2` 读 `simulate/config.yaml` 的
> `robot_scene`（默认平地）；带 `-s scene_terrain.xml` 才是坡道/台阶地形。

修改内容速览（核对用，细节见文件内注释）：

| 文件 | 修改 |
|---|---|
| `unitree_sdk2_bridge.h` | 新增 `ComputeFootForce()`（物理线程在 mj_step 后调用，提取足端 geom 与环境的法向接触力写入 `foot_force_cache_`）；run() 中把缓存填入 LowState/HighState 的 `foot_force`；G1 无该字段用 `if constexpr` 编译期裁剪；highstate 填充 base 四元数供 Ground Truth 使用 |
| `main.cc` | 全局 `std::atomic<UnitreeSDK2BridgeBase*> g_bridge`；物理循环 `mj_step` 后、`AddToHistory()` 前调 `ComputeFootForce()` |

### 4.3 步态控制器 gait_go2（测试激励必需）

已由 `sim_patch/example_cpp/` 提供（`gait_go2.cpp` + `CMakeLists.txt`），随 4.2 一并覆盖并编译，产出 `example/cpp/build/gait_go2`。要点：
- crawl 静步态（四腿 FR→RL→FL→RR 依次摆动，摆动占比 0.25，任意时刻至多 1 腿离地）——纯关节位置控制下 trot 必摔，这是实测结论；
- PD 增益 kp=50 / kd=3.5（kp=80 任何步态都摔）；
- 起立 3 s tanh 过渡 + 步态幅值 2 s 淡入（防目标角跳变冲击）；
- IMU yaw PD 航向闭环（kYawKp=1.2, kYawKd=0.08）；
- `main()` 开头必须 `ChannelFactory::Instance()->Init(1, "lo")`，结尾用 `std::_Exit(0)`（正常析构会因写线程访问成员而堆损坏）。

### 4.4 地形场景（可选，坡道测试用）

`sim_patch/scene_terrain.xml`（4.2 节已覆盖到 `unitree_robots/go2/`）相对官方版本的改动：把正前方 (1.5, 0) 的方台+圆柱障碍挪到 (1.5, 2.5)（否则直行必撞，实测摔倒），坡道改为 8° 缓坡置于 +x 正前方：

```xml
<geom pos="3.5 0.0 0.139" type="box" size="1.0 0.75 0.05"
      quat="0.9975640502598242 0.0 -0.0697564737441253 0.0" />
```

（quat 为绕 y 轴 -8°。原 28.7° 坡道 crawl 步态无法攀爬——爬坡失败属步态限制，非 CAPO 问题。平地测试用官方默认 `scene.xml`，无需改场景。）

### 4.5 DDS 配置

`simulate/config.yaml` 关键项（官方默认已如此，确认即可）：

```yaml
robot: "go2"
robot_scene: "scene.xml"   # 坡道测试传参覆盖为 scene_terrain.xml
domain_id: 1               # 必须与 ROS2 侧一致
interface: "lo"            # 回环网卡；真机部署时改实际网口
```

### 4.6 RL 步态激励 policy_runner（推荐激励，Test 1~5 均用此）

比 crawl 快 2.6 倍（0.5 m/s）、能原地转、能翻坡。源码已在 4.2 覆盖进
`example/cpp/`，还需准备 **onnxruntime** 与**策略模型**：

**① 安装 onnxruntime（CPU 版即可，50 Hz 推理单次 <2 ms）**

```bash
pip3 install onnxruntime==1.23.2          # .so 在 pip 包内，C++ 直接链接

# pip 包不含 C++ 头文件，单独下载（共 4 个文件，放到固定目录）
mkdir -p ~/onnxruntime_cpp/include ~/onnxruntime_cpp/lib
cd ~/onnxruntime_cpp/include
for f in onnxruntime_c_api.h onnxruntime_cxx_api.h onnxruntime_cxx_inline.h \
         onnxruntime_ep_c_api.h onnxruntime_float16.h; do
  wget https://raw.githubusercontent.com/microsoft/onnxruntime/v1.23.2/include/onnxruntime/core/session/$f
done

# 把 pip 包里的动态库软链过来（CMakeLists 按 ~/onnxruntime_cpp/lib 链接）
ln -s ~/.local/lib/python3.10/site-packages/onnxruntime/capi/libonnxruntime.so.1.23.2 \
      ~/onnxruntime_cpp/lib/libonnxruntime.so
ln -s ~/.local/lib/python3.10/site-packages/onnxruntime/capi/libonnxruntime.so.1.23.2 \
      ~/onnxruntime_cpp/lib/libonnxruntime.so.1
```

**② 放置策略模型**（policy_runner 默认读 `~/np3o/go2_policy.onnx`）

```bash
mkdir -p ~/np3o
cp ~/CAPO-Go2-Odometry/sim_patch/models/go2_policy.onnx ~/np3o/
# 也可用最后一个命令行参数指定任意路径：./policy_runner 30 0.5 0 0 /path/to/policy.onnx
```

模型 = NP3O `model_10000.pt` 的 actor（含 obs 归一化层）。
如需从 checkpoint 重新导出：`sim_patch/scripts/export_onnx.py`（需 torch，
并按其头部注释把 NP3O 仓库克隆到 `~/np3o`）。

**③ 编译**（4.2 的 cmake 已含 policy_runner 目标）

```bash
cd ~/unitree_mujoco/example/cpp/build && make -j$(nproc)
```

**④ 三大运行要点（sim2sim 调试换来的经验，务必遵守）**

```text
指令死区：   速度指令 |v|<0.2 m/s 被策略当站立（训练时被置零，分布外）→ 指令 ≥0.2
heading 模式：训练时 wz 由航向差闭环生成（0.5·wrap(target−yaw)，±1 限幅），
             恒定 wz 不在分布（转 0.56 rad 即停）。policy_runner 已内置该闭环：
             --script 第 5 列给绝对航向（度），转向段自动闭环纠偏
运行环境：   必须 export LD_LIBRARY_PATH（见第 7 节），否则 libddsc 0.10.5(ROS)
             与 libddscxx 0.10.2(unitree) 混载 → 随机堆损坏
启动呆滞：   静止→迈步有 5~24 s 延迟（打破对称需要时间），脚本段时长留余量
```

用法：

```bash
cd ~/unitree_mujoco/example/cpp/build
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib

./policy_runner 30 0.5 0 0        # 单指令：vx=0.5 m/s 走 30 s（起立 3 s 在前）
./policy_runner --teleop          # 键盘遥控：W/S 前后 Q/E 侧移 A/D 转向
                                   #           空格急停 X 退出（每键 ±0.05/±0.10）
./policy_runner --script ~/CAPO-Go2-Odometry/sim_patch/scripts/rl_rect.txt
                                  # 脚本序列：每行 <dur> <vx> <vy> <wz> [yaw_deg]
                                  # 第 5 列=目标绝对航向，段内持续闭环纠偏
```

---

## 5. 编译 unitree_ros2 桥接包（cyclonedds_ws）

CAPO 的 adapter 订阅 DDS 话题 `rt/lowstate` 需要 `unitree_go` 等 IDL 消息包：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
# 产出 install/ 下 unitree_api / unitree_go / unitree_hg
```

验证：

```bash
source install/setup.bash
ros2 interface show unitree_go/msg/LowState | head
```

---

## 6. 编译 CAPO-Go2-Odometry

```bash
# 把本仓库（含全部未提交修改！）放到新机器，例如 ~/CAPO-Go2-Odometry
mkdir -p ~/capo_ws
cd ~/capo_ws
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --base-paths ~/CAPO-Go2-Odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
```

验证：

```bash
source ~/capo_ws/install/setup.bash
ros2 pkg list | grep fusion_estimator
```

启动脚本 `scripts/run_capo_sim.sh`、`scripts/run_mujoco_sim.sh` 已随仓库提供。

---

## 7. 运行（四个终端）

DDS 环境三要素（脚本已内置，手工运行时必须一致）：

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 关键：SDK2 的 libddsc 0.10.2 必须优先于 ROS Humble 的 0.10.5，否则 DDS 断言崩溃
export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:${LD_LIBRARY_PATH}"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="lo" priority="default" multicast="default" />
</Interfaces></General>
<Discovery><Peers><Peer Address="localhost"/></Peers></Discovery>
</Domain></CycloneDDS>'
```

```bash
# Terminal 1 —— MuJoCo 仿真
bash ~/CAPO-Go2-Odometry/scripts/run_mujoco_sim.sh scene_terrain.xml
# （平地测试：run_mujoco_sim.sh，默认 scene.xml）

# Terminal 2 —— CAPO 全链（adapter + fusion + ground_truth + evaluator）
bash ~/CAPO-Go2-Odometry/scripts/run_capo_sim.sh
# 等待日志出现 "[中期] 样本 N | ..." 即四节点全部就绪且已收到 /lowstate

# Terminal 3 —— 步态激励（RL 步态推荐；crawl 为基线对照）
cd ~/unitree_mujoco/example/cpp/build
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
./policy_runner --teleop          # 键盘遥控（W/S/Q/E/A/D，空格急停，X 退出）
./policy_runner 30 0.5 0 0        # 或脚本化：vx=0.5 m/s 走 30 s
./gait_go2 walk 60 1.0            # crawl 基线：<stand|walk|turn> <dur> [speed]

# Terminal 4 —— rosbag（-o 目录必须不存在）
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash   # 必需，否则 /lowstate 类型无法识别
timeout 85 ros2 bag record -o ~/capo_bags/<新目录> \
  /lowstate /SMX/Odom /SMX/Odom_2D /ground_truth/odom /tf /tf_static
```

### 验证成功的标志

```text
Terminal 2 出现： [capo_evaluator_node]: [中期] 样本 180 | 当前误差 ... | 行程 GT=... CAPO=...
                  （说明 /lowstate → CAPO → /SMX/Odom_2D → GT 全链通）
Terminal 3 出现： 航向闭环基准 yaw0=...   （说明 gait 收到了 sim 的 /lowstate 反馈）
站立时 foot_force 约 30~60 N/足（bag 中 /lowstate.foot_force[]），
行走中摆动足接近 0。
评估 CSV 输出： /tmp/capo_eval.csv（列 t,capo_x..yaw,gt_x..yaw）
```

### 测试场景速查

RL 步态（policy_runner，推荐）——结果详见 `SIM_FINAL_REPORT.md` 第 5b 节：

| 测试 | 场景 | Terminal 3 命令 | 参考结果 |
|---|---|---|---|
| Test 1 静止 | 平地 | `./policy_runner 60 0 0 0` | GT 漂移 0.039 m，终点误差 0.050 m |
| Test 2 直线 | 平地 | `./policy_runner 30 0.5 0 0` | 15.33 m，相对误差 4.68%，行程比 1.048 |
| Test 3 原地转 | 平地 | `./policy_runner 30 0 0 0.5` | 643°，转角比 1.000，yaw RMSE 0.0006 rad |
| Test 4 矩形闭环 | 平地 | `./policy_runner --script .../sim_patch/scripts/rl_rect.txt` | 25.05 m，终点误差 0.36% |
| Test 5 坡道 | terrain | Terminal 1 用 `-s scene_terrain.xml`，再 `./policy_runner 30 0.5 0 0` | 全速翻坡（z 0.36→0.65），误差 ~4% |

运动段终点指标分析：`python3 ~/CAPO-Go2-Odometry/sim_patch/scripts/eval_final.py`
（读 `/tmp/capo_eval.csv`，自动截取运动段，输出行程比/终点误差/RMSE）。

crawl 基线（gait_go2，对照用）：

| 测试 | 场景 | 命令 | 参考结果 |
|---|---|---|---|
| 静止漂移 | 平地 | `./gait_go2 stand 300` | 漂移 ≈0.03 m/300 s |
| 直线行走 | 平地 | `./gait_go2 walk 60 1.0` | GT 16.91 m / CAPO 15.97 m，误差 3.64% |
| 坡道 | terrain | `./gait_go2 walk 60 1.0` | 堵坡底打滑，CAPO 欠估 55~63%（负结果） |
| 原地转 | 平地 | `./gait_go2 turn 30` | 摔倒（位置控制无法实现，负结果） |

---

## 8. 故障排除（本机全部踩过的坑）

| 症状 | 原因 | 解决 |
|---|---|---|
| sim 启动即崩，`dds_writecdr_impl_common` 断言 | ROS Humble 的 libddsc 0.10.5 与 SDK2 的 0.10.2 混用 | `LD_LIBRARY_PATH` 前置 `/opt/unitree_robotics/lib`（脚本已内置） |
| ROS2 侧收不到 /lowstate（`尚未收到 /lowstate` 警告不止） | lo 无组播接口上 CycloneDDS 跨版本单播发现不稳定 | ① `ros2 daemon stop` 清缓存；② CYCLONEDDS_URI 加 `<Peers><Peer Address="localhost"/>`；③ 重启 sim（通常 1~3 次内恢复）；④ 先起 CAPO 再起 sim 成功率更高 |
| bag 报 `Output folder already exists` | ros2 bag 的 `-o` 目录不能预先存在 | 每次换新目录名 |
| bag 警告 `/lowstate has unknown type 'unitree_go/msg/LowState'` | bag 环境没 source cyclonedds_ws | 先 `source ~/unitree_ros2/cyclonedds_ws/install/setup.bash` |
| 行走数分钟后 sim 无日志静默退出 | 桥接线程与物理线程并发访问 mjData（官方代码缺陷） | 已由 4.2 的线程分工方案修复；若仍偶发，重启 sim 即可 |
| 机器人走几步摔倒 | trot 步态 / kp 过大 / 无幅值淡入 | 用 crawl 步态、kp=50/kd=3.5、保留 2 s ramp（gait_go2 默认配置即是） |
| 步态朝 -x 走（机头反方向） | 摆动/支撑时序与机体朝向相反 | gait_go2 中 `kWalkDir=-1` 已反转（勿改回） |
| `pkill -f <脚本名>` 杀掉了自己正在执行的终端 | 模式匹配到执行命令的 shell 自身命令行 | 清理脚本写成独立 .sh 文件再执行（见 `/tmp/cleanup_chain.sh` 思路） |
| 评估 CSV 混入上一轮数据 | evaluator 仅启动时截断 CSV | 每轮测试前重启 CAPO（`run_capo_sim.sh` 所在终端） |
| **RL：低速指令机器人不动** | 训练指令死区 |v|<0.2 全被置零 | 指令 ≥0.2 m/s；teleop 按 4 次 W 才开走 |
| **RL：恒定 wz 只转一点就停**（实测 ~0.56 rad） | 训练用 heading 闭环生成 wz，恒定值不在分布 | 用 `--script` 第 5 列绝对航向，或 teleop（内置闭环） |
| **RL：policy_runner 随机崩溃**（malloc 断言 / free invalid pointer） | libddsc 解析到 ROS 的 0.10.5、libddscxx 到 unitree 的 0.10.2，ABI 混载 | 运行前 export 完整 LD_LIBRARY_PATH（第 7 节三行） |
| **RL：下达指令后 5~24 s 才迈步** | 策略打破对称需要时间（正常现象） | 脚本段时长留足；teleop 按住 W 等 10 s |
| **RL：直线走到 x≈2.3 摔落** | 用了官方原版 scene.xml（含台阶） | 覆盖 `sim_patch/go2_scene/scene.xml`（平地版） |
| **RL：侧移（vy）走路带前向漂移** | 策略 vy 执行有前向耦合（~23%） | 导航用 heading+vx 为主；侧移只做微调 |

---

## 9. 已知限制（引用结论前必读）

```text
foot_force：  mj_contactForce 法向分量合成，无噪声/延迟/零漂；真机阈值不能设 0.0
IMU：         仿真 IMU 理想无噪，yaw RMSE 0.001~0.004 rad 在真机不可复现
关节：        编码器与 dq 理想化，真机 dq 噪声显著
时间戳：      adapter 按接收时刻打戳（非 lowstate 原始 tick），DDS 抖动 <5 ms 量级
RTF：         VMware 无 GPU 时 mj_step 实时率偶发 <1；sim 长跑偶发静默崩溃需重启
步态：        crawl 仅 ~0.19 m/s；RL 步态 0.5 m/s 可原地转/翻 8° 坡，
              楼梯未复现（侧移前向耦合 + 启动呆滞，三次尝试失败，见 RL_GAIT_PLAN §13）
失效模式：    打滑→CAPO 欠估；摔倒挣扎→CAPO 过估（均已定量记录，见交付报告）
```

CAPO 数学核心全程零改动——仿真与真机使用同一套运行代码，这正是本验证的意义所在。
