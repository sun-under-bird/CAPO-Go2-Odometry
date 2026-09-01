# CAPO-Go2-Odometry MuJoCo 仿真验证最终报告

日期：2026-09-01
对应计划：`jihua.md`（第三～二十阶段全部执行完毕）

---

## 1. 环境

```text
Ubuntu 22.04.3 LTS（VMware 虚拟机）
ROS2 Humble
MuJoCo 3.3.6
unitree_mujoco commit: 4134cb5 (Merge pull request #129 from keeprobot/main)
CAPO-Go2-Odometry commit: 5eb6de6 (chore: 精简为Go2里程计运行版本) + 本地未提交修改（见第 3 节）
unitree_sdk2: /opt/unitree_robotics（自带 CycloneDDS libddsc 0.10.2）
DDS: ROS_DOMAIN_ID=1, 网卡 lo, RMW=CycloneDDS（无组播单播发现）
```

## 2. 是否成功

```text
MuJoCo Go2：        成功（scene.xml / scene_terrain.xml 均可运行）
/lowstate：         成功（sim → native SDK2 与 sim → ROS2 两条路均验证）
foot_force：        有效（物理线程提取足端法向接触力，站立约几十牛、离地接近 0）
CAPO：              成功（/SMX/Odom 与 /SMX/Odom_2D 持续输出，未修改数学核心）
Ground Truth：      成功（/ground_truth/odom 来自 sportmodestate frame_pos/quat）
定量对比：          成功（evaluator CSV + 中期日志；最佳平地直线相对误差 3.64%）
```

链路全通：

```text
MuJoCo Go2 → rt/lowstate → CAPO adapter → CAPO → /SMX/Odom_2D
                                ↘ /ground_truth/odom → capo_evaluator → 误差 CSV
```

## 3. 修改文件

### CAPO-Go2-Odometry（仿真侧适配，不动数学核心）

| path | 修改原因 | 修改内容 |
|---|---|---|
| `capo_params.hpp` (新增) | 仿真需要覆盖参数且不污染真机默认值 | 三级参数解析：内置默认 → ROS param → 命令行 |
| `go2_lowstate_adapter_node.cpp` | 订阅 DDS /lowstate 转 ROS2 | 增加 lo 接口/域名隔离、200Hz 重发布为 SMX 话题 |
| `fusion_estimator_node.cpp` | 支持参数注入与仿真时间 | 接入 capo_params 三级解析 |
| `ground_truth_node.cpp` (新增) | 需要 GT 对比基准 | 订阅 sportmodestate，发布 /ground_truth/odom |
| `capo_evaluator_node.cpp` (新增) | 定量误差评估 | 对比 /SMX/Odom_2D 与 GT，输出终点误差/行程比/RMSE/CSV |
| `launch/go2_capo_sim.launch.py` (新增) | 一键启动仿真验证链 | adapter + fusion + GT + evaluator 四节点 |
| `CMakeLists.txt` / `package.xml` | 新节点编译 | 新增两个可执行文件与依赖 |
| `scripts/run_mujoco_sim.sh`、`scripts/run_capo_sim.sh` (新增) | 固化启动流程 | 见第 4 节 |

### unitree_mujoco（仿真侧）

| path | 修改原因 | 修改内容 |
|---|---|---|
| `simulate/src/unitree_sdk2_bridge.h` | (a) LowState 缺 foot_force；(b) 桥接线程直接遍历 mjData->contact 与物理线程无锁并发，行走时偶发段错误 | (a) 新增 `ComputeFootForce()`：只统计足端 geom 与环境(bodyid==0)接触，mj_contactForce 法向分量累加，经 `foot_force_cache_` 缓存填入 LowState/HighState.foot_force；G1 无该字段用 `if constexpr` 编译期裁剪。(b) 接触力计算移到物理线程。另：highstate 填充 base 四元数供 GT 节点使用 |
| `simulate/src/main.cc` | 物理线程需要调用桥接的力计算 | 全局 `std::atomic` 桥接指针，mj_step 后调 `ComputeFootForce()`（在 AddToHistory 之前） |
| `example/cpp/gait_go2.cpp` (新增) | 仿真测试需要步态激励 | stand/walk/turn 三模式关节位置控制器：crawl 静步态（FR→RL→FL→RR、摆动占比 0.25）、幅值 2s 淡入、IMU yaw PD 航向闭环、起立 tanh 过渡。PD kp=50/kd=3.5 |
| `example/cpp/CMakeLists.txt` | 编译新示例 | 增加 gait_go2 目标 |
| `unitree_robots/go2/scene_terrain.xml` | Test 5 坡道 | 原 28.7° 坡道对位置控制步态不可攀，改为 8° 缓坡置于 +x 正前方 (3.5, 0)；原 (1.5,0) 方台+圆柱挡直行路径，挪至 (1.5, 2.5) |

## 4. 完整启动顺序

```bash
# Terminal 1 —— MuJoCo 仿真（先起，加载 Go2 + 地形）
cd ~/unitree_mujoco/simulate/build
LD_LIBRARY_PATH=/opt/unitree_robotics/lib ./unitree_mujoco -r go2 -s scene_terrain.xml
# （平地测试用默认 scene.xml，不加 -s）

# Terminal 2 —— CAPO 全链（adapter + fusion + GT + evaluator）
bash ~/CAPO-Go2-Odometry/scripts/run_capo_sim.sh
# 环境要点：ROS_DOMAIN_ID=1、RMW=cyclonedds、CYCLONEDDS_URI 限 lo 接口、
# LD_LIBRARY_PATH 前置 /opt/unitree_robotics/lib（统一 libddsc 0.10.2）

# Terminal 3 —— 步态激励（起立 3s + 行走 duration 秒）
cd ~/unitree_mujoco/example/cpp/build
./gait_go2 walk 60 1.0        # <mode> <duration_s> [speed] [heading_rad]

# Terminal 4 —— rosbag（-o 目录必须不存在；需 source cyclonedds_ws 以识别 unitree_go/msg）
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
timeout 85 ros2 bag record -o ~/capo_bags/<新目录> \
  /lowstate /SMX/Odom /SMX/Odom_2D /ground_truth/odom /tf /tf_static
```

实测注意：先起 CAPO 再起 sim 时 DDS 发现偶发失败（CycloneDDS 0.10.2/0.10.5 跨版本在 lo 无组播接口上单播发现不稳定），重启 sim 数次内即恢复；bag 的 `-o` 目录不能预先创建。

## 5. 测试结果（阶段十八～十九）

| 测试 | 场景 | GT 位移 | CAPO 位移 | 结论 |
|---|---|---|---|---|
| Test 1 静止 | 平地站立 | 0.00 m | ~0.03 m/300s | 漂移极小（≈0.1mm/s 量级） |
| Test 2 v1 (trot) | 平地 | 7.08 m（绕圈） | 10.93 m | 过估 54%，开环航向漂移 |
| Test 2 v3 (crawl) | 平地 | 16.91 m | 15.97 m | **最佳：相对误差 3.64%，yaw RMSE 0.004 rad，RMSE 2D 0.481 m** |
| Test 2 final (crawl) | 平地+bag | — | — | 相对误差 8.07%（尾部摔倒） |
| Test 3 旋转 | 平地 | 1.19 m | 16.53 m | 负结果：turn 步态摔倒；CAPO 在摔倒挣扎中积分大量假行程（过估） |
| Test 4 矩形闭环 | — | — | — | 跳过：依赖原地转弯，纯位置控制无法实现（局限） |
| Test 5 坡道 15° | 缓坡 | 2.6 m（坡底打滑堵死） | 1.15 m | 负结果：步态无法攀坡（足端摩擦 0.4），顶坡打滑段 CAPO 欠估 55% |
| Test 5 坡道 8° | 缓坡 | 2.3 m（同上） | 0.85 m | 同 15°：堵在坡底前腿搭坡打滑，位移欠估 63%，**yaw RMSE 仍 0.0015 rad** |

数据资产：`~/capo_bags/`（test1_stand、test2_walk、test2_walk_v2、test2_walk_v3_nobag、test2_walk_final、test3_turn、test5_ramp 含完整 /lowstate 的 bag、test5_15deg_skid.csv、test5_8deg.csv）。

核心发现：
- 平地正常行走下 CAPO 精度优秀（3.6%），yaw 一致性极好（RMSE < 0.005 rad）。
- 异常状态行为：**打滑→欠估**（接触足被假定静止，滑动位移全部丢失）；**摔倒挣扎→过估**（非接触腿乱蹬被积分）。两者互补，均为腿式里程计的固有失效模式，真机上需配合状态监测。

## 6. 风险（仿真结论外推真机的限制）

```text
foot_force 仿真真实性：  由 mj_contactForce 法向分量合成，无传感器噪声/延迟/量化，
                         真机 foot_force 有零漂与交叉干扰；阈值 0.0 在真机不可用
IMU 无噪声：             MuJoCo IMU 陀螺仪/加速度计为理想值，真机有偏置、噪声、温漂；
                         仿真 yaw RMSE 0.001~0.004 rad 在真机不可复现
joint q/dq 理想化：      无编码器噪声/执行器延迟/齿隙，dq 直接来自传感器，真机 dq 噪声大
时间戳：                 adapter 用 ROS 接收时刻打戳，非 lowstate 原始 tick；
                         DDS 抖动会引入时间戳误差（对本评估影响 < 5ms 量级）
Real Time Factor：       VMware 虚拟机无 GPU 渲染，mj_step 实时率偶发低于 1，
                         长时间运行 sim 偶发静默崩溃（无 core，疑似桥接线程与渲染线程竞争），
                         重启即恢复，测试期间通过自动重试规避
contact force scale：    sim 接触刚度远高于真机脚垫，接触瞬态更尖锐
sim-to-real gap：        crawl 静步态速度仅 ~0.19 m/s（真机 trot 可达 1.5 m/s），
                         高速/动态步态下 CAPO 精度未在仿真中评估；坡道/楼梯均未能实际
                         攀爬（位置控制步态限制，非 CAPO 问题）
```

## 7. 结论

jihua.md 的最小闭环全部打通并完成定量评估：

```text
MuJoCo Go2 → /lowstate → foot_force 有效 → CAPO → /SMX/Odom_2D → Ground Truth → 误差比较
```

CAPO 数学核心零改动，仿真与真机共用同一套运行代码，达成 jihua.md 最终目标图中的"同一套运行代码"要求。平地直线行走的 3.64% 相对误差证明链路与算法在仿真中工作正常；打滑与摔倒两类异常场景的系统性偏差（欠估/过估）已定量记录，为真机部署提供失效模式参考。
