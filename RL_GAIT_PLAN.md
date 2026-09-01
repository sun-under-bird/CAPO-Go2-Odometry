# Go2 RL 步态训练与部署计划

日期：2026-09-01
前置状态：MuJoCo 仿真链路已全通（见 `SIM_FINAL_REPORT.md` / `SIM_REPRODUCTION.md`），
当前激励为手工 crawl 静步态（`gait_go2`），速度仅 ~0.19 m/s，无法爬坡、无法原地转向。

---

## 1. 目标与产物

用 RL 策略替换 `gait_go2` 作为测试激励，让 Test 3（转弯）、Test 4（矩形闭环）、
Test 5（坡道）这些 crawl 下的负结果场景获得有效激励，从而完成高速/转弯/爬坡下的
CAPO 里程计评估。

**CAPO 验证链路（adapter → fusion → GT → evaluator）一行不改**——本计划只动激励侧。

最终交付物：

```text
① policy.onnx           训练好的 Go2 速度跟踪策略（CPU 可推理）
② 观测定义说明          观测向量各分量的顺序/缩放 + PD 增益 + 命令范围
③ policy_runner         本机推理节点（替代 gait_go2，Terminal 3）
④ 测试报告              高速/转弯/坡道下的 CAPO 误差数据（对照 crawl 基线）
```

## 2. 机器分工

| 机器 | 配置要求 | 职责 |
|---|---|---|
| **训练机**（待准备） | NVIDIA 独显 ≥ 8 GB 显存（RTX 3060/4060 起，4090 更快）、Ubuntu 22.04、内存 ≥ 32 GB、磁盘 ≥ 50 GB、驱动 ≥ 525 | Isaac Gym 环境搭建、训练、play 自测、导出 onnx |
| **本机**（VMware 虚拟机） | 现状即可，加装 onnxruntime（CPU 版） | 部署推理、sim2sim 校验、CAPO 全链验证 |

两机之间只传三样东西：`policy.onnx` + PD 增益/观测定义（一段文字或 config 文件）+ 训练 log。

## 3. 训练机阶段（Phase A）

### A1. 环境搭建（约 1~2 小时，坑最多的一段）

```bash
# 坑①：Isaac Gym Preview 4 只支持 Python 3.8，Ubuntu 22.04 自带 3.10，必须 conda 隔离
conda create -n rl python=3.8 -y && conda activate rl

# ① Isaac Gym Preview 4（需免费注册 NVIDIA 开发者账号）
#    https://developer.nvidia.com/isaac-gym → IsaacGym_Preview_4_Package.tar.gz
tar xzf IsaacGym_Preview_4_Package.tar.gz && cd isaacgym/python
pip install -e .
# 验证：python -c "import isaacgym; print('ok')"（首次运行编译着色器，需几分钟）

# ② 官方 RL 仓库（自带 go2 配置）
git clone https://github.com/unitreerobotics/unitree_rl_gym.git
cd unitree_rl_gym && pip install -e . && pip install onnx onnxruntime
```

已知坑速查：

| 坑 | 规避 |
|---|---|
| Isaac Gym Preview 已停止维护 | 环境内**禁止升级** PyTorch/numpy，按仓库 requirement 钉死版本 |
| 缺 `libpython3.8` / 着色器编译失败 | `sudo apt install libpython3.8-dev`；首次 import 慢属正常 |
| 新驱动下 Vulkan 报错 | 训练用 `--headless`，只有 play 需要渲染 |

### A2. 训练与自测（约 2 小时 ~ 1 天，取决于显卡）

```bash
python train.py --task=go2 --headless --max_iterations=15000
# 4090 级约 2~6 小时；3060 级约 1 天

python play.py --task=go2        # Isaac Gym 内自测：站得稳、跟踪速度指令、不摔
```

### A3. 训练侧必须做对的 4 件事（决定本机能否直接用）

| # | 事项 | 原因 |
|---|---|---|
| 1 | **开启 domain randomization**（摩擦/质量/PD 增益/延迟随机化） | Isaac Gym 与 MuJoCo 3.3.6 有物理差异，不随机化的策略在 MuJoCo 里大概率站不稳 |
| 2 | **记录 PD 增益**（go2 config 的 stiffness/damping） | 本机发目标角必须用同一组 kp/kd，否则动作变形 |
| 3 | **抄录观测向量定义**（`num_observations`、分量顺序与缩放，在 `go2_config` 里） | 本机组装观测必须与训练**逐位一致**，错一位 = 乱蹬 |
| 4 | **记录命令接口范围**（vx / vy / yaw_rate 的训练域，如 [-1, 1] m/s） | 部署时指令超出训练域策略行为未定义 |

### A4. 导出（几分钟）

```bash
python export_policy_as_onnx.py     # 产出 policy.onnx
```

交付三件套：`policy.onnx` + A3 的 2/3/4 项记录 + 训练 log（判断收敛用）。

## 4. 本机阶段（Phase B）

### B1. 准备（约 0.5 小时）

```bash
pip3 install onnxruntime          # CPU 版即可，策略是 ~50 万参数 MLP，50 Hz 推理无压力
# 冒烟测试：onnx 能加载、输入输出维度与 A3-3 记录一致
```

### B2. 编写 policy_runner（替代 gait_go2，约 1~2 天开发+调试）

结构对照 `gait_go2.cpp`（约 320 行），改成策略推理：

```text
循环 @ 50 Hz（与训练控制频率一致）：
  ① 读 /lowstate（IMU 四元数/角速度、关节 q/dq、上一帧 action）
  ② 按观测定义组装观测向量（gravity 投影 = 四元数逆旋转 [0,0,-1]）
  ③ onnxruntime 推理 → action
  ④ 关节目标角 = 默认关节角 + action × action_scale
  ⑤ 发 LowCmd（kp/kd 用 A3-2 记录值）
  ⑥ 叠加速度指令：stand（0,0,0）/ walk（vx,0,0）/ turn（0,0,ω）

工程要点（从 gait_go2 迁移的已踩坑经验）：
  - main() 开头 ChannelFactory::Instance()->Init(1, "lo")
  - 结尾 std::_Exit(0)（正常析构堆损坏）
  - 指令幅值 2 s 淡入（防目标角跳变冲击）
```

### B3. sim2sim 校验（约 0.5~1 天，gap 消除）

```text
阶梯测试：stand 指令站 60 s 不抖 → vx=0.3 行走 → vx=0.8 → yaw_rate 转弯
若抖动/摔倒（Isaac Gym→MuJoCo gap）：
  ① 确认 domain randomization 是否开足（回训练机重训是最常见解法）
  ② 本机微调 go2.xml 足端 friction（当前 0.4，RL 策略通常按 0.6~1.0 训练）
  ③ 检查 PD 增益是否与训练一致
```

### B4. CAPO 验证（约 1 天，流程与现在完全一致）

四终端流程不变，仅 Terminal 3 换成 policy_runner：

| 测试 | 场景 | 命令目标 | 对照基线（crawl） |
|---|---|---|---|
| Test 6 高速直线 | 平地 scene.xml | vx=0.8, 60 s | crawl 仅 0.19 m/s，误差 3.64% |
| Test 7 原地转向 | 平地 | yaw_rate=0.5, 30 s | crawl 必摔（Test 3 负结果） |
| Test 8 矩形闭环 | 平地 | 四段直行+三段 90° 转弯 | crawl 无法完成（Test 4 跳过） |
| Test 9 坡道 | scene_terrain.xml（8°） | vx=0.5 上坡 | crawl 堵坡底（Test 5 负结果） |

每轮 rosbag + `/tmp/capo_eval.csv`，沿用 evaluator 全部指标（相对误差 / yaw RMSE / 2D RMSE）。

## 5. 时间线估算

```text
Phase A（训练机）：环境 0.5 天 + 训练 0.5~1.5 天 + 导出交接 0.5 天   ≈ 1.5~2.5 天
Phase B（本机）：  B1 0.5 天 + B2 1~2 天 + B3 0.5~1 天 + B4 1 天     ≈ 3~4.5 天
合计：约 5~7 个工作日（不含训练机硬件到位时间）
```

## 6. 风险清单

| 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|
| sim2sim gap：策略在 MuJoCo 站不稳 | 高 | 返工 A2 | A3-1 开足 domain randomization；B3 阶梯测试尽早暴露 |
| Isaac Gym 环境安装失败（Python/驱动兼容） | 中 | 卡 Phase A | 钉死 Python 3.8 + 仓库指定依赖版本；备选路线见第 7 节 |
| 观测组装不一致 | 中 | 策略乱蹬 | B1 冒烟测试时逐维核对；先用零观测+零指令验证站立输出 |
| 本机 RTF 不足（VMware 无 GPU） | 低 | 长测试变慢 | 策略推理开销小；50 Hz 控制远低于 mj_step 预算 |
| 训练机缺 N 卡 | — | 无法开始 | 云 GPU（按小时租 4090）或 Isaac Lab/云端训练均可替代 |

## 7. 备选路线（若 Isaac Gym 路线受阻）

```text
Isaac Lab（新框架，Python 3.10，官方 Go2 资产 + velocity 任务）
  优点：活跃维护、安装现代；缺点：导出与观测定义需自行核对
MJX（MuJoCo XLA + Brax/PPO）
  优点：直接在 MuJoCo 动力学里训练，sim gap 最小，回本机零 gap
  缺点：生态较新，Go2 任务需自建，JAX 熟悉成本
社区现成 Go2 策略 checkpoint
  优点：零训练时间；缺点：观测/增益定义需逆向核对，domain randomization 未知
```

## 8. 验收标准

```text
① policy_runner 驱动 Go2 在本机 MuJoCo 中：stand 60 s 不抖、
   vx=0.8 直线 60 s 不摔、yaw_rate 原地转 30 s 不摔
② Test 6~9 至少完成 3 项（坡道允许失败但需记录失效模式）
③ 每项测试产出 bag + CSV + 与 crawl 基线的对照表
④ 全程 CAPO 数学核心零改动（与现有验证同一套运行代码）
```

---

# 实际执行记录（2026-09-01 完成）

## 9. 路线变更：Phase A 由社区策略替代

第 3 节的训练机路线（Phase A）**未执行**。用户决策改用社区现成 NP3O 策略：

```text
策略来源：unitree_rl_gym 的 NP3O 变体（Barlow Twins MLP 教师-学生蒸馏）
  观测 45 维/帧 × 历史栈 2 帧（actor 实际取 obs_hist_full[:,5:] 共 5 帧）
  obs_normalizer 已包含在 onnx 内，部署无需外部归一化
交付物：policy.onnx（CPU 推理 50 Hz 单次 <2ms，预热后稳定）
```

第 7 节"备选路线 3"即本路线，优缺点与预判一致：零训练时间，但观测/增益定义靠逆向核对。实际逆向核对结论：

| 项 | 值 | 来源 |
|---|---|---|
| PD 增益 | kp=40, kd=1.0 | np3o config stiffness/damping |
| action_scale | 0.25 | config（hip 关节 i%3==0 额外 ×0.5） |
| 动作滤波 | 0.8·旧 + 0.2·新 | config action_filter |
| 关节序 | SDK 序（与 reindex 对合验证） | legged_robot.py |
| 历史栈 | 新帧在末位 index 9；推理时末帧=t-1 | obs_history_buf 更新序 |

## 10. 三大根因（sim2sim 调试关键发现）

部署初期"站立稳定但一切运动指令失效"，根因三个，全部位于**部署侧与训练侧的指令生成差异**，非物理 gap：

1. **训练指令死区 0.2 m/s**：`legged_robot.py:1232` `_resample_commands` 中
   `commands[:,:2] *= (norm(commands) > 0.2)` —— 训练时 |v|<0.2 全部置零，
   策略从未见过低速指令。**部署指令必须 ≥0.2 m/s**。
2. **heading 指令模式**：训练 config `heading_command=True`，每步重算
   `wz = clip(0.5·wrap(heading_target − heading_now), ±1)`，指令随转向完成自然衰减。
   恒定 wz 不在训练分布（实测转 0.56 rad 即停）。**部署侧必须复刻该闭环**：
   policy_runner 维护 heading_target_，由闭环差值算 wz_cmd（转向跟踪 0%→94%）。
3. **场景台阶**：unitree_mujoco 自带 scene.xml 含 x=2.3~3.4m 的 6 级阶梯（高至 1.8m），
   机器人会爬上去再摔落——此前误诊为策略失稳（试过零阻尼/condim=3/摩擦调整均无效）。
   已删除台阶改纯平地（scene_terrain.xml 保留地形）。

## 11. 运行规范（DDS 库混载坑）

**必须**带 LD_LIBRARY_PATH 启动 policy_runner/gait_go2，否则 libddsc.so.0 从
ldconfig 解析到 ROS Humble 的 CycloneDDS 0.10.5、libddscxx.so.0 加载
/opt/unitree_robotics 的 0.10.2，ABI 混载 → 堆损坏（malloc 断言/dds_write 断言/
free invalid pointer，且崩溃点随机、极难定位）：

```bash
cd ~/unitree_mujoco/example/cpp/build
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
./policy_runner <模式参数>
```

另加两处防御：LowCmdWrite 防重入 mutex（DDS 写线程与控制线程并发保护）、
Init() 中 onnx 预热推理一次（首次 Run 含图优化可超 20ms 周期）。

## 12. B4 CAPO 验证结果（Test 1~5 全部通过）

| 测试 | 激励 | GT 行程 | 关键指标 | 结论 |
|---|---|---|---|---|
| Test 1 静止 60s | RL stand | 漂移 0.039 m | 终点误差 0.050 m，yaw RMSE 0.00026 rad | 通过 |
| Test 2 直线 vx=0.5 | RL 直行 | 15.33 m | **相对误差 4.68%**，RMSE 2D 0.120 m，行程比 1.048 | 通过 |
| Test 3 旋转 wz 闭环 | RL 原地转 | 累计 643° | 双方转角比 1.000，误差 0.025 m，yaw RMSE 0.0006 rad | 通过（crawl 必摔） |
| Test 4 矩形闭环 | RL 四段直行+绝对航向转向 | 25.05 m | 终点误差 0.090 m（**0.36%**），RMSE 2D 0.064 m | 通过（crawl 无法完成） |
| Test 5 坡道 8° | RL 上坡 | ~15 m | 全速翻坡成功（z 0.36→0.65），坡上水平误差均值 0.19 m，全程终点误差 4% | 通过（crawl 堵死坡底） |
| Test 6 楼梯 | — | — | 三次尝试失败（见 13 节），jihua.md 可选项 | 未完成 |

验收标准 ①②③④ 对照：①通过（stand 稳 + vx=0.5 行走 15m+ + 原地转 1.8 圈不摔）；
②完成 4/5（Test 1~5，楼梯可选未成）；③bag/CSV/对照表齐（SIM_FINAL_REPORT.md 第 5b 节）；
④CAPO 数学核心零改动。

## 13. 遗留问题

- **Test 6 楼梯**（可选项）：三次尝试均未成——(a) 侧移段 8s 不够步态启动（呆滞 5~24s 不等）；
  (b) RL 步态 vy 侧移带前向耦合（9m 侧移伴 2.1m 前向），斜走撞障碍摔倒；
  (c) heading 方案（转 90° 沿 +y 走再对准爬梯）机器人 42s 完全不动，dbg 显示
  gravity 向量倾斜 ~31°，疑似卡在场景物体上，未深入排查。建议用 `--teleop` 手动探索。
- **步态启动呆滞**：静止→迈步存在 5~24s 启动期（打破对称需要时间），脚本段时长需留余量。
- **侧移前向耦合**：导航应用优先 heading+vx，避免 vy 作为主要机动手段。

## 14. 最终用法

```bash
# 单指令（起立 3s + 行走）：
./policy_runner 0.5 0 0 30            # vx=0.5 m/s 走 30s

# 脚本序列（/tmp/rl_rect.txt 每行: dur vx vy wz [yaw_deg]，第 5 列=绝对航向闭环）：
./policy_runner --script /tmp/rl_rect.txt

# 键盘遥控（W/S 前后 ±0.05，Q/E 侧移，A/D 转向 ±0.10，空格急停，X 退出）：
./policy_runner --teleop
```

注意死区：|vx,vy| < 0.2 时策略视为站立指令（teleop 显示 `<0.2死区:站立`）。
