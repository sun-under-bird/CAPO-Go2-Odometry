# Marsdog 新 URDF 里程计优化

2026-11-09（Asia/Shanghai）。远端基线提交为 `82fc18d`，本地优化分支为 `marsdog-urdf-optimization`。

机械几何、质量、惯量和 IMU 安装位置采用 `/home/sunbird/marsdog_v17_source/quadruped.urdf`；该目录没有策略，因此按用户授权复用原 SDK 的 ONNX、运行器、PD 和电机转子惯量。14 个电机及两个活动腰关节均保留。

## 修正的问题

1. 新 URDF 导入的姿态传感器使用 MuJoCo `XBODY`（连杆坐标系），修正之前误用 `BODY`（惯量主轴系）造成的错误姿态输入；该问题曾使初次混合测试跌倒。修正后同一模型和策略完成全部测试。
2. 足端速度加入机身角速度项，腰部与腿部关节速度同时参与。
3. 球形脚掌持续支撑时，根据脚部总角速度积分几何接触位置的滚动量；重新落脚时建立新锚点。
4. 对临界载荷、与预测速度不一致的支撑脚降低融合权重。当前采用载荷裕量和 0.15 m/s 尺度的 Cauchy 权重，属于接触置信度启发式，尚未完成完整动力学补偿。
5. Marsdog 的预测矩阵按传感器时间戳计算间隔，修正固定 5 ms 与实际 2 ms 不一致；原 Go2 预设保持原参数。共享矩阵工作区每次按当前实例重载，避免配置相互影响。

## 十米方形

两版使用相同的新 URDF 与旧策略，98,090 帧输入文件完全一致，真实轨迹逐帧一致。全程位置 RMSE 从 **0.53814 m 降到 0.15033 m（−72.06%）**，最大误差从 **0.81736 m 降到 0.23672 m（−71.04%）**。每边距离由低估约 5.48%～6.03%，变为高估约 0.82%～1.35%。

终点差由 5.23 cm 变成 5.60 cm，没有改善；高度 RMSE 约 9.49 mm，也基本未改善。闭环存在误差抵消，报告同时保留沿途误差和终点误差。

[完整方形对比报告](reports/marsdog_optimization/report.md) · [轨迹图](reports/marsdog_optimization/comparison.png) · [PDF](reports/marsdog_optimization/comparison.pdf)

## 30 秒回归验证

每组前后对照的传感器输入 SHA-256 一致。速度列为指令值，实际行程按仿真真值计算。

| 场景 | 实际累计行程 | 基线终点误差 | 优化终点误差 |
|---|---:|---:|---:|
| 静止 | 0.112 m | 0.36 cm | 0.23 cm |
| 前进 0.15 m/s | 4.300 m | 25.78 cm | 6.01 cm |
| 前进 0.25 m/s | 7.215 m | 38.85 cm | 11.52 cm |
| 前进 0.35 m/s | 9.651 m | 50.15 cm | 17.10 cm |
| 倒退 −0.20 m/s | 4.025 m | 18.06 cm | 3.99 cm |
| 转向 0.30 rad/s | 0.995 m | 3.28 cm | 2.12 cm |

## 直行消融对照

同一段 7.215 m 真实运动，逐项加入修改。目录名 `measured-optimized` 是修正预测时间间隔之前的中间版本，配置中没有 `point_timestamp_prediction`；相当于当前 `weighted` 配置。最终版本为 `measured-final`。

| 累积修改 | 终点误差 | 位置 RMSE | 速度 RMSE |
|---|---:|---:|---:|
| 基线 | 0.3885 m | 0.2227 m | 0.0654 m/s |
| 加机身角速度 | 0.3706 m | 0.2122 m | 0.0562 m/s |
| 再加脚掌滚动 | 0.2788 m | 0.1610 m | 0.0562 m/s |
| 再加接触加权 | 0.2681 m | 0.1552 m | 0.0136 m/s |
| 再修正预测时间间隔 | 0.1152 m | 0.0675 m | 0.0127 m/s |

12 项单元测试通过：旧/新模型雅可比和腰部运动、机身转动与球形脚滚动、URDF 连杆/惯量/关节、IMU 坐标系、200/500 Hz 及不规则时间戳预测、同时间戳/时间跳变与实例隔离。ROS Release 构建通过。使用同一新编译核心的 Go2 5 秒直行回归通过，1.046 m 行程终点误差 2.904 cm，与基线一致。

## 复现

```bash
cd /home/sunbird/CAPO-Go2-Odometry
scripts/setup_marsdog_sim.sh
scripts/run_marsdog_sim.sh --scenario all --duration 30
scripts/run_marsdog_sim.sh --scenario square --side-length 10 --vx 0.25
```

默认新 URDF + 原 SDK 策略 + `optimized`。`--urdf` 指定模型，`--sdk` 指定策略运行包；`--legacy-sdk-model --estimator-profile baseline` 重现旧训练模型的适配基线。仅比较当前新模型下的旧算法时，使用 `--estimator-profile baseline`。消融配置依次为 `baseline`、`angular`、`motion`、`weighted`、`optimized`。带窗口和按实际时间运行，追加 `--viewer --realtime`。

```bash
/home/sunbird/capo_sim_venv/bin/python -B -m unittest discover -s tests -p "test_*.py" -v
/home/sunbird/capo_sim_venv/bin/python -B integration/report_marsdog_comparison.py .runtime/marsdog/measured-square-baseline/square .runtime/marsdog/measured-square-final/square reports/marsdog_optimization
```

每次新实验保存 `model.mjb` 及其哈希、URDF/网格/策略哈希、估计器源码哈希、配置、14 轴与 IMU 输入及里程计结果。报告数值见 [validation.json](reports/marsdog_optimization/validation.json)。

仍需验证真机力矩标定、时间同步、IMU 噪声与延迟；当前成绩仅代表平地理想传感器仿真，残余滑动与约 1% 的距离偏差尚未消除。
