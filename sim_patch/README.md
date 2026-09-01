# sim_patch —— unitree_mujoco 侧修改归档

对 `~/unitree_mujoco` 的全部修改（CAPO 数学核心零改动，只动仿真/激励侧）。
拷回方法：同名文件直接覆盖到 unitree_mujoco 对应路径后重新编译。

```text
unitree_sdk2_bridge.h   → simulate/src/unitree_sdk2_bridge.h
                          新增 ComputeFootForce()（物理线程提足端法向接触力，
                          经 foot_force_cache_ 填 LowState.foot_force）；
                          highstate 填充 base 四元数供 GT 节点使用
main.cc                 → simulate/src/main.cc
                          全局 atomic 桥接指针，mj_step 后调 ComputeFootForce()
scene_terrain.xml       → unitree_robots/go2/scene_terrain.xml
                          Test 5 坡道：8° 缓坡置于 (3.5,0)，方台/圆柱挪至 (1.5,2.5)
go2_scene/scene.xml     → unitree_robots/go2/scene.xml
                          平地测试场景：删除原版挡板+阶梯（RL 行走会爬 1.8m
                          阶梯后摔落，干扰直线评测；见 RL_GAIT_PLAN.md §10-3）

example_cpp/
  gait_go2.cpp          → example/cpp/    crawl 静步态激励（基线，~0.19 m/s）
  policy_runner.cpp     → example/cpp/    RL 步态激励（NP3O onnx 推理 50Hz，
                          观测组装/heading 闭环/--script/--teleop，
                          用法见 RL_GAIT_PLAN.md §14）
  CMakeLists.txt        → example/cpp/    增加 gait_go2、policy_runner 目标
                          （后者依赖 ~/onnxruntime_cpp 的 onnxruntime）

models/
  go2_policy.onnx       NP3O 行走策略（948KB，actor + obs 归一化一体）。
                          默认读取路径 ~/np3o/go2_policy.onnx，或
                          policy_runner 最后一个参数指定任意路径。
                          来源：zeonsunlightyu/LocomotionWithNP3O 的
                          model_10000.pt，经 scripts/export_onnx.py 提取导出

scripts/
  export_onnx.py        从 NP3O checkpoint 重新导出 onnx（torch 环境用）
  eval_final.py         RL 测试运动段终点误差分析（读 /tmp/capo_eval.csv，
                        输出行程比/终点误差/RMSE 2D/yaw RMSE）
  rl_rect.txt           Test 4 矩形脚本（绝对航向模式，5×13s）
  rl_stairs.txt         Test 6 楼梯尝试脚本（未成功，见 RL_GAIT_PLAN.md §13）
```

（模型已在 `models/go2_policy.onnx`，随仓库分发。）

## 运行规范（重要）

policy_runner / gait_go2 **必须**带 LD_LIBRARY_PATH 启动，否则 libddsc.so.0
解析到 ROS Humble 的 CycloneDDS 0.10.5、libddscxx.so.0 加载 unitree 0.10.2，
ABI 混载导致随机堆损坏（详见 RL_GAIT_PLAN.md §11）：

```bash
cd ~/unitree_mujoco/example/cpp/build
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
./policy_runner --teleop
```
