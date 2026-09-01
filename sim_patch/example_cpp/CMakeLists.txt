cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

# CAPO 仿真测试用步态控制器（阶段十八新增）
add_executable(gait_go2 gait_go2.cpp)
target_link_libraries(gait_go2  unitree_sdk2)

# RL 步态策略推理节点（RL_GAIT_PLAN.md Phase B2，替代 gait_go2 的激励源）
# 依赖 onnxruntime（头文件 ~/onnxruntime_cpp/include，lib 为 pip onnxruntime 包内的 .so 软链）
add_executable(policy_runner policy_runner.cpp)
target_include_directories(policy_runner PRIVATE /home/yahboom/onnxruntime_cpp/include)
target_link_directories(policy_runner PRIVATE /home/yahboom/onnxruntime_cpp/lib)
# rpath 指向 onnxruntime 库目录，免运行时手工设 LD_LIBRARY_PATH
set_target_properties(policy_runner PROPERTIES BUILD_RPATH "/home/yahboom/onnxruntime_cpp/lib"
                                               INSTALL_RPATH "/home/yahboom/onnxruntime_cpp/lib")
target_link_libraries(policy_runner unitree_sdk2 onnxruntime)

SET(CMAKE_BUILD_TYPE Release)