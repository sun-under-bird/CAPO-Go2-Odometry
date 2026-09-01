cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

# CAPO 仿真测试用步态控制器（阶段十八新增）
add_executable(gait_go2 gait_go2.cpp)
target_link_libraries(gait_go2  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)