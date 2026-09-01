cmake_minimum_required(VERSION 3.5)
project(fusion_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(unitree_go REQUIRED)

file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/FusionEstimator/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FusionEstimator/*.c"
)

add_executable(fusion_estimator_node
  fusion_estimator_node.cpp
  ${ESTIMATOR_SOURCES}
)

add_executable(go2_lowstate_adapter_node
  go2_lowstate_adapter_node.cpp
)

# ==== 仿真验证新增（阶段十六/十七）：Ground Truth 发布与误差评估节点 ====
add_executable(ground_truth_node
  ground_truth_node.cpp
)

add_executable(capo_evaluator_node
  capo_evaluator_node.cpp
)

target_include_directories(fusion_estimator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/FusionEstimator
)

ament_target_dependencies(fusion_estimator_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

ament_target_dependencies(go2_lowstate_adapter_node
  rclcpp
  sensor_msgs
  std_msgs
  unitree_go
  ament_index_cpp
)

ament_target_dependencies(ground_truth_node
  rclcpp
  nav_msgs
  unitree_go
)

ament_target_dependencies(capo_evaluator_node
  rclcpp
  nav_msgs
)

install(TARGETS
  fusion_estimator_node
  go2_lowstate_adapter_node
  ground_truth_node
  capo_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
