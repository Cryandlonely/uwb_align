cmake_minimum_required(VERSION 3.10)
project(uwb_align)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- 头文件 ----
include_directories(include)

# ---- UWB 串口解析库 ----
add_library(uwb_serial SHARED
  src/uwb_serial.cpp
)
target_include_directories(uwb_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uwb_serial pthread)

# ---- UWB 位姿发布节点（三边定位 → PoseWithCovarianceStamped）----
add_executable(uwb_pose_node
  src/main_pose.cpp
  src/uwb_pose_node.cpp
)
target_link_libraries(uwb_pose_node uwb_serial)
ament_target_dependencies(uwb_pose_node
  rclcpp
  geometry_msgs
  std_msgs
)

# ---- 对准控制节点（订阅 EKF 输出 → /cmd_vel）----
add_executable(uwb_align_controller
  src/main_controller.cpp
  src/uwb_align_controller.cpp
)
ament_target_dependencies(uwb_align_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# ---- 安装 ----
install(TARGETS uwb_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS uwb_pose_node uwb_align_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
