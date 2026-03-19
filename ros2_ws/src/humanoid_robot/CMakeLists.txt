cmake_minimum_required(VERSION 3.8)
project(humanoid_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ─────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ── imu_filter node ───────────────────────────────────────
add_executable(imu_filter src/imu_filter.cpp)
ament_target_dependencies(imu_filter
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ── balance_controller node ───────────────────────────────
add_executable(balance_controller src/balance_controller.cpp)
ament_target_dependencies(balance_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rcl_interfaces
)

# ── Install binaries ──────────────────────────────────────
install(TARGETS
  imu_filter
  balance_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config files ──────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Install launch files ──────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()