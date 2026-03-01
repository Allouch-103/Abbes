
cmake_minimum_required(VERSION 3.8)
project(humanoid_robot)

# Use C++17 (required by ROS2 Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings — catch bugs early
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ─────────────────────────────────────
# These are ROS2 packages installed on your system.
# find_package() locates their headers and libraries
# so our code can #include and link against them.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Build: imu_filter node ────────────────────────────────
# add_executable(name source_file)
#   Compiles src/imu_filter.cpp → binary called "imu_filter"
add_executable(imu_filter src/imu_filter.cpp)

# ament_target_dependencies(name dep1 dep2 ...)
#   Links the binary against ROS2 libraries.
#   Without this: "undefined reference to rclcpp::Node..."
ament_target_dependencies(imu_filter
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ── Install ───────────────────────────────────────────────
# Copies compiled binaries to install/humanoid_robot/lib/humanoid_robot/
# This is where 'ros2 run humanoid_robot imu_filter' looks for them.
install(TARGETS
  imu_filter
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (when we add them later)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Finalize ──────────────────────────────────────────────
# Required by all ament_cmake packages. Generates package
# metadata so ROS2 tools can discover this package.
ament_package()