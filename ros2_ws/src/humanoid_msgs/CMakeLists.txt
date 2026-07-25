cmake_minimum_required(VERSION 3.8)
project(humanoid_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Walk.action"
  "action/StandStill.action"
  "action/SitDown.action"
  "action/WaveHand.action"
  "action/Bow.action"
  "srv/Stop.srv"
  "srv/GetStatus.srv"
  "msg/BaseState.msg"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()