cmake_minimum_required(VERSION 3.8)
project(humanoid_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Walk.action"
  "action/StandStill.action"
  "action/SitDown.action"
  "action/WaveHand.action"
  "action/Bow.action"
  "srv/Stop.srv"
  "srv/GetStatus.srv"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()