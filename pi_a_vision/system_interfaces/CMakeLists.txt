cmake_minimum_required(VERSION 3.8)
project(system_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
# ROS2 message generation
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemMode.msg"
  "msg/Detect.msg"
  "msg/EndSignal.msg"
  "msg/TriggerCmd.msg"
  "msg/StartSignal.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

