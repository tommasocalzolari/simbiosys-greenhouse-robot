cmake_minimum_required(VERSION 3.8)
project(simbiosys_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/FlowerData.msg"
  "msg/PlantAnalysis.msg"
  "msg/TaskStatus.msg"
)

set(srv_files
  "srv/SendNamedArmPose.srv"
  "srv/SetRobotMode.srv"
)

set(action_files
  "action/MoveToTarget.action"
  "action/ExecuteArmMotion.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
