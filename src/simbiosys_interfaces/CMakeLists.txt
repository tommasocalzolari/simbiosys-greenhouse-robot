cmake_minimum_required(VERSION 3.8)
project(simbiosys_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Pixi/conda environments can expose Python to CMake's Python3 finder while
# rosidl_generator_py asks CMake's older Python finder for the same details.
# Seed both sets of cache variables so generated Python interfaces build
# reliably across native ROS and Pixi shells.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
set(Python_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH "" FORCE)
set(Python_INCLUDE_DIRS "${Python3_INCLUDE_DIRS}" CACHE PATH "" FORCE)
set(Python_LIBRARIES "${Python3_LIBRARIES}" CACHE FILEPATH "" FORCE)
set(Python_LIBRARY "${Python3_LIBRARIES}" CACHE FILEPATH "" FORCE)
set(Python_NumPy_INCLUDE_DIRS "${Python3_NumPy_INCLUDE_DIRS}" CACHE PATH "" FORCE)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/BedObservation.msg"
  "msg/BedRectangle.msg"
  "msg/BehaviorType.msg"
  "msg/DetectedTag.msg"
  "msg/FlowerData.msg"
  "msg/HarvestStatus.msg"
  "msg/MappingStatus.msg"
  "msg/MapMetadata.msg"
  "msg/NavigationStatus.msg"
  "msg/PlantAnalysis.msg"
  "msg/PlantHealth.msg"
  "msg/ScanProgress.msg"
  "msg/ScanPosition.msg"
  "msg/TaskStatus.msg"
)

set(srv_files
  "srv/CleanupMap.srv"
  "srv/DeleteBedRectangle.srv"
  "srv/GetHarvestEnabled.srv"
  "srv/LoadMapMetadata.srv"
  "srv/SaveMapWithMetadata.srv"
  "srv/SendNamedArmPose.srv"
  "srv/SetRobotMode.srv"
  "srv/SetHarvestEnabled.srv"
  "srv/SetScanPositions.srv"
  "srv/UpsertBedRectangle.srv"
)

set(action_files
  "action/ExecuteArmMotion.action"
  "action/ExecuteBehavior.action"
  "action/MoveToTarget.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
