cmake_minimum_required(VERSION 3.8)
project(warehouse_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Utterance.msg"
  "msg/PlanStep.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/ArucoMarker.msg"
  "msg/ArucoMarkerArray.msg"
  "msg/TaskStatus.msg"
  "action/ExecutePlan.action"
  "action/PickPlace.action"
  "srv/GetLocations.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
