cmake_minimum_required(VERSION 3.8)
project(robomini_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionToGimbal.msg"
  "msg/GimbalToVision.msg"
  "msg/RuneState.msg"
  "msg/RuneHit.msg"
  "msg/ScoreBoard.msg"
  "msg/TargetDetection.msg"
  "msg/TargetDetectionArray.msg"
  "srv/SetRuneMode.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
