cmake_minimum_required(VERSION 3.16)
project(mc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/FaultEvent.msg"
  "msg/FccMissionItem.msg"
  "msg/FccMission.msg"
  "msg/FccCommand.msg"
  "msg/FccMissionAck.msg"
  "msg/FccOffboardSetpoint.msg"
  "msg/GcsCommand.msg"
  "msg/LinkStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
