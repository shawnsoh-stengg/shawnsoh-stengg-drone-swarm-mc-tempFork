cmake_minimum_required(VERSION 3.16)
project(mc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/FaultEvent.msg"
  "msg/FccMissionItem.msg"
  "msg/FccMission.msg"
  "msg/FccCommand.msg"
  "msg/FccMissionAck.msg"
  "msg/VisualOdometry.msg"
  "msg/FccOffboardSetpoint.msg"
  "msg/GcsCommand.msg"
  "msg/LinkStatus.msg"
  "msg/MissionPhase.msg"
  "msg/FormationOffset.msg"
  "msg/SwarmPeer.msg"
  "msg/SwarmPeerArray.msg"
  "msg/VoronoiCell.msg"
  "msg/IsrRole.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "msg/VideoDownlinkStatus.msg"
  "msg/SystemHealth.msg"
  "msg/LocalisationMode.msg"
  "msg/MissionUpload.msg"
  "msg/StatusText.msg"
  "msg/GcsCommandAck.msg"
  "msg/OpticalFlow.msg"
  "msg/IndoorGnssSimStatus.msg"
  "msg/BaroAltitude.msg"
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
