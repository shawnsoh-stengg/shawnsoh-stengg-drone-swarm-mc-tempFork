cmake_minimum_required(VERSION 3.16)
project(gcs_comms)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavlink REQUIRED)
find_package(mc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# GCS-001: gcs_mavlink_node — raw-MAVLink Channel B transport (ICD §4.1/§4.2),
# GCS command decode, link-status heartbeat monitoring (SDD §11.1). Uses the
# same ros-humble-mavlink C headers as fcc_bridge's mission-protocol client
# rather than MAVROS — MAVROS is not installed anywhere in this repo's
# toolchain (docker/ros2-core/Dockerfile only provisions ros-humble-mavlink),
# so this matches the pattern fcc_bridge already established for the
# equivalent MC<->FCC problem instead of introducing a second, heavier
# MAVLink integration strategy.
add_library(gcs_mavlink_node_lib
  src/gcs_mavlink_node.cpp
)
target_include_directories(gcs_mavlink_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mavlink_INCLUDE_DIRS}
)
ament_target_dependencies(gcs_mavlink_node_lib PUBLIC rclcpp mc_msgs geometry_msgs geographic_msgs)

add_executable(gcs_mavlink_node
  src/gcs_mavlink_node_main.cpp
)
target_link_libraries(gcs_mavlink_node gcs_mavlink_node_lib)

# GCS-004: mission_orchestrator_node — MAVFTP server + MC_MISSION_PLAN_ACTIVATE/
# ACK over Channel C (ICD §12). Same raw-mavlink.h, no-MAVROS approach as
# gcs_mavlink_node_lib above; additionally links nlohmann_json for plan JSON
# parsing (ICD §12.4) — the one new library dependency this package didn't
# already have.
add_library(mission_orchestrator_node_lib
  src/mission_orchestrator_node.cpp
)
target_include_directories(mission_orchestrator_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mavlink_INCLUDE_DIRS}
)
ament_target_dependencies(mission_orchestrator_node_lib PUBLIC
  rclcpp mc_msgs geometry_msgs geographic_msgs)
target_link_libraries(mission_orchestrator_node_lib PUBLIC nlohmann_json::nlohmann_json)

add_executable(mission_orchestrator_node
  src/mission_orchestrator_node_main.cpp
)
target_link_libraries(mission_orchestrator_node mission_orchestrator_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS gcs_mavlink_node_lib mission_orchestrator_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS gcs_mavlink_node mission_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
# Documentation only — not consumed by the build (GCS-003, see this file's
# own header comment for why gcs_mavlink_node.cpp hand-packs against it
# instead of generating a header from it via mavgen).
install(
  FILES mc_mavlink_defs.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright only recognizes OSI license templates (apache2, bsd*,
  # mit, gpl*, ...); this package.xml declares license="Proprietary", which
  # has no matching template, so the check would always fail regardless of
  # header content. Skipped rather than attaching a misleading OSS notice.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gcs_mavlink_node test/test_gcs_mavlink_node.cpp)
  target_link_libraries(test_gcs_mavlink_node gcs_mavlink_node_lib)
  target_include_directories(test_gcs_mavlink_node PRIVATE ${mavlink_INCLUDE_DIRS})

  ament_add_gtest(test_mission_orchestrator_node test/test_mission_orchestrator_node.cpp)
  target_link_libraries(test_mission_orchestrator_node mission_orchestrator_node_lib)
  target_include_directories(test_mission_orchestrator_node PRIVATE ${mavlink_INCLUDE_DIRS})
endif()

ament_package()
