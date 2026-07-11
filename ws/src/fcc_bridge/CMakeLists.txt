cmake_minimum_required(VERSION 3.16)
project(fcc_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(mc_msgs REQUIRED)

add_library(fcc_backend
  src/factory.cpp
  src/xrce_dds_backend.cpp
  src/mission_protocol_client.cpp
)
target_include_directories(fcc_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mavlink_INCLUDE_DIRS}
)
ament_target_dependencies(fcc_backend PUBLIC rclcpp px4_msgs)

# FCC-004: fcc_bridge_node — thin adapter over FccBackend (SDD §3.4).
add_library(fcc_bridge_node_lib
  src/fcc_bridge_node.cpp
)
target_include_directories(fcc_bridge_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fcc_bridge_node_lib PUBLIC fcc_backend)
ament_target_dependencies(fcc_bridge_node_lib PUBLIC rclcpp mc_msgs)

add_executable(fcc_bridge_node
  src/fcc_bridge_node_main.cpp
)
target_link_libraries(fcc_bridge_node fcc_bridge_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS fcc_backend fcc_bridge_node_lib
  EXPORT export_fcc_backend
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS fcc_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
ament_export_targets(export_fcc_backend HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp px4_msgs mc_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright only recognizes OSI license templates (apache2, bsd*,
  # mit, gpl*, ...); this package.xml declares license="Proprietary", which
  # has no matching template, so the check would always fail regardless of
  # header content. Skipped rather than attaching a misleading OSS notice.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # FCC-001: interface contract + factory selection logic (no PX4 hardware
  # or px4_msgs types involved — a fake backend stands in for FccBackend).
  ament_add_gtest(test_fcc_backend_contract test/test_fcc_backend_contract.cpp)
  target_link_libraries(test_fcc_backend_contract fcc_backend)

  ament_add_gtest(test_factory test/test_factory.cpp)
  target_link_libraries(test_factory fcc_backend)

  # FCC-002: mission-protocol handshake correctness against a loopback UDP
  # peer that plays the FCC's part of the microservice.
  ament_add_gtest(test_mission_protocol_client test/test_mission_protocol_client.cpp)
  target_link_libraries(test_mission_protocol_client fcc_backend)
  target_include_directories(test_mission_protocol_client PRIVATE ${mavlink_INCLUDE_DIRS})

  # FCC-002: XrceDdsBackend state aggregation against a mock uORB publisher
  # (TP-NI style, SDD §3.2) — real px4_msgs types, no live FCC required.
  ament_add_gtest(test_xrce_dds_backend test/test_xrce_dds_backend.cpp)
  target_link_libraries(test_xrce_dds_backend fcc_backend)

  # FCC-004: fcc_bridge_node adapter — TP-NI-001/002/003 (publish rate,
  # content accuracy, link-loss fault), against a mock uORB publisher.
  ament_add_gtest(test_fcc_bridge_node test/test_fcc_bridge_node.cpp)
  target_link_libraries(test_fcc_bridge_node fcc_bridge_node_lib)
endif()

ament_package()
