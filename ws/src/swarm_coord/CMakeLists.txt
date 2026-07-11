cmake_minimum_required(VERSION 3.16)
project(swarm_coord)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(swarm_coordinator_node_lib
  src/swarm_coordinator_node.cpp
)
target_include_directories(swarm_coordinator_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(swarm_coordinator_node_lib PUBLIC rclcpp)

add_executable(swarm_coordinator_node src/swarm_coordinator_node_main.cpp)
target_link_libraries(swarm_coordinator_node swarm_coordinator_node_lib)

add_library(formation_controller_node_lib
  src/formation_controller_node.cpp
)
target_include_directories(formation_controller_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(formation_controller_node_lib PUBLIC rclcpp)

add_executable(formation_controller_node src/formation_controller_node_main.cpp)
target_link_libraries(formation_controller_node formation_controller_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS swarm_coordinator_node_lib formation_controller_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS swarm_coordinator_node formation_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
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

  ament_add_gtest(test_swarm_coordinator_node test/test_swarm_coordinator_node.cpp)
  target_link_libraries(test_swarm_coordinator_node swarm_coordinator_node_lib)

  ament_add_gtest(test_formation_controller_node test/test_formation_controller_node.cpp)
  target_link_libraries(test_formation_controller_node formation_controller_node_lib)
endif()

ament_package()
