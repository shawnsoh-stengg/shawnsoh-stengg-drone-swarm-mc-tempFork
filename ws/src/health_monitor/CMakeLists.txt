cmake_minimum_required(VERSION 3.16)
project(health_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(health_monitor_node_lib
  src/health_monitor_node.cpp
)
target_include_directories(health_monitor_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(health_monitor_node_lib PUBLIC rclcpp)

add_executable(health_monitor_node
  src/health_monitor_node_main.cpp
)
target_link_libraries(health_monitor_node health_monitor_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS health_monitor_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS health_monitor_node
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

  ament_add_gtest(test_health_monitor_node test/test_health_monitor_node.cpp)
  target_link_libraries(test_health_monitor_node health_monitor_node_lib)
endif()

ament_package()
