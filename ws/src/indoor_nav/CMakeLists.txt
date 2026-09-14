cmake_minimum_required(VERSION 3.16)
project(indoor_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mc_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavlink REQUIRED)

add_library(gnss_sim_node_lib
  src/gnss_sim_node.cpp
)
target_include_directories(gnss_sim_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mavlink_INCLUDE_DIRS}
)
ament_target_dependencies(gnss_sim_node_lib PUBLIC rclcpp mc_msgs nav_msgs)

add_executable(gnss_sim_node
  src/gnss_sim_node_main.cpp
)
target_link_libraries(gnss_sim_node gnss_sim_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS gnss_sim_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS gnss_sim_node
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

  ament_add_gtest(test_gnss_sim_node test/test_gnss_sim_node.cpp)
  target_link_libraries(test_gnss_sim_node gnss_sim_node_lib)

  ament_add_gtest(test_hil_gps_sender test/test_hil_gps_sender.cpp)
  target_link_libraries(test_hil_gps_sender gnss_sim_node_lib)
  target_include_directories(test_hil_gps_sender PRIVATE ${mavlink_INCLUDE_DIRS})
endif()

ament_package()
