cmake_minimum_required(VERSION 3.16)
project(mission_exec)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(mission_manager_node_lib
  src/mission_manager_node.cpp
)
target_include_directories(mission_manager_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_manager_node_lib PUBLIC rclcpp)

add_executable(mission_manager_node src/mission_manager_node_main.cpp)
target_link_libraries(mission_manager_node mission_manager_node_lib)

add_library(search_planner_node_lib
  src/search_planner_node.cpp
)
target_include_directories(search_planner_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(search_planner_node_lib PUBLIC rclcpp)

add_executable(search_planner_node src/search_planner_node_main.cpp)
target_link_libraries(search_planner_node search_planner_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mission_manager_node_lib search_planner_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS mission_manager_node search_planner_node
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

  ament_add_gtest(test_mission_manager_node test/test_mission_manager_node.cpp)
  target_link_libraries(test_mission_manager_node mission_manager_node_lib)

  ament_add_gtest(test_search_planner_node test/test_search_planner_node.cpp)
  target_link_libraries(test_search_planner_node search_planner_node_lib)
endif()

ament_package()
