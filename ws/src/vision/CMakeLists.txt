cmake_minimum_required(VERSION 3.16)
project(vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# --- visual_localisation_node -----------------------------------------
add_library(visual_localisation_node_lib
  src/visual_localisation_node.cpp
)
target_include_directories(visual_localisation_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(visual_localisation_node_lib PUBLIC rclcpp)

add_executable(visual_localisation_node
  src/visual_localisation_node_main.cpp
)
target_link_libraries(visual_localisation_node visual_localisation_node_lib)

# --- perception_down_node (renamed from detection_node) ------------------
add_library(perception_down_node_lib
  src/perception_down_node.cpp
)
target_include_directories(perception_down_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_down_node_lib PUBLIC rclcpp)

add_executable(perception_down_node
  src/perception_down_node_main.cpp
)
target_link_libraries(perception_down_node perception_down_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
install(
  TARGETS visual_localisation_node_lib perception_down_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS visual_localisation_node perception_down_node
  DESTINATION lib/${PROJECT_NAME}
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

  ament_add_gtest(test_visual_localisation_node test/test_visual_localisation_node.cpp)
  target_link_libraries(test_visual_localisation_node visual_localisation_node_lib)

  ament_add_gtest(test_perception_down_node test/test_perception_down_node.cpp)
  target_link_libraries(test_perception_down_node perception_down_node_lib)
endif()

ament_package()
