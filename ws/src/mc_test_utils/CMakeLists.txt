cmake_minimum_required(VERSION 3.8)
project(mc_test_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/mock_fcc_node
  scripts/mock_camera_node
  scripts/mock_peer_node
  scripts/mock_gcs_node
  scripts/mock_swarm_coord_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  test_data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # see docs/TESTING.md: no OSI license template matches "Proprietary"
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mavlink_gcs_client_items test/test_mavlink_gcs_client_items.py)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mock_fcc_node.py)
  add_launch_test(test/test_mock_camera_node.py)
  add_launch_test(test/test_mock_peer_node.py)
  add_launch_test(test/test_mock_gcs_node.py)
  add_launch_test(test/test_mock_swarm_coord_node.py)
  add_launch_test(test/test_mavlink_gcs_client.py)
endif()

ament_package()
