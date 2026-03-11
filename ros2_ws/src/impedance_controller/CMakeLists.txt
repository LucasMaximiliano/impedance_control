cmake_minimum_required(VERSION 3.8)
project(impedance_controller)

###################
#   Development   #
###################

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(impedance_controller_interfaces REQUIRED)
find_package(impedance_controller_params REQUIRED)

add_executable(impedance_controller_node
  src/main.cpp
  src/impedance_controller_base.cpp
  src/combined_controller_base.cpp
  src/combined_controller_ros2.cpp
)

target_include_directories(impedance_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(impedance_controller_node
  Eigen3::Eigen
  impedance_controller_params::impedance_controller_params
)

ament_target_dependencies(impedance_controller_node
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
  impedance_controller_interfaces
)

install(TARGETS impedance_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES config.toml # impedance_controller_foxglove_layout.json # This file is not yet needed, but might be in the future when we add a custom Foxglove layout.
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  impedance_controller_interfaces
  impedance_controller_params
)

ament_package()

############
#   Test   #
############

if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(tests/test_integration_clamp.py)
  add_ros_isolated_launch_test(tests/test_integration_publish.py)
  add_ros_isolated_launch_test(tests/test_integration_spawn.py)
  add_ros_isolated_launch_test(tests/test_integration_zero.py)
endif()