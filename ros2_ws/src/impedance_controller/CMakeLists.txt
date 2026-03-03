cmake_minimum_required(VERSION 3.8)
project(impedance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(impedance_controller_msgs REQUIRED) # This package is not yet needed, but might be in the future when we add custom messages.
find_package(impedance_controller_params REQUIRED)

add_executable(main
  src/main.cpp
)

target_include_directories(main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(main
  Eigen3::Eigen
  impedance_controller_params::impedance_controller_params
)

ament_target_dependencies(main
  rclcpp
  std_msgs
  # impedance_controller_msgs # See comment above about this package.
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES config.toml # impedance_controller_foxglove_layout.json # This file is not yet needed, but might be in the future when we add a custom Foxglove layout.
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  std_msgs
  # impedance_controller_msgs # See comment above about this package.
  impedance_controller_params
)

ament_package()
