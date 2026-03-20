cmake_minimum_required(VERSION 3.8)
project(impedance_controller_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFloat64.srv"
  "srv/SetFloat64MultiArray.srv"
  "srv/SetFloat32.srv"
  "srv/SetFloat32MultiArray.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()