cmake_minimum_required(VERSION 3.16)
project(impedance_controller_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

####################
#                  #
#    MATLAB MEX    #
#                  #
####################

# TODO: Update MATLAB path and OS architecture as needed
set(Matlab_ROOT_DIR "/Applications/MATLAB_R2025b.app")
set(Matlab_MEX_LIBRARY "/Applications/MATLAB_R2025b.app/bin/maca64/libmex.dylib")
set(Matlab_MEX_EXTENSION "mexmaca64")
set(CMAKE_OSX_ARCHITECTURES "arm64")

find_package(Matlab REQUIRED COMPONENTS MEX)
find_package(Eigen3 REQUIRED)

add_library(forward_finger_kinematics_mex SHARED
    src/mex/forward_finger_kinematics_mex.cpp
)

set_target_properties(forward_finger_kinematics_mex PROPERTIES
    PREFIX ""
    SUFFIX ".${Matlab_MEX_EXTENSION}"
)

target_include_directories(forward_finger_kinematics_mex PRIVATE
    ${Matlab_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../impedance_controller/include
)

target_link_libraries(forward_finger_kinematics_mex
    Matlab::MEX
    Eigen3::Eigen
)

target_compile_definitions(forward_finger_kinematics_mex PRIVATE
    MATLAB_MEX_FILE
)
