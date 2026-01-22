cmake_minimum_required(VERSION 3.16)
project(impedance_controller_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

####################
#    MATLAB MEX    #
####################

# TODO: Update MATLAB path as needed (depends on installation location, version, and OS)
set(Matlab_ROOT_DIR "/Applications/MATLAB_R2025b.app")
set(Matlab_MEX_LIBRARY "${Matlab_ROOT_DIR}/bin/maca64/libmex.dylib")
set(Matlab_MEX_EXTENSION "mexmaca64")

find_package(Matlab REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(forward_finger_kinematics_mex SHARED
    src/mex/forward_finger_kinematics_mex.cpp
)

set_target_properties(forward_finger_kinematics_mex PROPERTIES
    PREFIX ""
    SUFFIX ".${Matlab_MEX_EXTENSION}"
)

target_include_directories(forward_finger_kinematics_mex PRIVATE
    "${Matlab_ROOT_DIR}/extern/include"
    ${PROJECT_SOURCE_DIR}/../impedance_controller/include
)

target_link_libraries(forward_finger_kinematics_mex
    "${Matlab_MEX_LIBRARY}"
    "${Matlab_ROOT_DIR}/bin/maca64/libmx.dylib"
    "${Matlab_ROOT_DIR}/bin/maca64/libmat.dylib"
    Eigen3::Eigen
)

target_compile_definitions(forward_finger_kinematics_mex PRIVATE
    MATLAB_MEX_FILE
)
