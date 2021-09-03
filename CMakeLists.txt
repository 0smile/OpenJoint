cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(OpenJoint)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-msgs5 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo3 REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
add_library(OpenJoint SHARED library.cpp)
target_link_libraries(OpenJoint
        PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
        PRIVATE ignition-gazebo3::ignition-gazebo3
        PRIVATE ignition-transport8::ignition-transport8
        PRIVATE ignition-msgs5::ignition-msgs5)