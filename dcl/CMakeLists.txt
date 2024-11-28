cmake_minimum_required(VERSION 3.8)
project(dcl)

# Set C++ Standard to C++17 directly
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)

# Enable additional compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auv_core_helper REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_library(RML_LIBRARY NAMES rml)
find_library(CTRL_LIBRARY NAMES ctrl_toolbox)
find_library(IKCL_LIBRARY NAMES ikcl)
find_package(PkgConfig REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_library(SIX_DOF_MODEL_LIBRARY NAMES 6DOF_model REQUIRED)


# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${auv_core_helper_INCLUDE_DIRS}  # Include directories for auv_core_helper
)

# Add the library directory to the linker search paths (if necessary)
link_directories(${CMAKE_INSTALL_PREFIX}/auv_core_helper/lib)

# Set RPATH for both build and install phases
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Print RPATH for debugging
message(STATUS "CMAKE_BUILD_RPATH: ${CMAKE_BUILD_RPATH}")
message(STATUS "CMAKE_INSTALL_RPATH: ${CMAKE_INSTALL_RPATH}")


# Define executables for various nodes
add_executable(dynamic_control_layer_node
    src/dynamic_control_layer_node.cpp
    src/dcl/dynamic_control_layer.cpp
)

# Setup dependencies and linking for each executable
foreach(target dynamic_control_layer_node)
  ament_target_dependencies(${target}
    rclcpp
    visualization_msgs
    tf2_ros
    geometry_msgs
    Eigen3
    sensor_msgs
    tf2_geometry_msgs
    auv_core_helper
    rclcpp_lifecycle
    nav_msgs
  )
  target_link_libraries(${target}
    ${RML_LIBRARY}
    ${IKCL_LIBRARY}
    ${CTRL_LIBRARY}
    ${auv_core_helper_LIBRARIES}
    ${SIX_DOF_MODEL_LIBRARY}
  )
endforeach()

# Install the kcl executable
install(TARGETS dynamic_control_layer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers and additional directories
install(DIRECTORY include/
  DESTINATION include/
  OPTIONAL
)

# Install additional directories
install(DIRECTORY
  launch
  config
  models
  param
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
