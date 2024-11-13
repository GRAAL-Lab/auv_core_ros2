cmake_minimum_required(VERSION 3.8)
project(kcl_ros2)

# Set C++ Standard to C++17 directly
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Debug for detailed compiler output
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
find_package(auv_msgs_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_library(QPOASES_LIBRARY NAMES qpOASES)
find_library(RML_LIBRARY NAMES rml)
find_library(FSM_LIBRARY NAMES fsm)
find_library(CTRL_LIBRARY NAMES ctrl_toolbox)
find_library(IKCL_LIBRARY NAMES ikcl)
find_package(libconfig REQUIRED)
include_directories(${libconfig_INCLUDE_DIRS})
find_library(SISL_LIBRARY NAMES sisl_toolbox)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

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
  ${auv_msgs_ros2_INCLUDE_DIRS}
)

# Define a static library for states classes
add_library(states_lib STATIC
  src/kcl/states/BaseAUVState.cpp
  src/kcl/states/IdleState.cpp
  src/kcl/states/JoystickState.cpp
  src/kcl/states/TrajectoryPlanningState.cpp
  src/kcl/states/PathPlanningState.cpp
  src/kcl/states/HoldState.cpp
)
target_include_directories(states_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${QP_OASES_INCLUDE_DIR}
)
ament_target_dependencies(states_lib
  Eigen3
  auv_msgs_ros2
  ament_index_cpp
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
  sensor_msgs
  tf2_geometry_msgs
  rclcpp_lifecycle
  nav_msgs
)
target_link_libraries(states_lib
  ${QPOASES_LIBRARY}
  ${ament_index_cpp_LIBRARIES}
  ${RML_LIBRARY}
  ${SISL_LIBRARY}
  ${IKCL_LIBRARY}
  ${CTRL_LIBRARY}
)

# Define executables for various nodes with corrected paths
add_executable(kcl
    src/kcl.cpp
    src/kcl/states/BaseAUVState.cpp
    src/kcl/states/IdleState.cpp
    src/kcl/states/JoystickState.cpp
    src/kcl/states/TrajectoryPlanningState.cpp
    src/kcl/states/PathPlanningState.cpp
    src/kcl/states/HoldState.cpp
    src/kcl/kcl/kcl_class.cpp
)

# Setup dependencies and linking for each executable
foreach(target kcl)
  ament_target_dependencies(${target}
    rclcpp
    visualization_msgs
    tf2_ros
    geometry_msgs
    Eigen3
    sensor_msgs
    tf2_geometry_msgs
    auv_msgs_ros2
    rclcpp_lifecycle
    nav_msgs
    mavros_msgs
  )
  target_include_directories(${target} PRIVATE ${EIGEN3_INCLUDE_DIR} ${auv_msgs_ros2_INCLUDE_DIRS})
endforeach()

# Link libraries specific to executables
target_link_libraries(kcl states_lib ${FSM_LIBRARY} ${RML_LIBRARY} ${SISL_LIBRARY} ${IKCL_LIBRARY} ${CTRL_LIBRARY})

# Install directives for executables and libraries
install(TARGETS kcl
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS states_lib 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install additional directories
install(DIRECTORY
  launch
  config
  models
  param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
