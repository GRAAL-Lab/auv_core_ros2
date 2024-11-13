cmake_minimum_required(VERSION 3.8)
project(auv_core_helper)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(libconfig REQUIRED)

# Manually find qpOASES library
find_library(QPOASES_LIBRARY NAMES qpOASES REQUIRED)

# Generate ROS interfaces (messages and services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseStamped.msg"
  "srv/ControlCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Define the helper library
add_library(${PROJECT_NAME}_helper SHARED
  src/helper_lib.cpp
)

# Add dependencies to the helper library
ament_target_dependencies(${PROJECT_NAME}_helper
  Eigen3
  rclcpp
  geometry_msgs
  libconfig
  rosidl_default_runtime
)

# Include directories for the helper library
target_include_directories(${PROJECT_NAME}_helper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${libconfig_INCLUDE_DIRS}
)

# Link additional libraries
target_link_libraries(${PROJECT_NAME}_helper
  ${QPOASES_LIBRARY}
  config++
)

# Export dependencies and include directories
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_helper)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install library and parameter files
install(TARGETS ${PROJECT_NAME}_helper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # Only if the param directory is present
)

ament_package()
