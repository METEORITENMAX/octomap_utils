cmake_minimum_required(VERSION 3.8)
project(octomap_utils)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(visualization_msgs REQUIRED)




# Include directories
include_directories(include)



### ROS2 Node: OctoMapHeightModNode

add_executable(OctoMapHeightModNode src/OctoMapHeightModNode.cpp)

ament_target_dependencies(OctoMapHeightModNode
  rclcpp
  std_msgs
  sensor_msgs
  octomap_msgs
  octomap
  visualization_msgs
)

install(TARGETS
  OctoMapHeightModNode
  DESTINATION lib/${PROJECT_NAME})



### ROS2 Node: OctomapMockupPublisherNode

add_executable(OctomapMockupPublisherNode src/OctomapMockupPublisherNode.cpp)

ament_target_dependencies(OctomapMockupPublisherNode
  rclcpp
  octomap_msgs
  octomap
)

install(TARGETS
  OctomapMockupPublisherNode
  DESTINATION lib/${PROJECT_NAME})



### Libary: octomap_utils

# Add the library
add_library(octomap_utils SHARED
  include/octomap_utils/octomap_calculations.hpp
  include/octomap_utils/octomap_conversions.hpp
  include/octomap_utils/octomap_mockups.hpp
  include/octomap_utils/octomap_visualization.hpp
)

# Specify the language for the library
set_target_properties(octomap_utils PROPERTIES LINKER_LANGUAGE CXX)

# Specify include directories
target_include_directories(octomap_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export the library
ament_export_targets(export_octomap_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  octomap_msgs
  octomap
  visualization_msgs
)

# Install the library and headers
install(
  TARGETS octomap_utils
  EXPORT export_octomap_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)



ament_package()