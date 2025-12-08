cmake_minimum_required(VERSION 3.0.2)
project(uwb_fusion)

# GTSAM requires C++11 or later. C++14 is recommended.
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
  tf_conversions
)

# Find GTSAM System Library
find_package(GTSAM REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# Build the optimizer library/source
add_executable(uwb_optimizer_node 
  src/uwb_optimizer_node.cpp 
  src/uwb_gtsam_optimizer.cpp
)

target_link_libraries(uwb_optimizer_node
  ${catkin_LIBRARIES}
  gtsam
)