################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(move_base_ack)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  actionlib
  nav_msgs
  rospy
  move_base_msgs
  roscpp
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package()

################################################################################
# Build
################################################################################
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_executable(${PROJECT_NAME}_node_omar src/main_omar.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES})
  
target_link_libraries(${PROJECT_NAME}_node_omar
  ${catkin_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
################################################################################
# Install
################################################################################
install(DIRECTORY launch maps param rviz src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
