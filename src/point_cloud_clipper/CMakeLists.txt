cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_clipper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_clipper_node src/point_cloud_clipper_node.cpp)
target_link_libraries(point_cloud_clipper_node ${catkin_LIBRARIES})
