cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(local_mapping)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
 # darknet_ros_msgs
  message_generation
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  
 )

add_message_files(
  FILES
  detected_object_info.msg
  detected_object.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES darknet_ros_msgs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(local_map_  src/local_mapping_node.cpp)
target_link_libraries(local_map_  ${catkin_LIBRARIES})

