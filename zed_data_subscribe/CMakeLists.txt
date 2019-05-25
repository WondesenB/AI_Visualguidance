cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(zed_data_subscribe)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  darknet_ros_msgs
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
  darknet_ros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_depthdata_subscribe
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(zed_data_sub  src/zed_data_sub.cpp)
target_link_libraries(zed_data_sub  ${catkin_LIBRARIES})

