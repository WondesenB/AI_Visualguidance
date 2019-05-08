cmake_minimum_required(VERSION 2.8.3)
project(zed_data_subscribe)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  darknet_ros_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_depthdata_subscribe
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(zed_data_sub  src/zed_data_sub.cpp)
target_link_libraries(zed_data_sub  ${catkin_LIBRARIES})

