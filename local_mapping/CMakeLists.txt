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
  #message_generation
  
  
 )

# add_message_files(
#   FILES
#   detected_object_info.msg
#   detected_object.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
  
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES darknet_ros_msgs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#CATKIN_DEPENDS message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node  src/local_mapping_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

