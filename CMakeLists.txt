cmake_minimum_required(VERSION 2.8.3)
project(scl_agv)

find_package(catkin REQUIRED COMPONENTS 
  actionlib 
  move_base_msgs 
  roscpp 
  tf
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  vision_feedback.msg
)

add_service_files(
  FILES
  agv_niryo.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES AGV_SCL
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scl_agv_with_niryo_2 src/scl_agv_with_niryo_2.cpp)
target_link_libraries(scl_agv_with_niryo_2 ${catkin_LIBRARIES})
add_dependencies(scl_agv_with_niryo_2 agv_scl_generate_messages_cpp)
