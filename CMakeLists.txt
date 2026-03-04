cmake_minimum_required(VERSION 3.0.2)
project(com760cw1_b01041751)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  turtlesim
  tf2
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  B01041751LeaderMessage.msg
)

add_service_files(
  FILES
  B01041751GoToTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs turtlesim tf2 tf2_ros message_runtime
)
