cmake_minimum_required(VERSION 3.0.2)
project(ros_sockets_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation    
    std_msgs
)

add_message_files(
    FILES
    RolloutData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)