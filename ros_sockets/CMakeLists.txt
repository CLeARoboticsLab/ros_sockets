cmake_minimum_required(VERSION 3.0.2)
project(ros_sockets)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	ros_sockets_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES server
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(server
   src/server.cpp
)

add_executable(velocity_control
	src/velocity_control.cpp
	src/server.cpp
	src/control_server.cpp
	src/control_data.cpp
)
target_link_libraries(
	velocity_control
	${catkin_LIBRARIES}
)

add_executable(gazebo_tracker
	src/gazebo_tracker.cpp
)
target_link_libraries(
	gazebo_tracker
	${catkin_LIBRARIES}
)

add_executable(state_feedback
	src/state_feedback.cpp
	src/server.cpp
	src/feedback_server.cpp
)
target_link_libraries(
	state_feedback
	${catkin_LIBRARIES}
)

add_executable(experiment_time
	src/experiment_time.cpp
	src/server.cpp
	src/time_server.cpp
	src/command_data.cpp
)
target_link_libraries(
	experiment_time
	${catkin_LIBRARIES}
)

add_executable(array_publisher
	src/array_publisher.cpp
	src/server.cpp
	src/array_server.cpp
	src/command_data.cpp
)
target_link_libraries(
	array_publisher
	${catkin_LIBRARIES}
)

add_executable(rollout_data
	src/rollout_data.cpp
	src/server.cpp
	src/rollout_data_server.cpp
)
target_link_libraries(
	rollout_data
	${catkin_LIBRARIES}
)

install(TARGETS	velocity_control gazebo_tracker state_feedback experiment_time array_publisher server rollout_data
  	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)