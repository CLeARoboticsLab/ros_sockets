cmake_minimum_required(VERSION 3.0.2)
project(ros_sockets)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
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

install(TARGETS	velocity_control gazebo_tracker state_feedback experiment_time
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