cmake_minimum_required(VERSION 3.0.2)
project(ros_sockets)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(velocity_control
	src/velocity_control.cpp
	src/communication.cpp
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
)
target_link_libraries(
	state_feedback
	${catkin_LIBRARIES}
)

install(TARGETS	velocity_control gazebo_tracker state_feedback
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