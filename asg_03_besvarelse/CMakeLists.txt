cmake_minimum_required(VERSION 2.8.3)
project(pd_controller)

find_package(catkin REQUIRED roslaunch roscpp std_msgs geometry_msgs sensor_msgs message_generation)


generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	sensor_msgs
	)

# Tell packages that depend on us that they also need message_runtime
catkin_package(CATKIN_DEPENDS message_runtime)

catkin_package(
	INCLUDE_DIRS /home/eivind/Documents/master/project_ws/src/crustcrawler_lib/include/crustcrawler_lib
	LIBRARIES crustcrawler_lib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


# Add launch files so they are checked for faults
roslaunch_add_file_check(launch)

install(
	DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
##	DESTINATION src/$(pd_controller)
)

add_executable(controller src/controller.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/controller.cpp
)

add_dependencies(controller 
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(controller
	${catkin_LIBRARIES}	
)




