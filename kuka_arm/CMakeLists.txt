cmake_minimum_required(VERSION 2.8.3)
project(kuka_arm)
add_compile_options(-std=c++11)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	COMPONENTS
		message_generation
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		pluginlib
		geometric_shapes
		moveit_visual_tools
		roscpp
		roslint
		rospy
			)
find_package(Eigen3 REQUIRED)


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   CalculateIK.srv
#   Service2.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
	 geometry_msgs
	 trajectory_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(trajectory_sampler src/trajectory_sampler.cpp)
add_executable(stomp_trajectory_sampler src/stomp_trajectory_sampler.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(trajectory_sampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(stomp_trajectory_sampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(trajectory_sampler
   ${catkin_LIBRARIES}
 )

target_link_libraries(stomp_trajectory_sampler
		${catkin_LIBRARIES}
		)

 install(PROGRAMS
   scripts/random_spawn.py
	 scripts/safe_spawner.sh
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   config/spawn_locations.yaml
#   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
