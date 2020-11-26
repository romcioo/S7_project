cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_pkg
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
	CATKIN_DEPENDS
		geometry_msgs
		roscpp
		rospy
		std_msgs
		#DEPENDS system_lib
)



include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_pkg src/local_planner.cpp)

add_dependencies(test_pkg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_pkg ${catkin_LIBRARIES})
