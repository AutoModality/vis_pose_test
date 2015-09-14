cmake_minimum_required(VERSION 2.8.3)

project(vis_pose_test)

find_package(
	catkin REQUIRED 
	roscpp 
	geometry_msgs 
	std_msgs
	cv_bridge 
	genmsg 
	image_transport 
	sensor_msgs 
	control_toolbox 
	camera_info_manager
	mavros
	rosbag
)


set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    	roscpp 
    	geometry_msgs 
        std_msgs
		cv_bridge 
		genmsg 
		image_transport 
		sensor_msgs 
		control_toolbox 
		camera_info_manager
		mavros
	DEPENDS Boost mavlink 
)
catkin_package(
  DEPENDS Boost mavlink
)

include_directories(
	include
 	${catkin_INCLUDE_DIRS}
 	${mavlink_INCLUDE_DIRS}
)

link_directories(
	lib/x86_64
)

add_executable(rc_joystick src/rc_joystick.cpp)
target_link_libraries(rc_joystick ${catkin_LIBRARIES})
	
add_executable(vis_pose_test src/vis_pose_test.cpp src/locator.cpp src/controller.cpp src/kinematics.cpp)
target_link_libraries(vis_pose_test ${catkin_LIBRARIES})

