cmake_minimum_required(VERSION 2.8.3)

project(archie_description)

find_package(catkin REQUIRED COMPONENTS
		message_generation
		roscpp
		rospy
		std_msgs
		geometry_msgs
		urdf
		xacro
		message_generation
		gazebo_ros
	)

catkin_package(CATKIN_DEPENDS
		geometry_msgs
		roscpp
		rospy
		std_msgs
		gazebo_ros

	)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
