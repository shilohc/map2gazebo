cmake_minimum_required(VERSION 3.0.2)
project(map2gazebo)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(PROGRAMS src/map2gazebo.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
