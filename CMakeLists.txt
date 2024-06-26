cmake_minimum_required(VERSION 3.0.2)
project(scene_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene_calibration
#  CATKIN_DEPENDS  rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   scripts/main.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
