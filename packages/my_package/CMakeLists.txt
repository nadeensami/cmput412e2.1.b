cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

catkin_package(CATKIN_DEPENDS 
  message_runtime
  std_msgs
)

catkin_install_python(PROGRAMS src/led_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)