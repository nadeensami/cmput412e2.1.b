cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(DIRECTORY srv FILES ColorService.srv)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS
  src/led_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)