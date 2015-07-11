cmake_minimum_required(VERSION 2.8.3)
project(rcppm2ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosserial_arduino
  rosserial_python
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  rc_input.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_custom_target(rcppm2ros)
add_dependencies(rcppm2ros
  rcppm2ros_generate_messages_cpp
  rcppm2ros_generate_messages_python
)

#############
## Install ##
#############

install(FILES
  launch/rcppm2ros.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
