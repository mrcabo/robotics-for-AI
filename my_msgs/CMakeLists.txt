cmake_minimum_required(VERSION 2.8.3)
project(my_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        actionlib_msgs
        genmsg
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DummyMessage.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DummyService.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  DummyAction.action
  Example.action
  AddIntegers.action
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
)

catkin_package()
include_directories()