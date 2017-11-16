cmake_minimum_required(VERSION 2.8.3)
project(mybot_msg)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   msgMybot_jointInterface.msg
   msgMybot_basicMovement.msg
   msgMybot_detailMovement.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
#   sensor_msgs
   std_msgs 
#   geometry_msgs
#   m_operator_commands
 )





catkin_package(
#  INCLUDE_DIRS 
#	include
#  LIBRARIES operator_input
  CATKIN_DEPENDS 
   	message_runtime
#  DEPENDS system_lib
)



include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)





