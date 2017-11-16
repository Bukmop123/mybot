cmake_minimum_required(VERSION 2.8.3)
project(mybot_basic_mov_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  mybot_msg
  message_generation
  control_toolbox
)


 find_package(Boost REQUIRED COMPONENTS system)


# add_message_files(
#   FILES
# )

 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs 
   geometry_msgs
   mybot_msg
 )

catkin_package(
  CATKIN_DEPENDS 
	roscpp 
	sensor_msgs 
	std_msgs
	geometry_msgs
   	message_runtime
	control_toolbox
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("/usr/local/lib")

 add_executable(mybot_basic_mov_control src/mybot_basic_mov_control.cpp ${catkin_LIBRARIES})

 target_link_libraries(mybot_basic_mov_control
   ${catkin_LIBRARIES}
 )


install(TARGETS
  mybot_basic_mov_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
