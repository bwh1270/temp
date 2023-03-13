cmake_minimum_required(VERSION 3.0.2)
project(carrot_team)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  trajectory_msgs
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
    FILES
    poi.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs#   trajectory_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES carrot_team
   CATKIN_DEPENDS geometry_msgs roscpp rospy trajectory_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_executable(keyboard_control
        src/keyboard_control.cpp
        src/modules/orientation.cpp
        )
target_link_libraries(keyboard_control
        ${catkin_LIBRARIES}
        )
add_executable(control_main
        src/control_main.cpp
        src/modules/class.cpp
        src/modules/orientation.cpp
        )
target_link_libraries(control_main
        ${catkin_LIBRARIES}
        )