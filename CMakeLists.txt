cmake_minimum_required(VERSION 3.0.2)
project(slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  message_generation
)

find_package(PCL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate services in the 'srv' folder
add_service_files(
  FILES
  ClassifyColorSrv.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES perception_handling
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(perception_handling
  src/perception_handling/utils.cpp)

target_link_libraries(perception_handling
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cone_detection src/cone_detection.cpp)
add_executable(ground_removal src/ground_removal.cpp)
add_executable(slam_run src/slammer.cpp)
add_executable(color_classifier_server src/color_classifier_server.cpp)

target_link_libraries(cone_detection
  perception_handling
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(slam_run
  perception_handling
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(ground_removal
  perception_handling
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(color_classifier_server
  perception_handling
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/color_classifier_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)