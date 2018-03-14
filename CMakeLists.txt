cmake_minimum_required(VERSION 2.8.3)
project(safety_control)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  tf
  nav_msgs
  sensor_msgs
  laser_geometry
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

## Define custom messages
#add_message_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Define catkin exports
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sabertooth_usb
  CATKIN_DEPENDS pcl_conversions pcl_ros geometry_msgs trajectory_msgs roscpp rospy std_msgs sensor_msgs message_generation
  DEPENDS Boost system_lib
)


include_directories(
  include 
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(-std=c++11)


#add_library(
#   
#)

add_executable(safety_control_node src/safety_control_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(safety_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(safety_control_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sabertooth_usb sabertooth_usb_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sabertooth_usb.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
