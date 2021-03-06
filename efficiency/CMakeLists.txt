cmake_minimum_required(VERSION 2.8.3)
project(efficiency)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Efficiency_m.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Efficiency_s.srv
   Efficiency_s2.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs nav_msgs sensor_msgs 
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES efficiency
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(efficiency
#   src/${PROJECT_NAME}/efficiency.cpp
# )

## Declare a cpp executable
add_executable(Efficiency_s_server src/Efficiency_s_server.cpp)
target_link_libraries(Efficiency_s_server ${catkin_LIBRARIES})
add_dependencies(Efficiency_s_server efficiency_generate_messages_cpp efficiency_gencpp)

add_executable(Efficiency_s_server2 src/Efficiency_s_server2.cpp)
target_link_libraries(Efficiency_s_server2 ${catkin_LIBRARIES})
add_dependencies(Efficiency_s_server2 efficiency_generate_messages_cpp efficiency_gencpp)

add_executable(eff_subscriber src/eff_subscriber.cpp)
target_link_libraries(eff_subscriber ${catkin_LIBRARIES})
add_dependencies(eff_subscriber efficiency_generate_messages_cpp efficiency_gencpp)

add_executable(eff_subscriber2 src/eff_subscriber2.cpp)
target_link_libraries(eff_subscriber2 ${catkin_LIBRARIES})
add_dependencies(eff_subscriber2 efficiency_generate_messages_cpp efficiency_gencpp)

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
# install(TARGETS efficiency efficiency_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_efficiency.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
