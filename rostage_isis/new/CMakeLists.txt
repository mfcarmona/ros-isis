cmake_minimum_required(VERSION 2.8.3)
project(rostage_isis)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs sensor_msgs geometry_msgs rosgraph_msgs tf)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}/lib64
   CATKIN_DEPENDS roscpp rospy std_msgs rosgraph_msgs
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  include/Stage-4.1
)

link_directories(
${PROJECT_SOURCE_DIR}/lib64/
)

## Declare a cpp library
# add_library(intersim
# src/${PROJECT_NAME}/intersim.cpp
# )

## Declare a cpp executable
# add_executable(intersim_node src/intersim_node.cpp)
# add_dependencies(intersim_node intersim_generate_messages_cpp)

add_executable(rostage_isis src/rostage_isis.cpp)

target_link_libraries(rostage_isis
${catkin_LIBRARIES}
# ${LD_LIBRARY_PATH}
# ~/lib/stage4/libstage/libstage.so
# ~/ros_ws/src/vintersim/lib64/libstage.so
# ${PROJECT_SRC_DIR}/lib64/libstage.so
libstage.so
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(intersim_node intersim_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(intersim_node
# ${catkin_LIBRARIES}
# )


