cmake_minimum_required(VERSION 2.8.3)
project(anakin_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  compressed_image_transport
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  apriltags
  tf_conversions
)

find_package(Eigen3 REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES anakin_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_executable(anakin_detector_node
  src/anakin_detector_node.cpp )
target_link_libraries(anakin_detector_node ${catkin_LIBRARIES} )

## Declare a C++ library
add_library(${PROJECT_NAME}
 	src/anakin_detector_node.cpp
)

# catkin_add_nosetests(test)
