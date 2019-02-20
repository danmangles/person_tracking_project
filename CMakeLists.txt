cmake_minimum_required(VERSION 2.8.3)
project(multi_sensor_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs

)



catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_NAME}
  INCLUDE_DIRS include
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/kalman_filter.cpp
#   src/tracker.cpp
   src/centroid_estimator.cpp
 )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_read src/pcd_read.cpp)
target_link_libraries(pcd_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (voxel_grid src/voxel_grid.cpp)
target_link_libraries (voxel_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (passthrough src/passthrough.cpp)
target_link_libraries (passthrough ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable (clustering src/clustering.cpp)
target_link_libraries (clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (velodyne_tracker src/velodyne_tracker.cpp)
target_link_libraries (velodyne_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (average_tracker src/average_tracker.cpp)
target_link_libraries (average_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (kalman_test src/kalman_test.cpp)
target_link_libraries (kalman_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME})

add_executable (kalman_filter_tracker src/kalman_filter_tracker.cpp)
target_link_libraries (kalman_filter_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME})

#add_executable (nearest_cluster_tracker src/nearest_cluster_tracker.cpp)
#target_link_libraries (nearest_cluster_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME})

add_executable (tracker_node src/tracker_node.cpp src/tracker.cpp)
target_link_libraries (tracker_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME})
#add_executable (tracker src/tracker.cpp)
#target_link_libraries (tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME})
