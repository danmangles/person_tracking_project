#include "grid_map_cdt/Challenge.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_filters_demo");
  ros::NodeHandle nodeHandle("~");
  bool success;
  grid_map_demos::NavigationDemo navigationDemo(nodeHandle, success);
  if (success) ros::spin();
  return 0;
}
