/*


THIS FILE HAS BEEN SUPERCEDED BY THE PYTHON FILE average_tracker.py in ANAKIN_ROS


*/




















#include <ros/ros.h>
// tf includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  std::string vel_target_frame = "velodyne_person_est";
  // Create a ROS subscriber for the input point cloud
  //tf::TransformListener vel_listener;
  tf::TransformListener real_listener;
  tf::TransformBroadcaster br;

  ros::Rate rate(10.0);
  while (nh.ok()) {
    tf::StampedTransform real_tf;

    std::string base_frame = "base";
    //std::string real_target_frame = "realsense_person_est";
    std::string real_target_frame = "velodyne_person_est";
    
    try{
      real_listener.waitForTransform(real_target_frame, base_frame, ros::Time(0), ros::Duration(10.0) );
      real_listener.lookupTransform(real_target_frame, base_frame, ros::Time(0), real_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    std::cout << real_tf.getOrigin();


    rate.sleep();
  }
  return 0;

};