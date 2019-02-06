#include <ros/ros.h> // because this is a robot
// tf includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>




#include <Eigen/Dense>  // for all the lovely matrices
#include <visualization_msgs/Marker.h>

// sensor msgs includes specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "include/Centroid_Estimator.h" // for the centroid estimator

using namespace std;

// initialise the centroid estimator for use in cloud_cb




int main (int argc, char** argv)
{
    // initiailise ROS
    ros::init (argc, argv, "multi_sensor_tracker");

    ros::NodeHandle nh;
    int init_variable = 4;

    Centroid_Estimator ce(init_variable);





}
