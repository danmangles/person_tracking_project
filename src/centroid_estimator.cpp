

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>


#include "include/centroid_estimator.h"


using namespace Eigen;
using namespace std;

//ros::NodeHandle nh;

Centroid_Estimator::Centroid_Estimator(
        int init_variable)//, ros::NodeHandle &nh)
        : init_variable_(init_variable) {

    cout <<"this is the constructor" <<endl;
    // subscribe to pcl and use a callback function to execute stuff

    string input_topic = "/velodyne/point_cloud_filtered";
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_topic, 1,&Centroid_Estimator::cloud_cb); // subscribe to the pointcloud


} // if the constructor has any init variables, need to assign them here

void Centroid_Estimator::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    cout << "THis is a callback" <<endl;
    // INPUTS: cloud_msg: pointer to the raw input cloud from the velodyne
        // This is a callback function that reads in a pointcloud, gets velodyne coords, updates kalman filter

    // for each pcl that comes in:

    //ros::NodeHandle nh;
    publish_cluster_centroid(cloud_msg);

}


void Centroid_Estimator::publish_cluster_centroid (const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
  // INPUTS: input_cloud: pointer to the pointcloud produced by the velodyne relative to "base"
  //          centroid_cluster_array: pointer to output cloud which contains the clusters
  // This method takes the pcloud input_cloud, downsizes it with a voxel grid, splits up the grid into
  // clusters, and returns the largest cluster as "cloud_cluster", a pointer to a PointCloud

    cout <<"executing method generate_cluster_array"<<endl;

    // filter the pointcloud
    // cluster it
    // for each cluster
    //       get the coordinates
    //       add to coordinate vector
    //       publish to a transform

}
