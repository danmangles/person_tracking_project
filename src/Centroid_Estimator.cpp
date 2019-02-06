/*
 * Centroid_Estimator.cpp
 *
 *  Created on: 6 Feb 2019
 *      Author: dan
 */

// sensor msgs includes specific includes
#ifndef Centroid_Estimator_H
#define Centroid_Estimator_H

#include <ros/ros.h> // because this is a robot


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <vector>

#include "include/Centroid_Estimator.h"

#include <iostream>
#include <stdexcept>

using namespace Eigen;
using namespace std;


Centroid_Estimator::Centroid_Estimator(
        int init_variable)
        : init_variable_(init_variable) {
    cout <<"this is the constructor" <<endl;
    // subscribe to pcl and use a callback function to execute stuff

//    string input_topic = "/velodyne/point_cloud_filtered";
//    ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2> (input_topic, 1,&Centroid_Estimator::cloud_cb); // subscribe to the pointcloud







} // if the constructor has any init variables, need to assign them here

void Centroid_Estimator::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    cout << "THis is a callback" <<endl;
    // INPUTS: cloud_msg: pointer to the raw input cloud from the velodyne
        // This is a callback function that reads in a pointcloud, gets velodyne coords, updates kalman filter



}


void Centroid_Estimator::generate_cluster_array (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> centroid_cluster_array) {
  // INPUTS: input_cloud: pointer to the pointcloud produced by the velodyne relative to "base"
  //          centroid_cluster_array: pointer to output cloud which contains the clusters
  // This method takes the pcloud input_cloud, downsizes it with a voxel grid, splits up the grid into
  // clusters, and returns the largest cluster as "cloud_cluster", a pointer to a PointCloud

    cout <<"executing method generate_cluster_array"<<endl;

}

void Centroid_Estimator::update_centroid_coords(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        //process the pointcloud message and return a vector of coordinates

        cout << "executing method update_centroid_coords"<<endl;

        // Publish the cloud
        //pub_raw.publish (*cloud_msg); // publish the raw cloud


}

#endif // makes sure the ifndef works to prevent repeat class definitions
