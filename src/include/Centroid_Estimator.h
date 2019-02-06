/*
 * kalman_filter.h
 *
 *  Created on: 28 Jan 2019
 *      Author: ori
 */
#include <Eigen/Dense> // for more vectors
#include <vector> // for vectors

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h> // because this is a robot


#ifndef CENTROID_ESTIMATOR_H
#define CENTROID_ESTIMATOR_H

using namespace Eigen;
using namespace std;

class Centroid_Estimator {

public:
        // all public methods and variables go here
        Centroid_Estimator(
                int init_variable
                ); // could initiate with certain variables but at this point, hardcode everything into the object



        void update_centroid_coords(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:

         // Matrices for computation
        void generate_cluster_array (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> centroid_cluster_array);
        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
        int init_variable_; // define the centroid publisher here
       // ros::NodeHandle nh_; // internal copy of the node handle
};

#endif
