/*this file has a class definition:
*  - public
*      - constructor tracker(nh, kf_parmas);
*      - load_kf() {}
*      - update_kf() { updates kalman filter) and other get methods
*  - private has node handle
*     - THIS ENABLES DIFFERENT THINGS TO TALK TO EACH OTHER.
*     - void callback(msg);
*     - kf;
*/
#include <ros/ros.h> // because this is a robot
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
///////////////////

// sensor msgs includes specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <Eigen/Dense>

// TF includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>

// PCL includes
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// includes for cluster extraction
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h> //allows us to use pcl::transformPointCloud function.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h> // for the marker


#include <stdlib.h> // for random
#include <cmath> // used for euclidean norm
////////////////////////////////


//using namespace Eigen;
using namespace std;

#include "kalman_filter.h"
#ifndef tracker_H
#define tracker_H



class tracker {

public:
    tracker(ros::NodeHandle nh,
            int max_cluster_size,
            int min_cluster_size,
            double cluster_tolerance,
            double seg_dist_threshold,
            bool verbose); // initiate a constructor with a nodehandle and parameters for the kalman filter

    void setup_kalman_filter(VectorXd x0,
                             double dt,
                             const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& C,
                             const Eigen::MatrixXd& Q,
                             const Eigen::MatrixXd& R,
                             const Eigen::MatrixXd& P); // call the constructor of and initiate the kalman filter with params x0
    void update_kf(VectorXd y); // update the kalman filter with a new estimate y

private:
    void initialise_subscribers_publishers(); // initialises all the subscribers and publishers
    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg); // this method is called whenever the tracker sees a new pointcloud
    void apply_passthrough_filter(const sensor_msgs::PointCloud2ConstPtr input_cloud, sensor_msgs::PointCloud2 &output_cloud); // filters points outside of a defined cube
    void apply_base_odom_transformation(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &output_cloud); // transforms the cloud into the odom frame
    void get_centroids_of_clusters (vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector, vector<Eigen::VectorXd> &centroid_coord_array);
    void get_cluster_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_ptr, VectorXd &coord_centroid); //returns a vector of centroid coordinates
    void process_centroids(vector<VectorXd> centroid_coord_array);
    void publish_marker(VectorXd x_hat,double scale_x,double scale_y,double scale_z);
    void publish_transform(VectorXd coordinates, string target_frame_id); // publishes a transform at the 3D coordinate Vector coordinates
    void remove_out_of_plane_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr); // remove non-planar-inlying points
    void convert_sm2_to_pcl_ptr(sensor_msgs::PointCloud2 input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_ptr);
    void apply_voxel_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr);
    void assign_random_colour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
    void split_cloud_ptr_into_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_cluster_vector);
    vector<pcl::PointIndices> get_cluster_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
    VectorXd get_state(); // returns the current position estimate
    kalman_filter kf_; // our private copy of a kalman filter
    ros::Subscriber point_cloud_sub_; // private copy of subscriber
    // to publish our instantaneous, unfiltered estimates on
    tf::TransformBroadcaster br_;
    // a billion publishers
    ros::NodeHandle nh_;// setup 5 publishers to display point clouds at each stage
    ros::Publisher pub_raw_; // setup the publisher for the output point cloud
    ros::Publisher pub_trans_; // setup the publisher for the output point cloud
    ros::Publisher pub_zfilt_; // setup the publisher for the output point cloud
    ros::Publisher pub_ds_; // setup the publisher for the output point cloud
    ros::Publisher pub_centroid_; // setup the publisher for the output point cloud
    ros::Publisher pub_marker_; // to publish the markers on
    // HACK: setup the public transform listener so we can listen to the odom_base_tf
    boost::shared_ptr<tf::TransformListener> odom_base_ls_;
    int max_cluster_size_, min_cluster_size_;
    double cluster_tolerance_,seg_dist_threshold_; // how close a point must be to the model in order to be considered an inlier:
    bool verbose_;
};
#endif // tracker_H
