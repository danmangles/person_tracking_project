#include "include/tracker.h"

#include <cmath>
using namespace std;
//using namespace Eigen;

tracker::tracker(ros::NodeHandle nh,
                 double dt,
                 const Eigen::MatrixXd& A,
                 const Eigen::MatrixXd& C,
                 const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& P,
                 bool verbose
                 ) :
    nh_(nh), verbose_(verbose), kf_(dt, A, C, Q, R, P, false) // initiate the nodehandle and kalman_filter
{
    cout<< "tracker constructor called "<<endl;    // initiate the subscriber which calls the callback
    string input_topic = "/velodyne/point_cloud_filtered"; // topic is the pointcloud from the velodyne
    point_cloud_sub_ = nh_.subscribe(input_topic, 1, &tracker::callback, this); // Create a ROS subscriber for the input point cloud

    //*****************************************8
    // initialise the kalman filter NEED TO MAKE THIS INITIALISE FROM THE REALSENSE
    VectorXd x0(3);
    x0 << 0,0,0;
    init_kf(x0);

    // Create a ROS publisher for the output point cloud
    string topic_raw = "pcl_raw", topic_trans = "pcl_trans", topic_zfilt = "pcl_zfilt", topic_ds = "pcl_ds", topic_centroid = "pcl_centroid";

    pub_raw_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_raw, 1);
    pub_trans_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_trans, 1);
    pub_zfilt_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_zfilt, 1);
    pub_ds_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_ds, 1);
    pub_centroid_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_centroid, 1);
    pub_marker_ = nh.advertise<visualization_msgs::Marker>( "tracker_marker", 0 ); // this name shows up in RVIZ

    odom_base_ls_.reset(new tf::TransformListener); // initialise the odom_base transform listener- so we can transform our output into odom coords

}
void tracker::publish_marker(VectorXd x_hat,double scale_x,double scale_y,double scale_z) {
    ///
    /// This publishes a box of scales set by vector onto publisher vis_pub at location x_hat
    ///

    visualization_msgs::Marker marker; // initiate the marker

    marker.header.frame_id = "odom"; // we want to publish relative to the odom frame

    marker.header.stamp = ros::Time();
    marker.ns = "kalman_filter_marker";  // call our marker kalman_filter_marker
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER; // make it a cube
    marker.action = visualization_msgs::Marker::ADD;

    // assign the marker location according to x_hat
    marker.pose.position.x = x_hat[0];
    marker.pose.position.y = x_hat[1];
    marker.pose.position.z = x_hat[2];

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    //
    double scaler = .3;
    // set the marker size as an input params
    marker.scale.x = scale_x*scaler;
    marker.scale.y = scale_y*scaler;
    //    marker.scale.z = scale_z*scaler;
    marker.scale.z = 2;
    marker.color.a = 0.4; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    pub_marker_.publish( marker );


}
void tracker::publish_transform(VectorXd coordinates, string target_frame_id) {
    // publishes a transform on broadcaster br_ at the 3D coordinate Vector coordinates
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(coordinates[0],coordinates[1],coordinates[2]) );
    tf::Quaternion q; // initialise the quaternion q for the pose angle

    q.setEulerZYX(0, 0, 0);
    transform.setRotation(q);
    string velodyne_frame_id = "odom";
    //br.sendTransform(centre_point, q, ros::Time::now(), target_frame_id, velodyne_frame_id);
    // not sure if i put the ids the right way round
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), velodyne_frame_id, target_frame_id));
}
void tracker::callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) { // the callback fcn

    cout<< "************************************\nInitiating Callback\n***********************************"<<endl;

    // Publish the cloud
    pub_raw_.publish (*cloud_msg); // publish the raw cloud

    // apply a passthrough filter
    sensor_msgs::PointCloud2 bounded_cloud;
    apply_passthrough_filter(cloud_msg, bounded_cloud); // remove all points outside of a predefined bod

    pub_zfilt_.publish (bounded_cloud); // Publish the output

    //////////////////////* TRANSFORM THE INPUT CLOUD INTO THE ODOM FRAME  /////////////////////////////
    string target_frame = "odom", base_frame = "base"; // target frame for transform
    sensor_msgs::PointCloud2 transformed_cloud;
    if (verbose_)
        cout<<"waiting for transform"<<endl;
    odom_base_ls_->waitForTransform(target_frame, base_frame, ros::Time::now(), ros::Duration(10.0) );
    cout<<"transforming pcloud using time "<< ros::Time::now()<<endl;
    pcl_ros::transformPointCloud(target_frame, bounded_cloud, transformed_cloud, *odom_base_ls_); // perform the transformation

    //    transform_cloud(bounded_cloud, transformed_cloud, target_frame, transformed_cloud); // transform

    /////////////////////////////////////
    // Publish the cloud
    pub_trans_.publish (transformed_cloud); // this is publishing correctly//////////////////////////////////
    //   if cloud has less than 10 points, jump out of the callback
    if (transformed_cloud.width < 10) { // the velodyne reading is invalid- probably an issue with the transform
        cout << "*** THERE'S SOMETHING WRONG WITH THIS CLOUD. waiting for the next one. ****" << endl;
        cout << "Width: "<< transformed_cloud.width <<endl;
        return; // drop out of this callback and come back with the next transform
    }
    cout << "*** LOOKS OK TO ME***"<< endl;

    ////////////////////// DO A WEIRD VARIABLE CONVERSION THING SO WE END UP WITH A  "boost shared pointer"/////////////////////////////

    // Take the previously published output message and convert it to a pointcloud called "prefiltered cloud"
    pcl::PointCloud<pcl::PointXYZRGB> prefiltered_cloud;  // pcl version of the point cloud
    pcl::fromROSMsg (transformed_cloud, prefiltered_cloud); // wrong: output_msg of type smPC2 and transformed_cloud of
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (); //convert from an object to a boost shared pointer

    typedef pcl::PointCloud<pcl::PointXYZRGB>  mytype;
    boost::shared_ptr<mytype> prefiltered_cloud_ptr = boost::make_shared <mytype> (prefiltered_cloud);

    ////////////////// GET THE CLUSTER CENTROIDS
    vector <VectorXd> centroid_coord_array;
    generate_coord_array(prefiltered_cloud_ptr, centroid_coord_array); // generate a vector of coordinates
    cout << "There are "<<centroid_coord_array.size()<<" valid clusters in the pcl"<<endl;
    /////////////////////////////// print out the coordinates
    //if (centroid_coord_array.size() > 1){cout<<"*************************************************************************************************************************"<<endl;}
    //    cout <<"centroid_coord_array is :\n[";
    //    for (int i = 0; i<centroid_coord_array.size(); i++) { cout <<centroid_coord_array[i]<< "] "<<endl;} // print out the array

    //    // nb the old version used to publish centroids here

    /////////////////// PUBLISH THE CLUSTER CENTROIDS
    process_centroids(centroid_coord_array); // call the process centroids method with our vector of centroids



}
//void tracker::transform_pointcloud_to_odom()
void tracker::generate_coord_array (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, vector<Eigen::VectorXd> &centroid_coord_array) {
    // INPUTS: input_cloud: pointer to the pointcloud produced by the velodyne relative to "base"
    //          centroid_cluster_array: pointer to output cloud which contains the clusters
    // This method takes the pcloud input_cloud, downsizes it with a voxel grid, splits up the grid into
    // clusters, and returns the largest cluster as "cloud_cluster", a pointer to a PointCloud
    if(verbose_)
        cout << "generate_cluster_array() called" <<endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    // COMMENTED OUT THE DOWNSAMPLING BECAUSE OUR POINT CLOUD IS ALREADY SPARSE



    ///////////////// DOWNSAMPLE THE INPUT WITH A VOXEL GRID /////////////////

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    // make new point cloud pointer to contain the filtered point cloud
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // set vg input to input cloud
    vg.setInputCloud (input_cloud);

    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    // vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*cloud_filtered);
    cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl; //*



    //    cloud_filtered = input_cloud;

    // CONVERT FROM PCL:PC1 TO SM:PC2
    sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
    pcl::toROSMsg(*cloud_filtered,msg_to_publish ); // con
    pub_ds_.publish (msg_to_publish);


        /////////////////* DECOMPOSE cloud_filtered INTO CLUSTERS */////////////////
        if(verbose_)
            cout << "Initiating Segmentation objects" << endl;
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);


        seg.setDistanceThreshold (0.03); //
        // Loop through the cloud, performing the clustering operation
        int i=0, nr_points = (int) cloud_filtered->points.size (); // downsampling to a factor of 0.3
        double downsample_factor = 0.3;// downsampling to a factor of 0.3
        if(verbose_)
            cout << "Segmenting planar components" << endl;
        while (cloud_filtered->points.size () >downsample_factor* nr_points) // note
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                cout << "Could not estimate a planar model for the given dataset." << endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            //cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;

            //define cloud_f for extracting into
            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_filtered); //extract into cloud_filtered
        }

    // Create the KdTree object for the search method of the extraction
    //cout << "Creating Kdtree objects" << endl;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    //cout << "setting up cluster objects" << endl;
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

    ec.setClusterTolerance (0.3); // 2cm: too small, we split one object into many, too big, we merge objects into one.
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (150);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //////////////////////* Perform the extraction ////////////////////////////////////////

    cout << "extracting the clusters" << endl;
    int MAX_CLUSTER_SIZE = 150; // ditch all clusters with more points than this
    vector<pcl::PointIndices>::const_iterator it;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>); // make a local cloud in which to store the outputs

    int colour_counter = 0;
    double new_r,new_g,new_b;

    for (it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // loop through the clusters
    {
        //centroid_cluster_array[it]
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }

        // Create a random colour
        new_r = (colour_counter*40)%255;
        new_g = (255 - colour_counter*60)%255;
        new_b = (colour_counter*100)%255;

        cout << "Colouring this pointcloud as ("<<new_r<<","<<new_g<<","<<new_b<<")"<<endl;
        colour_counter += 1; // increment the colour counter
        for (size_t i = 0; i < cloud_cluster->points.size (); i++){ // colour the points according to this random colour
            cloud_cluster->points[i].r = new_r;
            cloud_cluster->points[i].g = new_g;
            cloud_cluster->points[i].b = new_b;
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;


        // publish it!
        sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
        pcl::toROSMsg(*cloud_cluster,msg_to_publish);
        msg_to_publish.header.frame_id = "odom"; // CHANGED THIS TO BASE INSTEAD OF ODOM BECAUSE WE WERE PUBLISHING POINTS IN THE WRONG PLACE
        pub_centroid_.publish (msg_to_publish); // this is not publishing correctly


        if (cloud_cluster->points.size() < MAX_CLUSTER_SIZE) { // if there are enough points in the cloud
            cout << "\n\n** Returning cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
            VectorXd coord_centroid(3); // because we are in 3d
            generate_centroid(cloud_cluster, coord_centroid);
            //            cout << "[inside generate_cluster_array()] coord_centroid is \n"<<coord_centroid<<endl;
            centroid_coord_array.push_back(coord_centroid); // we want to keep this cluster
        } else {
            //else just ignore and keep looping
            cout << "Ditching a cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
        }
    }
}
void tracker::generate_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_ptr, VectorXd &coord_centroid) {
    // loop through the point cloud cluster_ptr, adding points to a centroid

    cout << "generate_centroid() called" <<endl;

    pcl::CentroidPoint<pcl::PointXYZRGB> centroid; // Initialise a point to store the centroid inoutput_topic
    if (verbose_)
        cout<< "adding points to centroid"<<endl;
    for (size_t i = 0; i < cluster_ptr -> points.size (); ++i) {// add all the points to the centroid
        centroid.add (cluster_ptr -> points[i]);
    }

    pcl::PointXYZRGB c; // the centre point
    centroid.get (c);// Fetch centroid using `get()`
    if (verbose_) {
        cout << "\n Centroid is located at "
             << c
             << endl;

    }
    coord_centroid << c.x, c.y, c.z; // assign coords to coord_centroid
    cout << "generate_centroid() is returning a coord_centroid at :\n"<<coord_centroid<<endl;
}
void tracker::apply_passthrough_filter(const sensor_msgs::PointCloud2ConstPtr input_cloud, sensor_msgs::PointCloud2 &output_cloud) {

    // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input_cloud, *cloud); // convert cloud_msg into a pcl edition

    double radius = 8.0; // maximum distance from base we are interested in
    double max_height = 2.0; // max height of z filter in metres
    double min_height = -0.2; // min height of z filter in metres
    //    double min_height = -2;
    // setup the x filter
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-radius, radius); // limits
    pcl::PCLPointCloud2ConstPtr cloudPtr_x(cloud); // create a pointer called cloudPtr to use for our filter
    pass.setInputCloud (cloudPtr_x); //set the input cloud for our filter
    pcl::PCLPointCloud2* output_cloud_x = new pcl::PCLPointCloud2; // initiate our PC2 to send
    pass.filter(*output_cloud_x); // do the filtering operation


    // setup the  y filter
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-radius, radius); // limits
    pcl::PCLPointCloud2ConstPtr cloudPtr_y(output_cloud_x); // create a pointer called cloudPtr to use for our filter
    pass.setInputCloud (cloudPtr_y); //set the input cloud for our filter
    pcl::PCLPointCloud2* output_cloud_y = new pcl::PCLPointCloud2; // initiate our PC2 to send
    pass.filter(*output_cloud_y); // do the filtering operation

    // setup the z filter
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_height, max_height); // limits
    pcl::PCLPointCloud2ConstPtr cloudPtr_z(output_cloud_y); // create a pointer called cloudPtr to use for our filter
    pass.setInputCloud (cloudPtr_z); //set the input cloud for our filter
    pcl::PCLPointCloud2* output_cloud_z = new pcl::PCLPointCloud2; // initiate our PC2 to send
    pass.filter(*output_cloud_z); // do the filtering operation

    // Convert to ROS data type
    pcl_conversions::moveFromPCL(*output_cloud_z, output_cloud); // convert into the sensor_msgs format

}
void tracker::init_kf(VectorXd x0) { // initialises the kalman filter with initial vector x0
    cout<< "initiating kalman_filter"<<endl;
    kf_.init(0, x0); // initialise the kalman filter
    cout<<"kalman filter initiated"<<endl;
    return;

}
void tracker::update_kf(VectorXd y) { //update our state estimate with measurement y
    cout<<"update_kf()"<<endl;
    kf_.update(y); // call the update method of our (private) kalman filter

    // now publish the latest output of kf
}
VectorXd tracker::get_state() { // talks to our kalman filter
    return kf_.get_state();

}
void tracker::process_centroids(vector<VectorXd> centroid_coord_array) {
    // first get this working
    cout <<"process_centroids() with array :\n[";
    for (int i = 0; i<centroid_coord_array.size(); i++) { cout <<centroid_coord_array[i]<< "] "<<endl;} // print out the array

    // 1. Select which of the VectorXds in the centroid_coord_array is the correct one

    VectorXd previous_estimate(3), vector_from_prev(3);
    VectorXd new_measurement(3); //for the kalman filter which requires VectorXd
    double dist_from_prev;
    double best_dist_from_prev = 20; // this is the give up distance - 4m is too far

    ////////////////////////////////////
    previous_estimate = get_state();
    cout<<"previous_estimate =\n"<<previous_estimate<<endl;
    //    previous_estimate << 4.6, 7.5, 9.3;
    ///////////////////////////////////////
    int new_est_index = -1; // e.g. nothing found
    for (int i = 0; i<centroid_coord_array.size(); i++) {
        //publish a transform
        stringstream target_frame_id;
        target_frame_id << "centroid_"<<i;
        publish_transform(centroid_coord_array[i], target_frame_id.str());

        vector_from_prev = centroid_coord_array[i] - previous_estimate;
        //        cout << "vector_from_prev =\n"<< vector_from_prev<<endl;

        dist_from_prev = sqrt(vector_from_prev.squaredNorm());
        cout << "dist_from_prev = "<< dist_from_prev<<"m"<<endl;

        if (dist_from_prev < best_dist_from_prev) { // if this estimate is good enough and better than other centroids
            // it is the nearest centroid so publish it
            cout << "new best coordinate found at index "<<i<<endl;
            new_est_index = i;
            best_dist_from_prev = dist_from_prev; // update best distance to this one
        }
    }

    if (new_est_index != -1) { // e.g. if we have found a viable coordinate
        cout << "updating the filter with this coordinate"<<endl;
        new_measurement << centroid_coord_array[new_est_index][0],centroid_coord_array[new_est_index][1],centroid_coord_array[new_est_index][2];
        update_kf(new_measurement); //update the kalman filter with this estimate
        MatrixXd P;
        P = kf_.get_P(); //get covariance
        publish_marker(get_state(), P(0,0),P(1,1),P(2,2)); // publish the marker
        //        publish_transform(new_measurement);
    }
}
