#include "include/tracker.h"
using namespace std;
//using namespace Eigen;

Tracker::Tracker(ros::NodeHandle nh,
                 int max_cluster_size,
                 int min_cluster_size,
                 double cluster_tolerance,
                 double seg_dist_threshold,
                 bool verbose
                 ) :
    nh_(nh), max_cluster_size_(max_cluster_size), min_cluster_size_(min_cluster_size),cluster_tolerance_(cluster_tolerance), seg_dist_threshold_(seg_dist_threshold), verbose_(verbose) // initiate the nodehandle
{   // Constructor: sets up
    cout<< "Tracker constructor called "<<endl;
    initialiseSubscribersAndPublishers(); //initialise the subscribers and publishers
}

///// General Pointcloud Methods
void Tracker::callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) { // the callback fcn
    // this is called by pointclouds from the velodyne, processing them, decomposing into clusters, and publishing cluster coordinates
    cout<< "************************************\nInitiating Callback\n***********************************"<<endl;
    sensor_msgs::PointCloud2 msg_to_publish; // we will use this for all the pointclouds we need to publish
    // Publish the cloud
    pub_raw_.publish (*cloud_msg); // publish the raw cloud

    /////////// Apply a passthrough filter and publish the result
    sensor_msgs::PointCloud2 bounded_cloud;
    applyPassthroughFilter(cloud_msg, bounded_cloud); // remove all points outside of a predefined bod
    pub_zfilt_.publish (bounded_cloud); // Publish the output

    ////////// Transform the cloud into the odom frame to eliminate base motion
    sensor_msgs::PointCloud2 transformed_cloud;
    applyBaseOdomTransformation(bounded_cloud, transformed_cloud);
    pub_trans_.publish (transformed_cloud); // Publish the cloud

    /////////   if cloud has less than 10 points, jump out of the callback
    if (transformed_cloud.width < 10) {
        cout << "*** Cloud has too few points, waiting for the next one. ****" << endl;
        return;
    }


    //////// Downsample with a Voxel Grid and publish
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
    convertSM2ToPclPtr(transformed_cloud, cloud_ptr); // Convert variable to correct type for VoxelGrid
//    applyVoxelGrid(cloud_ptr); // apply the voxel_grid using a leaf size of 1cm
//    pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
//    pub_ds_.publish (msg_to_publish);

    //////// Remove non planar points e.g. outliers http://pointclouds.org/documentation/tutorials/planar_segmentation.php#id1
    removeOutOfPlanePoints(cloud_ptr);
    pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
    pub_seg_filter_.publish (msg_to_publish);

    /////////// Split up pointcloud into a vector of pointclouds, 1 for each cluster
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector;
    splitCloudPtrIntoClusters(cloud_ptr,cloud_cluster_vector);

    //////// Loop through clusters and put their centroids into a vector
    vector <VectorXd> centroid_coord_array;
    getCentroidsOfClusters(cloud_cluster_vector, centroid_coord_array); // generate a vector of coordinates

    /////// Publish the cluster centroids
    if (centroid_coord_array.size() != 0) // if there are any clusters visible
        processCentroidCoords(centroid_coord_array); // call the process centroids method with our vector of centroids
    else
        cout << "No valid clusters visible after getCentroidsOfClusters()"<<endl;
}

void Tracker::applyPassthroughFilter(const sensor_msgs::PointCloud2ConstPtr input_cloud, sensor_msgs::PointCloud2 &output_cloud) {
    if (verbose_)
        cout <<"Applying Passthrough Filter" << endl;
    // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input_cloud, *cloud); // convert cloud_msg into a pcl edition

    double radius = 8.0; // maximum distance from base we are interested in
    double max_height = 2.0; // max height of z filter in metres
    double min_height = -0.3; // min height of z filter in metres
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

void Tracker::applyBaseOdomTransformation(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &output_cloud) {
    // transforms the cloud into the odom frame from base frame
    if (verbose_)
        cout<<"Transforming Pointcloud into odom frame. Waiting for transform"<<endl;
    string target_frame = "odom", base_frame = "base"; // target frame and base frame for transformfor transform
    odom_base_ls_->waitForTransform(target_frame, base_frame, ros::Time::now(), ros::Duration(1.0) ); // wait until a tf is available before transforming

    if (verbose_)
        cout<<"transforming pcloud using time "<< ros::Time::now()<<endl;
    pcl_ros::transformPointCloud(target_frame, input_cloud, output_cloud, *odom_base_ls_); // perform the transformation

    cout << "1"<<endl;
    return;
}

void Tracker::removeOutOfPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr) {
    //////// Create a planar segmentation model <- NOT SURE WHAT THIS DOES EXACTLY, SEE http://pointclouds.org/documentation/tutorials/planar_segmentation.php#id1
    if(verbose_)
        cout << "Initiating Segmentation objects" << endl;
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()); // create a new pointcloud pointer to hold the planar cloud
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC); // using RANSAC to determine inliers
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (seg_dist_threshold_); // how close a point must be to the model in order to be considered an inlier: 3cm in this case

    // Loop through the cloud, performing the segmentation operation
    int i=0, nr_points = (int) cloud_ptr->points.size ();
    double downsample_factor = 0.3;// downsampling to a factor of 0.3
    if(verbose_)
        cout << "Segmenting planar components" << endl;
    while (cloud_ptr->points.size () > downsample_factor * nr_points) // note
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_ptr);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) { // check that there are inliers
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;}

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_ptr);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane); // Get the points associated with the planar surface, reject the rest
        if (verbose_)
            cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;
        // remove the outlying points from cloud_ptr
        extract.setNegative(true);
        extract.filter(*cloud_ptr);
        if (verbose_)
            cout << "cloud_ptr has "<<cloud_ptr->points.size ()<<" points left" << endl;
    }
    if (verbose_)
        cout <<"RANSAC filtering complete, returning cloud_ptr with "<<cloud_ptr->points.size ()<<" points"<<endl;
    return;
}

void Tracker::convertSM2ToPclPtr(sensor_msgs::PointCloud2 input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_ptr){
    // converts a sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    if (verbose_)
        cout << "converting sensor_msgs::pointcloud2 into a pointer"<<endl;
    pcl::PointCloud<pcl::PointXYZRGB> input_cloud_pcl;  // pcl version of the point cloud
    pcl::fromROSMsg (input_cloud, input_cloud_pcl); // convert from sm to pcl version
    // change to a boost shared pointer so we can use pcl::VoxelGrid
    typedef pcl::PointCloud<pcl::PointXYZRGB>  mytype;
    output_ptr = boost::make_shared <mytype> (input_cloud_pcl);
    return;
}

void Tracker::applyVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr){
    // applies a
    pcl::VoxelGrid<pcl::PointXYZRGB> vg; // Create the filtering object:
    vg.setInputCloud (cloud_ptr); // set vg input to input cloud
    vg.setLeafSize (0.01f, 0.01f, 0.01f); // downsample the dataset using a leaf size of 1cm
    vg.filter (*cloud_ptr);
    if (verbose_)
        cout << "PointCloud after filtering has: " << cloud_ptr->points.size ()  << " data points." << endl;
    return;
}

void Tracker::splitCloudPtrIntoClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_cluster_vector) {
    if (verbose_)
        cout << "splitting cloud_ptr with "<< cloud_ptr->points.size ()<<" points into clusters" <<endl;
    //////// Get a vector of index objects, 1 for each cluster
    vector<pcl::PointIndices> cluster_indices = getClusterIndices(cloud_ptr);

    //////// For each item in the index vector, extract the corresponding points into a pointcloud vector
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>); // make a local cloud_ptr in which to store the outputs
    vector<pcl::PointIndices>::const_iterator it;
    for (it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // loop through the clusters
    {
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) { // for this group of indices, move all the points from cloud_ptr into cloud_cluster
            cloud_cluster->points.push_back (cloud_ptr->points[*pit]); //*
        }
        assignRandomColour(cloud_cluster); // give cloud_cluster a random colour
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster_vector.push_back(cloud_cluster); // move cloud_cluster into the output vector
        if (verbose_)
            cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
    }
    return;
}

void Tracker::assignRandomColour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud) {
    // assigns a random colour to pointcloud ptr input_cloud, colour is set by colour_counter
    // Create a random colour
    int colour_counter = rand() % 100; // generate a random number between 0 and 99
    double new_r,new_g,new_b;
    new_r = (colour_counter*40)%255;
    new_g = (255 - colour_counter*60)%255;
    new_b = (colour_counter*100)%255;

    if (verbose_)
        cout << "Colouring this pointcloud as ("<<new_r<<","<<new_g<<","<<new_b<<")"<<endl;
    for (size_t i = 0; i < input_cloud->points.size (); i++){ // colour the points according to this random colour
        input_cloud->points[i].r = new_r;
        input_cloud->points[i].g = new_g;
        input_cloud->points[i].b = new_b;
    }
    return;
}

vector<pcl::PointIndices> Tracker::getClusterIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr) {
    // Create the KdTree object for the search method of the extraction
    if (verbose_)
        cout << "getting indices of each cluster from cloud_ptr with "<< cloud_ptr->points.size ()<<" points" << endl;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_ptr);

    //cout << "setting up cluster objects" << endl;
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (max_cluster_size_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_ptr);
    ec.extract (cluster_indices);
    if (verbose_)
        cout <<"cloud_ptr has been split into clusters, returning cluster_indices with "<< cluster_indices.size() << " elements" <<endl;
    return cluster_indices;
}

////// Centroid Pointcloud Methods
void Tracker::getCentroidsOfClusters (vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector, vector<Eigen::VectorXd> &centroid_coord_array) {
    // loops through cloud_cluster vector and gets the centroid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;
    int MAX_CLUSTER_SIZE = 140; // if there are more points than this
    for (int i = 0; i<cloud_cluster_vector.size(); i++) {
        cloud_cluster = cloud_cluster_vector[i]; // extract one cluster
        // publish it!
        sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
        pcl::toROSMsg(*cloud_cluster,msg_to_publish);
        msg_to_publish.header.frame_id = "odom"; // CHANGED THIS TO BASE INSTEAD OF ODOM BECAUSE WE WERE PUBLISHING POINTS IN THE WRONG PLACE
        pub_centroid_.publish (msg_to_publish); // this is not publishing correctly


        if (cloud_cluster->points.size() < MAX_CLUSTER_SIZE) { // if there are enough points in the cloud
            cout << "\n\n** Returning cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
            VectorXd coord_centroid(3); // because we are in 3d
            getClusterCentroid(cloud_cluster, coord_centroid);
            //            cout << "[inside generate_cluster_array()] coord_centroid is \n"<<coord_centroid<<endl;
            centroid_coord_array.push_back(coord_centroid); // we want to keep this cluster
        } else {
            //else just ignore and keep looping
            cout << "Ditching a cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
        }
    }

    ///////// Print out a line of stars if there are 2 centroids in the array because this is a
    cout << "There are "<<centroid_coord_array.size()<<" valid clusters in the pcl"<<endl;
    if (centroid_coord_array.size() > 1){cout<<"*************************************************************************************************************************"<<endl;}

}

void Tracker::getClusterCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_ptr, VectorXd &coord_centroid) {
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

void Tracker::processCentroidCoords(vector<VectorXd> centroid_coord_array) {
    // Loops through a vector of centroids and updates the kalman filter with the one closest to previous estimate.
    // Publishes transforms for all estimates

    int num_centroids = centroid_coord_array.size();
    int new_est_index = -1; // stores the index of the centroid to use. -1 indicates that no suitable estimate has been found
    VectorXd kf_prev_state; // for previous kalman filter state

    if (verbose_) {
        cout <<"processCentroids() with array :\n[";
        for (int i = 0; i<num_centroids; i++) { cout <<centroid_coord_array[i]<< "] "<<endl;}} // print out the array

    ////// If there is only 1 centroid, update kf with that
    if (num_centroids < 2) { // 1 centroid

        publishTransform(centroid_coord_array[0], "centroid_0"); // publish a transform
        new_est_index = 0; // use the 1st index

    } else { ///// If there are more than one centroid

        VectorXd innovation_vector(3); //new_measurement stores last kf output, innovation_vector stores distance between this and new measurement
        vector <double> distances_from_prev_est; // stores the distances between measurements and prior estimate in a vector
        kf_prev_state = getState(); // get the previous estimate from the Kalman Filter to compare against

        for (int i = 0; i<num_centroids; i++) { // loop through the centroids
            ////// publish a transform with an id based on i
            stringstream target_frame_id;
            target_frame_id << "centroid_"<<i;
            publishTransform(centroid_coord_array[i], target_frame_id.str());

            ///// update distance vector
            innovation_vector = centroid_coord_array[i] - kf_prev_state; // get difference vector between new measurement and previous
            distances_from_prev_est.push_back(sqrt(innovation_vector.squaredNorm())); // this value is in metres
            if (verbose_)
                cout << "distances_from_prev_est["<<i<<"] = "<< distances_from_prev_est[i]<<"m"<<endl;
        }
        ///// Set index of our centroid as the one with the smallest distance
        new_est_index = min_element(distances_from_prev_est.begin(),distances_from_prev_est.end()) - distances_from_prev_est.begin();
//        new_est_index = *min_element(distances_from_prev_est.begin(),distances_from_prev_est.end()); // take the minimum distance from the array
        distances_from_prev_est.clear(); // remove all the elements from the vector for next time
    }

    ////// update kf with new_est_index
    VectorXd new_measurement(3); // stores the measurement we will use
    MatrixXd P; // stores Kalman Filter Variance
    if (new_est_index != -1) { // e.g. if we have found a viable coordinate
        if (verbose_)
            cout << "updating the filter with coordinate at index "<<new_est_index<<endl;
        new_measurement << centroid_coord_array[new_est_index][0],centroid_coord_array[new_est_index][1],centroid_coord_array[new_est_index][2]; // extract the coordinate

        ///// Update the kalman_filter
        updateKf(new_measurement); //update the kalman filter with this estimate

        P = kf_.getP(); //get covariance
        kf_prev_state = getState();
        if (verbose_) {
            cout << "New measurement covariance is\n"<<P <<endl;
            cout << "new Kf state is\n"<<kf_prev_state <<endl; }
        publishMarker(kf_prev_state, P(0,0),P(1,1),P(2,2)); // publish the marker
        publishTransform(new_measurement, "kalman_filter_state");
    }
}

///// I/O Methods
void Tracker::publishTransform(VectorXd coordinates, string target_frame_id) {
    // publishes a transform on broadcaster br_ at the 3D coordinate Vector coordinates
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(coordinates[0],coordinates[1],coordinates[2]) );
    tf::Quaternion q; // initialise the quaternion q for the pose angle
    q.setEulerZYX(0, 0, 0);
    transform.setRotation(q);

    string base_frame_id = "odom"; // we are based in the odom frame
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, target_frame_id));
}

void Tracker::initialiseSubscribersAndPublishers() {
    string input_topic = "/velodyne/point_cloud_filtered"; // topic is the pointcloud from the velodyne
    point_cloud_sub_ = nh_.subscribe(input_topic, 1, &Tracker::callback, this); // Create a ROS subscriber for the input point cloud that calls the callback

    // Create ROS publishers for the output point clouds
    string topic_raw = "pcl_raw", topic_trans = "pcl_trans", topic_zfilt = "pcl_zfilt", topic_ds = "pcl_ds", topic_seg_filt = "pcl_seg_filter", topic_centroid = "pcl_centroid";

    pub_raw_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_raw, 1);
    pub_trans_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_trans, 1);
    pub_zfilt_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_zfilt, 1);
    pub_ds_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_ds, 1);
    pub_seg_filter_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_seg_filt, 1);
    pub_centroid_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_centroid, 1);
    // Create a publisher for the kf marker
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>( "Tracker_marker", 0 ); // this name shows up in RVIZ
    // Create a transformListener to enable translation from odom to base frames with our pointcloud.
    odom_base_ls_.reset(new tf::TransformListener); // initialise the odom_base transform listener- so we can transform our output into odom coords
    return;
}

void Tracker::publishMarker(VectorXd x_hat,double scale_x,double scale_y,double scale_z) {
    /// This publishes a cylinder, size set by scale_* onto publisher vis_pub at location x_hat

    visualization_msgs::Marker marker; // initiate the marker

    marker.header.frame_id = "odom"; // we want to publish relative to the odom frame
    marker.header.stamp = ros::Time();
    marker.ns = "kalman_filter_marker";  // call our marker kalman_filter_marker
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER; // make it a CYLINDER. CUBE and SPHERE also work
    marker.action = visualization_msgs::Marker::ADD;

    // assign the marker location according to x_hat
    marker.pose.position.x = x_hat[0];
    marker.pose.position.y = x_hat[1];
    marker.pose.position.z = x_hat[2];

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    double scaler = .3; // the overall marker size
    // set the marker size as an input params
    marker.scale.x = scale_x*scaler;
    marker.scale.y = scale_y*scaler;
    //    marker.scale.z = scale_z*scaler;
    marker.scale.z = 2; // keeping z at a constant 2 metres tall

    marker.color.a = 0.4; // Setting alpha to semi transparent
    marker.color.r = 0.0;
    marker.color.g = 1.0; // making it green
    marker.color.b = 0.0;
    pub_marker_.publish( marker ); // publish
}

////// Kalman Filter Methods
void Tracker::setupKalmanFilter(VectorXd x0,double dt,const Eigen::MatrixXd& A, const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P) { // initialises the kalman filter with initial vector x0
    cout<< "initiating kalman_filter"<<endl;
    kf_ = KalmanFilter(dt, A, C, Q, R, P, false); // call the kalman filter constructor
    kf_.init(0, x0); // initialise the kalman filter
    cout<<"kalman filter initiated"<<endl;
    return;
}

void Tracker::updateKf(VectorXd y) { //update our state estimate with measurement y
//    cout<<"updateKf()"<<endl;
    kf_.update(y); // call the update method of our (private) kalman filter
}

VectorXd Tracker::getState() { // talks to our kalman filter
    return kf_.getState();
}
