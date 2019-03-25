#include "include/mo_tracker.h" // includes a bunch of stuff inc kalman filter, tracklet and pairing

using namespace std;
//using namespace Eigen;


MOTracker::MOTracker(ros::NodeHandle nh,
                     pcl_param_struct pcl_params,
                     kf_param_struct kf_params,
                     tracker_param_struct tracker_params,
                     io_param_struct io_params,
                     bool verbose
                     ) :
    nh_(nh),  pcl_params(pcl_params), kf_params(kf_params), tracker_params(tracker_params), io_params(io_params), verbose_(verbose)// initiate the nodehandle
{
    // Constructor: sets up
    cout<< "MOTracker constructor called "<<endl;
    initialiseSubscribersAndPublishers(); //initialise the subscribers and publishers
    setupResultsCSV();

    //    results_file_.close();

}

///// General Pointcloud Methods
void MOTracker::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) { // the callback fcn
    // this is called by pointclouds from the velodyne, processing them, decomposing into clusters, and publishing cluster coordinates
    cout<< "************************************\nInitiating Callback\n***********************************"<<endl;
    sensor_msgs::PointCloud2 msg_to_publish; // we will use this for all the pointclouds we need to publish
    // Publish the cloud
    if (io_params.publishing)
        pub_raw_.publish (*cloud_msg); // publish the raw cloud

    /////////// Apply a passthrough filter and publish the result
    ///
    sensor_msgs::PointCloud2 bounded_cloud;

    if (pcl_params.apply_passthrough_filter == 1) {
        applyPassthroughFilter(cloud_msg, bounded_cloud); // remove all points outside of a predefined bod
        if (io_params.publishing)
            pub_zfilt_.publish (bounded_cloud); // Publish the output
    } else {
        cout<<"not applying passthrough_filter"<<endl;
        // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*cloud_msg, *cloud); // convert cloud_msg into a pcl edition
        // Convert to ROS data type
        pcl_conversions::moveFromPCL(*cloud, bounded_cloud); // convert into the sensor_msgs format
    }

    ////////// Transform the cloud into the odom frame to eliminate base motion
    sensor_msgs::PointCloud2 transformed_cloud;
    applyBaseOdomTransformation(bounded_cloud, transformed_cloud);
    if (io_params.publishing)
        pub_trans_.publish (transformed_cloud); // Publish the cloud

    /////////   if cloud has less than 10 points, jump out of the callback
    if (transformed_cloud.width < 10) {
        cout << "*** Cloud has too few points, waiting for the next one. ****" << endl;
        return;
    }

    //////// Downsample with a Voxel Grid and publish
    // Convert variable to correct type for VoxelGrid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
    convertSM2ToPclPtr(transformed_cloud, cloud_ptr);

    if (pcl_params.apply_voxel_grid == 1) {
        applyVoxelGrid(cloud_ptr, true); // apply the voxel_grid using a leaf size of 1cm
        // publish
        if (io_params.publishing) {
            cout << "publishing voxel grid"<<endl;
            pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
            pub_ds_.publish (msg_to_publish);
        }
    }
    //////// Remove non planar points e.g. outliers http://pointclouds.org/documentation/tutorials/planar_segmentation.php#id1
    if (pcl_params.apply_planar_outlier_removal == 1) {
        removeOutOfPlanePoints(cloud_ptr, true);
        if (io_params.publishing)
        {
            pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
            pub_seg_filter_.publish (msg_to_publish);
        }
    }

    /////////// Split up pointcloud into a vector of pointclouds, 1 for each cluster
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector;
    splitCloudPtrIntoClusters(cloud_ptr,cloud_cluster_vector);

    //////// Loop through clusters and put their centroids into a vector
    vector <VectorXd> centroid_coord_array;
    getCentroidsOfClusters(cloud_cluster_vector, centroid_coord_array); // generate a vector of coordinates

    /////// Publish the cluster centroids
    if (centroid_coord_array.size() != 0) // if there are any clusters visible
        manageTracklets(centroid_coord_array, cloud_msg->header.stamp.toSec(), false); // call the process centroids method with our vector of centroids
    else
        cout << "No valid clusters visible after getCentroidsOfClusters()"<<endl;
}

void MOTracker::poseArrayCallback(const geometry_msgs::PoseArray &pose_array)
{
    // this callback is called by the posearray
    if (verbose_)
        cout <<"\n\n**********poseArrayCallback()"<<endl;
    vector <VectorXd> realsense_coords; // initiate the array (empty
    // generate a vector of coordinates in the same manner as the pointclouds
    try {
        for (int i = 0; i < pose_array.poses.size(); i++) {
            VectorXd pose_coords(3);
            pose_coords[0] = pose_array.poses[i].position.x;
            pose_coords[1] = pose_array.poses[i].position.y;
            pose_coords[2] = pose_array.poses[i].position.z; // << pose_array.poses[i].position.y << pose_array.poses[i].position.z;
            cout<<"pose coords is \n"<<pose_coords<<endl;

            // convert these coordinates into the odom frame
            VectorXd translated_pose_coords(3);
            //            applyRealsenseOdomTransformation(pose_coords, translated_pose_coords, pose_array.header.stamp, true);

            realsense_coords.push_back(pose_coords);
        }
        // call manageTracklets with the time in seconds as a double
        manageTracklets(realsense_coords, pose_array.header.stamp.toSec(), true);
        cout << "centroid coords updated with a new pose"<<endl;

    } catch(const std::exception&)
    {
        cout <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!Got a transform error"<<endl;
    }
    ros::Duration(0.5).sleep(); //sleep for 0.5s
    return;

}

void MOTracker::applyPassthroughFilter(const sensor_msgs::PointCloud2ConstPtr input_cloud, sensor_msgs::PointCloud2 &output_cloud) {
    if (verbose_)
        cout <<"Applying Passthrough Filter" << endl;
    // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input_cloud, *cloud); // convert cloud_msg into a pcl edition

    //    double radius = pcl_params.; // maximum distance from base we are interested in
    double max_height = 2.0; // max height of z filter in metres
    //    double min_height = -0.3; // min height of z filter in metres
    //    double min_height = -2;
    // setup the x filter
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-pcl_params.box_x, pcl_params.box_x); // limits
    pcl::PCLPointCloud2ConstPtr cloudPtr_x(cloud); // create a pointer called cloudPtr to use for our filter
    pass.setInputCloud (cloudPtr_x); //set the input cloud for our filter
    pcl::PCLPointCloud2* output_cloud_x = new pcl::PCLPointCloud2; // initiate our PC2 to send
    pass.filter(*output_cloud_x); // do the filtering operation


    // setup the  y filter
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-pcl_params.box_y, pcl_params.box_y); // limits
    pcl::PCLPointCloud2ConstPtr cloudPtr_y(output_cloud_x); // create a pointer called cloudPtr to use for our filter
    pass.setInputCloud (cloudPtr_y); //set the input cloud for our filter
    pcl::PCLPointCloud2* output_cloud_y = new pcl::PCLPointCloud2; // initiate our PC2 to send
    pass.filter(*output_cloud_y); // do the filtering operation

    // setup the z filter
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (pcl_params.min_height, max_height); // limits
    pcl::PCLPointCloud2ConstPtr cloudPtr_z(output_cloud_y); // create a pointer called cloudPtr to use for our filter
    pass.setInputCloud (cloudPtr_z); //set the input cloud for our filter
    pcl::PCLPointCloud2* output_cloud_z = new pcl::PCLPointCloud2; // initiate our PC2 to send
    pass.filter(*output_cloud_z); // do the filtering operation

    // Convert to ROS data type
    pcl_conversions::moveFromPCL(*output_cloud_z, output_cloud); // convert into the sensor_msgs format
}

void MOTracker::applyBaseOdomTransformation(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &output_cloud) {
    // transforms the cloud into the odom frame from base frame
    if (verbose_)
        cout<<"Transforming Pointcloud into odom frame. Waiting for transform"<<endl;
    string target_frame = "odom", base_frame = "base"; // target frame and base frame for transformfor transform
    odom_base_ls_->waitForTransform(target_frame, base_frame, ros::Time::now(), ros::Duration(.1) ); // wait until a tf is available before transforming

    if (verbose_)
        cout<<"transforming pcloud using time "<< ros::Time::now()<<endl;
    pcl_ros::transformPointCloud(target_frame, input_cloud, output_cloud, *odom_base_ls_); // perform the transformation

    return;
}


void MOTracker::removeOutOfPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, bool verbose) {
    //////// Create a planar segmentation model <- NOT SURE WHAT THIS DOES EXACTLY, SEE http://pointclouds.org/documentation/tutorials/planar_segmentation.php#id1
    if(verbose)
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
    seg.setDistanceThreshold (pcl_params.seg_dist_threshold); // how close a point must be to the model in order to be considered an inlier: 3cm in this case

    // Loop through the cloud, performing the segmentation operation
    int i=0, nr_points = (int) cloud_ptr->points.size ();
    double downsample_factor = 0.7;// downsampling to a factor of 0.3
    if(verbose)
        cout << "Segmenting planar components" << endl;
    while (cloud_ptr->points.size () > pcl_params.downsample_factor * nr_points) // note
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
        if (verbose)
            cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;
        // remove the outlying points from cloud_ptr
        extract.setNegative(true);
        extract.filter(*cloud_ptr);
        if (verbose)
            cout << "cloud_ptr has "<<cloud_ptr->points.size ()<<" points left" << endl;
    }
    if (verbose)
        cout <<"RANSAC filtering complete, returning cloud_ptr with "<<cloud_ptr->points.size ()<<" points"<<endl;
    return;
}

void MOTracker::convertSM2ToPclPtr(sensor_msgs::PointCloud2 input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_ptr){
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

void MOTracker::applyVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, bool verbose){
    // applies a vg to the pointcloud cloud_ptr
    pcl::VoxelGrid<pcl::PointXYZRGB> vg; // Create the filtering object:
    vg.setInputCloud (cloud_ptr); // set vg input to input cloud
    vg.setLeafSize (0.01f, 0.01f, 0.01f); // downsample the dataset using a leaf size of 1cm
    vg.filter (*cloud_ptr);
    if (verbose)
        cout << "PointCloud after filtering has: " << cloud_ptr->points.size ()  << " data points." << endl;
    return;
}

void MOTracker::splitCloudPtrIntoClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_cluster_vector) {
    if (verbose_)
        cout << "splitting cloud_ptr with "<< cloud_ptr->points.size ()<<" points into clusters" <<endl;
    //////// Get a vector of index objects, 1 for each cluster
    vector<pcl::PointIndices> cluster_indices = getClusterIndices(cloud_ptr);


    vector<pcl::PointIndices>::const_iterator it;
    for (it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // loop through the clusters
    {
        //////// For each item in the index vector, extract the corresponding points into a pointcloud vector
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>); // make a local cloud_ptr in which to store the outputs

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

void MOTracker::assignRandomColour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud) {
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

vector<pcl::PointIndices> MOTracker::getClusterIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr) {
    // Create the KdTree object for the search method of the extraction
    if (verbose_)
        cout << "getting indices of each cluster from cloud_ptr with "<< cloud_ptr->points.size ()<<" points" << endl;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>); //http://pointclouds.org/documentation/tutorials/kdtree_search.php

    tree->setInputCloud (cloud_ptr);

    //cout << "setting up cluster objects" << endl;
    vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (pcl_params.cluster_tolerance);
    ec.setMinClusterSize (pcl_params.min_cluster_size);
    ec.setMaxClusterSize (pcl_params.max_cluster_size);
    //    ec.setClusterTolerance (.4);
    //    ec.setMinClusterSize (40);
    //    ec.setMaxClusterSize (150);

    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_ptr);
    ec.extract (cluster_indices);
    if (verbose_)
        cout <<"cloud_ptr has been split into "<< cluster_indices.size() << " clusters, returning cluster_indices" <<endl;
    return cluster_indices;
}

////// Centroid Pointcloud Methods
void MOTracker::getCentroidsOfClusters (vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector, vector<Eigen::VectorXd> &centroid_coord_array) {
    // loops through cloud_cluster vector and gets the centroid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;

    if (verbose_)
        cout << "investigating a vector with "<<cloud_cluster_vector.size()<<" clusters"<<endl;

    for (int i = 0; i<cloud_cluster_vector.size(); i++) {
        cloud_cluster = cloud_cluster_vector[i]; // extract one cluster
        // publish it!
        if (io_params.publishing)
        {
            sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
            pcl::toROSMsg(*cloud_cluster,msg_to_publish);
            msg_to_publish.header.frame_id = "odom"; // CHANGED THIS TO BASE INSTEAD OF ODOM BECAUSE WE WERE PUBLISHING POINTS IN THE WRONG PLACE
            pub_centroid_.publish (msg_to_publish); // this is not publishing correctly
        }
        if (verbose_)
            cout << "\n\n** Returning cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
        VectorXd coord_centroid(3); // because we are in 3d
        getClusterCentroid(cloud_cluster, coord_centroid);
        //            cout << "[inside generate_cluster_array()] coord_centroid is \n"<<coord_centroid<<endl;
        centroid_coord_array.push_back(coord_centroid); // we want to keep this cluster

    }

    ///////// Print out a line of stars if there are 2 centroids in the array because this is a
    cout << "There are "<<centroid_coord_array.size()<<" valid clusters in the pcl"<<endl;
    if (centroid_coord_array.size() > 1){cout<<"*************************************************************************************************************************"<<endl;}

}

void MOTracker::getClusterCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_ptr, VectorXd &coord_centroid) {
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

void MOTracker::updatePairings(vector<VectorXd> &unpaired_detections, double msg_time, bool isRGBD, bool verbose)
{
    if (verbose) {
        cout << "\nupdatePairings() with "<<unpaired_detections.size()<<" unpaired_detections"<<endl;
        cout << "and "<<tracklet_vector_.size()<<" live tracklets"<<endl;
    }

    double new_distance; // specific for each tracklet
    bool isPaired;
    for (int i = 0; i < tracklet_vector_.size(); i++) // loop through all live tracklets
    {
        Tracklet *this_tracklet = &tracklet_vector_[i];
        isPaired = false; // not paired this iteration

        if (verbose)
            cout << "*****assessing tracklet "<<this_tracklet->getID()<<endl;

        for (int j = 0; j < unpaired_detections.size(); j++) // loop through all unpaired detections
        {
            if (verbose)
                cout << "assessing detection "<<j<<endl;
            VectorXd new_detection = unpaired_detections[j];
            new_distance = this_tracklet->getDistance(new_detection); // get distance of detection from tracklet
            if (verbose)
                cout << "detection "<<j<<" is at distance "<<new_distance<<"m"<<endl;

            // if the tracklet is initialised, use a multiple of max gating distance

            if (new_distance < getMaxGatingDistance(this_tracklet, verbose = true)) // if detection is within range
            {
                if (verbose)
                    cout << "detection "<<j<<" in range, creating a new pairing.\n isRGBD: "<<isRGBD<<endl;
                //create a pairing instance with tracklet ID and detection

                Pairing new_pairing (this_tracklet->getID(), unpaired_detections[j], new_distance, isRGBD); // make a new pairing with MOTracker ID and detection coord

                unpaired_detections.erase(unpaired_detections.begin() + j); // move this one out of the array

                isPaired = true; // we have paired this tracklet

                pairing_vector_.push_back(new_pairing); // add to pairing vector
                if (verbose)
                    cout << "MOTracker "<<this_tracklet->getID()<<" has been paired"<<endl;
                cout << "pairing vector has length "<<pairing_vector_.size()<<endl;
            }
        }

        if (!isPaired && !isRGBD) // only record misses if we miss it in the pointcloud
        {
            cout << "Couldn't find a detection for tracklet "<< this_tracklet->getID()<<endl;
            cout<<"time is now "<<msg_time<<endl;
            this_tracklet->recordMiss(msg_time);
            cout << "has the tracklet been initialised? "<<this_tracklet->isInitialised()<<endl;

            if (this_tracklet->isInitialised()) {
                // update the kalman filter variance so the covariance circle grows for missed
                //                cout << "!!!!!!!Growing the covariance cylinder!!!!!!!"<<endl;
                KalmanFilter kf = this_tracklet->getKf(); // get the Kf from this tracklet vector
                MatrixXd P = kf.getP(); //get covariance
                VectorXd xhat = kf.getState();
                //                if (verbose) {
                //                    cout << "New measurement covariance is\n"<<P <<endl;
                //                    cout << "new Kf state is\n"<<xhat <<endl; }
                stringstream tracklet_name;
                tracklet_name << "tracklet_"<<this_tracklet->getID(); // identify this marker with the tracklet id
                /// create a title for this marker
                publishMarker(xhat,tracklet_name.str(), P(0,0),P(1,1),P(2,2)); // publish the marker


                //////////////////////////////////////////////////// ACTIVATE THIS WHEN WE HAVE TIME TO TEST
                cout << "TEST ME: I'M AT LINE 471"<<endl;
                VectorXd v = kf.getV();
                // order : detection XYZ, kf XYZ, kf covariance XYZ. Skip 4 cells so have 4+ 1 commas
                results_file_ <<msg_time<<",,,,,"<<this_tracklet->getID()<<","<< xhat[0]<<","<<xhat[1]<<","<<xhat[2]<<","<<P(0,0)<<","<<P(1,1)<<","<<P(2,2)<<","<<v[0]<<","<<v[1]<<"\n";
            }

        } else {
            cout << "this tracklet is paired" <<endl;
        }
    }
}
double MOTracker::getMaxGatingDistance(Tracklet *tracklet_ptr, bool verbose) {
    // returns the max distance at which a new detection can be associated
    if (tracklet_ptr->isInitialised()) {
        KalmanFilter kf = tracklet_ptr->getKf(); // get the Kf from this tracklet vector
        MatrixXd P = kf.getP(); //get covariance
        double max = sqrt(tracker_params.gating_dist_constant*(P(0,0) + P(1,1))/2);
        if (verbose)
            cout <<"MAX_GATING_DIST = "<<max<<"m"<<endl;
        return max; // return sqrt of average covariance is basically the std dev in x, y
    }
    else
    {
        return tracker_params.base_gating_dist;
    }

}
void MOTracker::updateTracklets(vector<VectorXd> &unpaired_detections, double msg_time,bool isRGBD, bool verbose)
{ /*param: vector<VectorXd> &unpaired_detections: address of the unpaired_detections vector
    param: double msg_time: time at which these detections were recorded
    param: bool isRGBD: true if the detection is from the RGBD detector, false if from LIDAR
    param: bool verbose: do we print stuff out?
    method: update the tracklets in tracklet_vector_ with the pairings in pairing_vector_, then put any unused pairings into unpaired_detections
  */
    if (verbose)
        cout << "\nupdateTracklets()"<<endl;

    for (int i = 0; i < tracklet_vector_.size(); i++) // loop through all live tracklets.
    {
        Tracklet *this_tracklet = &tracklet_vector_[i]; // create a pointer to current tracklet

        if (verbose)
            cout << "Looping through "<<pairing_vector_.size()<<" pairings for tracklet "<<this_tracklet->getID()<<endl;
        double best_distance = 100; //in m. try and beat this ////////////////////////////////////////////////
        int best_pairing_index = -1; // index for pairing with shortest distance

        for (int j = 0; j < pairing_vector_.size(); j++) // loop through all paired detections
        {
            if (pairing_vector_[j].getAssociatedTrackletID() == this_tracklet->getID()) // if the pairing is associated with this tracklet (same ID)
            {
                if (verbose)
                    cout << "found a pairing that matches, at distance "<<pairing_vector_[j].getDistanceToTracklet()<<"m"<<endl;
                // are we shorter than best_distance?
                if (pairing_vector_[j].getDistanceToTracklet() < best_distance) //if this pairing is closer than previous one
                {
                    best_pairing_index = j; // set the best pairing index
                    //                    cout << "updating best distance, best_pairing_index = "<<best_pairing_index<<endl;
                    best_distance = pairing_vector_[j].getDistanceToTracklet(); // update best distance
                } else {
                    cout << "not better than best distance" <<endl;
                }
            }

        }
        if(best_pairing_index != -1) // if we've paired this one
        {
            ///// using the best_pairing_index we've just found, update the tracklet and remove this pairing from the vector
            /// so it doesn't get associated with another tracker
            //            double new_time = ros::Time::now().toSec() - tracker_start_time;
            if (verbose) {
                cout << "Updating Tracklet "<<this_tracklet->getID()<< " with pairing at index "<<best_pairing_index<<endl;
                cout<<"\n!!!!!!!!!!!!!!!!!!!!!time is now "<<msg_time<<endl;
            }
            // make a pointer to the best pairing
            Pairing* best_pairing_ptr = &pairing_vector_[best_pairing_index];
            this_tracklet->updateTracklet(*best_pairing_ptr, msg_time); // update the tracklet with this pairing
            ///// now publish the output of this tracklet with a marker

            stringstream tracklet_name;
            tracklet_name << "tracklet_"<<this_tracklet->getID(); // identify this marker with the tracklet id
            if (io_params.publishing) {
                publishTransform(best_pairing_ptr->getDetectionCoord(),tracklet_name.str()); // publish a transform even if we are not initialised
                if (verbose)
                    cout <<" publishing a transform at \n"<<best_pairing_ptr->getDetectionCoord()<<" with name "<<tracklet_name.str()<<endl;
            }
            pairing_vector_.erase(pairing_vector_.begin() + best_pairing_index); // delete this pairing from pairing_vector_

            if (this_tracklet->isInitialised()) // start publishing if we are initialised
            {

                KalmanFilter kf = this_tracklet->getKf(); // get the Kf from this tracklet vector
                MatrixXd P = kf.getP(); //get covariance
                VectorXd xhat = kf.getState();
                VectorXd v = kf.getV();
                cout << "innovation is\n"<<v <<endl;
                //                if (verbose) {
                //                    cout << "New measurement covariance is\n"<<P <<endl;
                //                    cout << "new Kf state is\n"<<xhat <<endl; }

                /// create a title for this marker
                publishMarker(xhat,tracklet_name.str(), P(0,0),P(1,1),P(2,2)); // publish the marker

                /////////////// write to csv

                if (verbose)
                    cout << "writing tracklet data and detections to file"<<endl;
                VectorXd det_coord = best_pairing_ptr->getDetectionCoord();
                if (verbose)
                    cout <<det_coord[0]<<","<<det_coord[1]<<","<<det_coord[2]<<endl;

                // order : detection XYZ, kf XYZ, kf covariance XYZ. Skip 4 cells so have 4+ 1 commas
                results_file_ <<msg_time<<",,,,,"<<this_tracklet->getID()<<","<< xhat[0]<<","<<xhat[1]<<","<<xhat[2]<<","<<P(0,0)<<","<<P(1,1)<<","<<P(2,2)<<","<<v[0]<<","<<v[1]<<"\n";
            }
        }
        else
        {
            if (verbose)
                cout << "No pairings match this tracker"<<endl;

        }
    }
    ///// put any remaining pairings back in the unpaired detections
    if (verbose)
        cout << "Putting "<<pairing_vector_.size()<<" remaining pairings back into unpaired_detections"<<endl;
    for (int i = 0; i < pairing_vector_.size(); i++)
        unpaired_detections.push_back(pairing_vector_[i].getDetectionCoord()); // move the detection coordinate back onto detections
    pairing_vector_.clear(); // remove all elements from pairing_vector_, since we've just copied them across
}

int MOTracker::getNextTrackletID(bool verbose)
{
    // Generates an ID for a new tracker
    if(verbose)
        cout <<" **************************\nDead Tracklet IDs are ";
    for (int i = 0; i< dead_tracklet_IDs_.size(); i++){cout<<dead_tracklet_IDs_[i];}
    cout<<endl;

    if (!dead_tracklet_IDs_.empty())
    {
        if (verbose)
            cout << "getting a discarded ID"<<endl;
        // take the minimum ID from discarded ID array
        //        int min_pos = distance(dead_tracklet_IDs_.begin(),dead_tracklet_IDs_.end()
        //        int new_ID = min_element(dead_tracklet_IDs_.begin(),dead_tracklet_IDs_.end()) - dead_tracklet_IDs_.begin();
        int min_pos = distance(dead_tracklet_IDs_.begin(),min_element(dead_tracklet_IDs_.begin(),dead_tracklet_IDs_.end()));
        //        cout << "The distance is: " << min_pos << "|value is "<<*min_element(myvec.begin(),myvec.end())<<endl;
        int new_ID = *min_element(dead_tracklet_IDs_.begin(),dead_tracklet_IDs_.end());
        dead_tracklet_IDs_.erase(dead_tracklet_IDs_.begin()+min_pos); // erase this value
        return new_ID;

    } else {
        if (verbose)
            cout << "getting a new ID"<<endl;
        return tracklet_vector_.size(); // return an ID 1 greater than current one
    }
}
void MOTracker::createNewTracklets(vector<VectorXd> &unpaired_detections, bool verbose)
{
    ////// birth
    //    7. for unpaired detections
    //        1. create a new tracklet for each, with a unique id
    if (verbose)
        cout << "creating "<<unpaired_detections.size()<<" new Tracklets"<<endl;

    for (int i = 0; i < unpaired_detections.size(); i++) // loop though the unpaired detections
    {
        // create a new tracklet with a KF initialised at the last detection

        //        cout << "Kf_params.A is " << kf_params_.A << endl;
        //        int next_tracklet_ID = getNextTrackletID(true);
        //        int next_tracklet_ID = next_tracklet_ID_;
        Tracklet new_tracklet(next_tracklet_ID_,
                              unpaired_detections[i],
                              KalmanFilter(kf_params.delF, kf_params.delH, kf_params.delGQdelGT, kf_params.R, kf_params.P0, true));
        tracklet_vector_.push_back(new_tracklet);
        if (verbose)
            cout << "Tracklet with ID "<<next_tracklet_ID_<<" added to tracklet_vector_"<<endl;

        //        next_tracklet_ID_++; // update the next_tracklet_ID
        next_tracklet_ID_ ++;
        if (next_tracklet_ID_ > 20) // prevent the program getting an overflow after a really long time
            next_tracklet_ID_ = 0;

    }
}
void MOTracker::deleteDeadTracklets(bool verbose)
{
    //    // death
    //    8. for each tracklet
    //        1. if num_consecutive_misses > 3
    //            1. delete this tracker

    for (int i = 0; i < tracklet_vector_.size(); i++) // loop through all live tracklets.
    {
        if (verbose)
            cout << "Tracklet "<<tracklet_vector_[i].getID()<<" has had "<<tracklet_vector_[i].getNumConsecutiveMisses()<<" consecutive misses."<<endl;
        if (tracklet_vector_[i].getNumConsecutiveMisses() > tracker_params.max_consecutive_misses) // if we've missed this tracklet too many times in a row
        {
            dead_tracklet_IDs_.push_back(tracklet_vector_[i].getID()); // move ID back into tracklet vector
            tracklet_vector_.erase(tracklet_vector_.begin() + i); // delete this tracklet from tracklet_vector_
            if(verbose)
                cout<<"******************************************************\n!!!!!!!deleting this tracker!!!!!!!!!"<<endl;
        }
    }
}
void MOTracker::initiateLongTracklets(double msg_time, bool verbose)
{
    //    // kf initiation if tracklet reaches sufficient length
    //    9. for each tracklet
    //        1. if detections_vector.size() > 2
    //            1. initiate the kalman filter
    for (int i = 0; i < tracklet_vector_.size(); i++) // loop through all live tracklets.
    {
        if (verbose)
            cout << "Tracklet "<<tracklet_vector_[i].getID()<<" has had "<<tracklet_vector_[i].getLength()<<" detections.\n hasRGBDDetection: "<< tracklet_vector_[i].has_RGBD_detection()<<endl;
        //        if (tracklet_vector_[i].getLength() > min_initialisation_length && !tracklet_vector_[i].isInitialised()) // if this tracklet is long enough and not initialised
        // only allow initiation for RGBD detections

        if (tracklet_vector_[i].has_RGBD_detection() != 0 || tracker_params.only_init_rgb_tracklet == 0) { // if either we don't require RGB detection OR we have an RGB detection
            if (tracklet_vector_[i].getLength() > tracker_params.min_initialisation_length && !tracklet_vector_[i].isInitialised()) // if this tracklet is long enough and not initialised
            {
                if (verbose)
                    cout << "initiating kf at time "<<msg_time<<endl;
                tracklet_vector_[i].initKf(msg_time); // initialise this tracklet's kalman filter
            }
        }
    }
}
/////////////////////////////////// MANAGE THE TRACKLETS ///////////////////////////////////
void MOTracker::manageTracklets(vector<VectorXd> unpaired_detections, double msg_time, bool isRGBD) {
    // Loops through a vector of centroids and updates the kalman filter with the one closest to previous estimate.
    // Publishes transforms for all estimates. Uses time of message msg_time
    if (verbose_) {
        cout << "***********************************************\nmanageTracklets()"<<endl;
        cout << "There are "<<tracklet_vector_.size()<<" live tracklets."<<endl;
    }
    // if time not started yet, start the time
    if (tracker_start_time == -1) {
        tracker_start_time = msg_time;
    }
    // put msg_time relative to tracker time
    msg_time = msg_time - tracker_start_time;

    //// PUBLISH A TRANSFORM FOR EACH DETECTION
    ///
    for (int i = 0; i < unpaired_detections.size(); i++){
        if (io_params.publishing) {
            stringstream ss;
            ss << "detection_"<<i;
            publishTransform(unpaired_detections[i], ss.str());
        }
        // if in GND truth mode, write the detections straight to file
        cout << "writing raw detections to GND truth file"<<endl;
        gnd_file_<<msg_time<<","<<isRGBD<<","<<unpaired_detections[i][0]<<","<<unpaired_detections[i][1]<<","<<unpaired_detections[i][2]<<"\n";
    }

    ////// PERFORM THE TRACKLET ALGORITHM
    updatePairings(unpaired_detections,msg_time, isRGBD, false); // get a bunch of pairings between tracklets and detections
    updateTracklets(unpaired_detections, msg_time, isRGBD, true); // update each tracklet with the best pairing for that tracklet, increment the misses for tracklets without pairings
    createNewTracklets(unpaired_detections, false); // generate new tracklets from any unassociated pairings
    deleteDeadTracklets(false); // delete any tracklets that have been missed too many times
    initiateLongTracklets(msg_time, true); // initiate the kalman filters and publisher for any tracklets with a long sequence of detections
}

///// I/O Methods
void MOTracker::publishTransform(VectorXd coordinates, string target_frame_id) {
    // publishes a transform on broadcaster br_ at the 3D coordinate Vector coordinates
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(coordinates[0],coordinates[1],coordinates[2]) );
    tf::Quaternion q; // initialise the quaternion q for the pose angle
    q.setEulerZYX(0, 0, 0);
    transform.setRotation(q);

    //    string base_frame_id = "map"; // we are based in the odom frame
    string base_frame_id = "odom"; // we are based in the odom frame
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, target_frame_id));
}

void MOTracker::initialiseSubscribersAndPublishers() {
    string velodyne_topic = "/velodyne/point_cloud_filtered"; // topic is the pointcloud from the velodyne
    point_cloud_sub_ = nh_.subscribe(velodyne_topic, 1, &MOTracker::pointCloudCallback, this); // Create a ROS subscriber for the input point cloud that calls the callback

    // SUBSCRIBE TO A POSESTAMPED::TRANSFORM here from RGBD
    string pose_array_topic = "/realsense_detections_poseArray";
    realsense_poseArray_sub_ = nh_.subscribe(pose_array_topic,1, &MOTracker::poseArrayCallback, this); // Create a ROS subscriber for the input posearray

    // Create ROS publishers for the output point clouds
    string topic_raw = "pcl_raw", topic_trans = "pcl_trans", topic_zfilt = "pcl_zfilt", topic_ds = "pcl_ds", topic_seg_filt = "pcl_seg_filter", topic_centroid = "pcl_centroid";
    if (io_params.publishing)
    {
        pub_raw_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_raw, 1);
        pub_trans_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_trans, 1);
        pub_zfilt_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_zfilt, 1);
        pub_ds_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_ds, 1);
        pub_seg_filter_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_seg_filt, 1);
        pub_centroid_ = nh_.advertise<sensor_msgs::PointCloud2> (topic_centroid, 1);
    }
    // Create a publisher for the kf marker
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>( "Tracklet_markers", 0 ); // this name shows up in RVIZ
    // Create a transformListener to enable translation from odom to base frames with our pointcloud.

    odom_base_ls_.reset(new tf::TransformListener); // initialise the odom_base transform listener- so we can transform our output into odom coords
    odom_realsense_ls_.reset(new tf::TransformListener); // initialise the rs_detector_ls_ transform listener- so we can listen to the realsense
    return;
}

void MOTracker::publishMarker(VectorXd x_hat, string marker_name,double scale_x,double scale_y,double scale_z) {
    /// This publishes a cylinder, size set by scale_* onto publisher vis_pub at location x_hat



    visualization_msgs::Marker marker; // initiate the marker

    marker.header.frame_id = "odom"; // we want to publish relative to the odom frame
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(2);
    //    marker.ns = "kalman_filter_marker";  // call our marker kalman_filter_marker
    marker.ns = marker_name;
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

    double scaler = .5; // the overall marker size
    // set the marker size as an input params
    //    marker.scale.x = scale_x*scaler;
    //    marker.scale.y = scale_y*scaler;
    marker.scale.x = sqrt(scale_x)*scaler;
    marker.scale.y = sqrt(scale_y)*scaler;
    //    marker.scale.z = scale_z*scaler;
    marker.scale.z = 2; // keeping z at a constant 2 metres tall

    marker.color.a = 0.4; // Setting alpha to semi transparent
    marker.color.r = 0.0;
    marker.color.g = 1.0; // making it green
    marker.color.b = 0.0;
    pub_marker_.publish( marker ); // publish


}
void MOTracker::setupResultsCSV() {
    // results file
    cout<<"opening results file "<<io_params.res_filename<<endl;
    results_file_.open(io_params.res_filename);
    results_file_ << "Time,isRGBD,Detection_X,Detection_Y,Detection_Z,Tracklet_ID,KF_X,KF_Y,KF_Z,KF_cov_X,KF_cov_Y,KF_cov_Z,v_X,v_Y\n";

    // gnd file stores detections only at a higher frequency
    cout<<"opening gnd file "<<io_params.gnd_filename<<endl;
    gnd_file_.open(io_params.gnd_filename);
    gnd_file_ << "Time,isRGBD,Detection_X,Detection_Y,Detection_Z\n";

}
