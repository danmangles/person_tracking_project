#include "include/mo_tracker.h" // includes a bunch of stuff inc kalman filter, tracklet and pairing

using namespace std;
//using namespace Eigen;


MOTracker::MOTracker(ros::NodeHandle nh,
                     pcl_param_struct pcl_params,
                     kf_param_struct kf_params,
                     tracker_param_struct tracker_params,
                     io_param_struct io_params,
                     ogm_param_struct ogm_params,
                     bool verbose
                     ) :
    nh_(nh),  pcl_params(pcl_params), kf_params(kf_params), tracker_params(tracker_params), io_params(io_params),ogm_params(ogm_params), verbose_(verbose)// initiate the nodehandle
{
    // Constructor: sets up
    cout<< "Calling MOTracker constructor"<<endl;
    initialiseSubscribersAndPublishers(); //initialise the subscribers and publishers
    setupResultsCSV();
    initialiseOGM();
}

///// General Pointcloud Methods
void MOTracker::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) { // the callback fcn
    // this is called by pointclouds from the velodyne, processing them, decomposing into clusters, and publishing cluster coordinates
    cout<< "************************************\nInitiating Callback\n***********************************"<<endl;

    clock_t start_clock, stop_clock;
    double method_duration;
    start_clock = clock();

    sensor_msgs::PointCloud2 msg_to_publish; // we will use this for all the pointclouds we need to publish
    // Publish the cloud
    if (io_params.publishing)
        pub_raw_.publish (*cloud_msg); // publish the raw cloud

    /////////// Apply a passthrough filter and publish the result
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

    ////////// Transform the cloud into the odom frame to suppress base motion
    sensor_msgs::PointCloud2 transformed_cloud;
    transformFromBaseToFixedFrame(bounded_cloud, transformed_cloud);
    if (io_params.publishing)
        pub_trans_.publish (transformed_cloud); // Publish the cloud

    /////////   if cloud has less than 10 points, jump out of the callback
    if (transformed_cloud.width < 10) {
        cout << "*** Cloud has too few points, waiting for the next one. ****" << endl;
        return;
    }


    // Convert variable to correct type for further processing
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
    convertSM2ToPclPtr(transformed_cloud, cloud_ptr);


    /////// update an ogm and publish
    if (pcl_params.ogm_filter_mode != 0) // if method 1 or 2
    {
        updateOGM(cloud_ptr, false);
        if (pcl_params.ogm_filter_mode == 1) // if filtering point by point
        {
            cout << "mode is 1"<<endl;
            removeOccupiedPoints(cloud_ptr, false);
        }
        // publish this map
        if (io_params.publishing)
        {
            cout << "publishing ogm filtered cloud"<<endl;

            occupancy_map_.setTimestamp(cloud_msg->header.stamp.toNSec());
            grid_map_msgs::GridMap outputMessage;
            grid_map::GridMapRosConverter::toMessage(occupancy_map_, outputMessage);
            pub_ogm_.publish(outputMessage);

            pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
            pub_ogm_pcl_.publish (msg_to_publish);
        }
    }

    //////// Downsample with a Voxel Grid and publish
    if (pcl_params.apply_voxel_grid) {
        applyVoxelGrid(cloud_ptr, true); // apply the voxel_grid using a leaf size of 1cm
        // publish
        if (io_params.publishing) {
            cout << "publishing voxel grid"<<endl;
            pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
            pub_ds_.publish (msg_to_publish);
        }
    }
    //////// Remove non planar points e.g. outliers http://pointclouds.org/documentation/tutorials/planar_segmentation.php#id1
    //    if (pcl_params.apply_planar_outlier_removal) {
    //        removeOutOfPlanePoints(cloud_ptr, false);
    //        if (io_params.publishing)
    //        {
    //            pcl::toROSMsg(*cloud_ptr,msg_to_publish ); // convert from PCL:PC1 to SM:PC2
    //            pub_seg_filter_.publish (msg_to_publish);
    //        }
    //    }

    /////////// Split up pointcloud into a vector of pointclouds, 1 for each cluster
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector;
    splitCloudPtrIntoClusters(cloud_ptr,cloud_cluster_vector);

    //////// Loop through clusters and put their centroids into a vector
    vector <VectorXd> centroid_coord_array;
    getCentroidsOfClusters(cloud_cluster_vector, centroid_coord_array, false); // generate a vector of coordinates


    ///////////// time the method
    stop_clock = clock();
    method_duration = (stop_clock - start_clock) / (double)CLOCKS_PER_SEC;
    pcl_callback_time += method_duration;
    n_pcl_callback_time ++;
    cout <<"********************************************************************************************"<<endl;
    cout << "\nPointcloud Callback: Average Runtime for this method: "<<pcl_callback_time/(double)n_pcl_callback_time<<"s."<<endl;
    cout << "\nruntime for this mehthod is : "<<method_duration<<"s."<<endl;
    cout <<"********************************************************************************************"<<endl;

    /////// Publish the cluster centroids
    if (centroid_coord_array.size() != 0) // if there are any clusters visible
        manageTracklets(centroid_coord_array, cloud_msg->header.stamp.toSec(), false); // call the process centroids method with our vector of centroids
    else
        cout << "No valid clusters visible after getCentroidsOfClusters()"<<endl;
}

void MOTracker::updateOGM(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, bool verbose)
{
    if (verbose) cout <<"updateOGM() called"<<endl;
    // loop through the cloud
    for (int i = 0; i < cloud_ptr->size(); i++)
    {
        VectorXd this_point(2);
        this_point<< cloud_ptr->points[i].x, cloud_ptr->points[i].y;
        //        if (verbose) cout << "This point is ("<<this_point(0)<<","<<this_point(1)<<")"<<endl;

        // get grid map index corresponding to this point
        grid_map::Index pt_index;
        occupancy_map_.getIndex(this_point, pt_index);
        // update if the point falls inside the ogm
        if ( occupancy_map_.isInside(this_point) ){
            // label as updated
            occupancy_map_["current_occupancy"](pt_index(0),pt_index(1)) = 1;        }
        else
        {// we are outside the map, consider resizing it
            if (verbose) cout << "\n***************************************************outside of map arghhhhhhhhhhhhhhhhh\n*******************************"<<endl;
        }
    }

    if(verbose) cout<< "looping thru occ map to decrement missed cells"<<endl;

    // loop through the gridmap, decrementing the map for all indices that haven't been occupied
    for (GridMapIterator iterator(occupancy_map_); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Index pt_index(*iterator); // get the index corresponding to this iterator
        //        if (verbose) cout << "investigating ogm at index ("<<pt_index<<")"<<endl;
        //        cout << "occupied indices has size "<<occupied_indices.size()<<""<<endl;

        // if we haven't updated this cell this cycle
        if (occupancy_map_["current_occupancy"](pt_index(0),pt_index(1)) != 1)
        {
            //decrement windowed occupancy
            occupancy_map_["occupancy"](pt_index(0),pt_index(1)) -= ogm_params.decrement;

            // threshold at 0
            if (occupancy_map_["occupancy"](pt_index(0),pt_index(1)) < 0.0)
                occupancy_map_["occupancy"](pt_index(0),pt_index(1)) = 0.0;
        }
        else // if we HAVE updated this cell this cycle
        {
            // if we have set this value to 1, reset it to 0
            occupancy_map_["occupancy"](pt_index(0),pt_index(1)) += ogm_params.increment;
            occupancy_map_["current_occupancy"](pt_index(0),pt_index(1)) = 0; // and reset the bool
            // threshold at 10
            if (occupancy_map_["occupancy"](pt_index(0),pt_index(1)) > ogm_params.window_length)
                occupancy_map_["occupancy"](pt_index(0),pt_index(1)) = ogm_params.window_length;
        }
        // threshold at 14
        if (occupancy_map_["occupancy"](pt_index(0),pt_index(1)) > ogm_params.threshold)
            occupancy_map_["thresholded_occupancy"](pt_index(0),pt_index(1)) = 1;
        else
            occupancy_map_["thresholded_occupancy"](pt_index(0),pt_index(1)) = 0;

    }
    if (verbose) cout<<"exiting updateOGM()"<<endl;
}
void MOTracker::removeOccupiedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, bool verbose)
{
    // removes all the points in occupied cells in the ogm
    if(verbose) cout <<"removeOccupiedPoints() called"<<endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (int i = 0; i < cloud_ptr->size(); i++)
    {
        VectorXd this_point(2);
        this_point<< cloud_ptr->points[i].x, cloud_ptr->points[i].y;
        //        if (verbose) cout << "This point is ("<<this_point(0)<<","<<this_point(1)<<")"<<endl;

        // get grid map index corresponding to this point
        grid_map::Index pt_index;
        occupancy_map_.getIndex(this_point, pt_index);
        // update if the point falls inside the ogm
        if ( occupancy_map_.isInside(this_point) )
        {
            // label as updated
            if (occupancy_map_["thresholded_occupancy"](pt_index(0),pt_index(1)) == 1)
            {
                // if this point is occupied, delete it
                inliers->indices.push_back(i);
                //                if (verbose) cout << "added point with index "<<i<<" to the inliers set"<<endl;
            }
        }
        else
        {// we are outside the map, consider resizing it
            if (verbose) cout << "\n***************************************************outside of map arghhhhhhhhhhhhhhhhh\n*******************************"<<endl;
        }
    }

    if (verbose) cout <<"filtering cloud_ptr from "<<cloud_ptr->points.size()<<" points to ";
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_ptr);
    if (verbose) cout <<cloud_ptr->points.size()<<" points"<<endl;
}

void MOTracker::initialiseOGM()
{
    cout <<"initialising grid map"<<endl;
    // Sets up the grid map with the correct layers.
    //    occupancy_map_.add("local_occupancy", Matrix(100, 100)); // add the layer local occupancy for stuff

    // how to initialise this based on base frame location?
    double height = 70.0, width = 70.0, cell_scale = 0.2; // map params in metres
    int matrix_height = (int) (height/cell_scale), matrix_width = (int)(width/cell_scale); // n cells along one size
    occupancy_map_.setFrameId(io_params.fixed_frame);
    occupancy_map_.setGeometry(Length(height, width), cell_scale, Position(0.0,0.0));
    cout<<("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
           occupancy_map_.getLength().x(), occupancy_map_.getLength().y(),
           occupancy_map_.getSize()(0), occupancy_map_.getSize()(1),
           occupancy_map_.getPosition().x(), occupancy_map_.getPosition().y(), occupancy_map_.getFrameId().c_str());


    //    occupancy_map_.add("occupancy", grid_map::Matrix(matrix_height,matrix_width)); // add the layer local occupancy for stuff
    occupancy_map_.add("current_occupancy", grid_map::Matrix(matrix_height,matrix_width)); // add the layer local occupancy for stuff
    occupancy_map_.add("occupancy", grid_map::Matrix(matrix_height,matrix_width)); // add the layer local occupancy for stuff
    occupancy_map_.add("thresholded_occupancy", grid_map::Matrix(matrix_height,matrix_width)); // add the layer local occupancy for stuff
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
        if (realsense_coords.size() == 0) {
            cout <<"sending no coords for literally no reason"<<endl;
        }
        manageTracklets(realsense_coords, pose_array.header.stamp.toSec(), true);
        //        cout << "centroid coords updated with a new pose"<<endl;

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

void MOTracker::transformFromBaseToFixedFrame(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &output_cloud) {
    // transforms the cloud into the odom frame from base frame
    if (verbose_)
        cout<<"Transforming Pointcloud into odom frame. Waiting for transform"<<endl;
    string target_frame = io_params.fixed_frame, base_frame = "base"; // target frame and base frame for transformfor transform
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
void MOTracker::getCentroidsOfClusters (vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_cluster_vector, vector<Eigen::VectorXd> &centroid_coord_array, bool verbose) {
    // loops through cloud_cluster vector and gets the centroid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;

    if (verbose)
        cout << "investigating a vector with "<<cloud_cluster_vector.size()<<" clusters"<<endl;
    int pub_index = 0;
    for (int i = 0; i<cloud_cluster_vector.size(); i++) {
        if (verbose)
            cout <<"extracting cloud cluster from index "<<i<<endl;
        cloud_cluster = cloud_cluster_vector[i]; // extract one cluster

        if (verbose)
            cout << "\n\n** Investigating a centroid cluster with "<<cloud_cluster->points.size() <<" points in it"<<endl;

        Eigen::Vector4f xyz_centroid;
        compute3DCentroid (*cloud_cluster, xyz_centroid);


        if (io_params.publishing)
        {
            if (pub_index < 30) // don't publish if we've got too many people
            {
                sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
                pcl::toROSMsg(*cloud_cluster,msg_to_publish);
                msg_to_publish.header.frame_id = io_params.fixed_frame; // CHANGED THIS TO BASE INSTEAD OF ODOM BECAUSE WE WERE PUBLISHING POINTS IN THE WRONG PLACE
                pub_centroid_.at(pub_index).publish (msg_to_publish); // this is not publishing correctly
                pub_index++; // increment the publisher index
            }
        }

        if (pcl_params.ogm_filter_mode == 2) // if we are ogm filtering cluster by cluster
        {
            cout << "mode is 2"<<endl;
            //////////// STATIC OR DYNAMIC
            VectorXd this_point(2);
            this_point<< xyz_centroid(0), xyz_centroid(1);

            grid_map::Index pt_index;
            occupancy_map_.getIndex(this_point, pt_index);
            if (occupancy_map_["thresholded_occupancy"](pt_index(0),pt_index(1)) == 1)
            {
                // this is a static cluster
                cout <<"this is a static cluster!!!!!"<<endl;
                break;
            }
            else
            {
                if (verbose) cout << "this cluster is dynamic, might be a person!!"<<endl;
            }
        }

        //// get the covariance in X,Y,Z
        Matrix3f covariance_matrix;
        computeCovarianceMatrix (*cloud_cluster, xyz_centroid, covariance_matrix);
        if (verbose)
        {
            cout <<" this cluster has covariance matrix M = \n"<<covariance_matrix<<endl;
            cout <<" this cluster has centroid = \n"<<xyz_centroid<<endl;
        }
        float cov_X = covariance_matrix(0,0);
        float cov_Y = covariance_matrix(1,1);
        float cov_Z = covariance_matrix(2,2);

        if (cov_Z > 1.2* cov_X & cov_Z > 1.2 * cov_Y)
        {
            if (verbose)
            {
                cout <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!this cluster is a person!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                cout <<"V = \n"<<covariance_matrix<<endl;
            }


            // strip out the 4th coordinate which is a 1 for reverse compatibbility with other methods
            Vector3d coord_centroid(xyz_centroid[0],xyz_centroid[1],xyz_centroid[2]);
            centroid_coord_array.push_back(coord_centroid); // we want to keep this cluster
        }
        else
        {
            if (verbose)
            {
                cout <<"this cluster isn't a person"<<endl;
                cout <<"V = \n"<<covariance_matrix<<endl;
            }
        }

    }

    ///////// Print out a line of stars if there are 2 centroids in the array because this is a
    //    cout << "There are "<<centroid_coord_array.size()<<" valid clusters in the pcl"<<endl;
    //    if (centroid_coord_array.size() > 1){cout<<"*************************************************************************************************************************"<<endl;}

}

void MOTracker::getClusterCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_ptr, VectorXd &coord_centroid) {
    // loop through the point cloud cluster_ptr, adding points to a centroid

    //    cout << "getClusterCentroid() called" <<endl;

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
    //    cout << "generate_centroid() is returning a coord_centroid at :\n"<<coord_centroid<<endl;
}
double MOTracker::getMaxGatingDistance(Tracklet *tracklet_ptr, bool verbose) {
    // returns the max distance at which a new detection can be associated
    if (tracklet_ptr->isInitialised()) {
        KalmanFilter kf = tracklet_ptr->getKf(); // get the Kf from this tracklet vector
        MatrixXd P = kf.getP(); //get covariance
        double max = tracker_params.gating_dist_constant*sqrt((P(0,0) + P(1,1))/2)+0.5;
        // don't let it get too big
        if (max > tracker_params.base_gating_dist)
            max = tracker_params.base_gating_dist;
        if (verbose)
            cout <<"MAX_GATING_DIST = "<<max<<"m"<<endl;
        return max; // return sqrt of average covariance is basically the std dev in x, y
    }
    else
    {
        return tracker_params.base_gating_dist;
    }

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
        Tracklet new_tracklet(next_tracklet_ID_,
                              unpaired_detections[i],
                              KalmanFilter(kf_params.F, kf_params.H, kf_params.GQG, kf_params.R_rgbd,kf_params.R_velodyne, kf_params.P0, true));
        tracklet_vector_.push_back(new_tracklet);
        if (verbose)
            cout << "Tracklet with ID "<<next_tracklet_ID_<<" added to tracklet_vector_"<<endl;

        next_tracklet_ID_ ++; // increment ID so that the next tracklet will get a unique one
        if (next_tracklet_ID_ > 30) // prevent the program getting an overflow after a really long time
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

    cout << "***********************************************\nmanageTracklets()\n*******************************"<<endl;
    cout << "There are "<<tracklet_vector_.size()<<" live tracklets,"<<endl;
    cout << "There are "<<unpaired_detections.size()<<" unpaired_detections"<<endl;
    if (unpaired_detections.size() == 0)
    {
        cout <<"***************************888\nsomeone called manageTracklets with an empty detection: how?????\n***************************888\n";
        return;
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
        //        cout << "writing raw detections to GND truth file"<<endl;
        gnd_file_<<msg_time<<","<<isRGBD<<","<<unpaired_detections[i][0]<<","<<unpaired_detections[i][1]<<","<<unpaired_detections[i][2]<<"\n";
    }
    clock_t start_clock, stop_clock;
    double method_duration;
    start_clock = clock();

    // initiate a cost matrix to populate
    MatrixXd cost_matrix(unpaired_detections.size(),tracklet_vector_.size());
    populateCostMatrix(unpaired_detections, cost_matrix, false);
    updateTrackletsWithCM(unpaired_detections, cost_matrix, msg_time, isRGBD, false);

    stop_clock = clock();
    method_duration = (stop_clock - start_clock) / (double)CLOCKS_PER_SEC;
    assignment_algo_time += method_duration;
    n_assignment_algo_time ++;
    cout <<"********************************************************************************************"<<endl;
    cout << "\Assignment Algo: Average Runtime for this method: "<<assignment_algo_time/(double)n_assignment_algo_time<<"s."<<endl;
    cout << "\nruntime for this mehthod is : "<<method_duration<<"s."<<endl;
    cout <<"********************************************************************************************"<<endl;

    ////// PERFORM THE TRACKLET ALGORITHM
    //    updatePairings(unpaired_detections,msg_time, isRGBD, false); // get a bunch of pairings between tracklets and detections
    //    updateTracklets(unpaired_detections, msg_time, isRGBD, false); // update each tracklet with the best pairing for that tracklet, increment the misses for tracklets without pairings
    createNewTracklets(unpaired_detections, false); // generate new tracklets from any unassociated pairings
    deleteDeadTracklets(false); // delete any tracklets that have been missed too many times
    initiateLongTracklets(msg_time, false); // initiate the kalman filters and publisher for any tracklets with a long sequence of detections
}

void MOTracker::populateCostMatrix(vector<VectorXd> &unpaired_detections, MatrixXd &cost_matrix, bool verbose)
{
    // populates the cost_matrix:
    // tracklets are the COLUMNS
    // detections are the ROWS
    if (verbose) cout <<"populateCostMatrix()"<<endl;

    for (int i = 0; i < unpaired_detections.size(); i++) // loop through all unpaired detections
    {
        if (verbose) cout << "assessing detection "<<i<<endl;

        for (int j = 0; j < tracklet_vector_.size(); j++) // loop through all live tracklets
        {
            if (verbose) cout << "assessing tracklet "<<j<<endl;

            cost_matrix(i,j) = tracklet_vector_[j].getDistance(unpaired_detections[i]); // update the cost matrix
        }
    }
    if (verbose) cout <<"cost matrix populated"<<endl;

}

void MOTracker::updateTrackletsWithCM(vector<VectorXd> &unpaired_detections, MatrixXd &cost_matrix, double msg_time, bool isRGBD, bool verbose)
{
    vector<int> col_IDs, row_IDs;

    for (int i = 0; i <tracklet_vector_.size(); i++)
    {
        cout <<i<<": tracklet_"<<tracklet_vector_[i].getID()<<endl;
    }
    cout << "col_IDs:";
    for (int i = 0; i <tracklet_vector_.size(); i++)
    {
        col_IDs.push_back(i);
        cout <<i;
    }
    cout <<"\nrow_IDs";
    for (int i = 0; i < unpaired_detections.size(); i++)
    {
        row_IDs.push_back(i);
        cout <<i;
    }

    if (verbose) cout << "cost_matrix is now\n"<<cost_matrix<<endl;
    cout << "\nupdateTrackletsWithCM()"<<endl;

    if (tracklet_vector_.size() == 0)
    {
        if (verbose) cout << "no tracklets, exiting method"<<endl;
        return;
    }
    vector <int> paired_tracklet_indices;
    while (true) // this loop is broken out of by two conditions: all detections are far away or there are no detectinos left
    {
        if (verbose) cout << "cost_matrix is now\n"<<cost_matrix<<endl;


        //get location of minimum
        MatrixXd::Index minRow, minCol; // minRow is the row at which the detection in the min-distance-pair is located, minCol is the column at which the tracklet in that pair is.
        float min_dist = cost_matrix.minCoeff(&minRow, &minCol); // min_dist is the min distance between any detection and tracklet
        cout << "Min: " << min_dist << ", at: " << minRow << "," << minCol << endl;


        // create a pointer to the tracklet from this min_pair
        //        Tracklet * this_tracklet = &tracklet_vector_[minCol];

        Tracklet * this_tracklet = &tracklet_vector_[col_IDs[minCol]];
        paired_tracklet_indices.push_back(col_IDs[minCol]);
        // add the index to the list of paired_tracklet_indices
        //        paired_tracklet_IDs.push_back(this_tracklet->getID());
        // if this min_distance is not too great
        if (min_dist < getMaxGatingDistance(this_tracklet, false))
        {
            if (verbose) cout << "Updating tracklet_"<<this_tracklet->getID()<< " with detection at index "<<minRow<<endl;
            updateTracklet(this_tracklet, unpaired_detections[minRow], msg_time,isRGBD, false);

            // delete the detection from the array of unpaired detections and the tracklet from unpaired tracklets
            unpaired_detections.erase(unpaired_detections.begin() + minRow);

        }

        else // closest detection is too far away from tracklets
        {
            cout << "tracklet too far away" <<endl;
            break; //out of the loop
        }
        // delete the row and the column and repeat, delete these also from the rowids and col ids
        removeRow(cost_matrix, minRow);
        row_IDs.erase(row_IDs.begin() + minRow);
        removeColumn(cost_matrix, minCol);
        col_IDs.erase(col_IDs.begin() + minCol);
        //        row_adjuster += minRow, col_adjuster +=minCol;
        if (verbose)  cout <<"cost matrix now has the following dimensions: rows: "<<cost_matrix.rows()<<" cols: "<<cost_matrix.cols()<<endl;

        if (cost_matrix.rows() == 0)
        {
            if (verbose) cout<<"fewer measurements than tracklets (or both are empty)"<<endl;
            break;
        }
        if (cost_matrix.cols() == 0)
        {
            if (verbose) cout<<"fewer tracklets than measurements"<<endl;
            break;
        }
    }
    // now record misses with all remaining tracklets
    if (verbose)
        cout << "looping through "<<tracklet_vector_.size()<<" tracklets to assess which are unpaired"<<endl;
    for (int i = 0; i < tracklet_vector_.size(); i++)
    {
        // if paired_tracklet_indices contains i, we are ok
        if(find(paired_tracklet_indices.begin(), paired_tracklet_indices.end(), i) == paired_tracklet_indices.end())
        {
            Tracklet * this_tracklet = &tracklet_vector_[i]; // create a ptr for readibility
            // paired_tracklet_indices doesn't contain i
            cout << "Couldn't find a detection for tracklet_"<< this_tracklet->getID()<<endl;
            cout<<"time is now "<<msg_time<<endl;
            this_tracklet->recordMiss(msg_time);
            cout << "has the tracklet been initialised? "<<this_tracklet->isInitialised()<<endl;

            if (this_tracklet->isInitialised())
            {
                // update the kalman filter variance so the covariance circle grows for missed
                KalmanFilter kf = this_tracklet->getKf(); // get the Kf from this tracklet vector
                MatrixXd P = kf.getP(); //get covariance
                VectorXd xhat = kf.getState(), v = kf.getV();
                //                if (verbose) {
                //                    cout << "New measurement covariance is\n"<<P <<endl;
                //                    cout << "new Kf state is\n"<<xhat <<endl; }
                stringstream tracklet_name;
                tracklet_name << "tracklet_"<<this_tracklet->getID(); // identify this marker with the tracklet id
                /// create a title for this marker
                publishMarker(xhat,tracklet_name.str(), getMaxGatingDistance(this_tracklet, false),getMaxGatingDistance(this_tracklet, false),2); // publish the marker

                cout << "about to write all this to file "<<this_tracklet->getID();
                // order : detection XYZ, kf XYZ, kf covariance XYZ. Skip 4 cells so have 4+1 commas
                results_file_ <<msg_time<<",,,,,"<<this_tracklet->getID()<<","<< xhat[0]<<","<<xhat[1]<<","<<xhat[2]<<","<<P(0,0)<<","<<P(1,1)<<","<<P(2,2)<<","<<v[0]<<","<<v[1]<<"\n";
                cout << "results file is fine"<<endl;
            }

        }
        else
        {
            cout <<"tracklet vector "<<i<<" is paired"<<endl;
        }
    }
    if (verbose)
        cout <<"updateTrackletsWithCM() exiting with "<<unpaired_detections.size()<<" detections unpaired"<<endl;
}

void MOTracker::updateTracklet(Tracklet *tracklet, VectorXd det_coord, double msg_time, bool isRGBD, bool verbose)
{
    // updates tracklet this_tracklet with detection detection
    if (verbose)
    {
        cout <<"updateTracklet() called"<<endl;
    }
    tracklet->update(det_coord, msg_time, isRGBD, true); // update the tracklet with this detection
    ///// now publish the output of this tracklet with a marker
    stringstream tracklet_name;
    tracklet_name << "tracklet_"<<tracklet->getID(); // identify this marker with the tracklet id
    if (io_params.publishing)
    {
        publishTransform(det_coord,tracklet_name.str()); // publish a transform even if we are not initialised
        if (verbose)
            cout <<" publishing a transform at \n"<<det_coord<<" with name "<<tracklet_name.str()<<endl;
    }
    if (tracklet->isInitialised()) // start publishing if we are initialised
    {
        KalmanFilter kf = tracklet->getKf(); // get the Kf from this tracklet vector
        MatrixXd P = kf.getP(); //get covariance
        VectorXd xhat = kf.getState(), v = kf.getV();

        if (verbose) {
            cout << "New measurement covariance:\n"<<P <<endl;
            cout << "new Kf state is\n"<<xhat <<endl;
            cout << "innovation is\n"<<v <<endl;
        }

        /// create a title for this marker
        publishMarker(xhat,tracklet_name.str(), getMaxGatingDistance(tracklet, false),getMaxGatingDistance(tracklet, false),2); // publish the marker

        /////////////// write to csv
        if (verbose)
            cout << "writing tracklet data and detections to file"<<endl;


        // order : detection XYZ, kf XYZ, kf covariance XYZ. Skip 4 cells so have 4+ 1 commas
        results_file_ <<msg_time<<",,,,,"<<tracklet->getID()<<","<< xhat[0]<<","<<xhat[1]<<","<<xhat[2]<<","<<P(0,0)<<","<<P(1,1)<<","<<P(2,2)<<","<<v[0]<<","<<v[1]<<"\n";
    }
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
    string base_frame_id = io_params.fixed_frame; // we are based in the odom frame
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, target_frame_id));
}

void MOTracker::initialiseSubscribersAndPublishers() {
    string velodyne_topic = "/point_cloud_filter/velodyne/point_cloud_filtered"; // topic is the pointcloud from the velodyne
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
        pub_ogm_pcl_ = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_ogm_filt", 1);

        pub_ogm_ = nh_.advertise<grid_map_msgs::GridMap>("multi_sensor_tracker/filtered_map", 1);

        int numofpubs = 30;
        for (int i = 0; i<numofpubs; i++)
        {
            stringstream ss;
            ss << "pcl_centroid_"<<i;
            ros::Publisher new_pub = nh_.advertise<sensor_msgs::PointCloud2> (ss.str(), 1);
            pub_centroid_.push_back(new_pub);
        }

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

    marker.header.frame_id = io_params.fixed_frame; // we want to publish relative to the odom frame
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
    //    marker.pose.position.z = x_hat[2];

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    double scaler = .5; // the overall marker size
    // set the marker size as an input params
    //    marker.scale.x = scale_x*scaler;
    //    marker.scale.y = scale_y*scaler;
    //    marker.scale.x = sqrt(scale_x)*scaler;
    //    marker.scale.y = sqrt(scale_y)*scaler;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;


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
//void MOTracker::setupTimingVars() {
//    // sets up all the variables to measure the compute time for different methods

//}
void MOTracker::removeRow(MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void MOTracker::removeColumn(MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

