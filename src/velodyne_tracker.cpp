#include <ros/ros.h>

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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector> // for the vector of coordinates

#include <math.h>       /* sin */
using namespace std;
using namespace Eigen;

// HACK: setup the public transform listener so we can listen to the odom_base_tf
boost::shared_ptr<tf::TransformListener> odom_base_ls;

// setup 5 publishers to display point clouds at each stage
ros::Publisher pub_raw; // setup the publisher for the output point cloud
ros::Publisher pub_trans; // setup the publisher for the output point cloud
ros::Publisher pub_zfilt; // setup the publisher for the output point cloud
ros::Publisher pub_ds; // setup the publisher for the output point cloud
ros::Publisher pub_centroid; // setup the publisher for the output point cloud
ros::Publisher coords_publisher; // publisher to use for the vector of coords

//sensor_msgs::PointCloud2 output_msg; // general message to store data to be published on


void generate_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_ptr, Vector3f &coord_centroid) {
    // loop through the point cloud cluster_ptr, adding points to a centroid

    cout << "generate_centroid() called" <<endl;

    pcl::CentroidPoint<pcl::PointXYZ> centroid; // Initialise a point to store the centroid inoutput_topic

    for (size_t i = 0; i < cluster_ptr -> points.size (); ++i) {// add all the points to the centroid
      centroid.add (cluster_ptr -> points[i]);
    }

    pcl::PointXYZ c; // the centre point
    centroid.get (c);// Fetch centroid using `get()`
//    cout << "\n Centroid is located at "
//              << c
//              << endl;
    coord_centroid << c.x, c.y, c.z; // assign coords to coord_centroid
//    cout << "generate_centroid() is returning a coord_centroid at :\n"<<coord_centroid<<endl;
}
void publish_transform(tf::TransformBroadcaster br, Vector3f coord_centroid, string target_frame_id) {
    cout << "publish_transform()" <<endl;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(coord_centroid[0], coord_centroid[1], coord_centroid[2]) ); // set the origin of the transform to the centroid, converting to tf::Vector3
    tf::Quaternion q; // initialise the quaternion q for the pose angle
    //tf::TransformListener odom_base_ls; // literally no idea what this was doing here

    q.setEulerZYX(0, 0, 0); // set an arbitrary angle to the quaternion
    transform.setRotation(q);
    //q.setRPY(0, 0, msg->theta);
    string velodyne_frame_id = "odom";


    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), velodyne_frame_id, target_frame_id)); // send the transform

}

void apply_passthrough_filter(const sensor_msgs::PointCloud2ConstPtr input_cloud, sensor_msgs::PointCloud2 &output_cloud) {

    // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl_conversions::toPCL(*input_cloud, *cloud); // convert cloud_msg into a pcl edition

 double radius = 8.0; // maximum distance from base we are interested in
 double max_height = 2.0; // max height of z filter in metres
 double min_height = -0.2; // min height of z filter in metres
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
void transform_cloud(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 output_cloud, string target_frame, string base_frame) {

//    cout<<"**********************waiting for transform"<<endl;
    odom_base_ls->waitForTransform(target_frame, base_frame, ros::Time(0), ros::Duration(10.0) );
    cout<<"transforming pcloud using time "<< ros::Time(0)<<endl;
    pcl_ros::transformPointCloud(target_frame, input_cloud, output_cloud, *odom_base_ls); // perform the transformation


}

// METHOD CURRENTLY REDUNDANT
//void publish_coord_vector(vector <Vector3f> coord_vector) {
//    // publishes coord_vector to topic [topic_name]
//    coords_publisher.publish(coord_vector);




//}


void generate_coord_array (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, vector <Vector3f> &centroid_coord_array) {
  // INPUTS: input_cloud: pointer to the pointcloud produced by the velodyne relative to "base"
  //          centroid_cluster_array: pointer to output cloud which contains the clusters
  // This method takes the pcloud input_cloud, downsizes it with a voxel grid, splits up the grid into 
  // clusters, and returns the largest cluster as "cloud_cluster", a pointer to a PointCloud
 cout << "generate_cluster_array() called" <<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  /* COMMENTED OUT THE DOWNSAMPLING BECAUSE OUR POINT CLOUD IS ALREADY SPARSE



  ///////////////// DOWNSAMPLE THE INPUT WITH A VOXEL GRID /////////////////

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  // make new point cloud pointer to contain the filtered point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // set vg input to input cloud
  vg.setInputCloud (input_cloud);

  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  // vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);
  cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl; //*
  



  */
  cloud_filtered = input_cloud;

  // CONVERT FROM PCL:PC1 TO SM:PC2
  sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
  pcl::toROSMsg(*cloud_filtered,msg_to_publish ); // con
  pub_ds.publish (msg_to_publish);


  /////////////////* DECOMPOSE cloud_filtered INTO CLUSTERS */////////////////

  cout << "Initiating Segmentation objects" << endl;  
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);

  // Loop through the cloud, performing the clustering operation
  int i=0, nr_points = (int) cloud_filtered->points.size (); // downsampling to a factor of 0.3
  double downsample_factor = 0.3;// downsampling to a factor of 0.3
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
    pcl::ExtractIndices<pcl::PointXYZ> extract;
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
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  //cout << "setting up cluster objects" << endl;
  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.3); // 2cm: too small, we split one object into many, too big, we merge objects into one.
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

 //////////////////////* Perform the extraction ////////////////////////////////////////

  // we have used some dodgy coding here so we only loop ONCE- WE ONLY WANT THE LARGEST CLUSTER
  cout << "extracting the clusters" << endl;

  int MAX_CLUSTER_SIZE = 200; // ditch all clusters with more points than this

  vector<pcl::PointIndices>::const_iterator it;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>); // make a local cloud in which to store the outputs

  for (it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // loop through the clusters
  {
        //centroid_cluster_array[it]
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
          cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;

        // publish it!
       sensor_msgs::PointCloud2 msg_to_publish; // initiate intermediate message variable
       pcl::toROSMsg(*cloud_cluster,msg_to_publish);
       msg_to_publish.header.frame_id = "odom"; // CHANGED THIS TO BASE INSTEAD OF ODOM BECAUSE WE WERE PUBLISHING POINTS IN THE WRONG PLACE
       pub_centroid.publish (msg_to_publish); // this is not publishing correctly


        if (cloud_cluster->points.size() < MAX_CLUSTER_SIZE) { // if there are enough points in the cloud
            cout << "\n\n** Returning cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
            Vector3f coord_centroid;
            generate_centroid(cloud_cluster, coord_centroid);
            cout << "[inside generate_cluster_array()] coord_centroid is "<<coord_centroid<<endl;
            centroid_coord_array.push_back(coord_centroid); // we want to keep this cluster

        } else {
        //else just ignore and keep looping
            cout << "Ditching a cloud with "<<cloud_cluster->points.size() <<" points in it"<<endl;
    }
  }
}
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cout << "cloud_cb() called" <<endl;
  // INPUTS: cloud_msg: pointer to the raw input cloud from the velodyne
  // OUTPUTS: void
  // This is a callback function that reads in the velodyne message, removes the ground plane, 
  // calls get_cloud_cluster() to get the largest cluster, extracts the centroid from this, 
  // and publishes the centroid coordinates on the transform "velodyne_person_est"

    // Publish the cloud
     pub_raw.publish (*cloud_msg); // publish the raw cloud

     // apply a passthrough filter
   sensor_msgs::PointCloud2 bounded_cloud;
   apply_passthrough_filter(cloud_msg, bounded_cloud); // remove all points outside of a predefined bod

   pub_zfilt.publish (bounded_cloud); // Publish the output

  //////////////////////* TRANSFORM THE INPUT CLOUD INTO THE ODOM FRAME  /////////////////////////////

  string target_frame = "odom", base_frame = "base"; // target frame for transform
  sensor_msgs::PointCloud2 transformed_cloud;
  cout<<"*waiting for transform"<<endl;
   odom_base_ls->waitForTransform(target_frame, base_frame, ros::Time(0), ros::Duration(10.0) );
   cout<<"transforming pcloud using time "<< ros::Time(0)<<endl;
   pcl_ros::transformPointCloud(target_frame, bounded_cloud, transformed_cloud, *odom_base_ls); // perform the transformation

  //transform_cloud(bounded_cloud, transformed_cloud, target_frame, transformed_cloud); // transform
/////////////////////////////////////
  // Publish the cloud
  pub_trans.publish (transformed_cloud); // this is publishing correctly//////////////////////////////////


  //   if cloud has less than 10 points, jump out of the callback
    if (transformed_cloud.width < 10) { // the velodyne reading is invalid- probably an issue with the transform
          cout << "*** THERE'S SOMETHING WRONG WITH THIS CLOUD. waiting for the next one. ****" << endl;
          cout << "Width: "<< transformed_cloud.width <<endl;
        return; // drop out of this callback and come back with the next transform
    }
    cout << "*** LOOKS OK TO ME***"<< endl;



  ////////////////////// DO A WEIRD VARIABLE CONVERSION THING SO WE END UP WITH A  "boost shared poitner"/////////////////////////////

  // Take the previously published output message and convert it to a pointcloud called "prefiltered cloud"
  pcl::PointCloud<pcl::PointXYZ> prefiltered_cloud;  // pcl version of the point cloud
  pcl::fromROSMsg (transformed_cloud, prefiltered_cloud); // wrong: output_msg of type smPC2 and transformed_cloud of
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (); //convert from an object to a boost shared pointer

  typedef pcl::PointCloud<pcl::PointXYZ>  mytype;
  boost::shared_ptr<mytype> prefiltered_cloud_ptr = boost::make_shared <mytype> (prefiltered_cloud);

  ////////////////// GET THE CLUSTER CENTROIDS
  vector <Vector3f> centroid_coord_array;
  generate_coord_array(prefiltered_cloud_ptr, centroid_coord_array); // generate a vector of coordinates

  if (centroid_coord_array.size() > 1){cout<<"*************************************************************************************************************************"<<endl;}
  cout <<"centroid_coord_array is :\n[";
  for (int i = 0; i<centroid_coord_array.size(); i++) { cout <<centroid_coord_array[i]<< "] "<<endl;} // print out the array

  // nb the old version used to publish centroids here

  /////////////////// PUBLISH THE CLUSTER CENTROIDS
  //coords_publisher.publish(centroid_coord_array); // publish the vector to the coordinate publisher, at topic "centroid_coord_array"
  //publish_coord_vector(centroid_coord_array);

}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate r(1); // 10 hz


  odom_base_ls.reset(new tf::TransformListener); // initialise the odom_base transform listener- so we can transform our output into odom coords

  // Create a ROS subscriber for the input point cloud
  string input_topic = "/velodyne/point_cloud_filtered";
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  string topic_raw = "pcl_raw", topic_trans = "pcl_trans", topic_zfilt = "pcl_zfilt", topic_ds = "pcl_ds", topic_centroid = "pcl_centroid", topic_coords = "centroid_coord_array";
  
  pub_raw = nh.advertise<sensor_msgs::PointCloud2> (topic_raw, 1);
  pub_trans = nh.advertise<sensor_msgs::PointCloud2> (topic_trans, 1);
  pub_zfilt = nh.advertise<sensor_msgs::PointCloud2> (topic_zfilt, 1);
  pub_ds = nh.advertise<sensor_msgs::PointCloud2> (topic_ds, 1);
  pub_centroid = nh.advertise<sensor_msgs::PointCloud2> (topic_centroid, 1);

  //coords_publisher = nh.advertise<vector<Vector3f>> (topic_coords, 1); //setup the coordinate publisher

  // Spin
  ros::spin ();
  r.sleep();
}

