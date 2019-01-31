#include <ros/ros.h>

// sensor msgs includes specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


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

#include <math.h>       /* sin */
using namespace std;

// HACK: setup the public transform listener so we can listen to the odom_base_tf
boost::shared_ptr<tf::TransformListener> odom_base_ls;

// setup 5 publishers to display point clouds at each stage
ros::Publisher pub_raw; // setup the publisher for the output point cloud
ros::Publisher pub_trans; // setup the publisher for the output point cloud
ros::Publisher pub_zfilt; // setup the publisher for the output point cloud
ros::Publisher pub_ds; // setup the publisher for the output point cloud
ros::Publisher pub_centroid; // setup the publisher for the output point cloud

//sensor_msgs::PointCloud2 output_msg; // general message to store data to be published on

pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_cluster (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  // INPUTS: input_cloud: pointer to the pointcloud produced by the velodyne relative to "base"
  // OUTPUTS: cloud_cluster: pointer to the output cloud which contains the points from the largest cluster
  // This method takes the pcloud input_cloud, downsizes it with a voxel grid, splits up the grid into 
  // clusters, and returns the largest cluster as "cloud_cluster", a pointer to a PointCloud

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
  seg.setDistanceThreshold (0.01);

  // Loop through the cloud, performing the clustering operation
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  cout << "Segmenting planar componets" << endl;
  while (cloud_filtered->points.size () > 0.3 * nr_points)
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
    cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << endl;

    //define cloud_f for extracting into
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }


  
  // Convert to ROS data type
  // sensor_msgs::PointCloud pc1_pub_ds;

  // pcl_conversions::moveFromPCL(*cloud_filtered, pc1_pub_ds);


  // Create the KdTree object for the search method of the extraction
  cout << "Creating Kdtree objects" << endl;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  cout << "setting up cluster objects" << endl;
  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 2cm: too small, we split one object into many, too big, we merge objects into one.
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

 //////////////////////* Perform the extraction ////////////////////////////////////////

  // we have used some dodgy coding here so we only loop ONCE- WE ONLY WANT THE LARGEST CLUSTER
  cout << "extracting the clusters" << endl;
  
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
    return cloud_cluster; // actually we want to leave this loop after the first cluster
  }
}


//void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg, tf::TransformListener& odom_base_ls)
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // INPUTS: cloud_msg: pointer to the raw input cloud from the velodyne
  // OUTPUTS: void
  // This is a callback function that reads in the velodyne message, removes the ground plane, 
  // calls get_cloud_cluster() to get the largest cluster, extracts the centroid from this, 
  // and publishes the centroid coordinates on the transform "velodyne_person_est"

  //////////////////////////////////////
  // COMMENTED OUT BECAUSE I DON'T THINK CLOUD HAS BEEN DEFINED YET
  // cerr << "Raw Cloud: " << endl;
  // for (size_t i = 0; i < cloud_msg->points.size (); ++i)
  //   cerr << "    " << cloud_msg->points[i].x << " " 
  //                       << cloud_msg->points[i].y << " " 
  //                       << cloud_msg->points[i].z << endl;
  

  ///////////////////////////////////////

  
  //////////////////////* TRANSFORM THE INPUT CLOUD INTO THE ODOM FRAME  /////////////////////////////
  // Publish the cloud
  pub_raw.publish (*cloud_msg);

  string target_frame = "odom", base_frame = "base"; // target frame for transform
  sensor_msgs::PointCloud2 transformed_cloud; // initiate output cloud

  cout<<"**********************waiting for transform"<<endl;
  //odom_base_ls.waitForTransform(target_frame, base_frame, ros::Time::now(), ros::Duration(10.0) );
  odom_base_ls->waitForTransform(target_frame, base_frame, ros::Time(0), ros::Duration(10.0) );

  cout<<"transforming pcloud using time "<< ros::Time(0)<<endl;
  pcl_ros::transformPointCloud(target_frame, *cloud_msg, transformed_cloud, *odom_base_ls); // perform the transformation


  // Publish the cloud
  pub_trans.publish (transformed_cloud);

  //////////////////////* REMOVE ALL POINTS OUTSIDE OF 0.1, 100.0  /////////////////////////////
  // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(transformed_cloud, *cloud);


  //   if cloud has less than 10 points, jump out of the callback
    if (cloud->width < 10) { // the velodyne reading is invalid- probably an issue with the transform
          cout << "*** THERE'S SOMETHING WRONG WITH THIS CLOUD. waiting for the next one. ****" << endl;
          cout << "Width: "<< cloud->width <<endl;
        return; // drop out of this callback and come back with the next transform
    }
    cout << "*** LOOKS OK TO ME***"<< endl;

  // setup the filter
  pcl::PassThrough<pcl::PCLPointCloud2> pass; 
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 4.0);
  //pass.setFilterLimits (0.1, 100.0);
  //** set the input cloud
  //sor.setInputCloud (cloudPtr);

  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // create a pointer called cloudPtr to use for our filter

  pass.setInputCloud (cloudPtr); //set the input cloud for our filter
  

  // ** perform the filtering
  //sor.filter (cloud_filtered);
  pcl::PCLPointCloud2 output_cloud; // initiate our PC2 to send
  pass.filter(output_cloud);


  //////////////////////* PUBLISH THE RESULT ON pub/////////////////////////////

  // Convert to ROS data type
   sensor_msgs::PointCloud2 output_msg;
  pcl_conversions::moveFromPCL(output_cloud, output_msg);

  // // Publish the data
  pub_zfilt.publish (output_msg);

////////////////////////////////////////////////////////// NOT SURE WHAT THESE LINES DO.
  //pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  //cout <<" cloud_filtered size: "<< cloud_filtered.points.size ()<< endl;
  //cout << centroid_cloud.points.size ()<< endl;
  
  ////////////////////// GET THE GROUND CLUSTER /////////////////////////////

  // Take the previously published output message and convert it to a pointcloud called "prefiltered cloud"
  pcl::PointCloud<pcl::PointXYZ> prefiltered_cloud;  // pcl version of the point cloud

  pcl::fromROSMsg (output_msg, prefiltered_cloud);


  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (); //convert from an object to a boost shared pointer

//  using namespace boost;
  typedef pcl::PointCloud<pcl::PointXYZ>  mytype;
  boost::shared_ptr<mytype> prefiltered_cloud_ptr = boost::make_shared <mytype> (prefiltered_cloud);


  // create a cloud pointer centroid_cloud and populate it with the largest cluster from prefiltered_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  centroid_cloud_ptr = get_cloud_cluster(prefiltered_cloud_ptr);

 // publish it!
  sensor_msgs::PointCloud2 msg_to_publish2; // initiate intermediate message variable
  // // CONVERT FROM PCL:PC1 TO SM:PC2
  pcl::toROSMsg(*centroid_cloud_ptr,msg_to_publish2); // con
  // //cout<< centroid_cloud_ptr->points.size <<endl;
  msg_to_publish2.header.frame_id = "odom";
  pub_centroid.publish (msg_to_publish2); // OK APPARENTLY THIS WORKS

  // check the size
  cout << "There are "<<centroid_cloud_ptr->points.size ()<<" points in centroid_cloud_ptr"<< endl;
  

  ////////////////////// COMPUTE THE CENTROID OF THE CLUSTER /////////////////////////////
  // Initialise a point to store the centroid inoutput_topic
  pcl::CentroidPoint<pcl::PointXYZ> centroid;


  // loop through the point cloud, adding each to the centroid
  for (size_t i = 0; i < centroid_cloud_ptr -> points.size (); ++i) {
    centroid.add (centroid_cloud_ptr -> points[i]);
  }

  // Fetch centroid using `get()`
  pcl::PointXYZ c; // the centre point
  centroid.get (c);
  cout << "\n Centroid is located at "
            << c
            << endl; 



  ////////////////////// PUBLISH THE CENTROID ON A TRANSFORM /////////////////////////////
  // create a transformBroadcaster 
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(c.x,c.y,c.z) );
  tf::Quaternion q; // initialise the quaternion q for the pose angle
  tf::TransformListener odom_base_ls;
  //msg = ...?;
  q.setEulerZYX(0, 0, 0);
  transform.setRotation(q);
  //q.setRPY(0, 0, msg->theta);
  string velodyne_frame_id = "odom";
  string target_frame_id = "velodyne_person_est";
  //br.sendTransform(centre_point, q, ros::Time::now(), target_frame_id, velodyne_frame_id);
  // not sure if i put the ids the right way round
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), velodyne_frame_id, target_frame_id));
  
  
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate r(1); // 10 hz


  odom_base_ls.reset(new tf::TransformListener);
  // Create a ROS subscriber for the input point cloud
  string input_topic = "/velodyne/point_cloud_filtered";
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_topic, 1, boost::bind(&cloud_cb, _1, odom_base_ls));
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  string topic_raw = "pcl_raw", topic_trans = "pcl_trans", topic_zfilt = "pcl_zfilt", topic_ds = "pcl_ds", topic_centroid = "pcl_centroid";
  
  pub_raw = nh.advertise<sensor_msgs::PointCloud2> (topic_raw, 1);
  pub_trans = nh.advertise<sensor_msgs::PointCloud2> (topic_trans, 1);
  pub_zfilt = nh.advertise<sensor_msgs::PointCloud2> (topic_zfilt, 1);
  pub_ds = nh.advertise<sensor_msgs::PointCloud2> (topic_ds, 1);
  pub_centroid = nh.advertise<sensor_msgs::PointCloud2> (topic_centroid, 1);

  // Spin
  ros::spin ();
  r.sleep();
}

