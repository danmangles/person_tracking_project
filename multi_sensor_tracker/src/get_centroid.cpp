#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>

#include <pcl/common/centroid.h>

// includes for cluster extraction 
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <math.h>       /* sin */
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_cluster (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {

  //define cloud_f for extracting into
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (input_cloud);
  //vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  
  std::cout << "Initiating Segmentation objects" << std::endl;  
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  std::cout << "Segmenting planar componets" << std::endl;
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  std::cout << "Creating Kdtree objects" << std::endl;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::cout << "setting up cluster objects" << std::endl;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.7); // 2cm
  ec.setMinClusterSize (4);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::cout << "extracting the clusters" << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    return cloud_cluster; // actually we want to leave this loop after the first cluster
  }
}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2 output_cloud; // initiate our PC2 to send

  // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud2
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  //////////////////////////////////////
  /* COMMENTED OUT BECAUSE I DON'T THINK CLOUD HAS BEEN DEFINED YET
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
  */

  ///////////////////////////////////////
  // ** initialise a filter called pass and tune basic params
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //sor.setLeafSize (0.1, 0.1, 0.1);
  pcl::PassThrough<pcl::PCLPointCloud2> pass; 
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 100.0);
  //** set the input cloud
  //sor.setInputCloud (cloudPtr);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // create a cloudPtr to use for our filter
  pass.setInputCloud (cloudPtr); //set the input cloud for our filter
  

  // ** perform the filtering
  //sor.filter (cloud_filtered);
  pass.filter(output_cloud);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_msg;
  pcl_conversions::moveFromPCL(output_cloud, output_msg);

  // Publish the data
  pub.publish (output_msg);

//////////////////////////////////////////////////////////
  //pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  //std::cout <<" cloud_filtered size: "<< cloud_filtered.points.size ()<< std::endl;
  //std::cout << centroid_cloud.points.size ()<< std::endl;
  
  // Take the previously published output message and convert it to a pointcloud called "prefiltered cloud"
  pcl::PointCloud<pcl::PointXYZ> prefiltered_cloud;  // pcl version of the point cloud

  pcl::fromROSMsg (output_msg, prefiltered_cloud);


  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (); //convert from an object to a boost shared pointer

  using namespace boost;
  typedef pcl::PointCloud<pcl::PointXYZ>  mytype;
  shared_ptr<mytype> prefiltered_cloud_ptr = make_shared <mytype> (prefiltered_cloud);


  // create a cloud pointer centroid_cloud and populate it with the largest cluster from prefiltered_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);



  centroid_cloud_ptr = get_cloud_cluster(prefiltered_cloud_ptr);

  // centroid_cloud is now a pointer
  //pcl::PointCloud<pcl::PointXYZ> centroid_cloud = *centroid_cloud_ptr;  // pcl version of the point cloud

  ////////////////////////////////////////////////
  // check the size
  std::cout << centroid_cloud_ptr->points.size ()<< std::endl;
  
  // Initialise a point to store the centroid in 
  pcl::CentroidPoint<pcl::PointXYZ> centroid;


  // loop through the point cloud, adding each to the centroid
  for (size_t i = 0; i < centroid_cloud_ptr -> points.size (); ++i) {
    centroid.add (centroid_cloud_ptr -> points[i]);
  }

  // Fetch centroid using `get()`
  pcl::PointXYZ c; // the centre point
  centroid.get (c);
  std::cout << "\n Centroid is located at "
            << c
            << std::endl; 
//////////////////////////////////////////////////////////////
  // create a transformBroadcaster 
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(c.x,c.y,c.z) );
  tf::Quaternion q; // initialise the quaternion q for the pose angle

  //msg = ...?;
  q.setEulerZYX(0, 0, 0);
  transform.setRotation(q);
  //q.setRPY(0, 0, msg->theta);
  std::string velodyne_frame_id = "base"; 
  std::string target_frame_id = "velodyne_person_est";
  //br.sendTransform(centre_point, q, ros::Time::now(), target_frame_id, velodyne_frame_id);
  // not sure if i put the ids the right way round
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), velodyne_frame_id, target_frame_id));
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  std::string input_topic = "/velodyne/point_cloud_filtered";
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
}