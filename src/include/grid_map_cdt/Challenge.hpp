/*
 * NavigationDemo.hpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <string>
#include <chrono>


#include <tf/transform_listener.h>

namespace grid_map_demos {

/*!
 * Applies a chain of grid map filters to a topic and
 * republishes the resulting grid map.
 */
class NavigationDemo
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param success signalizes if filter is configured ok or not.
   */
  NavigationDemo(ros::NodeHandle& nodeHandle, bool& success);

  /*!
   * Destructor.
   */
  virtual ~NavigationDemo();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
  void callback(const grid_map_msgs::GridMap& message);


  // true - planning is good. send carrot.
  // false - planning isn't working or its disabled. don't send it
  bool planCarrot(const grid_map_msgs::GridMap& message,
    Eigen::Isometry3d pose_robot, grid_map::Position pos_goal,
    Eigen::Isometry3d& pose_chosen_carrot);

  void tic();
  std::chrono::duration<double> toc();

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Name of the input grid map topic.
  std::string inputTopic_;

  //! Name of the output grid map topic.
  std::string outputTopic_;

  //! Grid map subscriber
  ros::Subscriber subscriber_;

  //! Grid map publisher.
  ros::Publisher outputGridmapPub_, footstepPlanRequestPub_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;

  tf::TransformListener* listener_;


  std::chrono::high_resolution_clock::time_point lastTime_;

  bool verbose_;
  bool verboseTimer_;
  bool plannerEnabled_;
  bool demoMode_;

};

} /* namespace */
