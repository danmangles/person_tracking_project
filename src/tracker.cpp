/*
 *
 * tracker.cpp
 *
 *    tracker::tracker(nh, kf_params) : nh_(nh), kf_(kf_params) {
 *          // setup a subscriber which calls the callback
 *
 * }
 *
 */

#include <ros/ros.h> // because this is a robot
#include "include/tracker.h"

using namespace std;

tracker::tracker(ros::NodeHandle nh,
                 double dt,
                 const Eigen::MatrixXd& A,
                 const Eigen::MatrixXd& C,
                 const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& P
                 ) :
    nh_(nh), kf_(dt, A, C, Q, R, P) // initiate the nodehandle and kalman_filter
{
    cout<< "tracker constructor called "<<endl;
}

/*ros::NodeHandle &nh  //setup the tracker constructor
                 double dt,
                 const Eigen::MatrixXd& A,
                 const Eigen::MatrixXd& C,
                 const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R,
                 const Eigen::MatrixXd& P*/
