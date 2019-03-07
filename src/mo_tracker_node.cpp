/*
*
* tracker_node.cpp
*
* int main() {
*
*
*  - create nh
*  - kf_params is kf_params
*  - create tracker node ( nh, kf_params)
*  -
*/


#include <ros/ros.h>
#include "include/mo_tracker.h"
using namespace std;
//struct kf_param_struct {VectorXd x0; double dt; MatrixXd A; MatrixXd C; MatrixXd Q; MatrixXd R; MatrixXd P;}; // define this type of structure

kf_param_struct getTrackerKfParams() {
    // Sets the tracker's kalman filter kf's parameters dt,A,C,Q,R,P
    int n = 3; // Number of states
    int m = 3; // Number of measurements

    double dt = 1.0/10;

    MatrixXd I3(3,3); //define an identity matrix
    I3.setIdentity();
    cout << I3 << endl;

    MatrixXd delF(n, n); // System dynamics matrix
    MatrixXd delH(m, n); // map state space to observation space
    MatrixXd delGQdelGT(n, n); // Process noise covariance
    MatrixXd R(m, m); // Measurement noise covariance
    MatrixXd P0(n, n); // Estimate error covariance initial state

    // Assuming the person doesn't move, we are JUST X INITIALLY
    delF = I3; //I3
    delH = I3;

    delGQdelGT << 2, 0, 0, 0, 2, 0, 0, 0, .5; // MAKE THIS A FUNCTION OF TIMESTEP^2
    R << 20, 0, 0, 0, 20, 0, 0, 0, 1; //I3 * .05 // MAKE THIS A FUNCTION OF TIMESTEP^2
    P0 << 3, 0, 0, 0, 3, 0, 0, 0, 3;

    // initialise the kalman filter with the given parameters

    kf_param_struct kf_params = {.dt = dt, .delF = delF, .delH = delH, .delGQdelGT = delGQdelGT, .R = R, .P0 = P0}; // move all the params into the struct
    return kf_params; // return the parameters
}

int main (int argc, char** argv) // runs the tracker node
{
    cout << "Initiating tracker node"<<endl;

    // Initialize ROS
    ros::init (argc, argv, "tracker_node"); // initialise the node

    ros::NodeHandle nh; // setup a nodehandle for communication between methods
    int max_cluster_size = 150;
    int min_cluster_size = 40;
    double cluster_tolerance = .4; // too small, we split one object into many, too big, we merge objects into one. In metres
    double seg_dist_threshold = 0.03; // how close a point must be to the model in order to be considered an inlier in metres

    int file_index = 0; //index of next results file to generate
    MOTracker our_tracker(nh, max_cluster_size, min_cluster_size, cluster_tolerance, seg_dist_threshold, getTrackerKfParams(), false, true, true, 0); // construct a tracker called our_tracker

    ros::spin ();// spin ros
    return 0;
}



