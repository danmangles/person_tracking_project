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
    // kf params


//    struct kf_param_struct {VectorXd x0; double dt; MatrixXd A; MatrixXd C; MatrixXd Q; MatrixXd R; MatrixXd P;}; // define this type of structure


    int n = 3; // Number of states
    int m = 3; // Number of measurements
    double dt = 1.0/10; // Time step

    MatrixXd I3(3,3); //define an identity matrix
    I3.setIdentity();
    cout << I3 << endl;

    MatrixXd A(n, n); // System dynamics matrix
    MatrixXd C(m, n); // Output matrix
    MatrixXd Q(n, n); // Process noise covariance
    MatrixXd R(m, m); // Measurement noise covariance ///////////////////////////////DIMENSION???
    MatrixXd P(n, n); // Estimate error covariance

    // Assuming the person doesn't move, we are JUST X INITIALLY
    A = I3; //I3
    C = I3;

    Q << 2, 0, 0, 0, 2, 0, 0, 0, .5; // MAKE THIS A FUNCTION OF TIMESTEP^2
    R << 20, 0, 0, 0, 20, 0, 0, 0, 1; //I3 * .05 // MAKE THIS A FUNCTION OF TIMESTEP^2
    P << 3, 0, 0, 0, 3, 0, 0, 0, 3; //I3

    // initialise the kalman filter with the given parameters
//    VectorXd x0(3);
//    x0 << 0,0,0;

    kf_param_struct kf_params = {.dt = dt,.A = A,.C = C,.Q = Q,.R = R,.P = P}; // move all the params into the struct

//    our_tracker.setupKalmanFilter(x0, dt,A,C,Q,R,P);
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
    MOTracker our_tracker(nh, max_cluster_size, min_cluster_size, cluster_tolerance, seg_dist_threshold, getTrackerKfParams(), true, true); // construct a tracker called our_tracker


//    setTrackerKfParams(our_tracker); // setup the kalman filter inside the tracker

    ros::Rate r(10); // 10 hz
    ros::spin ();// spin ros

    return 0;
}



