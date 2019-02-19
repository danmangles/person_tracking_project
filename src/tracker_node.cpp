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
#include "include/tracker.h"
using namespace std;
void setTrackerKfParams(Tracker &our_tracker) {
    // Sets the tracker's kalman filter kf's parameters dt,A,C,Q,R,P
    // kf params

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

    Q << 3, 0, 0, 0, 3, 0, 0, 0, .5; // MAKE THIS A FUNCTION OF TIMESTEP^2
    R << 30, 0, 0, 0, 30, 0, 0, 0, 1; //I3 * .05 // MAKE THIS A FUNCTION OF TIMESTEP^2
    P << 2, 0, 0, 0, 2, 0, 0, 0, 2; //I3

    // initialise the kalman filter with the given parameters
    VectorXd x0(3);
    x0 << 0,0,0;
    our_tracker.setupKalmanFilter(x0, dt,A,C,Q,R,P);
    return;
}
int main (int argc, char** argv) // runs the tracker node
{
    cout << "Initiating tracker node"<<endl;

    // Initialize ROS
    ros::init (argc, argv, "tracker_node"); // initialise the node
    // ros::Rate r(10); // 10 hz

    ros::NodeHandle nh; // setup a nodehandle for communication between methods
    int max_cluster_size = 300;
    int min_cluster_size = 50;
    double cluster_tolerance = 0.3; // too small, we split one object into many, too big, we merge objects into one. In metres
    double seg_dist_threshold = 0.03; // how close a point must be to the model in order to be considered an inlier in metres

    Tracker our_tracker(nh, max_cluster_size, min_cluster_size, cluster_tolerance, seg_dist_threshold, true); // construct a tracker called our_tracker
    setTrackerKfParams(our_tracker); // setup the kalman filter inside the tracker

    ros::spin ();// spin ros

    return 0;
}



