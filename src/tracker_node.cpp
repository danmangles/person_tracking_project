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

int main (int argc, char** argv) // runs the tracker node
{
    cout << "Initiating tracker node"<<endl;
    //////////////////////////////////////////////

    // kf params

    int n = 3; // Number of states
    int m = 3; // Number of measurements
    VectorXd y(m); // define a variable y to hold all the input transforms. //NOT SURE IF d,m or m,d

    // 3. Create a Kalman filter to process the input

    double dt = 1.0/10; // Time step

    MatrixXd I3(3,3);
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
    R << 20, 0, 0, 0, 20, 0, 0, 0, 1; //I3 * .05 // MAKE THIS A FUNCTION OF TIMESTEP^2
    P << 2, 0, 0, 0, 2, 0, 0, 0, 2; //I3
    ////////////////////////////////////////////////

    // Initialize ROS
    ros::init (argc, argv, "tracker_node");
//    ros::Rate r(10); // 10 hz

    ros::NodeHandle nh;
    tracker our_tracker(nh,dt,A,C,Q,R,P); // construct a tracker called our_tracker
    ros::spin ();// spin ros

    return 0;
}

