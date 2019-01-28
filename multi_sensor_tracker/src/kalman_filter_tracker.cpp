#include <ros/ros.h> // because this is a robot
// tf includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "kalman_filter.h" // for the kalman filter
#include <Eigen/Dense>  // for all the lovely matrices
#include <unistd.h>
using namespace std;
using namespace Eigen; // to make matrix stuff more compact

int main (int argc, char** argv)
{
    cout <<"Initialising kalman_filter_tracker"<<endl;

    int n = 1; // Number of states
    int m = 1; // Number of measurements

    // 1. Initialise ROS

    ros::init (argc, argv, "multi_sensor_tracker");
    ros::NodeHandle nh;
    ros::Rate r(1); // 10 hz
    // 2. Subscribe to input transform

    tf::TransformListener vel_listener;
    tf::StampedTransform input_tf;

    string base_frame = "odom"; // define the transform origin we are looking for
    string target_frame = "velodyne_person_est"; // define the transform target we are looking for

    VectorXd y(m); // define a variable y to hold all the input transforms.

    // 3. Create a Kalman filter to process the inputs


    double dt = 1.0/30; // Time step

    MatrixXd A(n, n); // System dynamics matrix
    MatrixXd C(m, n); // Output matrix
    MatrixXd Q(n, n); // Process noise covariance
    MatrixXd R(m, m); // Measurement noise covariance
    MatrixXd P(n, n); // Estimate error covariance

    // Assuming the person doesn't move, we are JUST X INITIALLY
    //A << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    A << 1; // assume the person doesn't move

    //C << 1, 0, 0;
    C << 1;
    // Reasonable covariance matrices
    //Q << .05, .05, .0, .05, .05, .0, .0, .0, .0; // I DON'T KNOW HOW TO TUNE THIS
    Q << .001;
    R << 5; //OR THIS
    //P << .1, .1, .1, .1, 10000, 10, .1, 10, 100; //OR THIS, FOR THAT MATTER
    P << 1;
    //print out the chosen matrices
    cout << "A: \n" << A << endl;
    cout << "C: \n" << C << endl;
    cout << "Q: \n" << Q << endl;
    cout << "R: \n" << R << endl;
    cout << "P: \n" << P << endl;

    // create the filter
    kalman_filter kf(dt, A, C, Q, R, P);
    VectorXd x0(n);
    x0 << 0;
    kf.init(0, x0); // initialise the kalman filter
    // 4. Create a broadcaster on which to publish the outputs
     tf::TransformBroadcaster br;
     string output_frame = "kalman_filter_est";


    ros::Time curr_time, prev_time;
    ros::Duration delta;
    prev_time = ros::Time(0);
//     * 5. Loop:
     while (nh.ok()) {
         // receive the input transform

         try{
           vel_listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(5.0) );
           vel_listener.lookupTransform(base_frame, target_frame, ros::Time(0), input_tf);
         }
         catch (tf::TransformException ex){ // for things like- can't see the input transform
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
         }

         ///////////////////////////////////
         prev_time = curr_time;
         curr_time = ros::Time::now();
         delta = curr_time - prev_time;
         ///////////////////////////////////
         // extract coordinates from transform
         tf::Vector3 input_vector;
         input_vector = input_tf.getOrigin();

         //y << input_vector.getX(), input_vector.getY(), input_vector.getZ();
         y << input_vector.getX();// we are only getting one measurement- put this in y
         kf.update(y, delta.toSec(), A);
         cout << "t = " << curr_time << ", dt = "<<delta<<", y =" << y.transpose()
             << ", x_hat = " << kf.state().transpose() << endl;


        r.sleep();
     }

//     *      publish the output



}
