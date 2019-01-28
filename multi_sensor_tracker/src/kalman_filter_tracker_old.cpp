
#include <ros/ros.h> // because this is a robot
// tf includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense> // for matrices
#include <iostream>   // also for matrices i think

//#include <Transform.h>
using namespace std;
bool verbose = 1;

int filter_length = 5;

void update_average(tfScalar measurements[5][3], tfScalar latest_measurement[3],tfScalar output_estimate[3], int filter_length) {
/*
Inputs: measurements[filter_length][n_dimensions]: the list of recorded position measurements
                latest_measurement[n_dimensions]: the latest measurement recorded from tf listener
                output_estimate[n_dimensions]: our weighted average estimate of the position
*/
        cout << "\n\n**********UPDATING THE AVERAGE ***********************8"<<endl;
        // Extended Kalman Filter:
  // 0. Initialise step:
  // matrices: https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
  Matrix3f Q, R; //Q, R are process, sensor noise covariance/PSD
  Matrix3f P_curr, P_next; //P_curr, P_next are current and next estimates of Kalman Filter covariance
  Vector3d x_hat_prev, x_hat_curr, x_hat_next, y_curr, Kf;
  Matrix3f eye;
  eye << 1, 0, 0,
      0, 1, 0,
      0, 0, 1;// setup for use later
  Q << .2, 0, 0,
      0, .2, 0,
      0, 0, 0.01; // populate process noise with how far we estimate a person will walk in 1 timestep (how long is a timestep). Assume not much motion in z

  R << 4, 0, 0,
      0, 5, 0,
      0, 0, 1;// populate sensor noise with some basic estimates of sensor performance

  P_curr = eye;// populate this with our intitial matrix
  Vector3d C ( 1, 1, 1);// we measure the state directly, therefore C is 1 1 1 (SHOULD THIS BE TRANSPOSED?)

  Vector3d y_curr(latest_measurement[0],latest_measurement[1],latest_measurement[2]); // move latest_measurement into the correct format

  cout << "Q: "<<Q<<endl;
  cout << "R: "<<R<<endl;
  cout << "P_curr: "<<P_curr<<endl;
  cout << "y_curr: "<<y_curr<<endl;

        // 1. Prediction Step: increase the Covariance of the prior estimate
  x_hat_curr = x_hat_prev; // assume person doesn't fundamentally move
  P_curr = Q;

        // 2. measure step: shift the mean using the last estimate + input estimate
  x_hat_next = x_hat_curr + Kf*(y_curr - C*x_hat_curr) // update the state
  P_next = (eye - Kf*C)*P_curr*(eye - Kf*C).transposeInPlace() + Kf*R*Kf.transposeInPlace();
  Kf = P_next*C.transposeInPlace()*(P_next + R).inverse(); // these lines might all be wrong
        /*
        we want to estimate the updated state estimate
                x_hat = x_hat_prev - K * yk
        where K = Kalman filter gain, yk = latest measurement residual:
                yk = zk - h(x_hat_prev) << what is h??? CALCULATE OUR MEASUREMENT RESIDUAL
                Sk = Hk*Pk*HT + Rk


        */



}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "multi_sensor_tracker");
  ros::NodeHandle nh;

  string vel_target_frame = "velodyne_person_est";

  // Create a ROS subscriber for the input transform
  tf::TransformListener vel_listener;
  // create a broadcaster to use on output
  tf::TransformBroadcaster br;


  tfScalar measurements[filter_length][3] = {{0,2,3},{4,5,0},{0,0,0},{4,5,0},{0,0,0}}; // initiate the average we will use to store our predictions

  tfScalar latest_measurement[3] = {0,0,0};
  tfScalar output_estimate[3]= {0,0,0};

  cout << "starting the loop" << endl;
  // make the loop

  ros::Rate rate(10.0);

  while (nh.ok()) {
    // initiate a transform in which to store inputs
    tf::StampedTransform input_tf;

    string base_frame = "odom";
    //string real_target_frame = "realsense_person_est";
    string target_frame = "velodyne_person_est";

    try{
      vel_listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(5.0) );
      vel_listener.lookupTransform(base_frame, target_frame, ros::Time(0), input_tf);
    }
    catch (tf::TransformException ex){ // for things like- can't see the input transform
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    string output_frame = "moving_average_est";

    // extract coordinates from transform
    tf::Vector3 input_vector;
    input_vector = input_tf.getOrigin();

    tfScalar x = input_vector.getX(), y = input_vector.getY(), z = input_vector.getZ();


    latest_measurement[0] = x;
    latest_measurement[1] = y;
    latest_measurement[2] = z;

    cout<< "input x is ("<<x<<","<<y<<","<<z<<")"<<endl;

    cout << filter_length<<endl;
    update_average(measurements, latest_measurement, output_estimate, filter_length); //pass it the address of average

         // Print the average
          cout<<"\naverage  is "<< endl;
          for (int i = 0; i < filter_length; i++){
                 cout<<"("<<measurements[i][0] <<","<<measurements[i][1] <<","<<measurements[i][2] <<")"<< endl;
          }

        cout<<"\noutput_estimate = ("<<output_estimate[0] <<","<<output_estimate[1] <<","<<output_estimate[2] <<")"<< endl;

    //Now publish the output
        tf::Transform output_tf;
        output_tf.setOrigin( tf::Vector3(output_estimate[0],output_estimate[1],output_estimate[2]) );

        tf::Quaternion q; // initialise the quaternion q for the pose angle
        q.setEulerZYX(0, 0, 0);
        output_tf.setRotation(q);
    //

    br.sendTransform(tf::StampedTransform(output_tf, ros::Time(0), base_frame, output_frame));

    rate.sleep();
  }
  return 0;

};
