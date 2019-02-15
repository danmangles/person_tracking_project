#include <ros/ros.h>
// tf includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include <Transform.h>
using namespace std;
bool verbose = 1;

int filter_length = 5;

void update_average(tfScalar average[5][3], tfScalar input_estimate[3],tfScalar output_estimate[3], int filter_length) {
/*
Inputs: average
*/
	cout << "\n\n**********UPDATING THE AVERAGE ***********************8"<<endl;
  int n_coords = 3; // xyz
  //double weights[filter_length] = {.4, .25, .15, .1, .1}; //
  double weights[filter_length] = {40, 30, 20, 10, 5}; //these are the weights to assign to recent samples vs older samples. 
  // need to program these to actually adjust when we change filter length
  
  double mass_of_weights = 0;
  for (int i = 0; i < filter_length; i ++) {mass_of_weights += weights[i];} // get the mass of weights to normalise them

  cout << "mass of weights is "<<mass_of_weights <<endl;
  cout<<"new_estimate is ("<<input_estimate[0] <<","<<input_estimate[1] <<","<<input_estimate[2] <<")"<< endl;



  // Print the average
  cout<<"\naverage  is "<< endl;
  for (int i = 0; i < filter_length; i++){
  	 cout<<"("<<average[i][0] <<","<<average[i][1] <<","<<average[i][2] <<")"<< endl;
  }
  
  
  
  for (int i = filter_length-1; i > 0; i--) { // count down from end of array to 1
    
  	for (int j = 0; j < n_coords; j++){ // loop across the x y z
  		// cout << i<<","<<j<<endl;
  		// cout << average[i][j]<<endl;
  		// cout << average[i-1][j]<<endl;
  		average[i][j] = average[i-1][j]; //shift the old estimates down
  	}
  }
  for (int j = 0; j < n_coords; j++){ // loop across the x y z
  		
  	average[0][j] = input_estimate[j]; //shift the old estimates down
  }

  // NOW WE NEED TO RETURN OUTPUT ESTIMATE
  tfScalar x = 0,y=0,z=0;
  for (int j = 0; j < n_coords; j++){ // loop across the x y z
  	output_estimate[j] = 0; // clear the estimate
  	 for (int i = 0; i < filter_length; i++) { //loop back across the average
  	 	//output_estimate[j] += average[i][j]; // add to the output average
  	 	output_estimate[j] += average[i][j]*weights[i]; // add to the output average
  	 	
  	 	cout << "output_estimate["<<j<<"] = "<<output_estimate[j]<<endl;
  	 }
  	output_estimate[j] = output_estimate[j]/mass_of_weights; // now divide by number of terms to compute the average

  	 cout << "divided out: output_estimate["<<j<<"]"<<output_estimate[j]<<endl;
  }


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


  tfScalar average[filter_length][3] = {{0,2,3},{4,5,0},{0,0,0},{4,5,0},{0,0,0}}; // initiate the average we will use to store our predictions
  
  tfScalar input_estimate[3] = {0,0,0};
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


    input_estimate[0] = x;
    input_estimate[1] = y;
    input_estimate[2] = z;

    cout<< "input x is ("<<x<<","<<y<<","<<z<<")"<<endl;

    cout << filter_length<<endl;
    update_average(average, input_estimate, output_estimate, filter_length); //pass it the address of average
	
	 // Print the average
	  cout<<"\naverage  is "<< endl;
	  for (int i = 0; i < filter_length; i++){
	  	 cout<<"("<<average[i][0] <<","<<average[i][1] <<","<<average[i][2] <<")"<< endl;
	  }

	cout<<"\noutput_estimate = ("<<output_estimate[0] <<","<<output_estimate[1] <<","<<output_estimate[2] <<")"<< endl;
    
    //////////////////////////////////////////////Now publish the FIRST COORDINATE OF THE OUTPUT ON A TRANSFORM
  	tf::Transform output_tf;
  	output_tf.setOrigin( tf::Vector3(output_estimate[0],output_estimate[1],output_estimate[2]) );

  	tf::Quaternion q; // initialise the quaternion q for the pose angle
  	q.setEulerZYX(0, 0, 0);
  	output_tf.setRotation(q);
    //

    br.sendTransform(tf::StampedTransform(output_tf, ros::Time::now(), base_frame, output_frame));
  
    rate.sleep();
  }
  return 0;

};
