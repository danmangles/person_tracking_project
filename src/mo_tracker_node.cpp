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

#include <ctime>
#include <ros/ros.h>
#include "include/mo_tracker.h"
using namespace std;
//struct kf_param_struct {VectorXd x0; double dt; MatrixXd A; MatrixXd C; MatrixXd Q; MatrixXd R; MatrixXd P;}; // define this type of structure
pcl_param_struct getPclParams() {
    // sets all the parameters for the pcl_params struct:
    return {.apply_passthrough_filter = true,
                .apply_planar_outlier_removal = true, ///////////////////////////////////// TODO
                .apply_voxel_grid = false,
                .max_cluster_size = 250,
                .min_cluster_size = 40,
                .cluster_tolerance = .4, // too small, we split one object into many, too big, we merge objects into one. In metres
                .seg_dist_threshold = .1, // how close a point must be to the model in order to be considered an inlier in metres
                .box_x = 10.0,
                .box_y = 9.0,
                .min_height = -0.1, // min height of z filter in metres below base
                .downsample_factor = 0.7
    };
}

tracker_param_struct getTrackerParams() {
    // Sets the tracker's kalman filter kf's parameters dt,A,C,Q,R,P
    // add a velocity state
    return {.gating_dist_constant = 0.65, //this is used in max_gating_dist calcs e.g.
                          // max = tracker_params.gating_dist_constant*sqrt((P(0,0) + P(1,1))/2);
            .base_gating_dist = 1, // gating distance for uninitiated tracklets in m
            .max_consecutive_misses = 6,// if we've missed this tracklet too many times in a row, delete it
            .min_initialisation_length = 4,// min number of detections needed to start the kalman filter
            .only_init_rgb_tracklet = false
    };
}

kf_param_struct getKfParams() {
    // Sets the tracker's kalman filter kf's parameters dt,A,C,Q,R,P
    // add a velocity state

    int n = 6; // Number of states (velocity
    int m = 3; // Number of measurements
    double dt = 1.0/5;
    MatrixXd I3(3,3); //define an identity matrix
    I3.setIdentity();

    ///// Setup the matrices
    // delF
    MatrixXd delF(n, n); // System dynamics matrix
    delF.setIdentity(); // delF has a diagonal of ones, don't add the velocity state to the first estimate
    // delH
    MatrixXd delH(m, n); // map state space to observation space
    delH.setZero();
    delH.block(0,0,3,3) = I3; // set left 3x3 block to identity
    //delGQdelGT
    MatrixXd delGQdelGT(n, n); // Process noise covariance
    delGQdelGT.setIdentity(); //MAKE THIS A FUNCTION OF TIMESTEP^2
    delGQdelGT = delGQdelGT*2; // to do with walking speed
    //R
    MatrixXd R(m, m); // Measurement noise covariance
    R << 20, 0, 0, 0, 20, 0, 0, 0, 1; //I3 * .05 // MAKE THIS A FUNCTION OF TIMESTEP^2
    //P0
    MatrixXd P0(n, n); // Estimate error covariance initial state
    P0 = delGQdelGT; // set to same as process noise initially


   // initialise the kalman filter with the given parameters
    //print out the chosen matrices
    cout << "delF: \n" << delF << endl;
    cout << "delH: \n" << delH << endl;
    cout << "delGQdelGT: \n" << delGQdelGT << endl;
    cout << "R: \n" << R << endl;
    cout << "P0: \n" << P0 << endl;

    return {.dt = dt, .delF = delF, .delH = delH, .delGQdelGT = delGQdelGT, .R = R, .P0 = P0}; // move all the params into the struct
}

io_param_struct getIOParams(){
    // returns all the IO parameters

    // get date for filename
    time_t now = time(0);
    tm *ltm = localtime(&now);
//    cout << "Month: "<< 1 + ltm->tm_mon<< endl;
//    cout << "Day: "<<  ltm->tm_mday << endl;
//    cout << "Time: "<< 1 + ltm->tm_hour << ":";
//    cout << 1 + ltm->tm_min << ":";
//    cout << 1 + ltm->tm_sec << endl;

    stringstream res_filename, gnd_filename;
    res_filename << "results_CSVs/res_0"<<1+ ltm->tm_mon<<  ltm->tm_mday<<"_"<<ltm->tm_hour<<1 + ltm->tm_min<< ".csv";
    gnd_filename << "results_CSVs/gnd_0"<<1+ ltm->tm_mon<<  ltm->tm_mday<<"_"<<ltm->tm_hour<<1 + ltm->tm_min<< ".csv";

    return {.publishing = true,
                .res_filename = res_filename.str(),
                .gnd_filename = gnd_filename.str()
    };
}

int main (int argc, char** argv) // runs the tracker node
{
    cout << "Initiating tracker node"<<endl;

    // Initialize ROS
    ros::init (argc, argv, "tracker_node"); // initialise the node

    ros::NodeHandle nh; // setup a nodehandle for communication between methods
    // construct a tracker called our_tracker verbose = false
    MOTracker our_tracker(nh, getPclParams(), getKfParams(),getTrackerParams(),getIOParams(), false);
    ros::spin ();// spin ros
    return 0;
}



