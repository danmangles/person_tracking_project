

#include "kalman_filter.h"
#include <vector>
//#include "pairing.h" // includes Eigen
using namespace std;


#ifndef TRACKLET_H
#define TRACKLET_H

class Tracklet {
public:
    Tracklet(int ID,
             VectorXd initial_detection,
             KalmanFilter kf);

    VectorXd getState(); // either returns KF state OR last measurement if KF not initialised

    void update(VectorXd detection, double current_time, bool isRGBD, bool verbose); // update the tracklet with a new detection registered at time current_time

    int getNumConsecutiveMisses() { return num_consecutive_misses;};
    void recordMiss(double time); // increment number of consecutive misses
    int getID(){return ID_;};

    double getDistance(VectorXd detection);// return the distance to this detection
    int getLength(){return tracklet_length_;}; // return the number of detections registered with this tracklet
    bool isInitialised() {return isInitialised_;}; // have we started the kalman_filter?
    void initKf(double current_time); // initialise the kalman filter at the last observation
    KalmanFilter getKf() {return kf_;}; // get the private kalman filter
    bool has_RGBD_detection(){return has_RGBD_detection_;} // get method

private:
    KalmanFilter kf_; // private copy of kalman filter
    int ID_;
    int num_consecutive_misses = 0;
    int tracklet_length_ = 1; // number of detections registered with this tracklet. We start with 1 detection
    vector <VectorXd> detection_vector_;
    bool isInitialised_ = false; // have we started the kalman_filter?
    bool has_RGBD_detection_ = 0;
};

#endif // TRACKLET_H
