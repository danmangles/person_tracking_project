#include "include/tracklet.h"
#include "include/pairing.h"
using namespace std;

Tracklet::Tracklet(int ID,
                   VectorXd initial_detection,
                   KalmanFilter kf) :
    ID_(ID), kf_(kf)
{
    cout << "Tracklet constructor called" <<endl;
    detection_vector_.push_back(initial_detection); // add the first detection to detection_vector_
}

void Tracklet::update(VectorXd detection, double current_time, bool isRGBD, bool verbose) {
    // Update the tracklet with a new detection of type isRGBD taken at time current_time
    if (verbose) cout<<"Tracklet::update()"<<endl;

    detection_vector_.push_back(detection); // update this tracklet with the detection coord
    if (verbose) cout << "pushed back new coord"<<endl;
    tracklet_length_++; // increase tracklet length
    num_consecutive_misses = 0; // reset num of consecutive misses
    if (isRGBD)
        has_RGBD_detection_ = true; // register an RGBD detection
    if (isInitialised_)
    {
        if (verbose) cout << "predicting and updating kf for Tracklet_"<<ID_<<endl;
        kf_.predict(current_time, false);
        kf_.update(detection, isRGBD, false);
    }
}
double Tracklet::getDistance(VectorXd detection)
{
    // add check if KF initialised functionality
    VectorXd state = detection_vector_.back(); // use last detection in vector
    return sqrt((detection - state).squaredNorm()); // return euclidean norm
}

void Tracklet::initKf(double current_time){
    kf_.init(current_time, detection_vector_.back());
    isInitialised_ = true;
}

void Tracklet::recordMiss(double current_time)
{
    cout<<"recordMiss()"<<endl;
    num_consecutive_misses++;
    if (isInitialised_)
    {
//        cout << "predicting kf for Tracklet_"<<ID_<<endl;
        kf_.predict(current_time, false); // predict even if we don't have a measurement
    } else{
        cout << "filter not initialised anyway" <<endl;
    }
}
