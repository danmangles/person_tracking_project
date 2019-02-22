

#include "kalman_filter.h"
#include <vector>
#include "pairing.h" // includes Eigen
using namespace std;


#ifndef TRACKLET_H
#define TRACKLET_H

class Tracklet {
public:
    Tracklet(int ID,
             VectorXd initial_detection,
             KalmanFilter kf);

    VectorXd getState(); // either returns KF state OR last measurement if KF not initialised

    void updateTracklet(Pairing pairing); // update the tracklet with a new pairing

    int getNumConsecutiveMisses() { return num_consecutive_misses;};
    void recordMiss(){num_consecutive_misses++;}; // increment number of consecutive misses
    int getID(){return ID_;};

    double getDistance(VectorXd detection);// return the distance to this detection

private:
    KalmanFilter kf_; // private copy of kalman filter
    int ID_;
    int num_consecutive_misses = 0;
    vector <VectorXd> detection_vector_;

};

#endif // TRACKLET_H
