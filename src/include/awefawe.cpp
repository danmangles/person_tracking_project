

#include "kalman_filter.h"
#include <vector>
#include <Eigen/Dense>
#include "pairing.h"
using namespace std;


#ifndef TRACKLET_H
#define TRACKLET_H

class Tracklet {
public:
    Tracklet(int ID,
             int num_consecutive_misses,
             vector <VectorXd> detections,
             KalmanFilter kf);

    VectorXd getState(); // either returns KF state OR last measurement if KF not initialised

    void updateTracklet(Pairing pairing); // update the tracklet with a new pairing

    int getNumConsecutiveMisses() { return num_consecutive_misses;}

private:
    KalmanFilter kf_; // private copy of kalman filter
    int ID_;
    int num_consecutive_misses;

}

#endif // TRACKLET_H












#include <Eigen/Dense>
using namespace std;

#ifndef PAIRING_H
#define PAIRING_H

class Pairing {
public:
    Pairing(int associated_tracklet_ID, VectorXd detection_coord); // initiate

private:
    int associated_tracklet_ID_;
    VectorXd detection_coord_;

}

#endif // PAIRING_H
