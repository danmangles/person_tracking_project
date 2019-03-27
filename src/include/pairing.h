#include <Eigen/Dense>
#include <iostream> // for couts
using namespace Eigen;
using namespace std;

#ifndef PAIRING_H
#define PAIRING_H

class Pairing {
public:
    Pairing(int associated_tracklet_ID, VectorXd detection_coord, double distance_to_tracklet, bool isRGBD); // initiate

    // get methods for private variables
    VectorXd getDetectionCoord() {return detection_coord_;};
    int getAssociatedTrackletID() {return associated_tracklet_ID_;};
    double getDistanceToTracklet() {return distance_to_tracklet_;};

    bool isRGBD_;
private:
    int associated_tracklet_ID_;
    VectorXd detection_coord_;
    double distance_to_tracklet_;

};

#endif // PAIRING_H
