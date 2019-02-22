#include "include/tracklet.h"

Tracklet::Tracklet(int ID,
                   VectorXd initial_detection,
                   KalmanFilter kf) : ID_(ID), kf_(kf)
{
    detection_vector_.push_back(initial_detection); // add the first detection to detection_vector_
}

Tracklet::updateTracklet(Pairing pairing) {
    // Update the tracklet with a new pairing
    detection_vector_.push_back(pairing.detection_coord_);

    // DELETE THE PAIRING (how do I do this?)
}
Tracklet::getDistance(VectorXd detection)
{
    // add check if KF initialised functionality
    VectorXd state = detection_vector_.back(); // use last detection in vector
    return sqrt((detection - state).squaredNorm()); // return euclidean norm
}
