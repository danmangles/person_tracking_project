#include "include/pairing.h"

// empty constructor just assigns things
Pairing::Pairing(int associated_tracklet_ID, VectorXd detection_coord, double distance_to_tracklet, bool isRGBD) :
    associated_tracklet_ID_(associated_tracklet_ID), detection_coord_(detection_coord), distance_to_tracklet_(distance_to_tracklet), isRGBD_(isRGBD){};
