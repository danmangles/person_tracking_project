/*this file has a class definition:
*  - public
*      - constructor tracker(nh, kf_parmas);
*      - load_kf() {}
*      - update_kf() { updates kalman filter) and other get methods
*  - private has node handle
*     - THIS ENABLES DIFFERENT THINGS TO TALK TO EACH OTHER.
*     - void callback(msg);
*     - kf;
*/
#include <ros/ros.h> // because this is a robot
#include <ros/console.h>
#include <Eigen/Dense>

#include "kalman_filter.h"
#ifndef tracker_H
#define tracker_H



class tracker {

public:
    tracker(ros::NodeHandle nh,
            double dt,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::MatrixXd& P); //ros::NodeHandle &nh);

private:
    kalman_filter kf_; // our private copy of a kalman filter
    ros::NodeHandle nh_;
};
#endif // tracker_H
