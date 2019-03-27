/*
 * kalman_filter.h
 *
 *  Created on: 28 Jan 2019
 *      Author: ori
 */
#include <Eigen/Dense>
#include <cmath>
#define KALMAN_FILTER_H_

using namespace Eigen;
class KalmanFilter {
public:
    // all public methods and variables go here
    /**
      * Create a Kalman filter with the specified matrices.
      *   A - System dynamics matrix
      *   C - Output matrix
      *   Q - Process noise covariance
      *   R - Measurement noise covariance
      *   P - Estimate error covariance
      */

    KalmanFilter(
            const MatrixXd& F,
            const MatrixXd& H,
            const MatrixXd& GQG,
            const MatrixXd& R_rgbd,
            const MatrixXd& R_velodyne,
            const MatrixXd& P0,
            bool verbose
    );
    void init();
    /*
     * Initial states are zero
     */

        /*
         * Initialise the filier witha  a guess for initial states
         */
        void init(double t0, const VectorXd& x0);

        /*
          * Update the estimated state based on measured values. The
          * time step is assumed to remain constant.
          */

        void update(const VectorXd& z, bool isRGBD, bool verbose);
        void predict(double time, bool verbose);
    /*
     * Return current state and time
     */
    VectorXd getState();
    MatrixXd getP(); // return covariance matrix
    VectorXd getV();
    double time() { return t_; };

private:
    // Matrices for computation
    MatrixXd F, H, GQG, R_rgbd, R_velodyne, P, S, W, I;
    MatrixXd R;
    // System dimensions
    int m, n;

    // current time and discrete time step
    double t_,tau_;

    // Is the filter initialized?
    bool initialized;

    // Estimated states, measurement, innovation
    VectorXd x_hat,  z_pred, v;

    bool verbose_;
};
