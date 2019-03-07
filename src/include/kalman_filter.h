/*
 * kalman_filter.h
 *
 *  Created on: 28 Jan 2019
 *      Author: ori
 */
#include <Eigen/Dense>
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
            double dt,
            const MatrixXd& delF,
            const MatrixXd& delH,
            const MatrixXd& delGQdelGT,
            const MatrixXd& R,
            const MatrixXd& P0,
            bool verbose
    );
    /**
      * Create a blank estimator.
      */
    void init();
    /*
     * Initial states are zero
     */

        /*
         * Initialise the filier witha  aguess for initial states
         */
        void init(double t0, const VectorXd& x0);

        /*
          * Update the estimated state based on measured values. The
          * time step is assumed to remain constant.
          */
        void update(const VectorXd& z);

    /*
     * Return current state and time
     */
    VectorXd getState();
    MatrixXd getP(); // return covariance matrix
    double time() { return t; };

private:
    // Matrices for computation
    MatrixXd delF, delH, delGQdelGT, R, P, S, W, I;

    // System dimensions
    int m, n;

    // Initial and current time
    double t, t0;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // Estimated states, measurement, innovation
    VectorXd x_hat,  z_pred, v;

      bool verbose_;
};
