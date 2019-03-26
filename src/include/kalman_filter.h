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
            const MatrixXd& delF,
            const MatrixXd& delH,
            const MatrixXd& delGQdelGT,
            const MatrixXd& R,
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

        void update(const VectorXd& z, bool verbose);
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
    MatrixXd delF, delH, delGQdelGT, R, P, S, W, I;

    // System dimensions
    int m, n;

    // current time and discrete time step
    double t_,dt_;

    // Is the filter initialized?
    bool initialized;

    // Estimated states, measurement, innovation
    VectorXd x_hat,  z_pred, v;

    bool verbose_;
};
