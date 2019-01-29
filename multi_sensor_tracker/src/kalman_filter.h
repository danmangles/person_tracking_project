/*
 * kalman_filter.h
 *
 *  Created on: 28 Jan 2019
 *      Author: ori
 */
#include <Eigen/Dense>
#define KALMAN_FILTER_H_
using namespace Eigen;

class kalman_filter {
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

	kalman_filter(
			double dt,
			const MatrixXd& A,
			const MatrixXd& C,
			const MatrixXd& Q,
			const MatrixXd& R,
			const MatrixXd& P
	);
	/**
	  * Create a blank estimator.
	  */
	kalman_filter(); // what does this line do????
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
        void update(const VectorXd& y);

        /*
         * Update estimate state using measured values, using give time step and dynamics matrix
         */
        void update(const VectorXd& y, double dt, const MatrixXd A);

	/*
	 * Return current state and time
	 */
    VectorXd state() { return x_hat; };
	double time() { return t; };

private:
	 // Matrices for computation
	  MatrixXd A, C, Q, R, P, K, P0;

	  // System dimensions
      int m, nd;

	  // Initial and current time
	  double t0, t;

	  // Discrete time step
	  double dt;

	  // Is the filter initialized?
	  bool initialized;

	  // n-size identity
      MatrixXd I;

          // Estimated states
      VectorXd x_hat, x_hat_new;
};
