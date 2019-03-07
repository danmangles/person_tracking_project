/*
 * kalman_filter.cpp
 *
 *  Created on: 28 Jan 2019
 *      Author: ori
 */

#include "include/kalman_filter.h"
#include <iostream>
#include <stdexcept>

using namespace Eigen;
using namespace std;


KalmanFilter::KalmanFilter(
        double dt,
        const MatrixXd& delF,
        const MatrixXd& delH,
        const MatrixXd& delGQdelGT,
        const MatrixXd& R,
        const MatrixXd& P0,
        bool verbose)

    : delF(delF), delH(delH), delGQdelGT(delGQdelGT), R(R), P(P0), //populate matrices values given in constructor
      m(delH.rows()), n(delF.rows()), initialized(false), //populate m with number of rows in C, n with number of rows in A
      I(n, n), x_hat(n), verbose_(verbose)
{
    cout << "KalmanFilter constructor called" << endl;
    I.setIdentity();
    //print out the chosen matrices
    cout << "delF: \n" << delF << endl;
    cout << "delH: \n" << delH << endl;
    cout << "delGQdelGT: \n" << delGQdelGT << endl;
    cout << "R: \n" << R << endl;
    cout << "P: \n" << P << endl;

}

//constructor with lots of params
void KalmanFilter::init(double t0, const VectorXd& x0) {
    //Initialise all values
    if (verbose_)
        cout << "initialising Kalman Filter" <<endl;
    x_hat = x0;
    //    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
    if (verbose_) {
        cout << "P = \n"<<P <<endl;
        cout << "delF = \n"<<delF <<endl;
    }
}

// default constructor
void KalmanFilter::init() {
    x_hat.setZero();
    //    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
}

void KalmanFilter::update(const VectorXd& z) {
    /*
    //check if we are initialised
    if (!initialized)
        throw std::runtime_error("Filter is not initialised... :3");

    if (verbose_)
    {
        cout << "A = "<<A <<endl;
        cout << "*************\ny = "<<y<<endl;
    }
    x_hat_new = A*x_hat; // PREDICT
    P = A*P*A.transpose() + Q; //PREDICT

    if (verbose_) {
        cout << "*PREDICT*\nx_hat_new = "<<x_hat_new<<endl;
        cout << "P = \n"<<P<<endl;} //
*/
    //check if we are initialised
    if (!initialized)
        throw runtime_error("Filter is not initialised... :3");
    /////// Prediction
    x_hat = delF*x_hat; // predicted  state = plant_model(old_state) but using a linear plant model delF
    P = delF*P*delF.transpose() + delGQdelGT; // predicted covariance = transformed old covariance + process noise
    z_pred = delH*x_hat; // predicted observation

    if (verbose_) {
        cout << "*PREDICT*\nx_hat_pred = \n"<<x_hat<<endl;
        cout << "P_pred = \n"<<P<<endl;
        cout << "z_pred = \n"<<z_pred<<endl;
    }

    /*

    K = P*C.transpose()*(C*P*C.transpose() + R).inverse(); //UPDATE


    x_hat_new += K * (y - C*x_hat_new); // predict


    P = (I - K*C)*P; // UPDATE
    if (verbose_) {
        cout << "*UPDATE*\nK = \n"<<K<<endl;
        cout << "UPDATED x_hat_new = "<<x_hat_new<<endl;
        cout << "UPDATED P = "<<P<<endl;}

    x_hat = x_hat_new; // UPDATE
    if (verbose_)
        cout << "x_hat is now\n" << x_hat<<endl;
        */
    /////// Update
    v = z - z_pred; // innovation = difference between measurement and predicted measurement
    S = delH*P*delH.transpose() + R; // innovation covariance = kf covariance in observation space + sensor noise R
    W = P*delH.transpose()*S.inverse(); // Kalman gain = kf covariance/innovation covariance

    x_hat = x_hat + W*v; // estimate = prediction + Gain*Innovation
    P = P - W*S*W.transpose(); // covariance is decreased by the update

    if (verbose_)
        cout << "*UPDATE*\nv = \n"<<v<< "\nS = \n"<<S<<"\nW = \n"<<W<<"\nx_hat_new = \n"<<x_hat<< "\nP_new = \n"<<P<<endl;

    t += dt;

}
MatrixXd KalmanFilter::getP()
{

    if (!initialized)
        throw std::runtime_error("Filter is not initialised... :3");
    if (verbose_)
        cout << "KalmanFilter::getP() P =" <<P<<endl;
    return P;
}
VectorXd KalmanFilter::getState()
{
    if (!initialized)
        throw std::runtime_error("Filter is not initialised... :3");
    return x_hat;
}



