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
        const MatrixXd& F,
        const MatrixXd& H,
        const MatrixXd& GQG,
        const MatrixXd& R,
        const MatrixXd& P0,
        bool verbose)

    : F(F), H(H), GQG(GQG), R(R), P(P0), //populate matrices values given in constructor
      m(H.rows()), n(F.rows()), initialized(false), //populate m with number of rows in C, n with number of rows in A
      I(3, 3), x_hat(n), verbose_(verbose), v(n)
{
    cout << "KalmanFilter constructor called" << endl;
    I.setIdentity();

    if (verbose_) {
        //print out the chosen matrices
        cout << "F: \n" << F << endl;
        cout << "H: \n" << H << endl;
        cout << "GQG: \n" << GQG << endl;
        cout << "R: \n" << R << endl;
        cout << "P: \n" << P << endl;
    }

}

//constructor with lots of params
void KalmanFilter::init(double t0, const VectorXd& x0) {
    //Initialise all values
    if (verbose_)
        cout << "initialising Kalman Filter" <<endl;
    x_hat = x0;
    v.setZero(); //set v as zero so there is something to reference the first time
    t_ = t0;
    initialized = true;
    if (verbose_) {
        cout << "P = \n"<<P <<endl;
        cout << "F = \n"<<F <<endl;
    }
}

void KalmanFilter::predict(double time, bool verbose){
    //check if we are initialised
    if (!initialized)
        throw runtime_error("Filter is not initialised... :3");

    /////// Prediction
    tau_ = time - t_; // compute tau_ as new time - previous time
    t_ = time; // update previous time

    if (tau_ < 0)
        cout<<"\n********************************\n!!!!!! dt is negative!!!"<<endl;

//    MatrixXd I3(3,3);
//    I3.setIdentity();

    // change the matrices which are a function of time
    F.block(0,3,3,3) = I*tau_; // set the top right to make fcn of dt
    /////////////////
    /// \brief var_pos
    double timestep = 0.1; //s e.g. we are splitting time into timesteps
    double var_pos = pow((0.5*tau_/timestep),2); // e.g. we have progressed dt/timestep timesteps since the past predict; and in each our position uncertainty has grown 0.5m
    double var_vel = pow((0.5*tau_/timestep),2); // e.g. we have progressed dt/timestep timesteps since the past predict; and in each our velocity uncertainty has grown 3ms-1

    GQG.block(0,0,3,3) = I*var_pos; //update GQG in 2 blocks.
    GQG.block(3,3,3,3) = I*var_vel;

    if (verbose)
    {
        cout <<"time is "<<t_<<endl;
        cout <<"dt is "<<tau_<<endl;
        cout <<"F is \n"<<F<<endl;
        cout <<"GQG is \n"<<GQG<<endl;
    }

    x_hat = F*x_hat; // predicted  state = plant_model(old_state) but using a linear plant model F
    x_hat[2] = 2;
    P = F*P*F.transpose() + GQG; // predicted covariance = transformed old covariance + process noise

    z_pred = H*x_hat; // predicted observation

    if (verbose)
    {
        cout << "*PREDICT*\nx_hat_pred = \n"<<x_hat<<endl;
        cout << "P_pred = \n"<<P<<endl;
        cout << "z_pred = \n"<<z_pred<<endl;
    }
}

void KalmanFilter::update(const VectorXd& z, bool verbose) {
    //check if we are initialised
    if (!initialized)
        throw runtime_error("Filter is not initialised... :3");

    /////// Update
    v = z - z_pred; // innovation = difference between measurement and predicted measurement
    S = H*P*H.transpose() + R; // innovation covariance = kf covariance in observation space + sensor noise R
    W = P*H.transpose()*S.inverse(); // Kalman gain = kf covariance/innovation covariance

    x_hat = x_hat + W*v; // estimate = prediction + Gain*Innovation
    P = P - W*S*W.transpose(); // covariance is decreased by the update

    if (verbose)
        cout << "*UPDATE*\nv = \n"<<v<< "\nS = \n"<<S<<"\nW = \n"<<W<<"\nx_hat_new = \n"<<x_hat<< "\nP_new = \n"<<P<<endl;

    // hack: keep the z value at 1.5
    x_hat[2] = 2;

}

MatrixXd KalmanFilter::getP()
{
    cout <<"returning P"<<endl;
    if (!initialized)
    {
        throw std::runtime_error("Can't return P when filter not initialised ... :3");
    }
    return P;
}

VectorXd KalmanFilter::getV()
{
    cout <<"returning v"<<endl;
    if (!initialized)
    {
        throw std::runtime_error("Can't return v when filter not initialised ... :3");
    }
    return v;
}

VectorXd KalmanFilter::getState()
{
    cout <<"returning xhat"<<endl;
    if (!initialized)
    {
        throw std::runtime_error("Can't return xhat when filter not initialised ... :3");
    }
    return x_hat;
}



