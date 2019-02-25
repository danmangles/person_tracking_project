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
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P,
        bool verbose)

    : A(A), C(C), Q(Q), R(R), P0(P), //populate A,C,Q,R,P0 with values given in constructor
      m(C.rows()), n(A.rows()), dt(dt), initialized(false), //populate m with number of rows in C, n with number of rows in A
      I(n, n), x_hat(n), x_hat_new(n), verbose_(verbose)
{
    cout << "KalmanFilter constructor called" << endl;
    I.setIdentity();
    //print out the chosen matrices
    cout << "A: \n" << A << endl;
    cout << "C: \n" << C << endl;
    cout << "Q: \n" << Q << endl;
    cout << "R: \n" << R << endl;
    cout << "P: \n" << P << endl;

}

KalmanFilter::KalmanFilter() {}

//constructor with lots of params
void KalmanFilter::init(double t0, const VectorXd& x0) {
    //Initialise all values
    if (verbose_)
        cout << "initialising Kalman Filter" <<endl;
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
    if (verbose_) {
        cout << "P = "<<P <<endl;
        cout << "A = "<<A <<endl;
    }
}

// default constructor
void KalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
}

void KalmanFilter::update(const VectorXd& y) {
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
void KalmanFilter::update(const VectorXd& y, double dt, const MatrixXd A) {

    this->A = A;
    this->dt = dt;
    update(y);
}


