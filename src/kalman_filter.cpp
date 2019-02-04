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

kalman_filter::kalman_filter(
	double dt,
	const Eigen::MatrixXd& A,
	const Eigen::MatrixXd& C,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R,
	const Eigen::MatrixXd& P)

  : A(A), C(C), Q(Q), R(R), P0(P), //populate A,C,Q,R,P0 with values given in constructor
    m(C.rows()), nd(A.rows()), dt(dt), initialized(false), //populate m with number of rows in C, n with number of rows in A
    I(nd, nd), x_hat(nd), x_hat_new(nd)
	{
		I.setIdentity();
	}

kalman_filter::kalman_filter() {}

//constructor with lots of params
void kalman_filter::init(double t0, const VectorXd& x0) {
	//Initialise all values
	x_hat = x0;
	P = P0;
	this->t0 = t0;
	t = t0;
	initialized = true;
}

// default constructor
void kalman_filter::init() {
	x_hat.setZero();
	P = P0;
	t0 = 0;
	t = t0;
	initialized = true;
}

void kalman_filter::update(const VectorXd& y) {
	//check if we are initialised
	if (!initialized)
		throw std::runtime_error("Filter is not initialised... :3");



        cout << "*************\ny = "<<y<<endl;
    x_hat_new = A*x_hat; // PREDICT

        cout << "*PREDICT*\nx_hat_new = "<<x_hat_new<<endl;

    P = A*P*A.transpose() + Q; //PREDICT

        cout << "P = "<<P<<endl;




    K = P*C.transpose()*(C*P*C.transpose() + R).inverse(); //UPDATE

        cout << "*UPDATE*\nK = "<<K<<endl;

	x_hat_new += K * (y - C*x_hat_new); // predict

        cout << "UPDATED x_hat_new = "<<x_hat_new<<endl;

    P = (I - K*C)*P; // UPDATE

        cout << "UPDATED P = "<<P<<endl;

    x_hat = x_hat_new; // UPDATE

	t += dt;
}

void kalman_filter::update(const VectorXd& y, double dt, const MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}


