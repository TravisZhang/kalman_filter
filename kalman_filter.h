#ifndef MOTION_CONTROL_KALMAN_FILTER_H
#define MOTION_CONTROL_KALMAN_FILTER_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cstdlib>

#include <memory> // for smart pointer

#include <Eigen/Dense> // for eigen3 support

namespace xmotors
{
namespace kalman_filter
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter
{
private:
    MatrixXd F_; // state transition matrix
    MatrixXd B_; // control input matrix
    MatrixXd P_; // state covariance matrix
    MatrixXd Q_; // process covariance matrix
    MatrixXd R_; // observation covariance
    MatrixXd H_; // observation/measurement matrix
    VectorXd x_; // state vector;
    VectorXd z_; // observation vector
    VectorXd K_; // kalman gain
    double dt_ = 0.1; // sampling time
public:
    KalmanFilter(/* args */);
    ~KalmanFilter();

    virtual void Init(const VectorXd &x, const MatrixXd &P, const double dt); // initialize state&state cov

    virtual void Predict(const VectorXd &u);

    virtual void Update(const VectorXd &z);

    virtual void MotionModel(const VectorXd &u);

    virtual void ObervationModel();

    virtual MatrixXd CalJacobianF(const VectorXd &u);

    virtual MatrixXd CalJacobianH();


};

KalmanFilter::KalmanFilter(/* args */)
{
}

KalmanFilter::~KalmanFilter()
{
}





} // namespace kalman_filter

} // namespace xmotors



#endif