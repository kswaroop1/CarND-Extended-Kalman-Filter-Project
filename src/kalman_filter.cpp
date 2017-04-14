#include "kalman_filter.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Matrix;

KalmanFilter::KalmanFilter() : I (Matrix4d::Identity()) {
  noise_ax_ = noise_ay_ = 9.0; // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.

  x_ = VectorXd(4);   // state vector
  x_ << 1.0, 1.0, 1.0, 1.0;

  // Set the process and measurement noises
  P_ << // state covariance matrix
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;
  F_ << // state transistion matrix
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
  Q_ << // process covariance matrix (/noise/stochatic/random/assumed normally distributed)
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0;

  H_laser_ << // measurement mapper - laser
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;
  Ht_laser_ = H_laser_.transpose();

  R_laser_ << //measurement covariance matrix - laser
    0.0225, 0.0,
    0.0,    0.0225;
  R_radar_ << //measurement covariance matrix - radar
    0.09, 0.0,    0.0,
    0.0,  0.0009, 0.0,
    0.0,  0.0,    0.09;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict(double dt) {
  //update the state
  //1. Modify the F matrix so that the time is integrated
  F_(0, 2) = F_(1, 3) = dt;

  //2. Set the process covariance matrix Q
  auto dt2 = dt*dt, dt3 = dt*dt*dt / 2, dt4 = dt*dt*dt*dt / 4;
  Q_(0, 0) = dt4*noise_ax_;
  Q_(1, 1) = dt4*noise_ay_;
  Q_(2, 0) = Q_(0, 2) = dt3*noise_ax_;
  Q_(3, 1) = Q_(1, 3) = dt3*noise_ay_;
  Q_(2, 2) = dt2*noise_ax_;
  Q_(3, 3) = dt2*noise_ay_;

  //new estimate
  x_      = F_ * x_;
  auto Ft = F_.transpose();
  P_      = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Vector2d &z) {
  // update the state by using Kalman Filter equations
  auto z_pred = H_laser_ * x_;
  auto y      = z - z_pred;
  auto S      = H_laser_ * P_ * Ht_laser_ + R_laser_;
  auto Si     = S.inverse();
  auto PHt    = P_ * Ht_laser_;
  auto K      = PHt * Si ;

  // new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const Vector3d &z) {
  // update the state by using Extended Kalman Filter equations
  auto Hj = CalculateJacobian();
  auto Htj = Hj.transpose();
  auto S = Hj * P_ * Htj + R_radar_;
  auto Si = S.inverse();
  auto PHt = P_ * Htj;
  auto K = PHt * Si;

  auto f1     = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  auto z_pred = Vector3d{ f1, atan2(x_[1], x_[0]), (x_[0] * x_[2] + x_[1] * x_[3]) / f1 }; // polar coords
  auto y      = Vector3d{ z - z_pred };     // need to declare type, else the expression result defaults to 
  if (y(1) > M_PI) y(1) -= 2 * M_PI;        // bring the angle phi 
  else if (y(1) < -M_PI) y(1) += 2 * M_PI;  // between -pi and pi

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
}

Matrix<double, 3, 4> KalmanFilter::CalculateJacobian() {
  Matrix<double, 3, 4> Hj;

  //recover state parameters
  auto px = x_(0);
  auto py = x_(1);
  auto vx = x_(2);
  auto vy = x_(3);

  //pre-compute a set of terms to avoid repeated calculation
  auto c1 = px*px + py*py;
  auto c2 = sqrt(c1);
  auto c3 = (c1*c2);

  //check division by zero
  if (fabs(c1) < EPSILON) {
    std::cerr << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj <<
    (px / c2),  (py / c2), 0.0, 0.0,
    -(py / c1), (px / c1), 0.0, 0.0,
    py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

  return Hj;
}
