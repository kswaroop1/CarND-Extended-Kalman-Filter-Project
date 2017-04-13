#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Matrix;

class KalmanFilter {
public:
  static constexpr double MIN_VAL = 0.01; /// minimum value to overcome divide by zero

  KalmanFilter();
  virtual ~KalmanFilter();

  const Vector4d& x() const                 { return x_; }
  double x(int idx) const                   { return x_(idx); }
  void SetInitState(Vector4d init_state)    { x_ = init_state; }

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt The amount of time elapsed from previous measurement in seconds
   */
  void Predict(double dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Vector2d &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Vector3d &z);

private:
  Matrix4d I;   // identity matrix

  Vector4d x_;  // state vector
  Matrix4d P_;  // state covariance matrix
  Matrix4d F_;  // state transistion matrix
  Matrix4d Q_;  // process covariance matrix (/noise/stochatic/random/assumed normally distributed)

  // measurement matrix - laser
  Matrix<double, 2, 4> H_laser_;
  Matrix<double, 4, 2> Ht_laser_;

  // measurement covariance matrix
  Matrix2d R_laser_;
  Matrix3d R_radar_;

  // noise
  double noise_ax_;
  double noise_ay_;

  Matrix<double, 3, 4> CalculateJacobian(); // A helper method to calculate Jacobians.
};

#endif /* KALMAN_FILTER_H_ */
