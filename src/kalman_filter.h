#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:
  static constexpr double EPSILON = 0.0001; /// minimum value to overcome divide by zero

  KalmanFilter();
  virtual ~KalmanFilter();

  const Eigen::Vector4d& x() const                      { return x_; }
  double x(int idx) const                               { return x_(idx); }
  void SetInitState(const Eigen::Vector4d& init_state)  { x_ = init_state; }

  const Eigen::Matrix4d& P() const                      { return P_; }

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
  void Update(const Eigen::Vector2d &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::Vector3d &z);

private:
  Eigen::Matrix4d I;   // identity matrix

  Eigen::Vector4d x_;  // state vector
  Eigen::Matrix4d P_;  // state covariance matrix
  Eigen::Matrix4d F_;  // state transistion matrix
  Eigen::Matrix4d Q_;  // process covariance matrix (/noise/stochatic/random/assumed normally distributed)

  // measurement matrix - laser
  Eigen::Matrix<double, 2, 4> H_laser_;
  Eigen::Matrix<double, 4, 2> Ht_laser_;

  // measurement covariance matrix
  Eigen::Matrix2d R_laser_;
  Eigen::Matrix3d R_radar_;

  // noise
  double noise_ax_;
  double noise_ay_;

  Eigen::Matrix<double, 3, 4> CalculateJacobian(); // A helper method to calculate Jacobians.
};

#endif /* KALMAN_FILTER_H_ */
