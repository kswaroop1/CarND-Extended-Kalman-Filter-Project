#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class FusionEKF {
public:
  FusionEKF();
  FusionEKF(bool verboseMode);
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * A helper method to calculate RMSE.
  */
  static Eigen::Vector4d CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  void initialize(const MeasurementPackage& measurement_pack);

  bool verboseMode_;
  bool is_initialized_;   // check whether the tracking toolbox was initiallized or not (first measurement)
  long long previous_timestamp_;
};

#endif /* FusionEKF_H_ */
