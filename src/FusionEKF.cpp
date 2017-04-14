#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>
#include <iomanip>

using namespace std;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using std::vector;

FusionEKF::FusionEKF() : FusionEKF(false) {}
FusionEKF::FusionEKF(bool verboseMode) : verboseMode_(verboseMode), is_initialized_(false), previous_timestamp_(0) {}

FusionEKF::~FusionEKF() {}

void FusionEKF::initialize(const MeasurementPackage &measurement_pack) {
  // Initialize the state ekf_.x_ with the first measurement.
  if (verboseMode_) std::cout << "EKF: " << setprecision(3) << endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  Vector4d init_state;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Convert radar from polar to cartesian coordinates and initialize state.
    auto rho = measurement_pack.raw_measurements_[0];
    auto phi = measurement_pack.raw_measurements_[1];
    auto rate = measurement_pack.raw_measurements_[2];
    init_state << rho*cos(phi), rho*sin(phi), rate*cos(phi), rate*sin(phi);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    init_state << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
  }
  if (init_state[0] == 0.0) init_state[0] = 0.01; // init (to 0.01), to
  if (init_state[1] == 0.0) init_state[1] = 0.01; // overcome divide by zero

  ekf_.SetInitState(init_state);

  is_initialized_ = true;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    initialize(measurement_pack);
    return;   // done initializing, no need to predict or update
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //0. compute the time elapsed between the current and previous measurements
  auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update, as per sensor type
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    auto z = Vector3d{ measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2] };
    ekf_.UpdateEKF(z);
  } else {
    // Laser updates
    auto z = Vector2d{ measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1] };
    ekf_.Update(z);
  }

  // print the output
  if (verboseMode_) {
    std::cout << "x_ = \t" << ekf_.x(0) << "\t" << ekf_.x(1) << "\t" << ekf_.x(2) << "\t" << ekf_.x(3) << endl;
    //std::cout << "P_ = " << ekf_.P() << endl;
  }
}

/*static*/ Vector4d FusionEKF::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  Vector4d rmse{ 0.0, 0.0, 0.0, 0.0 };

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cerr << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array(); //coefficient-wise multiplication
    rmse += residual;
  }

  rmse = rmse / (double)estimations.size();       //calculate the mean
  rmse = rmse.array().sqrt();                     //calculate the squared root

  return rmse;
}
