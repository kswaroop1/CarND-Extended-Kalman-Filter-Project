#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[], bool& verboseMode) {
  string usage_instructions{ "Usage instructions: " };
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  auto has_valid_args = verboseMode = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc == 4) {
    has_valid_args = verboseMode = true;
  }
  else if (argc > 4) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

void read_data(std::ifstream &in_file, std::vector<MeasurementPackage> &measurement_pack_list, std::vector<GroundTruthPackage> &gt_pack_list)
{
  string line;
  
  while (getline(in_file, line)) { // prep the measurement packages (each line represents a measurement at a	timestamp)
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {  // LASER MEASUREMENT
      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      double x, y;
      iss >> x >> y >> timestamp;
      meas_package.raw_measurements_ << x, y;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0) { // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      double ro, phi, ro_dot;
      iss >> ro >> phi >> ro_dot >> timestamp;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    double x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }
}

int main(int argc, char* argv[]) {
  auto verboseMode = false;
  check_arguments(argc, argv, verboseMode);

  string in_file_name_{ argv[1] };
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_{ argv[2] };
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  // read the data file
  read_data(in_file_, measurement_pack_list, gt_pack_list);

  // Create a Fusion EKF instance
  FusionEKF fusionEKF{ verboseMode };

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    auto x = fusionEKF.ekf_.x();
    out_file_ << x(0) << "\t" << x(1) << "\t" << x(2) << "\t" << x(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      auto ro = measurement_pack_list[k].raw_measurements_(0);
      auto phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    auto gt = gt_pack_list[k].gt_values_;
    out_file_ << gt(0) << "\t" << gt(1) << "\t" << gt(2) << "\t" << gt(3) << "\n";
    if (verboseMode) {
      cout << "gt\t" << gt(0) << "\t" << gt(1) << "\t" << gt(2) << "\t" << gt(3) << endl;
      cout << "d" << k << "\t" << x(0)-gt(0) << "\t" << x(1)-gt(1) << "\t" << x(2)-gt(2) << "\t" << x(3)-gt(3) << endl << endl;
    }

    estimations.push_back(fusionEKF.ekf_.x());
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  VectorXd rmse(fusionEKF.CalculateRMSE(estimations, ground_truth));

  //cout << "Accuracy - RMSE:" << endl << rmse << endl;
  cout << "RMSE" << endl;
  for (auto i=0; i<rmse.size(); i++)
	  cout << rmse[i] << endl;

  // close files
  if (out_file_.is_open()) out_file_.close();
  if (in_file_.is_open()) in_file_.close();

  return 0;
}
