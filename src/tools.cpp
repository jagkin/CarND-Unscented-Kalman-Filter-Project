#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse;
  unsigned int num_entries = estimations.size();
  if (num_entries == 0) {
    cout << "Tools::CalculateRMSE: Invalid Estimation vector\n";
    return rmse;
  }

  if (num_entries != ground_truth.size()) {
    cout << "Tools::CalculateRMSE: Estimation and ground_truth vectors are of different sizes\n";
    return rmse;
  }

  // Initialize sum of squared diffs
  unsigned int dims = estimations[0].size();
  VectorXd sum_diffs(dims);
  sum_diffs = VectorXd::Zero(dims);
  for (unsigned int i = 0u; i != num_entries; i++) {
    // Calculate diff
    VectorXd diff = estimations[i] - ground_truth[i];
    // Add to sum of squared diffs
    VectorXd diff_sqrd = diff.array() * diff.array();
    sum_diffs += diff_sqrd;
  }
  // Calculate mean
  VectorXd mean = sum_diffs/num_entries;

  // Calculate and return RMSE
  rmse = mean.array().sqrt();

#if VERBOSE_PRINTS
  // Dump latest estimations/ground truth and RMSE to console
  cout << "Estimation:   ";
  for (unsigned int i = 0u; i != dims; i++) {
     cout << estimations[num_entries-1](i) << "\t";
  }
  cout << endl;

  cout << "Ground truth: ";
  for (unsigned int i = 0u; i != dims; i++) {
     cout << ground_truth[num_entries-1](i) << "\t";
  }
  cout << endl;

  cout << "RMSE:         ";
  for (unsigned int i = 0u; i != dims; i++) {
     cout << rmse(i) << "\t";
  }
  cout << endl << endl;
#endif

#if DUMP_GROUND_TRUTH_STATS
  static double prev_vx = 0, prev_vy = 0, max_ax = 0, max_ay = 0;
  // ground truth is px py vx vy yaw yawrate
  double vx = ground_truth[num_entries-1](2);
  double vy = ground_truth[num_entries-1](3);
  double ax = 0, ay = 0;
  if (num_entries > 1) {
	  ax = (vx - prev_vx)/0.05;
	  ay = (vy - prev_vy)/0.05;
	  if (ax > max_ax) max_ax = ax;
	  if (ay > max_ay) max_ay = ay;
  }
  cout << "vx:" << vx << " vy:" << vy << " ax:" << ax << " ay:" << ay << " max_ax:" << max_ax << " max_ay:" << max_ay << endl;
  prev_vx = vx;
  prev_vy = vy;
#endif

  return rmse;
}
