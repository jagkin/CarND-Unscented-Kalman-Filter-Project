#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cstdlib>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  const char* env_var(NULL);

  use_laser_ = true;
  env_var = getenv("DISABLE_LASER");
  if (env_var != NULL) use_laser_ = atoi(env_var) ? false : true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  env_var = getenv("DISABLE_RADAR");
  if (env_var != NULL) use_radar_ = atoi(env_var) ? false : true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // Choose initial value of 0.231 based on google search for "average bike acceleration"
  std_a_ = 1.75;
  env_var = getenv("STD_A");
  if (env_var != NULL) std_a_ = atof(env_var);

  // Process noise standard deviation yaw acceleration in rad/s^2
  // Choose initial value of M_PI/80 assuming bike to angular speed to change 10% of typical angular speed (M_PI/8) described in lecture.
  // With M_PI/80 : observed RADAR NIS of about 30 with dataset 1 and about 25 with dataset 2.
  // With M_PI/16 : observed RADAR NIS of about 5 with dataset 1 and about 11 with dataset 2.
  // With M_PI/8 and M_PI/4 : observed RADAR NIS of about 4 with dataset 1 and about 9 with dataset 2.
  std_yawdd_ = M_PI/14;
  env_var = getenv("STD_YAWDD");
  if (env_var != NULL) std_yawdd_ = atof(env_var);
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  // Time
  time_us_ = 0;

  // Flag indicating if Filter is initialized
  is_initialized_ = false;

  // Sigma points spreading parameter
  lambda_ = 3 - n_aug_ ;

  // Augmented state dimension
  n_aug_ = 7;

  // state dimension
  n_x_ = 5;

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_) {
    x_ = VectorXd(n_x_);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "Using RADAR\n";
      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      float px = ro * sin(theta);
      float py = ro * cos(theta);
      x_ << px, py, 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "Using LASER\n";
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      x_ << px, py, 0, 0, 0;
    }

    // Initialize co-variance matrix to Identity as suggested in "What to Expect from the Project" part of the lecture.
    P_ = MatrixXd::Identity(n_x_, n_x_);

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double_t dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  cout << "Predicting after " << dt << " sec\n";
  Prediction(dt);
  time_us_ = meas_package.timestamp_;

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	cout << "Updating using RADAR meas\n";
    UpdateRadar(meas_package);
  } else {
    cout << "Updating using LASER meas\n";
	UpdateLidar(meas_package);
  }

  return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_ + 0) = 0; // nu_a
  x_aug(n_x_ + 1) = 0; // nu_yawdotdot

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_ + 0,n_x_ + 0) = std_a_*std_a_;
  P_aug(n_x_ + 1,n_x_ + 1) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose() ;
  }

  return;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  //set measurement dimension, Lidar can measure px and py
  int n_z = 2;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
	   meas_package.raw_measurements_[1];

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
	double weight = 0.5/(n_aug_+lambda_);
	weights(i) = weight;
  }

  //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
	 // extract values for better readibility
	 double p_x = Xsig_pred_(0,i);
	 double p_y = Xsig_pred_(1,i);

	 // measurement model
	 Zsig(0,i) = p_x; //px
	 Zsig(1,i) = p_y; //py
   }

   //mean predicted measurement
   VectorXd z_pred = VectorXd(n_z);
   z_pred.fill(0.0);
   for (int i=0; i < 2*n_aug_+1; i++) {
	   z_pred = z_pred + weights(i) * Zsig.col(i);
   }

   //innovation covariance matrix S
   MatrixXd S = MatrixXd(n_z,n_z);
   S.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
	 //residual
	 VectorXd z_diff = Zsig.col(i) - z_pred;
	 S = S + weights(i) * z_diff * z_diff.transpose();
   }

   //add measurement noise covariance matrix
   MatrixXd R = MatrixXd(n_z,n_z);
   R <<    std_laspx_*std_laspx_, 0,
		   0,std_laspy_*std_laspy_;
   S = S + R;

   //create matrix for cross correlation Tc
   MatrixXd Tc = MatrixXd(n_x_, n_z);
   //calculate cross correlation matrix
   Tc.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
	 //residual
	 VectorXd z_diff = Zsig.col(i) - z_pred;

	 // state difference
	 VectorXd x_diff = Xsig_pred_.col(i) - x_;

	 Tc = Tc + weights(i) * x_diff * z_diff.transpose();
   }

   //Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   //residual
   VectorXd z_diff = z - z_pred;

   //update state mean and covariance matrix
   x_ = x_ + K * z_diff;
   P_ = P_ - K*S*K.transpose();

   // NIS
   double nis = z_diff.transpose() * S.inverse() * z_diff;
   static const double line_95percent = 5.991;
   static uint32_t n_total = 0, n_aboveLine = 0;
   n_total++;
   if (nis > line_95percent) n_aboveLine++;
   cout << "LIDAR NIS: Current: " << nis << ". \n %age of values above 95%line: ("<<n_aboveLine<<"/"<<n_total << ")*100 = " << float(n_aboveLine*100)/n_total << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //transform sigma points into measurement space
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

     // extract values for better readibility
     double p_x = Xsig_pred_(0,i);
     double p_y = Xsig_pred_(1,i);
     double v  = Xsig_pred_(2,i);
     double yaw = Xsig_pred_(3,i);

     double v1 = cos(yaw)*v;
     double v2 = sin(yaw)*v;

     // measurement model
     Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
     Zsig(1,i) = atan2(p_y,p_x);                                 //phi
     Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
   }

   //mean predicted measurement
   VectorXd z_pred = VectorXd(n_z);
   z_pred.fill(0.0);
   for (int i=0; i < 2*n_aug_+1; i++) {
       z_pred = z_pred + weights(i) * Zsig.col(i);
   }

   //innovation covariance matrix S
   MatrixXd S = MatrixXd(n_z,n_z);
   S.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
     //residual
     VectorXd z_diff = Zsig.col(i) - z_pred;

     //angle normalization
     while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

     S = S + weights(i) * z_diff * z_diff.transpose();
   }

   //add measurement noise covariance matrix
   MatrixXd R = MatrixXd(n_z,n_z);
   R <<    std_radr_*std_radr_, 0, 0,
           0, std_radphi_*std_radphi_, 0,
           0, 0,std_radrd_*std_radrd_;
   S = S + R;


   //create matrix for cross correlation Tc
   MatrixXd Tc = MatrixXd(n_x_, n_z);
   //calculate cross correlation matrix
   Tc.fill(0.0);
   for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

     //residual
     VectorXd z_diff = Zsig.col(i) - z_pred;
     //angle normalization
     while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

     // state difference
     VectorXd x_diff = Xsig_pred_.col(i) - x_;
     //angle normalization
     while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
     while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

     Tc = Tc + weights(i) * x_diff * z_diff.transpose();
   }

   //Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   //residual
   VectorXd z_diff = z - z_pred;

   //angle normalization
   while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
   while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

   //update state mean and covariance matrix
   x_ = x_ + K * z_diff;
   P_ = P_ - K*S*K.transpose();

   // NIS
   double nis = z_diff.transpose() * S.inverse() * z_diff;
   static const double line_95percent = 7.8;
   static uint32_t n_total = 0, n_aboveLine = 0;
   n_total++;
   if (nis > line_95percent) n_aboveLine++;
   cout << "RADAR NIS: Current: " << nis << ". \n %age of values above 95%line: ("<<n_aboveLine<<"/"<<n_total << ")*100 = " << float(n_aboveLine*100)/n_total << endl;
}
