#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  std_a_ = 6.0;
  std_yawdd_ = 1.0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2*n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < weights_.size(); i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  time_us_ = 0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {
    is_initialized_ = true; 

    // initialize state and process noise matrix
    x_.fill(0.0);
    if (meas_package.sensor_type_ == meas_package.LASER) {
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
      
      P_(0, 0) = std_laspx_ * std_laspx_;
      P_(1, 1) = std_laspy_ * std_laspy_;

    } else if (meas_package.sensor_type_ == meas_package.RADAR) {
      x_(0) = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      x_(1) = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);

      P_(0, 0) = P_(1, 1) = std_radr_ * std_radr_;
      P_(2, 2) = std_radrd_ * std_radrd_;
      P_(3, 3) = P_(4, 4) = std_radphi_ * std_radphi_;
    }
  } else {
    // perform a prediction step
    double dt = (meas_package.timestamp_ - time_us_) / 1000000;
    Prediction(dt);
  }

  time_us_ = meas_package.timestamp_;

  if (use_laser_ && meas_package.sensor_type_ == meas_package.LASER) {
    UpdateLidar(meas_package);
  }
    
  if (use_radar_ && meas_package.sensor_type_ == meas_package.RADAR) {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // create augmented states
  VectorXd x_aug = VectorXd(7);
  MatrixXd P_aug = MatrixXd(7, 7);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  // predict sigma points
  for (int i = 0; i < Xsig_aug.cols(); i++) {
    // extract parameters
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted position
    double px_p, py_p;

    // perform different calculations if yaw rate is 0
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // write values to predicted sigma point matrix. add noise
    Xsig_pred_(0, i) = px_p + (0.5 * nu_a * delta_t * delta_t * cos(yaw));
    Xsig_pred_(1, i) = py_p + (0.5 * nu_a * delta_t * delta_t * sin(yaw));
    Xsig_pred_(2, i) = v_p + (nu_a * delta_t);
    Xsig_pred_(3, i) = yaw_p + (0.5 * nu_yawdd * delta_t * delta_t);
    Xsig_pred_(4, i) = yawd_p + (nu_yawdd * delta_t);
  }

  // compute predicted mean and covariance
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Initialize measurement vector
  int n_z = 2;
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];

  // Initialize state->measurement transition matrix
  MatrixXd H = MatrixXd(n_z, n_x_);
  H.fill(0.0);
  H(0, 0) = H(1, 1) = 1;

  // Compute predicted measurement using predicted sigma points of state
  MatrixXd Z_sig = H * Xsig_pred_;

  // compute mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1;++i){
    z_pred += Z_sig.col(i) * weights_(i);  
  }

  // compute measurement covariance
  MatrixXd R = MatrixXd::Identity(n_z, n_z);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  S += R;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) { 
    // difference
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // compute process-measurement cross-correlation
  MatrixXd T = MatrixXd(n_x_, n_z);
  T.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) { 
    // measurement difference
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    
    // state difference 
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    T = T + weights_(i) * x_diff * z_diff.transpose();
  }

  // update state
  MatrixXd K = T * S.inverse();
  VectorXd err = z - z_pred;

  x_ = x_ + K * err;
  P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}