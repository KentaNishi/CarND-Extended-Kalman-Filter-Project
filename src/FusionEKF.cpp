#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  /* ekf_ is KalmanFilter type.
          * // state vector ,this is going to be set at each process. 
          Eigen::VectorXd x_;

          // state covariance matrix , initial value is set in this constructor function.
          Eigen::MatrixXd P_;

          // state transition matrix, this depends on dt, so ,receiving sensor data,its value is going to be set,
          Eigen::MatrixXd F_;

          // process covariance matrix, this depends on dt, so ,receiving sensor data,its value is going to be set,
          Eigen::MatrixXd Q_;

          // measurement matrix, this is based on which senser is used, so I set this just before each measurement process.
          // its shape also depends sensor type. It is impossible to initialize in this constructor.
          Eigen::MatrixXd H_;

          // measurement covariance matrix, this is based on which senser is used, so I set this just before each measurement process.
          // its shape also depends sensor type. It is impossible to initialize in this constructor.
          Eigen::MatrixXd R_;
  */
  // state vector x_
  

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0,     0,
             0, 1, 0,     0,
             0, 0, 1000,  0,
             0, 0, 0,     1000;

  // initializing state transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  // initializing process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);

  

  // measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  // measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  // measurement matrix - laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // initializing measurement matrices
  Hj_ = MatrixXd(3, 4);



}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1,
               1,
               1,
               1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);

      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);

      ekf_.x_ << px,
                 py,
                 vx,
                 vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_(0),
                 measurement_pack.raw_measurements_(1),
                 ekf_.x_(2),
                 ekf_.x_(3);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  /*  Predict function requires x_, P_, Q_, F_
      x_ and P_ are already prepared.
  */
  // acceleration noise components
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  // calculate time difference between the current and previous measurement.
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // set the value of F_
  ekf_.F_ << 1, 0, dt,  0,
             0, 1,  0, dt,
             0, 0,  1,  0,
             0, 0,  0,  1;

  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  // set the value of Q_
  ekf_.Q_ << dt4/4.0*noise_ax,  0,                  dt3/2.0*noise_ax,  0,
             0,                 dt4/4.0*noise_ay,   0,                 dt3/2.0*noise_ay,
             dt3/2.0*noise_ax,  0,                  dt2*noise_ax,      0,
             0,                 dt3/2.0*noise_ay,   0,                 dt2*noise_ay;

  // x_ and P_ is updated.
  ekf_.Predict(); 
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  /*  Predict function requires x_, P_, H_, R_
      x_ and P_ are already prepared.
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.H_ = MatrixXd(3, 4);
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    if ((Hj_.array()==0).all()==false){
      ekf_.H_ = Hj_;
      ekf_.R_ = MatrixXd(3, 3);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    
  } else {
    // TODO: Laser updates
    cout << "laser updates" << endl;
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ = H_laser_;
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
