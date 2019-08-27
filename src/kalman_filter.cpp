#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}



void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  x_ = F_*x_;
  P_ = F_ * P_ * F_.transpose()  + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // z is true value from sensor,z_pred is prediction value.
  // y is error between true value and predict value.
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd S_ = H_ * P_ *H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  // new state estimate
  x_ = x_ + (K_ * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K_* H_)* P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // z is true value from sensor
  // y is error between true value and predict value.

  // to calculate K_

  // calculate z = h(x_), not equal Hj_ * x_. 
  double px = x_(0);
  double py = x_(1);
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = c2;
  double phi = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / c2;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  /* */
  while (y(1) < -M_PI || M_PI < y(1)) {
      if (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
      } else if (M_PI < y(1)) {
        y(1) -= 2 * M_PI;
      }
    }
  MatrixXd S_ = H_ * P_ *H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  // new state estimate
  x_ = x_ + (K_ * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K_* H_)* P_;

}
