#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */
// It will be better too use numpy in place of Eigen library
// Constructor
KalmanFilter::KalmanFilter() {}
// Destructor
KalmanFilter::~KalmanFilter() {}
// Function for prediction and namescope operator
void KalmanFilter::Predict() {
  /**
   * THis is step for prediction 
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO-Done: update the state by using Extended Kalman Filter equations
   */
  VectorXd y = z - radarMeasurementFunction(x_);
// Calling pointer inside heap memoery
  double *phi = &y[1];
  while (*phi <= -M_PI) {
    *phi += M_PI;
  }
  while (*phi > M_PI) {
    *phi -= M_PI;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
  
}

Eigen::VectorXd KalmanFilter::radarMeasurementFunction(const VectorXd &predictedState){
  VectorXd result = VectorXd(3);

  double px = predictedState[0];
  double py = predictedState[1];
  double vx = predictedState[2];
  double vy = predictedState[3];

  double r1 = std::sqrt(std::pow(px, 2.0) + std::pow(py, 2.0));
  double p1 = std::atan2(py, px);
  if (fabs(r1) < 0.0001) {
      r1 = 0.0001;
   }
  double rho_dot = (px * vx + py * vy) / r1;

  result << r1, p1, rho_dot;
  return result;

}
