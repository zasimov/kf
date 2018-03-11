#include <cassert>
#include <iostream>

#include "gaussian.h"
#include "kalman_filter.h"
#include "math.h"

#include "ekf.h"


/*
 * Simple Kalman Filter (the base for LinearKalmanFilter and ExtendedKalmanFilter)
 */
SimpleKalmanFilter::SimpleKalmanFilter(std::shared_ptr<Gaussian> state,
				       const Eigen::MatrixXd &R):
  AbstractKalmanFilter(state), R_(R) {

  F_ = Eigen::MatrixXd(4, 4);

  /*
   *   px' = 1 * px + 0 * py +  dt * vx + 0 * vy
   *   py' = 0 * px + 1 * py +  0 * vx + dt * vy
   *   vx' = 0 * px + 0 * py +  1 * vx + 0 * vy
   *   vy' = 0 * px + 0 * py +  0 * vx + 1 * vy
   */
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  I_ = Eigen::MatrixXd::Identity(state_->x_.size(),
				 state_->x_.size());
}


const Eigen::VectorXd& SimpleKalmanFilter::Predict(const double dt, const Eigen::VectorXd &u) {
  const Eigen::MatrixXd Q = GetQ(dt);
  SetDT(dt);  // change F_ matrix

  state_->x_ = F_ * state_->x_ + u;
  state_->P_ = F_ * state_->P_ * F_.transpose() + Q;

  return state_->x_;
}


void SimpleKalmanFilter::Update(const Eigen::VectorXd &z) {
  const Eigen::VectorXd y = GetY(z);

  const Eigen::MatrixXd H = GetH();
  const Eigen::MatrixXd H_transpose = H.transpose();

  const Eigen::MatrixXd S = (H * state_->P_ * H_transpose) + R_;
  const Eigen::MatrixXd K = state_->P_ * H_transpose * S.inverse();  // Kalman gain

  state_->x_ = state_->x_ + K * y;
  state_->P_ = (I_ - K * H) * state_->P_;
}


void SimpleKalmanFilter::SetDT(const double dt){
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}


const Eigen::MatrixXd SimpleKalmanFilter::GetQ(const double dt) const {
  double dt_2 = dt   * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // set the acceleration noise components
  double noise_ax = 9;
  double noise_ay = 9;

  // set the process covariance matrix Q
  Eigen::MatrixXd Q(4, 4);
  Q << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
    0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
    dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

  return Q;
}


/*
 * LinearKalmanFilter (aka Kalman Filter)
 */

LinearKalmanFilter::LinearKalmanFilter(std::shared_ptr<Gaussian> state,
				       const Eigen::MatrixXd &R,
				       const Eigen::MatrixXd &H)
  : SimpleKalmanFilter(state, R), H_(H) {
}


void LinearKalmanFilter::Init(const Eigen::VectorXd &z) {

  assert(z.size() == 2);

  state_->x_(0) = z(0);
  state_->x_(1) = z(1);
}


const Eigen::MatrixXd LinearKalmanFilter::GetH() const {
  return H_;
}


Eigen::VectorXd LinearKalmanFilter::GetY(const Eigen::VectorXd &z) const {
  return z - H_ * state_->x_;
}


/*
 * ExtendedKalmanFilter (EKF)
 */

ExtendedKalmanFilter::ExtendedKalmanFilter(std::shared_ptr<Gaussian> state,
					   const Eigen::MatrixXd &R)
  : SimpleKalmanFilter(state, R) {

}


void ExtendedKalmanFilter::Init(const Eigen::VectorXd &z) {
  assert(z.size() == 3);

  Eigen::VectorXd c = ToCartesian(z);

  state_->x_(0) = c(0);
  state_->x_(1) = c(1);
}


const Eigen::MatrixXd ExtendedKalmanFilter::GetH() const {
  return CalculateJacobian(state_->x_);
}


Eigen::VectorXd ExtendedKalmanFilter::GetY(const Eigen::VectorXd &z) const {
  Eigen::VectorXd z_predicted = ToPolar(state_->x_);
  Eigen::VectorXd y = z - z_predicted;
  // normalize phi
  y(1) = NormalizeAngle(y(1));
  return y;
}
