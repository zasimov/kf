#include <cassert>

#include "gaussian.h"
#include "kalman_filter.h"
#include "math.h"


AbstractKalmanFilter::AbstractKalmanFilter(std::shared_ptr<Gaussian> state,
					   const Eigen::MatrixXd &R):
  state_(state), R_(R) {

  I_ = Eigen::MatrixXd::Identity(state_->x_.size(),
				 state_->x_.size());
}


const Eigen::VectorXd& AbstractKalmanFilter::Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::VectorXd &u) {
  state_->x_ = F * state_->x_ + u;
  state_->P_ = F * state_->P_ * F.transpose() + Q;
  return state_->x_;
}


void AbstractKalmanFilter::Update(const Eigen::VectorXd &z) {
  const Eigen::VectorXd y = GetY(z);

  const Eigen::MatrixXd H = GetH();
  const Eigen::MatrixXd H_transpose = H.transpose();

  const Eigen::MatrixXd S = (H * state_->P_ * H_transpose) + R_;
  const Eigen::MatrixXd K = state_->P_ * H_transpose * S.inverse();  // Kalman gain

  state_->x_ = state_->x_ + K * y;
  state_->P_ = (I_ - K * H) * state_->P_;
}


const Eigen::VectorXd& AbstractKalmanFilter::GetEstimate() const {
  return state_->x_;
}


LinearKalmanFilter::LinearKalmanFilter(std::shared_ptr<Gaussian> state,
				       const Eigen::MatrixXd &R,
				       const Eigen::MatrixXd &H)
  : AbstractKalmanFilter(state, R), H_(H) {
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


ExtendedKalmanFilter::ExtendedKalmanFilter(std::shared_ptr<Gaussian> state,
					   const Eigen::MatrixXd &R)
  : AbstractKalmanFilter(state, R) {

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
