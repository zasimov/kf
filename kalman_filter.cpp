#include <cassert>
#include <iostream>

#include "ctrv.h"
#include "gaussian.h"
#include "kalman_filter.h"
#include "math.h"


AbstractKalmanFilter::AbstractKalmanFilter(std::shared_ptr<Gaussian> state):
  state_(state) {
  // nothing to do
}


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


/*
 * CtrvUnscendedKalmanFilter
 */

CtrvUnscendedKalmanFilter::CtrvUnscendedKalmanFilter(std::shared_ptr<Gaussian> state, const Eigen::VectorXd &stdv, const Eigen::MatrixXd &R)
  : AbstractKalmanFilter(state), stdv_(stdv), R_(R) {

  Q_ = Eigen::MatrixXd(stdv_.size(), stdv_.size());
  // calculate Q_ matrix
  Q_.fill(0.0);
  for (unsigned i = 0; i < stdv_.size(); i++) {
    Q_(i, i) = stdv_(i) * stdv_(i);
  }

  // calculate weights
  lambda_ = STDLAMBDA(GetNaug());
  weights_ = CalculateSigmaWeights(lambda_, GetNaug());

  Xsig_pred_ = Eigen::MatrixXd(GetNx(), 2 * GetNaug() + 1);
}


const Eigen::VectorXd& CtrvUnscendedKalmanFilter::Predict(const double dt, const Eigen::VectorXd &u) {

  // calculate augmented sigma points
  Gaussian g_aug = AugmentGaussian(*state_, Q_);
  Eigen::MatrixXd Xsig_aug = CalculateSigmaPoints(lambda_, g_aug.x_, g_aug.P_);

  assert(Xsig_aug.cols() == 2 * GetNaug() + 1);

  // map augmented sigma points to predicted sigma points
  // (n_aug, 2 * n_aug + 1) => (n_x, 2 * n_aug + 1)
  for (unsigned i = 0; i < Xsig_aug.cols(); i++) {
    Xsig_pred_.col(i) = CtrvProcessModel(Xsig_aug.col(i), dt);
  }

  // calculate predicted gaussian
  auto norm_dx = [this](Eigen::VectorXd &dx) {
    this->NormDx(dx);
  };
  *state_ = PredictGaussian(weights_, Xsig_pred_, norm_dx);
}


void CtrvUnscendedKalmanFilter::Update(const Eigen::VectorXd &z) {
  Eigen::MatrixXd Zsig_pred(GetNz(), Xsig_pred_.cols());

  // map predicted sigma points from state space to measurement space
  for (unsigned i = 0; i < Xsig_pred_.cols(); i++) {
    Zsig_pred.col(i) = MapXtoZ(Xsig_pred_.col(i));;
  }

  // calculate predicted gaussian
  auto norm_dz = [this](Eigen::VectorXd &dz) {
    NormDz(dz);
  };
  Gaussian g_z_pred = PredictGaussian(weights_, Zsig_pred, norm_dz);
  const Eigen::VectorXd &z_pred = g_z_pred.x_;
  const Eigen::MatrixXd S = g_z_pred.P_ + R_;  // add measurement noise

  // calculate crosscorrelation matrix
  const Eigen::MatrixXd Tc = CalculateCrosscorrelationMatrix(Xsig_pred_, Zsig_pred, state_->x_, z_pred);

  // calculate Kalman gain
  const Eigen::MatrixXd K = Tc * S.inverse();

  const Eigen::VectorXd dz = CalculateZdiff(z, z_pred);

  // update step
  state_->x_ = state_->x_ + K * dz;
  state_->P_ = state_->P_ - K * S * K.transpose();
}


int CtrvUnscendedKalmanFilter::GetNx() const {
  return state_->x_.size();
}


int CtrvUnscendedKalmanFilter::GetNaug() const {
  return GetNx() + stdv_.size();
}


Eigen::MatrixXd CtrvUnscendedKalmanFilter::CalculateCrosscorrelationMatrix(const Eigen::MatrixXd &Xsig_pred,
									   const Eigen::MatrixXd &Zsig_pred,
									   const Eigen::VectorXd &x_pred,
									   const Eigen::VectorXd &z_pred) {
  assert(Xsig_pred.cols() == Zsig_pred.cols());
  assert(Xsig_pred.cols() == weights_.size());

  Eigen::MatrixXd Tc(x_pred.size(), z_pred.size());
  Tc.fill(0.0);

  for (unsigned i = 0; i < Xsig_pred.cols(); i++) {
    Eigen::VectorXd x_diff = CalculateXdiff(Xsig_pred.col(i), x_pred);
    Eigen::VectorXd z_diff = CalculateZdiff(Zsig_pred.col(i), z_pred);
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  return Tc;
}


/*
 * LazerUKF
 */
void LazerUKF::Init(const Eigen::VectorXd &z) {
  // the same as in LinearKalmanFilter

  assert(z.size() == 2);

  state_->x_(0) = z(0);
  state_->x_(1) = z(1);
}


void LazerUKF::NormDx(Eigen::VectorXd &dx) const {
  // normalize angle
  dx(3) = NormalizeAngle(dx(3));
}


void LazerUKF::NormDz(Eigen::VectorXd &dz) const {
  // nothing to norm
}


Eigen::VectorXd LazerUKF::MapXtoZ(const Eigen::VectorXd &x) const {
  return CtrvLazerMeasurementModel(x);
}


/*
 * RadarUKF
 */
void RadarUKF::Init(const Eigen::VectorXd &z) {
  // the same as in EKF
  assert(z.size() == 3);

  Eigen::VectorXd c = ToCartesian(z);

  state_->x_(0) = c(0);
  state_->x_(1) = c(1);
}


void RadarUKF::NormDx(Eigen::VectorXd &dx) const {
  // normalize angle
  dx(3) = NormalizeAngle(dx(3));
}


void RadarUKF::NormDz(Eigen::VectorXd &dz) const {
  // normalize rho
  dz(1) = NormalizeAngle(dz(1));
}


Eigen::VectorXd RadarUKF::MapXtoZ(const Eigen::VectorXd &x) const {
  return CtrvRadarMeasurementModel(x);
}
