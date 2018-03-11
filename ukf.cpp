#include <cassert>
#include <iostream>

#include "ctrv.h"
#include "gaussian.h"
#include "kalman_filter.h"
#include "math.h"

#include "ukf.h"


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
