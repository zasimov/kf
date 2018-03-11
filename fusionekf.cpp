#include <iostream>
#include <stdexcept>

#include "fusionekf.h"
#include "logging.h"
#include "math.h"


const double kInitialStateUncertanty = 1000;


// 4 is a dimension of state space
FusionEKF::FusionEKF()
  : kStateSpaceDim_(4),
    LazerRadarFusion(std::make_shared<Gaussian>(4)),
    F_(4, 4) {

  // initialize shared gaussian
  ResetState();

  // H_lazer - drop velocity
  Eigen::MatrixXd H_lazer(2, kStateSpaceDim_);
  H_lazer << 1, 0, 0, 0,
             0, 1, 0, 0;

  Eigen::MatrixXd R_lazer(2, 2);
  R_lazer << 0.0225, 0,
             0,      0.0225;

  lazer_filter_ = std::make_shared<LinearKalmanFilter>(state_, R_lazer, H_lazer);

  Eigen::MatrixXd R_radar(3, 3);
  R_radar << 0.09, 0,      0,
             0,    0.0009, 0,
             0,    0,      0.09;

  radar_filter_ = std::make_shared<ExtendedKalmanFilter>(state_, R_radar);
}


Eigen::VectorXd FusionEKF::GetZeroU() const {
  Eigen::VectorXd u(kStateSpaceDim_);
  u.fill(0.0);
  return u;
}


void FusionEKF::ResetState() {
  state_->x_ << 1, 1, 1, 1;

  state_->P_ << 1, 0, 0,                       0,
                0, 1, 0,                       0,
                0, 0, kInitialStateUncertanty, 0,
                0, 0, 0,                       kInitialStateUncertanty;

  state_->is_initialized_ = false;

  clock_.Init(0);
}


Eigen::VectorXd FusionEKF::ToEstimate(const Eigen::VectorXd &x) const {
  Eigen::VectorXd estimate(x);
  return estimate;
}
