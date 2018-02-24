#include <iostream>
#include <stdexcept>

#include "fusionekf.h"
#include "logging.h"
#include "math.h"
#include "matrices.h"


const double kInitialStateUncertanty = 1000;


Clock::Clock(long long initial): time_(initial), start_(initial) {
}


void Clock::Init(long long now) {
  time_ = now;
  start_ = now;
}


long long Clock::Update(long long now) {
  long long dt = now - time_;
  time_ = now;
  return dt;
}


std::string Clock::NowStr() const {
  return "[" + std::to_string(US2S(time_ - start_)) + "] ";
}


FusionEKF::FusionEKF()
  : clock_(0), F_(4, 4), u_(4) {

  state_ = std::make_shared<KalmanFilterState>(4);  // 4 is a dimension of state space

  state_->x_ << 1, 1, 1, 1;
  state_->P_ << 1, 0, 0,                       0,
                0, 1, 0,                       0,
                0, 0, kInitialStateUncertanty, 0,
                0, 0, 0,                       kInitialStateUncertanty;

  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  u_ << 0, 0, 0, 0;

  // H_lazer - drop velocity
  Eigen::MatrixXd H_lazer(2, 4);
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


bool FusionEKF::Init(const struct measurement &m) {
  if (state_->is_initialized_) {
    return false;
  }

  std::shared_ptr<AbstractKalmanFilter> filter = ChooseFilter(m);

  // TODO: Should filter calculate z?
  Eigen::VectorXd z = GetZ(m);

  filter->Init(z);

  clock_.Init(m.timestamp);

  state_->is_initialized_ = true;

  return true;
}


std::shared_ptr<AbstractKalmanFilter> FusionEKF::ChooseFilter(const struct measurement &m) const {

  switch (m.sensor_type) {
  case measurement::kLazer:
    return lazer_filter_;
  case measurement::kRadar:
    return radar_filter_;
  default:
    logging::fatal("fusion ekf can process kLazer or kRadar measurements");
  }
}


Eigen::VectorXd FusionEKF::GetZ(const struct measurement &m) const {

  if (m.sensor_type == measurement::kLazer) {
    Eigen::VectorXd z(2);

    z << m.lazer.px, m.lazer.py;

    return z;
  } else if (m.sensor_type == measurement::kRadar) {
    Eigen::VectorXd z(3);

    // FIXME: theta?
    z << m.radar.rho, m.radar.theta, m.radar.rho_dot;

    return z;
  } else {
    logging::fatal("cannot calculate z-vector for measurement");
  }
}


const Eigen::VectorXd& FusionEKF::ProcessMeasurement(const struct measurement &m) {
  if (Init(m)) {
    return state_->x_;
  }

  std::shared_ptr<AbstractKalmanFilter> filter = ChooseFilter(m);
  double dt = US2S(clock_.Update(m.timestamp));
  SetDT(F_, dt);
  Eigen::MatrixXd Q = CalculateQ(dt);

  filter->Predict(F_, Q, u_);

  Eigen::VectorXd z = GetZ(m);
  filter->Update(z);

  return state_->x_;
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


void FusionEKF::PrintState() const {
  std::cout << clock_.NowStr() << "x = " << state_->x_.transpose() << std::endl;
  std::cout << clock_.NowStr() << "P = " << std::endl << state_->P_ << std::endl;
}
