#include <iostream>
#include <stdexcept>

#include "fusionekf.h"
#include "logging.h"
#include "math.h"


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


Fusion::Fusion(std::shared_ptr<Gaussian> state)
  : state_(state), clock_(0) {
}


const Eigen::VectorXd Fusion::ProcessMeasurement(const struct measurement &m, const Eigen::VectorXd &u) {
  if (Init(m)) {
    return ToEstimate(state_->x_);
  }

  std::shared_ptr<AbstractKalmanFilter> filter = ChooseFilter(m);
  double dt = US2S(clock_.Update(m.timestamp));

  filter->Predict(dt, u);

  Eigen::VectorXd z = GetZ(m);
  filter->Update(z);

  return ToEstimate(state_->x_);
}


Eigen::VectorXd Fusion::GetZ(const struct measurement &m) const {

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


void Fusion::PrintState() const {
  std::cout << clock_.NowStr() << "x = " << state_->x_.transpose() << std::endl;
  std::cout << clock_.NowStr() << "P = " << std::endl << state_->P_ << std::endl;
}



bool LazerRadarFusion::Init(const struct measurement &m) {
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


std::shared_ptr<AbstractKalmanFilter> LazerRadarFusion::ChooseFilter(const struct measurement &m) const {

  switch (m.sensor_type) {
  case measurement::kLazer:
    return lazer_filter_;
  case measurement::kRadar:
    return radar_filter_;
  default:
    logging::fatal("fusion ekf can process kLazer or kRadar measurements");
  }
}


// 4 is a dimension of state space
FusionEKF::FusionEKF()
  : LazerRadarFusion(std::make_shared<Gaussian>(4)), F_(4, 4) {

  // initialize shared gaussian
  ResetState();

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


FusionUKF::FusionUKF()
  : LazerRadarFusion(std::make_shared<Gaussian>(5)) {

  ResetState();

  // Process noise standard deviation longitudinal acceleration in m/s^2
  const double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  const double std_yawdd = 0.2;

  Eigen::VectorXd stdv(2);
  stdv << std_a, std_yawdd;

  Eigen::MatrixXd R_radar(3, 3);
  Eigen::MatrixXd R_lazer(2, 2);

  // Radar measurements noise std
  const double kStdRho = 0.3;     // in meters
  const double kStdPhi = 0.03;    // in radians
  const double kStdRhoDot = 0.3;  // in meters/second

  // Lazer measurements noise std
  const double kStdPx = 0.15; // in meters
  const double kStdPy = 0.15; // in meters

  // Lazer measurmenets noise std

  R_radar << kStdRho * kStdRho, 0, 0,
    0, kStdPhi * kStdPhi, 0,
    0, 0, kStdRhoDot * kStdRhoDot;

  R_lazer << kStdPx * kStdPx, 0.0,
    0.0, kStdPy * kStdPy;

  lazer_filter_ = std::make_shared<LazerUKF>(state_, stdv, R_lazer);
  radar_filter_ = std::make_shared<RadarUKF>(state_, stdv, R_radar);
}


void FusionUKF::ResetState() {
  state_->x_ << 1, 1, 1, 1, 1;

  state_->P_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

  state_->is_initialized_ = false;

  clock_.Init(0);
}


Eigen::VectorXd FusionUKF::ToEstimate(const Eigen::VectorXd &x) const {
    Eigen::VectorXd estimate(4);

    const double px = x(0);
    const double py = x(1);
    const double v  = x(2);
    const double yaw = x(3);

    estimate << px, py, cos(yaw) * v, sin(yaw) * v;

    return estimate;
}
