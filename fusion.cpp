#include "logging.h"

#include "fusion.h"


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
