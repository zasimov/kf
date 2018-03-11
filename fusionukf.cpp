#include "fusionukf.h"


// 5 is a dimension of state space
FusionUKF::FusionUKF()
  : kStateSpaceDim_(5),
    LazerRadarFusion(std::make_shared<Gaussian>(5)) {

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


Eigen::VectorXd FusionUKF::GetZeroU() const {
  Eigen::VectorXd u(kStateSpaceDim_);
  u.fill(0.0);
  return u;
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
