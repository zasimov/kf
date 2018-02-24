#include <cmath>

#include "logging.h"
#include "math.h"


bool IsZero(double d) {
  return fabs(d) < kZero;
}


const unsigned kRadarZDim = 3;


Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_predicted) {
  Eigen::MatrixXd Hj(kRadarZDim, x_predicted.size());

  // recover state parameters
  double px = x_predicted(0);
  double py = x_predicted(1);
  double vx = x_predicted(2);
  double vy = x_predicted(3);

  // pre-compute a set of terms to avoid repeated calculation
  double c1 = px * px + py * py;

  // check division by zero
  if (IsZero(c1)) {
    logging::error("Division by zero in CalculateJacobian");
    Hj.setZero();
    return Hj;
  }

  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  // compute the Jacobian matrix
  Hj <<  (px/c2),                (py/c2),               0,     0,
        -(py/c1),                (px/c1),               0,     0,
         py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}


Eigen::VectorXd ToPolar(const Eigen::VectorXd &x) {
  assert(x.size() == 4);

  double px = x[0];
  double py = x[1];
  double vx = x[2];
  double vy = x[3];

  if (IsZero(px)) {
    px = kZero;
  }

  double rho = sqrt(px * px + py * py);
  double rho_dot;
  if (IsZero(rho)) {
    rho_dot = 0;
  } else {
    rho_dot = (px * vx + py * vy) / rho;
  }

  double phi = atan2(py, px);

  Eigen::VectorXd polar(3);
  polar << rho, phi, rho_dot;

  return polar;
}


Eigen::VectorXd ToCartesian(const Eigen::VectorXd &v) {
  double rho = v[0];
  double phi = v[1];
  double rho_dot = v[2];

  Eigen::VectorXd cartesian(4);
  cartesian << rho * cos(phi),
    rho * sin(phi),
    rho_dot * cos(phi),
    rho_dot * sin(phi);

  return cartesian;
}


Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
			      const std::vector<Eigen::VectorXd> &ground_truth) {
  Eigen::VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0){
    return rmse;
  }

  // accumulate squared residuals
  for(unsigned i = 0; i < estimations.size(); ++i){
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}


// normalize the angle to be within -pi to pi
double NormalizeAngle(const double angle)
{
    double normalized_angle = angle;

    if (fabs(angle) > M_PI) {
        static const double two_pi = 2 * M_PI;
        normalized_angle -= round(normalized_angle / two_pi) * two_pi;
    }

    return normalized_angle;
}
