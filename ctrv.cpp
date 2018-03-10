/*
 * ctrv.cpp - The CTRV model
 */

#include <cmath>

#include "ctrv.h"
#include "math.h"


/*
 * CtrvProcessModel calculates `x_pred` using augmented `x_aug` and `dt`.
 *
 * You can use this function to calculate prediced sigma points.
 *
 */
Eigen::VectorXd CtrvProcessModel(const Eigen::VectorXd &x_aug, const double dt) {
  Eigen::VectorXd x_pred(kCtrvStateDim);

  const double dt2 = dt * dt;

  double px = x_aug(0);
  double py = x_aug(1);
  double v = x_aug(2);
  double yaw = x_aug(3);
  double yawd = x_aug(4);
  double nua = x_aug(5);
  double nuyawdd = x_aug(6);

  double px_p, py_p;

  if (not IsZero(yawd)) {
    px_p = px + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
    py_p = py + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
  } else {
    px_p = px + v * dt * cos(yaw);
    py_p = py + v * dt * sin(yaw);
  }

  double v_p = v;
  double yaw_p = yaw + yawd * dt;
  double yawd_p = yawd;

  // add noise
  px_p = px_p + 0.5 * nua * dt2 * cos(yaw);
  py_p = py_p + 0.5 * nua * dt2 * sin(yaw);
  v_p = v_p + nua * dt;
  yaw_p = yaw_p + 0.5 * nuyawdd * dt2;
  yawd_p = yawd_p + nuyawdd * dt;

  x_pred << px_p, py_p, v_p, yaw_p, yawd_p;

  return x_pred;
}


/*
 * You can use this function to calculate sigma points in measurement space.
 *
 * Do not forget about `CtrvRadarMeasurementNoise`.
 *
 */
Eigen::VectorXd CtrvRadarMeasurementModel(const Eigen::VectorXd &x) {
  const double px = x(0);
  const double py = x(1);
  const double v  = x(2);
  const double yaw = x(3);

  const double v1 = cos(yaw) * v;
  const double v2 = sin(yaw) * v;

  Eigen::VectorXd z(kCtrvRadarZDim);
  z(0) = sqrt(px * px + py * py);                        //r
  // FIXME: check atan2(0, 0)
  z(1) = atan2(py, px);                                  //phi
  z(2) = (px * v1 + py * v2) / sqrt(px * px + py * py);  //r_dot

  return z;
}


Eigen::MatrixXd CtrvRadarMeasurementNoise(const double std_radr, const double std_radphi, const double std_radrd) {
  Eigen::MatrixXd R(kCtrvRadarZDim, kCtrvRadarZDim);
  R << std_radr * std_radr, 0, 0,
    0, std_radphi * std_radphi, 0,
    0, 0, std_radrd * std_radrd;
  return R;
}


Eigen::VectorXd CtrvLazerMeasurementModel(const Eigen::VectorXd &x) {
  Eigen::VectorXd z(kCtrvLazerZDim);

  z << x(0), x(1);

  return x;
}
