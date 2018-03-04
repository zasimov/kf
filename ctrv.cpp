/*
 * ctrv.cpp - The CTRV model
 */

#include <cmath>

#include "ctrv.h"
#include "math.h"


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
