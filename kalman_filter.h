#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

/*
 * There is an Abstract Kalman Filter only.
 */

#include <memory>

#include <Eigen/Dense>

#include "gaussian.h"


class AbstractKalmanFilter {
 public:

  AbstractKalmanFilter(std::shared_ptr<Gaussian> state)
    : state_(state) {
  }

  virtual ~AbstractKalmanFilter() {
  }

  /**
   * Should be called once (with first measurement).
   */
  virtual void Init(const Eigen::VectorXd &z) = 0;

  /**
   * Predict implements "Predict" step.
   *
   *  F - transition matrix for dt
   *  Q - process covariance matrix for dt
   *  u - control vector
   *
   * Predict changes `x_` and `P_`. `x_` fill contain prediction for (t + 1)
   *
   */
  virtual const Eigen::VectorXd& Predict(const double dt, const Eigen::VectorXd &u) = 0;

  /**
   * Update implements "Update" step"
   *
   *  z is a measurement vector.
   */
  virtual void Update(const Eigen::VectorXd &z) = 0;

 protected:
  std::shared_ptr<Gaussian> state_;
};


#endif /* __KALMAN_FILTER_H */
