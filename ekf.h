#ifndef __EKF_H
#define __EKF_H

/*
 * Extended Kalman Filter implementation
 *
 */


#include "kalman_filter.h"


class SimpleKalmanFilter: public AbstractKalmanFilter {
 public:
  // `F` is an initial transition matrix
  SimpleKalmanFilter(std::shared_ptr<Gaussian> state,
		     const Eigen::MatrixXd &R);

  virtual const Eigen::VectorXd& Predict(const double dt, const Eigen::VectorXd &u);

  virtual void Update(const Eigen::VectorXd &z);

 protected:
  /**
   * GetH should return H matrix (X to Z projection).
   */
  virtual const Eigen::MatrixXd GetH() const = 0;

  // FIXME: it seems `SetDT` and `GetQ` is a part of "physical model" but not KF
  // Now SimpleKalmanFilter works with "hardcoded" physical model.

  /**
   * Apply `dt` to `F` matrix.
   */
  virtual void SetDT(const double dt);

  /**
   * Calculate process covariance matrix
   */
  virtual const Eigen::MatrixXd GetQ(const double dt) const;

  /**
   * GetY calculates `y` vector.
   */
  virtual Eigen::VectorXd GetY(const Eigen::VectorXd &z) const = 0;

 private:
  /*
   *   px' = 1 * px + 0 * py +  dt * vx + 0 * vy
   *   py' = 0 * px + 1 * py +  0 * vx + dt * vy
   *   vx' = 0 * px + 0 * py +  1 * vx + 0 * vy
   *   vy' = 0 * px + 0 * py +  0 * vx + 1 * vy
   */
  Eigen::MatrixXd F_;  // transition matrix

  Eigen::MatrixXd R_;  // measurement noise matrix

  Eigen::MatrixXd I_;  // identity matrix
};


class LinearKalmanFilter : public SimpleKalmanFilter {
 public:
  LinearKalmanFilter(std::shared_ptr<Gaussian> state,
		     const Eigen::MatrixXd &R,
		     const Eigen::MatrixXd &H);

  void Init(const Eigen::VectorXd &z);

 protected:
  const Eigen::MatrixXd GetH() const;
  Eigen::VectorXd GetY(const Eigen::VectorXd &z) const;

 private:
  Eigen::MatrixXd H_;  // measurement function

};


class ExtendedKalmanFilter : public SimpleKalmanFilter {
 public:
  ExtendedKalmanFilter(std::shared_ptr<Gaussian> state,
		       const Eigen::MatrixXd &R);

  void Init(const Eigen::VectorXd &z);
 protected:
  const Eigen::MatrixXd GetH() const;
  Eigen::VectorXd GetY(const Eigen::VectorXd &z) const;
};


#endif
