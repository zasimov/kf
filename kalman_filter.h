#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <memory>

#include <Eigen/Dense>

#include "gaussian.h"


class AbstractKalmanFilter {
 public:

  AbstractKalmanFilter(std::shared_ptr<Gaussian> state);

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
  virtual const Eigen::VectorXd& Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::VectorXd &u) = 0;

  /**
   * Update implements "Update" step"
   *
   *  z is a measurement vector.
   */
  virtual void Update(const Eigen::VectorXd &z) = 0;

  /**
   * GetEstimate returns current estimate of state.
   */
  const Eigen::VectorXd& GetEstimate() const;


 protected:
  std::shared_ptr<Gaussian> state_;
};


class SimpleKalmanFilter: public AbstractKalmanFilter {
 public:
  SimpleKalmanFilter(std::shared_ptr<Gaussian> state,
		     const Eigen::MatrixXd &R);

  virtual const Eigen::VectorXd& Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::VectorXd &u);

  virtual void Update(const Eigen::VectorXd &z);

 protected:
  /**
   * GetH should return H matrix (X to Z projection).
   */
  virtual const Eigen::MatrixXd GetH() const = 0;

  /**
   * GetY calculates `y` vector.
   */
  virtual Eigen::VectorXd GetY(const Eigen::VectorXd &z) const = 0;

 private:
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


#endif /* __KALMAN_FILTER_H */
