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


class CtrvUnscendedKalmanFilter : public AbstractKalmanFilter {
 public:
  /*
   * `stdv` - noise vector
   *
   * `n_aug` = state->x_.size() + stdv.size()
   * `lambda` - lambda (STDLAMBDA(n_aug) is a good choice)
   */
  CtrvUnscendedKalmanFilter(std::shared_ptr<Gaussian> state, const Eigen::VectorXd &stdv, const Eigen::MatrixXd &R);

  virtual const Eigen::VectorXd& Predict(const double dt, const Eigen::VectorXd &u);

  virtual void Update(const Eigen::VectorXd &z);

 protected:
  /*
   * Calculate residual (x - x_pred)
   */
  Eigen::VectorXd CalculateXdiff(const Eigen::VectorXd &x, const Eigen::VectorXd &x_pred) {
    Eigen::VectorXd dx = x - x_pred;
    NormDz(dx);
    return dx;
  }

  /*
   * Calculate residual (z - z_pred)
   */
  Eigen::VectorXd CalculateZdiff(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred) {
    Eigen::VectorXd dz = z - z_pred;
    NormDz(dz);
    return dz;
  }

  /*
   * Normalize `dx` vector inplace.
   */
  virtual void NormDx(Eigen::VectorXd &dx) const = 0;

  /*
   * Normalize 'dz' vector inplace (dz = z - z_pred)
   */
  virtual void NormDz(Eigen::VectorXd &dz) const = 0;

  /*
   * Map state vector to measurement vector.
   */
  virtual Eigen::VectorXd MapXtoZ(const Eigen::VectorXd &x) const = 0;

  /*
   * Calculate Croscorrelation Matrix (n_x, n_z) using sigm points and pair of predicted vectors
   * Should be different for lazer and radar (for radar with normalization)
   */
  Eigen::MatrixXd CalculateCrosscorrelationMatrix(const Eigen::MatrixXd &Xsig_pred,
						  const Eigen::MatrixXd &Zsig_pred,
						  const Eigen::VectorXd &x_pred,
						  const Eigen::VectorXd &z_pred);

  /*
   * Return dimension of state space.
   */
  int GetNx() const;

  /*
   * Return dimension of measurement space.
   */
  virtual int GetNz() const = 0;

  /*
   * Return dimension of augmented space: GetNx() + stdv_.size()
   */
  int GetNaug() const;

 protected:
  double lambda_;
  Eigen::VectorXd stdv_;
  Eigen::MatrixXd Q_;   // calculated by `stdv_`
  Eigen::VectorXd weights_;  // const
  Eigen::MatrixXd R_;  // measurement noise

 private:
  Eigen::MatrixXd Xsig_pred_;  // predicted sigma points, `Predict` calculates Xsig_pred_ and `Update` uses.
};


class LazerUKF: public CtrvUnscendedKalmanFilter {
 public:
  using CtrvUnscendedKalmanFilter::CtrvUnscendedKalmanFilter;

  void Init(const Eigen::VectorXd &z);

 protected:
  void NormDx(Eigen::VectorXd &dx) const;

  void NormDz(Eigen::VectorXd &dz) const;

  Eigen::VectorXd MapXtoZ(const Eigen::VectorXd &x) const;

  int GetNz() const {
    return 2;
  }
};


class RadarUKF: public CtrvUnscendedKalmanFilter {
 public:
  using CtrvUnscendedKalmanFilter::CtrvUnscendedKalmanFilter;

  void Init(const Eigen::VectorXd &z);

 protected:
  void NormDx(Eigen::VectorXd &dx) const;

  void NormDz(Eigen::VectorXd &dz) const;

  Eigen::VectorXd MapXtoZ(const Eigen::VectorXd &x) const;

  int GetNz() const {
    return 3;
  }
};


#endif /* __KALMAN_FILTER_H */
