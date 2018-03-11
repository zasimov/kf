#ifndef __UKF_H
#define __UKF_H

/*
 * Unscended Kalman Filter Implementation
 *
 * `CtrvUnscendedKalmanFilter` is a base class for `LazerUKF` and `RadarUKF`.
 * `CtrvUnscendedKalmanFilter` implements full `predict` and `update` workflows.
 *
 * `LazerUKF` processes lazer measurements.
 *
 * `RadarUKF` processes radar measurements.
 *
 *
 * `FusionUKF` uses both `LazerUKF` and `RadarUKF`.
 *
 */


#include "kalman_filter.h"


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
  const Eigen::VectorXd stdv_;
  Eigen::MatrixXd Q_;   // calculated by `stdv_`
  Eigen::VectorXd weights_;  // const
  const Eigen::MatrixXd R_;  // measurement noise

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


#endif
