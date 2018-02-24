#ifndef __FUSIONEKF_H
#define __FUSIONEKF_H


#include <memory>

#include <Eigen/Dense>

#include "kalman_filter.h"
#include "measurement_package.h"


/**
 * US2S converts microseconds to seconds
 */
#define US2S(us) ((us) / 1000000.0)


class Clock {
public:
  Clock(long long initial);

  void Init(long long now);
  long long Update(long long now);

  std::string NowStr() const;

private:
  long long time_;
  long long start_;
};


class FusionEKF {
 public:
  FusionEKF();

  /**
   * Init initializes Kalman State and Clock.
   *
   * Returns true if pipeline is initializes on current step.
   *
   */
  bool Init(const struct measurement &m);

  /**
   * Process processes measurement and returns estimate.
   */
  const Eigen::VectorXd& ProcessMeasurement(const struct measurement &m);

  /**
   * GetEstimate returns current estimation of state.
   */
  const Eigen::VectorXd GetEstimate() const;

  std::shared_ptr<KalmanFilterState> GetState() const {
    return state_;
  }

  /**
   * Reset state_
   */
  void ResetState();

  /**
   * Print kalman filter state (for debug purposes).
   */

  void PrintState() const;

  std::string NowStr() const {
    return clock_.NowStr();
  }

 private:
  /**
   * ChooseFilter chooses appropriate filter.
   */
  std::shared_ptr<AbstractKalmanFilter> ChooseFilter(const struct measurement &m) const;

  /**
   * GetZ converts measurement to Eigen vector.
   */
  Eigen::VectorXd GetZ(const struct measurement &m) const;

 private:
  struct std::shared_ptr<KalmanFilterState> state_;
  Clock clock_;

  // matrices

  /*
   *   px' = 1 * px + 0 * py +  dt * vx + 0 * vy
   *   py' = 0 * px + 1 * py +  0 * vx + dt * vy
   *   vx' = 0 * px + 0 * py +  1 * vx + 0 * vy
   *   vy' = 0 * px + 0 * py +  0 * vx + 1 * vy
   */
  Eigen::MatrixXd F_;

  // control vector
  Eigen::VectorXd u_;

  std::shared_ptr<LinearKalmanFilter> lazer_filter_;
  std::shared_ptr<ExtendedKalmanFilter> radar_filter_;
};


#endif /* __FUSIONEKF_H */
