#ifndef __FUSIONEKF_H
#define __FUSIONEKF_H

#include <cmath>
#include <iostream>
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


class Fusion {
 protected:
  Fusion(std::shared_ptr<Gaussian> state);

 public:
  /**
   * Init initializes Kalman State and Clock.
   *
   * Returns true if pipeline is initializes on current step.
   *
   */
  virtual bool Init(const struct measurement &m) = 0;

  /**
   * Process processes measurement and returns estimate.
   */
  const Eigen::VectorXd ProcessMeasurement(const struct measurement &m, const Eigen::VectorXd &u);

  /**
   * Return "zero" control vector.
   */
  virtual Eigen::VectorXd GetZeroU() const = 0;

  /**
   * Reset state_
   *
   * uWS server calls this method when client is connected
   *
   */
  virtual void ResetState() = 0;

  /**
   * Print kalman filter state (for debug purposes).
   */

  void PrintState() const;

  std::shared_ptr<Gaussian> GetState() const {
    return state_;
  }

  std::string NowStr() const {
    return clock_.NowStr();
  }

 protected:
  /**
   * ChooseFilter chooses appropriate filter.
   */
  virtual std::shared_ptr<AbstractKalmanFilter> ChooseFilter(const struct measurement &m) const = 0;


  /**
   * GetZ converts measurement to Eigen vector.
   */
  Eigen::VectorXd GetZ(const struct measurement &m) const;

  /*
   * Convert state vector to estimate vector
   */
  virtual Eigen::VectorXd ToEstimate(const Eigen::VectorXd &x) const = 0;

 protected:
  std::shared_ptr<Gaussian> state_;  // kalman filter shared state
  Clock clock_;
};



class LazerRadarFusion: public Fusion {
 public:
  bool Init(const struct measurement &m);

 protected:
  LazerRadarFusion(std::shared_ptr<Gaussian> state)
    : Fusion(state) {
  }

  /**
   * ChooseFilter chooses appropriate filter.
   */
  std::shared_ptr<AbstractKalmanFilter> ChooseFilter(const struct measurement &m) const;

 protected:
  // child must initialize these fields
  std::shared_ptr<AbstractKalmanFilter> lazer_filter_;
  std::shared_ptr<AbstractKalmanFilter> radar_filter_;
};


class FusionEKF: public LazerRadarFusion {
 public:
  FusionEKF();

  Eigen::VectorXd GetZeroU() const;

  void ResetState();

 protected:
  Eigen::VectorXd ToEstimate(const Eigen::VectorXd &x) const;

 private:
  const int kStateSpaceDim_;

  // matrices
  Eigen::MatrixXd F_;
};


class FusionUKF: public LazerRadarFusion {
 public:
  FusionUKF();

  Eigen::VectorXd GetZeroU() const;

  void ResetState();

 protected:
  Eigen::VectorXd ToEstimate(const Eigen::VectorXd &x) const;

  private:
    const int kStateSpaceDim_;
};


#endif /* __FUSIONEKF_H */
