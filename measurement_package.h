#ifndef __MEASUREMENT_PACKAGE_H
#define __MEASUREMENT_PACKAGE_H

#include <cmath>

#include <Eigen/Dense>


struct lazer_measurement {
  float px;
  float py;
};


struct radar_measurement {
  float rho;
  float theta;
  float rho_dot;
};


struct measurement {
  enum sensor_type {
    kUnknown = 0,
    kLazer,
    kRadar
  } sensor_type;

  long long timestamp;

  union {
    struct lazer_measurement lazer;
    struct radar_measurement radar;
  };

  measurement(): sensor_type(kUnknown), timestamp(0) {
  }

  measurement(long long timestamp_, const struct lazer_measurement &lazer_)
  : sensor_type(kLazer), timestamp(timestamp_) {
    lazer = lazer_;
  }

  measurement(long long timestamp_, const struct radar_measurement &radar_)
  : sensor_type(kRadar), timestamp(timestamp_) {
    radar = radar_;
  }

  /*
   * Return init state based on current measurement.
   *
   * `dim` is a dimension of state space.
   *
   */
  Eigen::VectorXd GetInitState(int dim, const Eigen::VectorXd &rest) const {
    assert(dim > 2);

    Eigen::VectorXd initv(dim);
    initv.setZero();

    switch (sensor_type) {
    case kLazer:
      initv(0) = lazer.px;
      initv(1) = lazer.py;
      break;
    case kRadar:
      initv(0) = radar.rho * cos(radar.theta);
      initv(1) = radar.rho * sin(radar.theta);
      break;
    default:
      break;
    }

    for (unsigned i = 2; i < dim; i++) {
      initv(i) = rest(i - 2);
    }

    return initv;
  }
};

#endif
