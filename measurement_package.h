#ifndef __MEASUREMENT_PACKAGE_H
#define __MEASUREMENT_PACKAGE_H


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
};

#endif
