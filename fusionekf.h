#ifndef __FUSIONEKF_H
#define __FUSIONEKF_H

#include <cmath>

#include <Eigen/Dense>

#include "fusion.h"
#include "ekf.h"
#include "measurement_package.h"


/*
 * Extended Kalman Filter Fusion
 */


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
