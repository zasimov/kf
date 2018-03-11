#ifndef __FUSIONUKF_H
#define __FUSIONUKF_H

/*
 * Unscended Kalman Filter Fusion
 */

#include <cmath>

#include <Eigen/Dense>

#include "fusion.h"
#include "ukf.h"
#include "measurement_package.h"


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


#endif /* __FUSIONUKF_H */
