#ifndef __CTRV_H
#define __CTRV_H

#include <Eigen/Dense>


static const int kCtrvStateDim = 5;
static const int kCtrvAugStateDim = 7;
static const int kCtrvRadarZDim = 3;
static const int kCtrvLazerZDim = 2;


/*
 * Map augmented sigma point to predicted sigma points.
 *
 * Input: 7-vector
 * Output: 5-vector
 *
 */
Eigen::VectorXd CtrvProcessModel(const Eigen::VectorXd &x_aug, const double dt);


/*
 * Map sigma point to measurement space.
 */
Eigen::VectorXd CtrvRadarMeasurementModel(const Eigen::VectorXd &x);

/*
 * Calculate measurement noise matrix
 *
 * std_radr - radar measurement noise standard deviation radius in m
 * std_radphi - radar measurement noise standard deviation angle in rad
 * std_radrd - radar measurement noise standard deviation radius change in m/s
 *
 */
Eigen::MatrixXd CtrvRadarMeasurementNoise(const double std_radr, const double std_radphi, const double std_radrd);


/*
 * Map a state vector to measurement vector (for Lazer)
 */
Eigen::VectorXd CtrvLazerMeasurementModel(const Eigen::VectorXd &x);



#endif
