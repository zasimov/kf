#ifndef __CTRV_H
#define __CTRV_H

#include <Eigen/Dense>


static const int kCtrvStateDim = 5;
static const int kCtrvAugStateDim = 7;


/*
 * Map augmented sigma point to predicted sigma points.
 *
 * Input: 7-vector
 * Output: 5-vector
 *
 */
Eigen::VectorXd CtrvProcessModel(const Eigen::VectorXd &x_aug, const double dt);


#endif
