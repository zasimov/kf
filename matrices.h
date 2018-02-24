#ifndef __MATRICES_H
#define __MATRICES_H

#include <Eigen/Dense>

/**
 * Set dt for F matrix.
 *
 * F matrix is 4x4 matrix and represents following equations
 *
 *   px' = 1 * px + 0 * py +  dt * vx + 0 * vy
 *   py' = 0 * px + 1 * py +  0 * vx + dt * vy
 *   vx' = 0 * px + 0 * py +  1 * vx + 0 * vy
 *   vy' = 0 * px + 0 * py +  0 * vx + 1 * vy
 *
 * SetDT function changes
 *   F(0, 2) = dt
 *   F(1, 3) = dt
 *
 */
void SetDT(Eigen::MatrixXd &F, double new_df);


/**
 * CalculateQ calculates process covariance matrix.
 */
Eigen::MatrixXd CalculateQ(double d);

#endif
