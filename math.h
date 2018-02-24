#ifndef __MY_MATH_H
#define __MY_MATH_H

/**
 * Math tools.
 */

#include <vector>

#include <Eigen/Dense>


static const double kZero = 0.0001;

/**
 * IsZero returns `true` if d is almost zero or zero.
 *
 * IsZero compares absolute of d with kZero.
 *
 */
bool IsZero(double d);


/**
 * CalculateJacobian (Hj)
 */
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_predicted);


/**
 * ToPolar Converts vector v = <px, py, vx, vy> to Polar vector <rho, phi, rho_dot>.
 */
Eigen::VectorXd ToPolar(const Eigen::VectorXd &v);

/**
 * ToCartesian converts vector v = <rho, phi, rho_dot> to cartezian <px, py, vx, vy>
 */
Eigen::VectorXd ToCartesian(const Eigen::VectorXd &v);


Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
			      const std::vector<Eigen::VectorXd> &ground_truth);


double NormalizeAngle(const double angle);

#endif /* __MY_MATH_H */
