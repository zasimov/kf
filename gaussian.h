#ifndef __GAUSSIAN_H
#define __GAUSSIAN_H


#include <Eigen/Dense>


struct Gaussian {
  Eigen::VectorXd x_;  // mean vector
  Eigen::MatrixXd P_;  // state covariance matrix

  bool is_initialized_;

  Gaussian(unsigned x_size)
  : x_(x_size), P_(x_size, x_size), is_initialized_(false) {
  }
};


/*
 * CalculateSigmaPoints returns (n, 2n + 1) matrix where n is a size of `x_` vector.
 *
 * `lambda` defines a scale
 */
Eigen::MatrixXd CalculateSigmaPoints(double lambda, const Eigen::VectorXd &x, const Eigen::MatrixXd &P);

#endif
