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

#endif
