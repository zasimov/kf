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



#define STDLAMBDA(n_x) (3 - (n_x))


/*
 * CalculateSigmaPoints returns (n, 2n + 1) matrix where n is a size of `x_` vector.
 *
 * `lambda` defines a scale
 */
Eigen::MatrixXd CalculateSigmaPoints(double lambda, const Eigen::VectorXd &x, const Eigen::MatrixXd &P);

/*
 * Augment gaussian `g`
 *
 * `stdv` - process noise vector (standard deviations)
 *
 */
Gaussian AugmentGaussian(const Gaussian &g, const Eigen::VectorXd &stdv);


/*
 * Predict gaussian using predicted sigma points
 */
Gaussian PredictGaussian(const Eigen::MatrixXd &Xsig_pred, const double lambda, const int n_aug);

#endif
