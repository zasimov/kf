#include <cmath>

#include "gaussian.h"


Eigen::MatrixXd CalculateSigmaPoints(const double lambda, const Eigen::VectorXd &x, const Eigen::MatrixXd &P) {
  const int n_x = x.size();
  
  // create a sigma point matrix (this is a result)
  Eigen::MatrixXd Xsig(n_x, 2 * n_x + 1);

  // calculate a square root of P
  Eigen::MatrixXd A = P.llt().matrixL();

  Xsig.col(0) = x;

  const double k = std::sqrt(lambda + n_x);

  for (unsigned i = 0; i < n_x; i++) {
    Xsig.col(i + 1)       = x + k * A.col(i);
    Xsig.col(i + 1 + n_x) = x - k * A.col(i);
  }

  return Xsig;
}


Gaussian AugmentGaussian(const Gaussian &g, const Eigen::VectorXd stdv) {
  Eigen::VectorXd xaug(g.x_.size() + stdv.size());
  Eigen::MatrixXd Paug(xaug.size(), xaug.size());

  xaug.head(g.x_.size()) = g.x_;
  for(unsigned i = g.x_.size(); i < xaug.size(); i++) {
    xaug(i) = 0;
  }

  Paug.fill(0);
  Paug.topLeftCorner(g.x_.size(), g.x_.size()) = g.P_;
  for(unsigned i = g.x_.size(); i < xaug.size(); i++) {
    Paug(i, i) = stdv(i - g.x_.size()) * stdv(i - g.x_.size());
  }

  Gaussian g2(xaug.size());
  g2.x_ = xaug;
  g2.P_ = Paug;
  g2.is_initialized_ = true;

  return g2;
}
