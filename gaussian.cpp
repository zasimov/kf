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
