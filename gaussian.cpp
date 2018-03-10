#include <iostream>
#include <cmath>

#include "gaussian.h"
#include "math.h"


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


Gaussian AugmentGaussian(const Gaussian &g, const Eigen::MatrixXd &Q) {
  assert(Q.rows() == Q.cols());

  Eigen::VectorXd xaug(g.x_.size() + Q.rows());
  Eigen::MatrixXd Paug(xaug.size(), xaug.size());

  xaug.head(g.x_.size()) = g.x_;
  for(unsigned i = g.x_.size(); i < xaug.size(); i++) {
    xaug(i) = 0;
  }

  Paug.fill(0);
  Paug.topLeftCorner(g.x_.size(), g.x_.size()) = g.P_;
  for(unsigned i = g.x_.size(); i < xaug.size(); i++) {
    auto j = i - g.x_.size();
    Paug(i, i) = Q(j, j);
  }

  Gaussian g2(xaug.size());
  g2.x_ = xaug;
  g2.P_ = Paug;
  g2.is_initialized_ = true;

  return g2;
}


Eigen::VectorXd CalculateSigmaWeights(const double lambda, const int n_aug) {
  Eigen::VectorXd weights(2 * n_aug + 1);

  weights(0) = lambda / (lambda + n_aug);
  for (unsigned i = 1; i < 2 * n_aug + 1; i++) {
    weights(i) = 0.5 / (lambda + n_aug);
  }

  return weights;
}


Eigen::VectorXd CalculatePredictedMean(const Eigen::VectorXd &weights, const Eigen::MatrixXd &Xsig_pred) {
  Eigen::VectorXd x(Xsig_pred.rows());

  x.fill(0);

  for (unsigned i = 0; i < Xsig_pred.cols(); i++) {
    x = x + weights(i) * Xsig_pred.col(i);
  }

  return x;
}


Eigen::MatrixXd CalculatePredictedCovMatrix(const Eigen::VectorXd &x, const Eigen::VectorXd &weights, const Eigen::MatrixXd &Xsig_pred,
					    std::function<void(Eigen::VectorXd&)> norm_df) {
  Eigen::MatrixXd P(Xsig_pred.rows(), Xsig_pred.rows());

  P.fill(0.0);

  for (unsigned i = 0; i < Xsig_pred.cols(); i++) {
    Eigen::VectorXd df = Xsig_pred.col(i) - x;
    norm_df(df);

    P = P + weights(i) * df * df.transpose();
  }

  return P;
}


Gaussian PredictGaussian(const Eigen::VectorXd &weights, const Eigen::MatrixXd &Xsig_pred, std::function<void(Eigen::VectorXd&)> norm_df) {
  const Eigen::VectorXd x = CalculatePredictedMean(weights, Xsig_pred);
  const Eigen::MatrixXd P = CalculatePredictedCovMatrix(x, weights, Xsig_pred, norm_df);

  Gaussian g(x.size());
  g.x_ = x;
  g.P_ = P;
  g.is_initialized_ = true;

  return g;
}
