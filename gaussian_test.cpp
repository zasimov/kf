#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "gaussian.h"
#include "math.h"

#include <gtest/gtest.h>


namespace {

  // This test uses examples from lessons.
  TEST(TestSigmaPoints, Lessons) {
    const int n_x = 5;

    Eigen::VectorXd x(n_x);
    x << 5.7441,
      1.3800,
      2.2049,
      0.5015,
      0.3528;

    Eigen::MatrixXd P(n_x, n_x);
    P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
      -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
      0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
      -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
      -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

    const double lambda = STDLAMBDA(n_x);

    const Eigen::MatrixXd sigma_points = CalculateSigmaPoints(lambda, x, P);

    Eigen::MatrixXd expected(n_x, 2 * n_x + 1);
    expected << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441,
      1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38,
      2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049,
      0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015,
      0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879;

    ASSERT_EQ(sigma_points.rows(), sigma_points.rows());
    ASSERT_EQ(sigma_points.cols(), sigma_points.cols());

    ASSERT_TRUE(sigma_points.isApprox(expected, 1e-6));
  }

  // This test uses examples from lessons.
  TEST(TestAugmentGaussian, Lessons) {
    const int n_x = 5;

    Eigen::VectorXd x(n_x);
    x << 5.7441,
      1.3800,
      2.2049,
      0.5015,
      0.3528;

    Eigen::MatrixXd P(n_x, n_x);
    P << 0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
      0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    Gaussian g(x.size());

    g.x_ = x;
    g.P_ = P;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    const double std_a = 0.2;

    // Process noise standard deviation yaw acceleration in rad/s^2
    const double std_yawdd = 0.2;

    Eigen::VectorXd stdv(2);
    stdv << std_a, std_yawdd;

    Eigen::MatrixXd Q(2, 2);
    Q.fill(0.0);
    Q(0, 0) = stdv(0) * stdv(0);
    Q(1, 1) = stdv(1) * stdv(1);

    Gaussian gaug = AugmentGaussian(g, Q);

    Eigen::MatrixXd sigma_points = CalculateSigmaPoints(STDLAMBDA(gaug.x_.size()), gaug.x_, gaug.P_);

    const int n_x_aug = 7;
    ASSERT_EQ(sigma_points.rows(), n_x_aug);
    ASSERT_EQ(sigma_points.cols(), 2 * n_x_aug + 1);

    Eigen::MatrixXd expected(n_x_aug, 2 * n_x_aug + 1);
    expected << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
      1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38,
      2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049,
      0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015,
      0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528,
      0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0,
      0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;

    ASSERT_TRUE(sigma_points.isApprox(expected, 1e-6));
  }


  // This test uses examples from lessons (24. Predict Mean and Covariance)
  TEST(TestPredictGaussian, Lessons) {
    const int n_x = 5;
    const int n_aug = 7;

    Eigen::MatrixXd Xsig_pred(n_x, 2 * n_aug + 1);
    Xsig_pred << 5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
      1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
      2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
      0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
      0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

    const Eigen::VectorXd weights = CalculateSigmaWeights(STDLAMBDA(n_aug), n_aug);
    const Gaussian g = PredictGaussian(weights, Xsig_pred, [](Eigen::VectorXd &df) { df(3) = NormalizeAngle(df(3)); });

    Eigen::VectorXd expected_x(n_x);
    expected_x <<  5.93637, 1.49035, 2.20528, 0.536853, 0.353577;

    ASSERT_TRUE(g.x_.isApprox(expected_x, 1e-6));

    Eigen::MatrixXd expected_P(n_x, n_x);
    expected_P << 0.00543425, -0.0024053, 0.00341576, -0.00348196, -0.00299378,
      -0.0024053, 0.010845, 0.0014923, 0.00980182, 0.00791091,
      0.00341576, 0.0014923, 0.00580129, 0.000778632, 0.000792973,
      -0.00348196, 0.00980182, 0.000778632, 0.0119238, 0.0112491,
      -0.00299378, 0.00791091, 0.000792973, 0.0112491, 0.0126972;

    ASSERT_TRUE(g.P_.isApprox(expected_P, 1e-5));
  }

}
