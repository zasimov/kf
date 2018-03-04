#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "gaussian.h"

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

    Gaussian gaug = AugmentGaussian(g, stdv);

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
  
  
}
