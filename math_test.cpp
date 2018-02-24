#include <cmath>
#include <Eigen/Dense>

#include "math.h"

#include <gtest/gtest.h>


namespace {
  using namespace std;

  TEST(ToPolar, Zero) {
    Eigen::VectorXd x(4);
    x << 0.0, 0.0, 0.0, 0.0;

    Eigen::VectorXd polar = ToPolar(x);

    Eigen::VectorXd expected(3);
    expected << sqrt(kZero * kZero), atan2(0.0, 1.0), 0;

    ASSERT_TRUE(polar.isApprox(expected));
  }

  TEST(ToPolar, One) {
    Eigen::VectorXd x(4);
    x << 1.0, 1.0, 1.0, 1.0;

    Eigen::VectorXd polar = ToPolar(x);

    Eigen::VectorXd expected(3);
    expected << sqrt(2.0), atan2(1.0, 1.0), 2 / sqrt(2);

    ASSERT_TRUE(polar.isApprox(expected));
  }

  TEST(ToCartesian, Zero) {
    Eigen::VectorXd p(3);
    p << 0.0, 0.0, 0.0;

    Eigen::VectorXd c = ToCartesian(p);

    Eigen::VectorXd expected(4);
    expected << 0.0, 0.0, 0.0, 0.0;

    ASSERT_TRUE(c.isApprox(expected));
  }

  TEST(ToCartesian, One) {
    Eigen::VectorXd p(3);
    p << sqrt(2), M_PI / 4, 2 / sqrt(2);

    Eigen::VectorXd c = ToCartesian(p);

    Eigen::VectorXd expected(4);
    expected << 1.0, 1.0, 1.0, 1.0;

    ASSERT_TRUE(c.isApprox(expected));
  }
}
