/*
 * tests for measurement_package.h
 */

#include "measurement_package.h"

#include <Eigen/Dense>

#include <gtest/gtest.h>


namespace {
  TEST(GetInitState, Lazer3D) {
    const struct lazer_measurement lazer = { .px = 10.0, .py = 11.0 };
    const struct measurement m(0, lazer);
    Eigen::VectorXd rest(1);
    rest << 300.0;

    const Eigen::VectorXd initv = m.GetInitState(3, rest);

    Eigen::VectorXd expected(3);
    expected << 10.0, 11.0, 300.0;
    ASSERT_TRUE(initv.isApprox(expected));
  }

  TEST(GetInitState, Radar3D) {
    const struct radar_measurement radar = { .rho = 1.0, .theta = M_PI, .rho_dot = 0.0 };
    const struct measurement m(0, radar);
    Eigen::VectorXd rest(1);
    rest << 300.0;

    const Eigen::VectorXd initv = m.GetInitState(3, rest);

    Eigen::VectorXd expected(3);
    expected << -1.0, 0.0, 300.0;

    ASSERT_TRUE(initv.isApprox(expected, 1e-7));
  }

  TEST(GetInitState, Lazer5D) {
    const struct lazer_measurement lazer = { .px = 10.0, .py = 11.0 };
    const struct measurement m(0, lazer);
    Eigen::VectorXd rest(3);
    rest << 12.0, 13.0, 14.0;

    const Eigen::VectorXd initv = m.GetInitState(5, rest);

    Eigen::VectorXd expected(5);
    expected << 10.0, 11.0, 12.0, 13.0, 14.0;
    ASSERT_TRUE(initv.isApprox(expected));
  }

  TEST(GetInitState, Radar5D) {
    const struct radar_measurement radar = { .rho = 1.0, .theta = M_PI, .rho_dot = 0.0 };
    const struct measurement m(0, radar);
    Eigen::VectorXd rest(3);
    rest << 12.0, 13.0, 14.0;

    const Eigen::VectorXd initv = m.GetInitState(5, rest);

    Eigen::VectorXd expected(5);
    expected << -1.0, 0.0, 12.0, 13.0, 14.0;

    ASSERT_TRUE(initv.isApprox(expected, 1e-7));
  }
};
