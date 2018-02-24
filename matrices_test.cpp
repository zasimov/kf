#include <Eigen/Dense>

#include <gtest/gtest.h>

#include "matrices.h"

namespace {
  TEST(SetDF, Simple) {
    Eigen::MatrixXd F(4, 4);

    double initial_dt = 99;
    double new_dt = 39;

    // NOTE: initial_dt should not be equal new_dt

    F << 1, 0, initial_dt, 0,
      0, 1 ,0, initial_dt,
      0, 0, 1, 0,
      0, 0, 0, 1;

    Eigen::MatrixXd RF(4, 4);

    RF << 1, 0, new_dt, 0,
      0, 1 ,0, new_dt,
      0, 0, 1, 0,
      0, 0, 0, 1;

    SetDT(F, new_dt);

    ASSERT_TRUE(F.isApprox(RF));
    
  }
} // namespace
