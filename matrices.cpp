#include "matrices.h"


void SetDT(Eigen::MatrixXd &F, double new_df) {
  F(0, 2) = new_df;
  F(1, 3) = new_df;
}


Eigen::MatrixXd CalculateQ(double dt) {
  double dt_2 = dt   * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // set the acceleration noise components
  double noise_ax = 9;
  double noise_ay = 9;

  // set the process covariance matrix Q
  Eigen::MatrixXd Q(4, 4);
  Q << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
    0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
    dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

  return Q;
}
