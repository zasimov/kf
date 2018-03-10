/*
 * Read measurements from file and calculate RMSE
 */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "fusionekf.h"
#include "ground_truth.h"
#include "math.h"
#include "measurement_package.h"
#include "parser.h"


void die(char *filename, int lineno, const std::string &msg) {
  std::cerr << "error: " << filename << ": " << lineno << ": " << msg << std::endl;
  exit(1);
}


Eigen::VectorXd GetGroundTruthVector(const struct ground_truth &g) {
  Eigen::VectorXd v(4);

  v << g.x, g.y, g.vx, g.vy;

  return v;
}


void test_file_or_die(char *filename) {
  FusionEKF fusion;

  // fake control vector
  Eigen::VectorXd u(4);
  u << 0.0, 0.0, 0.0, 0.0;

  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  std::ifstream infile(filename);

  std::string line;

  struct measurement m;
  struct ground_truth g;

  int lineno = 1;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (not parse(iss, &m, &g)) {
      die(filename, lineno, "malformed line");
    }

    if (fusion.Init(m)) {
      continue;
    }

    const Eigen::VectorXd &estimate = fusion.ProcessMeasurement(m, u);
    Eigen::VectorXd gtv = GetGroundTruthVector(g);

    estimations.push_back(estimate);
    ground_truth.push_back(gtv);

    lineno++;
  }

  Eigen::VectorXd rmse = CalculateRMSE(estimations, ground_truth);

  std::cout << filename << ": " << rmse.transpose() << std::endl;
}


int main(int argc, char **argv) {
  for (int i = 1; i < argc; i++) {
    test_file_or_die(argv[i]);
  }
  return 0;
}
