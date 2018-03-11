/*
 * Read measurements from file and calculate RMSE
 */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
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


void test_file_or_die(const std::string &mode, std::shared_ptr<Fusion> fusion, char *filename) {
  // fake control vector
  Eigen::VectorXd u = fusion->GetZeroU();

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

    if (fusion->Init(m)) {
      continue;
    }

    Eigen::VectorXd estimate = fusion->ProcessMeasurement(m, u);
    Eigen::VectorXd gtv = GetGroundTruthVector(g);

    estimations.push_back(estimate);
    ground_truth.push_back(gtv);

    lineno++;
  }

  Eigen::VectorXd rmse = CalculateRMSE(estimations, ground_truth);

  std::cout << mode << ": " << filename << ": " << rmse.transpose() << std::endl;
}


void help() {
  std::cout << "objpose [--ekf | --ukf] file1.txt ..." << std::endl;
  std::cout << std::endl;
  std::cout << "Calculate RMSE using measurements from file." << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "    -h, --help - show help and exit with code 1" << std::endl;
  std::cout << "    --ekf - use Extended Kalman Filter for the following files" << std::endl;
  std::cout << "    --ukf - use Unscended Kalman Filter for the following files" << std::endl;
  std::cout << std::endl;

  exit(1);
}

enum Opt {
  kOptUnknown = 0,
  kOptHelp,
  kOptEkf,
  kOptUkf
};


Opt GetOpt(const std::string &opt) {
  if (opt == "-h" or opt == "--help") return kOptHelp;
  if (opt == "--ekf") return kOptEkf;
  if (opt == "--ukf") return kOptUkf;
  return kOptUnknown;
};


int main(int argc, char **argv) {
  Opt opt;
  bool opt_use_ukf = true;

  for (int i = 1; i < argc; i++) {
    opt = GetOpt(argv[i]);

    if (opt == kOptHelp) {
      help();
    }
  }

  for (int i = 1; i < argc; i++) {
    opt = GetOpt(argv[i]);

    if (opt == kOptEkf) {
      opt_use_ukf = false;
      continue;
    } else if (opt == kOptUkf) {
      opt_use_ukf = true;
      continue;
    }

    if (opt_use_ukf) {
      std::shared_ptr<Fusion> fusion = std::make_shared<FusionUKF>();
      test_file_or_die("UKF", fusion, argv[i]);
    } else {
      std::shared_ptr<Fusion> fusion = std::make_shared<FusionEKF>();
      test_file_or_die("EKF", fusion, argv[i]);
    }
  }

  return 0;
}
