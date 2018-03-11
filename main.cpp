#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <uWS/uWS.h>

#include "fusionekf.h"
#include "ground_truth.h"
#include "kalman_filter.h"
#include "logging.h"
#include "math.h"
#include "measurement_package.h"
#include "protocol.h"


const unsigned kListenPort = 4567;


Eigen::VectorXd GetGroundTruthVector(const struct ground_truth &g) {
  Eigen::VectorXd v(4);

  v << g.x, g.y, g.vx, g.vy;

  return v;
}


void help() {
  std::cout << "kf [--ekf | --ukf]" << std::endl;
  std::cout << std::endl;
  std::cout << "Run Kalman Filter application in server mode." << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "    -h, --help - show help and exit with code 1" << std::endl;
  std::cout << "    --ekf - use Extended Kalman Filter" << std::endl;
  std::cout << "    --ukf - use Unscended Kalman Filter" << std::endl;
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
  uWS::Hub hub;

  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  std::shared_ptr<Fusion> fusion;

  bool opt_use_ukf = true;

  // parse arguments
  for (int i = 1; i < argc; i++) {
    switch (GetOpt(argv[i])) {

    case kOptHelp:
      help();
      break;

    case kOptEkf:
      opt_use_ukf = false;
      break;

    case kOptUkf:
      opt_use_ukf = true;
      break;

    default:
      std::cout << "unknown option: " << argv[i] << std::endl << std::endl;
      help();
    }
  }

  if (opt_use_ukf) {
    fusion = std::make_shared<FusionUKF>();
    logging::info("Using UKF");
  } else {
    fusion = std::make_shared<FusionEKF>();
    logging::info("Using EKF");
  }

    // fake control vector
  const Eigen::VectorXd u = fusion->GetZeroU();


  if (! hub.listen(kListenPort)) {
    logging::fatal("cannot start server, probably port is busy");
    exit(1);
  }

  /**
   * onConnection and onDisconnection are added to notify user about
   * important events.
   */
  hub.onConnection([&hub, fusion, &estimations, &ground_truth](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest request) {
      logging::info("Client connected.");

      // FIXME: hack, reinit state and drop clock
      fusion->ResetState();
      estimations.clear();
      ground_truth.clear();
    });

  hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
      logging::info("Client disconnected.");
    });

  /**
   * Fusion!!!
   */
  hub.onMessage([&fusion, &estimations, &ground_truth, &u]
		(uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
       struct measurement m;
       struct ground_truth g;

       if (! IsWebSocketMessageEvent(data, length)) {
	 logging::info("*** Drop ws message");
	 SendManual(ws);
	 return;
       }

       // Process Sensor Measurements
       if (! GetMeasurement(data, &m, &g)) {
	 SendManual(ws);
	 return;
       }

       if (m.sensor_type != measurement::kLazer && m.sensor_type != measurement::kRadar) {
	 // drop unknown measurement
	 logging::fatal("measurement of unknown type");
       }

       std::cout << "*** Process measurement: " << m.sensor_type << std::endl;

       Eigen::VectorXd estimate = fusion->ProcessMeasurement(m, u);
       Eigen::VectorXd gtv = GetGroundTruthVector(g);

       estimations.push_back(estimate);
       ground_truth.push_back(gtv);

       Eigen::VectorXd rmse = CalculateRMSE(estimations, ground_truth);

       SendEstimate(ws, estimate, rmse);

       std::cout << fusion->NowStr() << "GT = " << gtv.transpose() << std::endl;
       fusion->PrintState();
       std::cout << fusion->NowStr() << "RMSE = " << rmse.transpose() << std::endl;
     });

  logging::info("Listening to " + std::to_string(kListenPort) + " port");
  hub.run();

  return 0;
}
