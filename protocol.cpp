#include <sstream>

#include <uWS/uWS.h>

#include "json.hpp"

#include "ground_truth.h"
#include "parser.h"
#include "protocol.h"
#include "measurement_package.h"


/**
 * As I understand client (simulator) sends one message per line. Each message starts with 42 where
 *   4 - message code
 *   2 - event code
 * After follows "event body". Event body is a JSON array like
 *   ["event type", <event data>]
 *
 * Server processes events with "event type" == "telemetry"
 *
 */


/**
 * WS-protocol related constants
 */
const char kWebSocketMessageCode = '4';
const char kWebSocketEventCode = '2';

const char *kEventTypeTelemetry = "telemetry";

const char *kManual = "42[\"manual\",{}]";


// for convenience
using json = nlohmann::json;


void SendManual(uWS::WebSocket<uWS::SERVER> *ws) {
  ws->send(kManual);
}


bool IsWebSocketMessageEvent(char *data, size_t length) {
  if (length <= 2) {
    return false;
  }

  return data[0] == kWebSocketMessageCode && data[1] == kWebSocketEventCode;
}


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


bool GetMeasurement(char *data, struct measurement *m, struct ground_truth *g) {
  auto data_string = HasData(std::string(data));

  if (data_string == "") {
    return false;
  }

  auto j = json::parse(data_string);

  std::string event_type = j[0].get<std::string>();

  if (event_type != kEventTypeTelemetry) {
    return false;
  }

  std::string sensor_measurement = j[1]["sensor_measurement"];

  std::istringstream iss(sensor_measurement);

  return parse(iss, m, g);
}


void SendEstimate(uWS::WebSocket<uWS::SERVER> *ws,
		  const Eigen::VectorXd &estimate, const Eigen::VectorXd &RMSE) {
  json json_message;

  json_message["estimate_x"] = estimate(0);
  json_message["estimate_y"] = estimate(1);
  json_message["rmse_x"] = RMSE(0);
  json_message["rmse_y"] = RMSE(1);
  json_message["rmse_vx"] = RMSE(2);
  json_message["rmse_vy"] = RMSE(3);

  std::string response = "42[\"estimate_marker\"," + json_message.dump() + "]";
  ws->send(response.c_str());
}
