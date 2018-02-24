#ifndef __PROTOCOL_H
#define __PROTOCOL_H


#include <string>

#include <Eigen/Dense>

#include "ground_truth.h"
#include "measurement_package.h"


/**
 * Send "Manual" (I really don't know what it means)
 */
void SendManual(uWS::WebSocket<uWS::SERVER> *ws);


/**
 * IsWebSocketMessageEvent return `true` if current message contains
 * measurement.
 */
bool IsWebSocketMessageEvent(char *data, size_t length);


/**
 * GetMeasurement applies WebSocket message and fills
 * measurement structure.
 *
 * Returns true only if `m` contains filled measurement.
 *
 */
bool GetMeasurement(char *data, struct measurement *m, struct ground_truth *g);


void SendEstimate(uWS::WebSocket<uWS::SERVER> *ws,
		  const Eigen::VectorXd &estimate, const Eigen::VectorXd &RMSE);


#endif /* __PROTOCOL_H */
