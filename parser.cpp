/*
 * Parse measurements.
 *
 * Input: measurement string
 * Output: measurement from measurement_package.h
 *
 * Naming convention:
 *   - mstr - measurement string (like "L ..." or "R ...")
 *   - is - input stream
 */

#include "parser.h"

/*
 * Parse measurement string from `is` input stream.
 *
 * Fill `m` structure. m->sensor_type will by kLazer, m->lazer will be filled.
 *
 * Return false if `is` doesn't contain measurement string.
 */
bool parse_lazer(std::istream &is, struct measurement *m) {
  is >> m->lazer.px;
  if (is.fail()) {
    return false;
  }

  is >> m->lazer.py;
  if (is.fail()) {
    return false;
  }

  is >> m->timestamp;
  if (is.fail()) {
    return false;
  }

  m->sensor_type = measurement::kLazer;

  return true;
}


bool parse_radar(std::istream &is, struct measurement *m) {
  is >> m->radar.rho;
  if (is.fail()) {
    return false;
  }

  is >> m->radar.theta;
  if (is.fail()) {
    return false;
  }

  is >> m->radar.rho_dot;
  if (is.fail()) {
    return false;
  }

  is >> m->timestamp;
  if (is.fail()) {
    return false;
  }

  m->sensor_type = measurement::kRadar;

  return true;
}


bool parse_ground_truth(std::istream &is, struct ground_truth *g) {
  if (!(is >> g->x)) {
    return false;
  }

  if (!(is >> g->y)) {
    return false;
  }

  if (!(is >> g->vx)) {
    return false;
  }

  if (!(is >> g->vy)) {
    return false;
  }

  return true;
}


bool parse(std::istream &is, struct measurement *m, struct ground_truth *g) {
  std::string sensor_type;
  if (!(is >> sensor_type))
    return false;
  if (sensor_type == "L") {
    return parse_lazer(is, m) and parse_ground_truth(is, g);
  } else if (sensor_type == "R") {
    return parse_radar(is, m) and parse_ground_truth(is, g);
  } else {
    return false;
  }
}
