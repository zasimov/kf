#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "parser.h"

namespace {

  using namespace std;

  const string kLazerMeasurement = "L	4.632272e-01	6.074152e-01	1477010443000000	6.000000e-01	6.000000e-01	2.199937e+00	0	0	6.911322e-03";
  const string kRadarMeasurement = "R	8.986584e-01	6.176736e-01	1.798602e+00	1477010443050000	7.099968e-01	6.000190e-01	2.199747e+00	7.601581e-04	3.455661e-04	1.382155e-02";

  const float kLazerPx = 4.632272e-01;
  const float kLazerPy = 6.074152e-01;
  const long long kLazerTimestamp = 1477010443000000;
  const struct ground_truth kLazerGroundTruth = {
    6.000000e-01,
    6.000000e-01,
    2.199937e+00,
    0
  };

  const float kRadarRho = 8.986584e-01;
  const float kRadarTheta = 6.176736e-01;
  const float kRadarRhoDot = 1.798602e+00;
  const float kRadarTimestamp = 1477010443050000;
  const struct ground_truth kRadarGroundTruth = {
    7.099968e-01,
    6.000190e-01,
    2.199747e+00,
    7.601581e-04
  };

  TEST(ParseLazer, ValidReturnsTrue) {
    istringstream iss(kLazerMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;

    ASSERT_TRUE(parse_lazer(iss, &m));
  }

  TEST(ParseLazer, ValidSensorTypeIsLazer) {
    istringstream iss(kLazerMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;
    m.sensor_type = measurement::kRadar;

    ASSERT_TRUE(parse_lazer(iss, &m));
    ASSERT_EQ(m.sensor_type, measurement::kLazer);
  }

  TEST(ParseLazer, ValidCoords) {
    istringstream iss(kLazerMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;

    ASSERT_TRUE(parse_lazer(iss, &m));
    ASSERT_FLOAT_EQ(m.lazer.px, kLazerPx);
    ASSERT_FLOAT_EQ(m.lazer.py, kLazerPy);
  }

  TEST(ParseLazer, ValidTimestamp) {
    istringstream iss(kLazerMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;

    ASSERT_TRUE(parse_lazer(iss, &m));
    ASSERT_EQ(m.timestamp, kLazerTimestamp);
  }

  TEST(ParseLazer, NegativeEmpty) {
    istringstream iss("");

    struct measurement m;

    ASSERT_FALSE(parse_lazer(iss, &m));
  }

  TEST(ParseRadar, ValidReturnsTrue) {
    istringstream iss(kRadarMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;

    ASSERT_TRUE(parse_radar(iss, &m));
  }

  TEST(ParseRadar, ValidSensorTypeIsRadar) {
    istringstream iss(kRadarMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;
    m.sensor_type = measurement::kLazer;

    ASSERT_TRUE(parse_radar(iss, &m));
    ASSERT_EQ(m.sensor_type, measurement::kRadar);
  }

  TEST(ParseRadar, ValidCoords) {
    istringstream iss(kRadarMeasurement);
    string sensor_type;
    iss >> sensor_type;

    struct measurement m;

    ASSERT_TRUE(parse_radar(iss, &m));
    ASSERT_FLOAT_EQ(m.radar.rho, kRadarRho);
    ASSERT_FLOAT_EQ(m.radar.theta, kRadarTheta);
    ASSERT_FLOAT_EQ(m.radar.rho_dot, kRadarRhoDot);
  }

  TEST(Parse, Lazer) {
    istringstream iss(kLazerMeasurement);
    struct measurement m;
    struct ground_truth g;

    ASSERT_TRUE(parse(iss, &m, &g));
    ASSERT_EQ(m.sensor_type, measurement::kLazer);
  }

  TEST(Parse, Radar) {
    istringstream iss(kRadarMeasurement);
    struct measurement m;
    struct ground_truth g;

    ASSERT_TRUE(parse(iss, &m, &g));
    ASSERT_EQ(m.sensor_type, measurement::kRadar);
  }

  TEST(ParseGroundTruth, Lazer) {
    istringstream iss(kLazerMeasurement);
    struct measurement m;
    struct ground_truth g;

    ASSERT_TRUE(parse(iss, &m, &g));
    ASSERT_FLOAT_EQ(g.x, kLazerGroundTruth.x);
    ASSERT_FLOAT_EQ(g.y, kLazerGroundTruth.y);
    ASSERT_FLOAT_EQ(g.vx, kLazerGroundTruth.vx);
    ASSERT_FLOAT_EQ(g.vy, kLazerGroundTruth.vy);
  }

  TEST(ParseGroundTruth, Radar) {
    istringstream iss(kRadarMeasurement);
    struct measurement m;
    struct ground_truth g;

    ASSERT_TRUE(parse(iss, &m, &g));
    ASSERT_FLOAT_EQ(g.x, kRadarGroundTruth.x);
    ASSERT_FLOAT_EQ(g.y, kRadarGroundTruth.y);
    ASSERT_FLOAT_EQ(g.vx, kRadarGroundTruth.vx);
    ASSERT_FLOAT_EQ(g.vy, kRadarGroundTruth.vy);
  }
}
