# The C++ Implementation of Extended Kalman Filter

This repository contains implementations of Kalman Filter and Extended
Kalman Filter.


## How to build

The following system packages should be installed:

  * `cmake`
  * `make`
  * `gcc`
  * `libuv` and development files for `libuv`

Note: the `pre-build.sh` script can be used to build the required dependencies.

`googletest` is used as a primary test framework. You should

    git clone https://github.com/google/googletest
    cd googletest
    mkdir build-googletest
    cd build-gooletest
    cmake .. && make && make install

`Eigen` is used as linear algebra library. Download Eigen 3.3.4 from http://eigen.tuxfamily.org/, unpack and copy Eigen folder to /usr/local/include. Direct link is http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz

    wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
    tar zxf 3.3.4.tar.gz
    sudo cp -R eigen-eigen-5a0156e40feb/Eigen /usr/local/include

Use the following commands to build applications:

    mkdir build
    cd build
    cmake .. && make && make test

Make sure that the parser works and `ob_pose-laser-radar-synthetic-input.txt` is valid:

    ./testfile ./obj_pose-laser-radar-synthetic-input.txt && echo "OK!"


## Applications

`build` folder contains the following applications that can be used
for experiments:

  * `ekf` runs WebSocket-server (port 4567) and can be used as a server for Udacity Term 2 Simulator

  * `objpose` reads measurements from a file and prints RMSE

## Code exploration

The main modules are `fusionekf` and `kalman_filter`. `fusionekf` is an
implementation of fusion pipeline. `kalman_filter` contains
implementation of kalman filter and extended kalman filter.

### fusionekf module

  * `FusionEKF::FusionEKF` initializes matrices (F, R_lazer, H_lazer,
    R_radar) and creates kalman filter as well as extended kalman filter.

  * `FusionEKF::Init` initializes kalman filter state using the first measurement

  * `FusionEKF::ProcessMeasurement` implements predict-update step.

### kalman_filter module

  * `AnstractKalmanFilter::Predict` implements `predict` step for
    kalman filter and extended kalman filter (there is the same code
    for KF and EKF)

  * `AbstractKalmanFilter::Update` implements common `update` steps.

The difference between `LinearKalmanFilter` and `ExtendedKalmanFilter` is
the way to calculate `H` matrix and `y` vector. See
`LinearKalmanFilter::GetH`, `LinearKalmanFilter::GetY`,
`ExtendedKalmanFilter::GetH`, `ExtendedKalmanFilter::GetY`.

### json.hpp

The latest `json.hpp` can be found here - https://github.com/nlohmann/json

## Experiment

    build/objpose obj_pose-laser-radar-synthetic-input.txt

The resulting RMSE is

    0.0745332  0.074456  0.265846  0.249984


## Udacity Term 2 Simulator v1.45

The applications were tested with https://github.com/udacity/self-driving-car-sim/releases/download/v1.45/term2_sim_linux.zip