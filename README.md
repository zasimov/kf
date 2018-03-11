# The C++ Implementation of Kalman Filters

This repository contains implementations of Kalman Filter, Extended
Kalman Filter (EKF) and Unscended Kalman Filter (UKF).

## How to build (light way)

### Ubuntu

    sudo apt-get update && sudo apt-get install -y cmake g++ make libssl-dev libuv-dev git
    git clone https://github.com/zasimov/kf
    cd kf && ./pre-build.sh && mkdir build && cd build && cmake .. && make
    export LD_LIBRARY_PATH=$(pwd)../uWebSockets  # optional

Note: you don't need to install the latest version of uWebSockets to your `/usr`.
      `ekf` will use `uWS.so` from `kf/uWebSockets` directory.

### Mac OS

    brew install libuv openssl
    git clone https://github.com/zasimov/kf
    cd kf && ./pre-build.sh && mkdir build && cd build
    cmake -DOPENSSL_LIBRARIES=/usr/local/opt/openssl/lib -DOPENSSL_INCLUDE_DIR=/usr/local/opt/openssl/include ..
    make
    export DYLD_LIBRARY_PATH=$(pwd)/../uWebSockets/

## How to build (hard way)

Note: do not forget to checkout submodules

    git submodule update --init

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
    cmake .. && make && sudo make install

(It is not neccesary to build googletest if you don't want to run tests)

`Eigen` is used as linear algebra library. Download Eigen 3.3.4 from http://eigen.tuxfamily.org/, unpack and copy Eigen folder to /usr/local/include. Direct link is http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz

    wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
    tar zxf 3.3.4.tar.gz
    sudo cp -R eigen-eigen-5a0156e40feb/Eigen /usr/local/include

Note: Eigen is included to project as git-submodule.

Build uWebSockets:

    make -C uWebSockets
    sudo make install

Use the following commands to build applications:

    mkdir build
    cd build
    cmake .. && make && make test

(`make test` is optional step)

Make sure that the parser works and `ob_pose-laser-radar-synthetic-input.txt` is valid:

    ./testfile ./obj_pose-laser-radar-synthetic-input.txt && echo "OK!"


## Applications

`build` folder contains the following applications that can be used
for experiments:

  * `kf` runs WebSocket-server (port 4567) and can be used as a server for Udacity Term 2 Simulator

  * `objpose` reads measurements from a file and prints RMSE


### kf application

`kf` implements both EKF fusion and UKF fusion.

To run UKF fusion use `kf --ukf` or just `kf`.

To run EKF fusion use `kf --ekf`.


### objpose application

`objpose` can be used to calculate RMSE for EKF or UKF fusion.

Use `objpose --ukf <filename>` to calculate RMSE using UKF fusion.

Use `objpose --ekf <filename>` to calculate RMSE using EKF fusion.

You can calculate RMSE both for UKF and EKF using

    objpose --ukf obj_pose-laser-radar-synthetic-input.txt --ekf obj_pose-laser-radar-synthetic-input.txt

## Code exploration

The main modules are

  * `fusionekf` and `ekf` implement EKF fusion pipeline
  * `fusionukf` and `ukf` implement UKF fusion pipeline

### EKF

#### fusionekf module

  * `FusionEKF::FusionEKF` initializes matrices (F, R_lazer, H_lazer,
    R_radar) and creates kalman filter as well as extended kalman filter.

  * `FusionEKF::Init` initializes kalman filter state using the first measurement

  * `FusionEKF::ProcessMeasurement` implements predict-update step.

#### ekf module

`SimpleKalmanFilter` class is a base class for `LinearKalmanFilter`
and `ExtendedKalmanFilter`.

  * `SimpleKalmanFilter::Predict` implements `predict` step for
    kalman filter and extended kalman filter (there is the same code
    for KF and EKF)

  * `SimpleKalmanFilter::Update` implements common `update` steps.

The difference between `LinearKalmanFilter` and `ExtendedKalmanFilter` is
the way to calculate `H` matrix and `y` vector. See
`LinearKalmanFilter::GetH`, `LinearKalmanFilter::GetY`,
`ExtendedKalmanFilter::GetH`, `ExtendedKalmanFilter::GetY`.


### UKF

UKF fusion pipeline is more compilcated. Please see

  * `ctrv` module implements CTRV-model equations

  * `gaussian` module implements functions for sigma points calculation

     * `CalculateSigmaPoints` calculates sigma points

	 * `AugmentGaussian` builds augmented gaussian using `Q` matrix

	 * `CalculateSigmaWeights` calculates `weights`

	 * `PredictGaussian` "restores" a gaussian by predicted sigma
       points. `PredictGaussian` uses an external function to
       normalize angles.

  * `ukf` module implements two filters: `LazerUKF` and
    `RadarUKF`. Both `LazerUKF` and `RadarUKF` is based on
    `CtrvUnscendedKalmanFilter` class which implementets whole
    `Predict` and `Update` pipeline. `LazerUKF` and `RadarUKF`
    implement additional initialization logic, normalization logic and
    equation `MapXtoZ` `Update` method uses `MapXtoZ` to calculate
    predicted sigma points in measurement space.

### json.hpp

The latest `json.hpp` can be found here - https://github.com/nlohmann/json

## Experiment

    build/objpose --ekf obj_pose-laser-radar-synthetic-input.txt \
	              --ukf obj_pose-laser-radar-synthetic-input.txt

The resulting RMSE is

    EKF: 0.0745332  0.074456  0.265846  0.249984
    UKF: 0.0720093  0.064579  0.182584  0.288182


## Udacity Term 2 Simulator v1.45

The applications were tested with https://github.com/udacity/self-driving-car-sim/releases/download/v1.45/term2_sim_linux.zip

## How to build using AWS Ubuntu 16.04 (t2.micro ami-66506c1c)

    sudo apt-get update
    sudo apt-get install -y cmake g++ make libssl-dev libuv-dev git
    # clone kf
    git clone https://github.com/zasimov/kf
    cd kf && git submodule update --init && ./pre-build.sh
    mkdir build && cd build
    cmake .. && make

`ekf` depends on following libraries:

    linux-vdso.so.1 =>  (0x00007ffd92df2000)
    libuWS.so => /usr/lib/libuWS.so (0x00007f54b8401000)
    libz.so.1 => /lib/x86_64-linux-gnu/libz.so.1 (0x00007f54b81e7000)
    libssl.so.1.0.0 => /lib/x86_64-linux-gnu/libssl.so.1.0.0 (0x00007f54b7f7e000)
    libstdc++.so.6 => /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007f54b7bfc000)
    libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007f54b78f3000)
    libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007f54b76dd000)
    libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007f54b7313000)
    /lib64/ld-linux-x86-64.so.2 (0x00007f54b862e000)
    libcrypto.so.1.0.0 => /lib/x86_64-linux-gnu/libcrypto.so.1.0.0 (0x00007f54b6ecf000)
    libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007f54b6ccb000)

Make sure you have the same versions at your local machine.
