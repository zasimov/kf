#!/bin/bash

if [ -f /proc/cpuinfo ]; then
    NUM_CPU=$(cat /proc/cpuinfo | grep "^processor" | wc -l)
else
    NUM_CPU=1
fi

git submodule update --init

# Build googletest
if [ "$WITH_TESTS" == "yes" ]; then
    mkdir build-googletest
    pushd build-googletest
    cmake ../googletest
    make -j${NUM_CPU}
    popd
fi

# Build uWebSockets
make -C uWebSockets -j${NUM_CPU}

mkdir -p include/uWS
cp uWebSockets/src/*.h include/uWS

