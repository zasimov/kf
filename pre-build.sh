#!/bin/bash

NUM_CPU=$(cat /proc/cpuinfo | grep "^processor" | wc -l)

# git submodule update --init

# Build googletest
mkdir build-googletest
pushd build-googletest
cmake ../googletest
make -j${NUM_CPU}
popd

# Build uWebSockets
make -C uWebSockets -j${NUM_CPU}

echo
echo "***************************************************************"
echo "to finish installation you should run these commands from root:"
echo "***************************************************************"
echo "make -C build-googletest install | tee googletest.log"
echo "make -C uWebSockets install | tee uWebSockets.log"
echo "cp -r eigen-git-mirror/Eigen /usr/local/include"
