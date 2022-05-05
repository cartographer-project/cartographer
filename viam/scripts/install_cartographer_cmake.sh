#!/bin/sh
set -o errexit

# Build and install Cartographer.
pushd cartographer

rm -rf build
mkdir build
pushd build

cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH=`brew --prefix` -DQt5_DIR=$(brew --prefix qt5)/lib/cmake/Qt5
ninja
sudo ninja install
popd
popd