# Cartographer Project Overview

Cartographer is a system that provides real-time simultaneous localization and
mapping
([SLAM](http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping))
across multiple platforms and sensor configurations.

## Installation with ROS

See https://github.com/googlecartographer/cartographer_ros

## Installation without ROS

On Ubuntu 14.04 (Trusty) or Ubuntu 16.04 (Xenial):

    # Install the required libraries that are available as debs
    sudo apt-get install \
      g++ \
      google-mock \
      libboost-all-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      liblua5.2-dev \
      libprotobuf-dev \
      libsuitesparse-dev \
      libwebp-dev \
      ninja-build \
      protobuf-compiler \
      python-sphinx

    # Build and install Ceres
    git clone https://ceres-solver.googlesource.com/ceres-solver
    cd ceres-solver
    mkdir build && cd build
    cmake ..
    make
    sudo make install

    # Build Cartographer
    cd cartographer
    mkdir build && cd build
    cmake .. -G Ninja
    ninja
