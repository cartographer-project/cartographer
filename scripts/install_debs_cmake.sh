#!/bin/bash

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

# Install the required libraries that are available as debs.
sudo apt-get update

# Install CMake 3.2 for Debian Jessie.
sudo apt-get install lsb-release -y
if [[ "$(lsb_release -sc)" = "jessie" ]]
then
  sudo sh -c "echo 'deb [check-valid-until=no] http://archive.debian.org/debian jessie-backports main' >> /etc/apt/sources.list"
  sudo sh -c "echo 'Acquire::Check-Valid-Until \"false\";' >> /etc/apt/apt.conf"
  sudo apt-get update
  sudo apt-get -t jessie-backports install cmake -y
else
  sudo apt-get install cmake -y
fi

sudo apt-get install -y \
    clang \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    ninja-build

if [[ "$(lsb_release -sc)" = "focal" ]]
then
    sudo apt-get install -y python3-sphinx libgmock-dev
else
    sudo apt-get install -y python-sphinx
fi

# Install Ceres Solver on Ubuntu Bionic. No need to build it ourselves.
if [[ "$(lsb_release -sc)" = "bionic" ]]
then
  sudo apt-get install -y libceres-dev
fi

# Install Ceres Solver and Protocol Buffers support on Ubuntu Focal. No need to build it ourselves.
if [[ "$(lsb_release -sc)" = "focal" ]]
then
  sudo apt-get install -y libceres-dev protobuf-compiler
fi
