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
sudo apt-get install -y \
    clang \
    cmake \
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
    lsb-release \
    ninja-build \
    stow

# Install Ceres Solver and Protocol Buffers support if available.
# No need to build it ourselves.
if [[ "$(lsb_release -sc)" = "focal" || "$(lsb_release -sc)" = "buster" ]]
then
  sudo apt-get install -y python3-sphinx libgmock-dev libceres-dev protobuf-compiler
else
  sudo apt-get install -y python-sphinx
  if [[ "$(lsb_release -sc)" = "bionic" ]]
  then
    sudo apt-get install -y libceres-dev
  fi
fi


