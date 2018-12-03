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

FROM ubuntu:trusty

ARG cc
ARG cxx

# Set the preferred C/C++ compiler toolchain, if given (otherwise default).
ENV CC=$cc
ENV CXX=$cxx

COPY scripts/install_debs_cmake.sh cartographer/scripts/
RUN cartographer/scripts/install_debs_cmake.sh && rm -rf /var/lib/apt/lists/*
COPY scripts/install_ceres.sh cartographer/scripts/
RUN cartographer/scripts/install_ceres.sh && rm -rf ceres-solver
COPY scripts/install_proto3.sh cartographer/scripts/
RUN cartographer/scripts/install_proto3.sh && rm -rf protobuf
COPY scripts/install_grpc.sh cartographer/scripts/
RUN cartographer/scripts/install_grpc.sh && rm -rf grpc
COPY scripts/install_async_grpc.sh cartographer/scripts/
RUN cartographer/scripts/install_async_grpc.sh && rm -rf async_grpc
COPY scripts/install_prometheus_cpp.sh cartographer/scripts/
RUN cartographer/scripts/install_prometheus_cpp.sh && rm -rf prometheus-cpp
COPY . cartographer
RUN cartographer/scripts/install_cartographer_cmake_with_grpc.sh && rm -rf cartographer
