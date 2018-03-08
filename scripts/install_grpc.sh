#!/bin/sh

# Copyright 2017 The Cartographer Authors
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

VERSION="v1.10.0"
# Digest: 474c5950686e3962bd339c93d27e369bf64f568f

# Build and install gRPC.
git clone --branch ${VERSION} --depth 1 https://github.com/grpc/grpc
cd grpc
git submodule update --init
make
sudo make install
