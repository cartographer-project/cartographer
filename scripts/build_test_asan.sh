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

# Installing the following packages may be required:
# sudo apt-get install clang-3.8 llvm-3.8

cd cartographer
mkdir -p build-asan
cd build-asan
cmake .. -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DFORCE_DEBUG_BUILD=True \
  -DCMAKE_CXXFLAGS="-fno-omit-frame-pointer -fsanitize=address \
    -fsanitize-address-use-after-scope -O1" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address" \
  -DCMAKE_C_COMPILER=clang-3.8 \
  -DCMAKE_CXX_COMPILER=clang++-3.8
ninja
PATH="/usr/lib/llvm-3.8/bin/:$PATH" \
  ASAN_OPTIONS="detect_leaks=1 detect_stack_use_after_return=true" \
  ctest --output-on-failure -j20 --timeout 60 \
    --test-arguments="--gtest_color=yes"
