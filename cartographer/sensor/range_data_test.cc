/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/sensor/range_data.h"

#include <tuple>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;

MATCHER(NearPointwise, std::string(negation ? "Doesn't" : "Does") + " match.") {
  return std::get<0>(arg).isApprox(std::get<1>(arg), 0.001f);
}

MATCHER_P(Near, point, std::string(negation ? "Doesn't" : "Does") + " match.") {
  return arg.isApprox(point, 0.001f);
}

class RangeDataTest : public ::testing::Test {
 protected:
  RangeDataTest() : origin_(Eigen::Vector3f(1, 1, 1)) {
    returns_.emplace_back(0, 1, 2);
    returns_.emplace_back(4, 5, 6);
    returns_.emplace_back(0, 1, 2);
    misses_.emplace_back(7, 8, 9);
  }
  Eigen::Vector3f origin_;
  std::vector<Eigen::Vector3f> returns_;
  std::vector<Eigen::Vector3f> misses_;
};

}  // namespace
}  // namespace sensor
}  // namespace cartographer
