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
using ::testing::PrintToString;

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

TEST_F(RangeDataTest, Compression) {
  const RangeData expected_data = {origin_, returns_, misses_};
  const RangeData actual_data = Decompress(Compress(expected_data));
  EXPECT_THAT(expected_data.origin, Near(actual_data.origin));
  EXPECT_EQ(3, actual_data.returns.size());
  EXPECT_EQ(1, actual_data.misses.size());

  // Returns may be reordered, so we compare in an unordered manner.
  for (const auto& expected : expected_data.returns) {
    EXPECT_THAT(actual_data.returns, Contains(Near(expected)));
  }
  for (const auto& expected : expected_data.misses) {
    EXPECT_THAT(actual_data.misses, Contains(Near(expected)));
  }
}

TEST_F(RangeDataTest, CompressedRangeDataToAndFromProto) {
  const auto expected = CompressedRangeData{
      origin_, CompressedPointCloud(returns_), CompressedPointCloud(misses_)};
  const auto actual = FromProto(ToProto(expected));
  EXPECT_THAT(expected.origin, Near(actual.origin));
  EXPECT_EQ(expected.returns, actual.returns);
  EXPECT_EQ(expected.misses, actual.misses);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
