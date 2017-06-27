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

#include <typeinfo>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::PrintToString;
constexpr float kPrecision = 0.001f;

// TODO (brandon-northcutt): place this function in a common location with
// identical function in compressed_point_cloud_test.cc
MATCHER_P(ApproximatelyEquals, expected,
          string("is equal to ") + PrintToString(expected)) {
  return (arg - expected).isZero(kPrecision);
}

class RangeDataTest : public ::testing::Test {
 protected:
  RangeDataTest() {
    origin_ = Eigen::Vector3f(1, 1, 1);
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
  const RangeData range_data = {origin_, returns_, misses_};
  const RangeData actual = Decompress(Compress(range_data));
  EXPECT_TRUE(actual.origin.isApprox(origin_, 1e-6));
  EXPECT_EQ(3, actual.returns.size());
  EXPECT_EQ(1, actual.misses.size());

  // Returns may be reordered, so we compare in an unordered manner.
  for (const auto& expected : returns_) {
    EXPECT_THAT(actual.returns, Contains(ApproximatelyEquals(expected)));
  }
  for (const auto& expected : misses_) {
    EXPECT_THAT(actual.misses, Contains(ApproximatelyEquals(expected)));
  }
}

TEST_F(RangeDataTest, RangeDataToAndFromProto) {
  const auto expected = RangeData{origin_, returns_, misses_};
  const auto actual = FromProto(ToProto(expected));

  EXPECT_TRUE(typeid(expected) == typeid(actual));
  EXPECT_TRUE(expected.origin.isApprox(actual.origin, kPrecision));

  for (uint i = 0; i < expected.returns.size(); ++i) {
    EXPECT_TRUE(
        expected.returns[i].isApprox(actual.returns[i], kPrecision));
  }
  for (uint i = 0; i < expected.misses.size(); ++i) {
    EXPECT_TRUE(
        expected.misses[i].isApprox(actual.misses[i], kPrecision));
  }
}

TEST_F(RangeDataTest, CompressedRangeDataToAndFromProto) {
  const auto expected = CompressedRangeData{
      origin_, CompressedPointCloud(returns_), CompressedPointCloud(misses_)};
  const auto actual = FromProto(ToProto(expected));

  EXPECT_TRUE(typeid(expected) == typeid(actual));
  EXPECT_TRUE(expected.origin.isApprox(actual.origin, kPrecision));

  EXPECT_EQ(expected.returns.size(), actual.returns.size());
  EXPECT_TRUE(expected.returns.empty() == actual.returns.empty());
  EXPECT_EQ(expected.misses.size(), actual.misses.size());
  EXPECT_TRUE(expected.misses.empty() == actual.misses.empty());

  for (uint i = 0; i < expected.returns.size(); ++i) {
    EXPECT_EQ(expected.returns.point_data()[i], actual.returns.point_data()[i]);
  }
}
}  // namespace
}  // namespace sensor
}  // namespace cartographer
