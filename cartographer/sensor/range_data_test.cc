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
#include <vector>
#include <typeinfo>
#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::FloatNear;
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
  void RangeDataToProtoTest() {}

  void CompressedRangeDataToProtoTest() {
    const CompressedPointCloud compressed_returns(returns_);
    const CompressedPointCloud compressed_misses(misses_);
    const CompressedRangeData compressed_range_data = {
        origin_, compressed_returns, compressed_misses
    };
    const auto proto = ToProto(compressed_range_data);
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
  EXPECT_TRUE(actual.misses[0].isApprox(misses_[0], 0.001f));

  // Returns may be reordered, so we compare in an unordered manner.
  for (uint i = 0; i < returns_.size(); ++i) {
    EXPECT_THAT(actual.returns, Contains(ApproximatelyEquals(returns_[i])));
  }
  for (uint i = 0; i < misses_.size(); ++i) {
    EXPECT_THAT(actual.misses, Contains(ApproximatelyEquals(misses_[i])));
  }
}

TEST_F(RangeDataTest, RangeDataToProto) {
  const auto proto = ToProto(RangeData{origin_, returns_, misses_});

  EXPECT_TRUE(proto.has_origin());
  EXPECT_TRUE(proto.has_returns());
  EXPECT_TRUE(proto.has_misses());
  EXPECT_EQ(returns_.size(), proto.returns().x_size());
  EXPECT_EQ(returns_.size(), proto.returns().y_size());
  EXPECT_EQ(returns_.size(), proto.returns().z_size());
  EXPECT_EQ(misses_.size(), proto.misses().x_size());
  EXPECT_EQ(misses_.size(), proto.misses().y_size());
  EXPECT_EQ(misses_.size(), proto.misses().z_size());
  EXPECT_NEAR(proto.origin().x(), origin_.x(), kPrecision);
  EXPECT_NEAR(proto.origin().y(), origin_.y(), kPrecision);
  EXPECT_NEAR(proto.origin().z(), origin_.z(), kPrecision);

  for (int i = 0; i < proto.returns().x_size(); ++i) {
    EXPECT_NEAR(proto.returns().x(i), returns_[i].x(), kPrecision);
    EXPECT_NEAR(proto.returns().y(i), returns_[i].y(), kPrecision);
    EXPECT_NEAR(proto.returns().z(i), returns_[i].z(), kPrecision);
  }
  for (int i = 0; i < proto.misses().x_size(); ++i) {
    EXPECT_NEAR(proto.misses().x(i), misses_[i].x(), kPrecision);
    EXPECT_NEAR(proto.misses().y(i), misses_[i].y(), kPrecision);
    EXPECT_NEAR(proto.misses().z(i), misses_[i].z(), kPrecision);
  }
}

TEST_F(RangeDataTest, RangeDataFromProto) {
  const auto range_data =
      FromProto(ToProto(RangeData{origin_, returns_, misses_}));

  EXPECT_NEAR(range_data.origin.x(), origin_.x(), kPrecision);
  EXPECT_NEAR(range_data.origin.y(), origin_.y(), kPrecision);
  EXPECT_NEAR(range_data.origin.z(), origin_.z(), kPrecision);
  EXPECT_EQ(returns_.size(), range_data.returns.size());
  EXPECT_EQ(misses_.size(), range_data.misses.size());

  for (int i = 0; i < range_data.returns.size(); ++i) {
    EXPECT_NEAR(range_data.returns[i].x(), returns_[i].x(), kPrecision);
    EXPECT_NEAR(range_data.returns[i].y(), returns_[i].y(), kPrecision);
    EXPECT_NEAR(range_data.returns[i].z(), returns_[i].z(), kPrecision);
  }
  for (int i = 0; i < range_data.misses.size(); ++i) {
    EXPECT_NEAR(range_data.misses[i].x(), misses_[i].x(), kPrecision);
    EXPECT_NEAR(range_data.misses[i].y(), misses_[i].y(), kPrecision);
    EXPECT_NEAR(range_data.misses[i].z(), misses_[i].z(), kPrecision);
  }
}

TEST_F(RangeDataTest, CompressedRangeDataToProto) {
  const auto proto = ToProto(CompressedRangeData{
      origin_, CompressedPointCloud(returns_), CompressedPointCloud(misses_)});

  EXPECT_TRUE(proto.has_origin());
  EXPECT_TRUE(proto.has_returns());
  EXPECT_TRUE(proto.has_misses());
  EXPECT_NEAR(proto.origin().x(), origin_.x(), kPrecision);
  EXPECT_NEAR(proto.origin().y(), origin_.y(), kPrecision);
  EXPECT_NEAR(proto.origin().z(), origin_.z(), kPrecision);
}

TEST_F(RangeDataTest, CompressedRangeDataFromProto) {
  const auto compressed_range_data = FromProto(ToProto(CompressedRangeData{
      origin_, CompressedPointCloud(returns_), CompressedPointCloud(misses_)}));

  EXPECT_NEAR(compressed_range_data.origin.x(), origin_.x(), kPrecision);
  EXPECT_NEAR(compressed_range_data.origin.y(), origin_.y(), kPrecision);
  EXPECT_NEAR(compressed_range_data.origin.z(), origin_.z(), kPrecision);
  EXPECT_EQ(returns_.size(), compressed_range_data.returns.size());
  EXPECT_EQ(misses_.size(), compressed_range_data.misses.size());
}
}  // namespace
}  // namespace sensor
}  // namespace cartographer
