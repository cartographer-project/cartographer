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

#include "cartographer/sensor/compressed_point_cloud.h"

#include "gmock/gmock.h"

namespace Eigen {

// Prints Vector3f in a readable format in matcher ApproximatelyEquals when
// failing a test. Without this function, the output is formated as hexadecimal
// 8 bit numbers.
void PrintTo(const Vector3f& x, std::ostream* os) {
  *os << "(" << x[0] << ", " << x[1] << ", " << x[2] << ")";
}

}  // namespace Eigen

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::FloatNear;
using ::testing::PrintToString;

constexpr float kPrecision = 0.001f;

// Matcher for 3-d vectors w.r.t. to the target precision.
MATCHER_P(ApproximatelyEquals, expected,
          std::string("is equal to ") + PrintToString(expected)) {
  return (arg - expected).isZero(kPrecision);
}

// Helper function to test the mapping of a single point. Includes test for
// recompressing the same point again.
void TestPoint(const Eigen::Vector3f& p) {
  CompressedPointCloud compressed({p});
  EXPECT_EQ(1, compressed.size());
  EXPECT_THAT(*compressed.begin(), ApproximatelyEquals(p));
  CompressedPointCloud recompressed({*compressed.begin()});
  EXPECT_THAT(*recompressed.begin(), ApproximatelyEquals(p));
}

TEST(CompressPointCloudTest, CompressesPointsCorrectly) {
  TestPoint(Eigen::Vector3f(8000.f, 7500.f, 5000.f));
  TestPoint(Eigen::Vector3f(1000.f, 2000.f, 3000.f));
  TestPoint(Eigen::Vector3f(100.f, 200.f, 300.f));
  TestPoint(Eigen::Vector3f(10.f, 20.f, 30.f));
  TestPoint(Eigen::Vector3f(-0.00049f, -0.0005f, -0.0015f));
  TestPoint(Eigen::Vector3f(0.05119f, 0.0512f, 0.05121));
  TestPoint(Eigen::Vector3f(-0.05119f, -0.0512f, -0.05121));
  TestPoint(Eigen::Vector3f(0.8405f, 0.84f, 0.8396f));
  TestPoint(Eigen::Vector3f(0.8395f, 0.8394f, 0.8393f));
  TestPoint(Eigen::Vector3f(0.839f, 0.8391f, 0.8392f));
  TestPoint(Eigen::Vector3f(0.8389f, 0.8388f, 0.83985f));
}

TEST(CompressPointCloudTest, Compresses) {
  const CompressedPointCloud compressed({Eigen::Vector3f(0.838f, 0, 0),
                                         Eigen::Vector3f(0.839f, 0, 0),
                                         Eigen::Vector3f(0.840f, 0, 0)});
  EXPECT_FALSE(compressed.empty());
  EXPECT_EQ(3, compressed.size());
  const PointCloud decompressed = compressed.Decompress();
  EXPECT_EQ(3, decompressed.size());
  EXPECT_THAT(decompressed,
              Contains(ApproximatelyEquals(Eigen::Vector3f(0.838f, 0, 0))));
  EXPECT_THAT(decompressed,
              Contains(ApproximatelyEquals(Eigen::Vector3f(0.839f, 0, 0))));
  EXPECT_THAT(decompressed,
              Contains(ApproximatelyEquals(Eigen::Vector3f(0.840f, 0, 0))));
}

TEST(CompressPointCloudTest, CompressesEmptyPointCloud) {
  CompressedPointCloud compressed;
  EXPECT_TRUE(compressed.empty());
  EXPECT_EQ(0, compressed.size());
}

// Test for gaps.
// Produces a series of points densly packed along the x axis, compresses these
// points (twice), and tests, whether there are gaps between two consecutive
// points.
TEST(CompressPointCloudTest, CompressesNoGaps) {
  PointCloud point_cloud;
  for (int i = 0; i < 3000; ++i) {
    point_cloud.push_back(Eigen::Vector3f(kPrecision * i - 1.5f, 0, 0));
  }
  const CompressedPointCloud compressed(point_cloud);
  const PointCloud decompressed = compressed.Decompress();
  const CompressedPointCloud recompressed(decompressed);
  EXPECT_EQ(decompressed.size(), recompressed.size());

  std::vector<float> x_coord;
  for (const Eigen::Vector3f& p : compressed) {
    x_coord.push_back(p[0]);
  }
  std::sort(x_coord.begin(), x_coord.end());
  for (size_t i = 1; i < x_coord.size(); ++i) {
    EXPECT_THAT(std::abs(x_coord[i] - x_coord[i - 1]),
                FloatNear(kPrecision, 1e-7f));
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
