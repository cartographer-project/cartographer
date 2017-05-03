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

#include <utility>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::PrintToString;

// Custom matcher for Eigen::Vector3f entries.
MATCHER_P(ApproximatelyEquals, expected,
          string("is equal to ") + PrintToString(expected)) {
  return (arg - expected).isZero(0.001f);
}

TEST(RangeDataTest, Compression) {
  const std::vector<Eigen::Vector3f> returns = {Eigen::Vector3f(0, 1, 2),
                                                Eigen::Vector3f(4, 5, 6),
                                                Eigen::Vector3f(0, 1, 2)};
  const RangeData range_data = {
      Eigen::Vector3f(1, 1, 1), returns, {Eigen::Vector3f(7, 8, 9)}};
  const RangeData actual = Decompress(Compress(range_data));
  EXPECT_TRUE(actual.origin.isApprox(Eigen::Vector3f(1, 1, 1), 1e-6));
  EXPECT_EQ(3, actual.returns.size());
  EXPECT_EQ(1, actual.misses.size());
  EXPECT_TRUE(actual.misses[0].isApprox(Eigen::Vector3f(7, 8, 9), 0.001f));

  // Returns will be reordered, so we compare in an unordered manner.
  EXPECT_EQ(3, actual.returns.size());
  EXPECT_THAT(actual.returns,
              Contains(ApproximatelyEquals(Eigen::Vector3f(0, 1, 2))));
  EXPECT_THAT(actual.returns,
              Contains(ApproximatelyEquals(Eigen::Vector3f(4, 5, 6))));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
