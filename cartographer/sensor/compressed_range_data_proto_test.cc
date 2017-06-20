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
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cartographer/common/port.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(CompressedRangeDataProtoTest, ToProto) {
  auto origin = Eigen::Vector3f(0.11f, 0.22f, 0.33f);
  PointCloud returns;
  PointCloud misses;

  for (int i = 0; i < 10; ++i) {
    returns.emplace_back(Eigen::Vector3f(0.0f, 0.0f, 1.0f / 10.0f));
    misses.emplace_back(Eigen::Vector3f(1.0f / 10.0f, 0.0f, 0.0f));
  }

  CompressedPointCloud compressed_returns(returns);
  CompressedPointCloud compressed_misses(misses);
  const CompressedRangeData compressed_range_data = {origin, compressed_returns,
                                                     compressed_misses};
  const auto proto = ToProto(compressed_range_data);

  EXPECT_EQ(proto.returns().num_points(), 10);
  EXPECT_EQ(proto.misses().num_points(), 10);

  const CompressedRangeData from_proto = FromProto(proto);
  const auto decompressed_returns = from_proto.returns.Decompress();
  const auto decompressed_misses = from_proto.misses.Decompress();

  EXPECT_EQ(proto.origin().x(), from_proto.origin.x());
  EXPECT_EQ(proto.origin().y(), from_proto.origin.y());
  EXPECT_EQ(proto.origin().z(), from_proto.origin.z());

  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(returns[0], Eigen::Vector3f(0.0f, 0.0f, 1.0f / 10.0f));
    EXPECT_EQ(misses[0], Eigen::Vector3f(1.0f / 10.0f, 0.0f, 0.0f));
    EXPECT_EQ(decompressed_returns[i].x(), returns[i].x());
    EXPECT_EQ(decompressed_misses[i].x(), misses[i].x());
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
