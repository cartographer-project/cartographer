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

#include "cartographer/mapping_3d/submaps.h"

#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_3d {
namespace {

TEST(SubmapsTest, ToFromProto) {
  const Submap expected(0.05, 0.25,
                        transform::Rigid3d(Eigen::Vector3d(1., 2., 0.),
                                           Eigen::Quaterniond(0., 0., 0., 1.)));
  mapping::proto::Submap proto;
  expected.ToProto(&proto);
  EXPECT_FALSE(proto.has_submap_2d());
  EXPECT_TRUE(proto.has_submap_3d());
  const auto actual = Submap(proto.submap_3d());
  EXPECT_TRUE(expected.local_pose().translation().isApprox(
      actual.local_pose().translation(), 1e-6));
  EXPECT_TRUE(expected.local_pose().rotation().isApprox(
      actual.local_pose().rotation(), 1e-6));
  EXPECT_EQ(expected.num_range_data(), actual.num_range_data());
  EXPECT_EQ(expected.finished(), actual.finished());
  EXPECT_NEAR(expected.high_resolution_hybrid_grid().resolution(), 0.05, 1e-6);
  EXPECT_NEAR(expected.low_resolution_hybrid_grid().resolution(), 0.25, 1e-6);
}

}  // namespace
}  // namespace mapping_3d
}  // namespace cartographer
