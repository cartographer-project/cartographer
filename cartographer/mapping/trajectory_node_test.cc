/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/trajectory_node.h"

#include <limits>

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_node_data.pb.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(TrajectoryNodeTest, ToAndFromProto) {
  const TrajectoryNode::Data expected{
      common::FromUniversal(42),
      Eigen::Quaterniond(1., 2., -3., -4.),
      sensor::CompressedPointCloud({{1.f, 2.f, 0.f}, {0.f, 0.f, 1.f}})
          .Decompress(),
      sensor::CompressedPointCloud({{2.f, 3.f, 4.f}}).Decompress(),
      sensor::CompressedPointCloud({{-1.f, 2.f, 0.f}}).Decompress(),
      Eigen::VectorXf::Unit(20, 4),
      transform::Rigid3d({1., 2., 3.},
                         Eigen::Quaterniond(4., 5., -6., -7.).normalized())};
  const proto::TrajectoryNodeData proto = ToProto(expected);
  const TrajectoryNode::Data actual = FromProto(proto);
  EXPECT_EQ(expected.time, actual.time);
  EXPECT_TRUE(actual.gravity_alignment.isApprox(expected.gravity_alignment));
  EXPECT_EQ(expected.filtered_gravity_aligned_point_cloud,
            actual.filtered_gravity_aligned_point_cloud);
  EXPECT_EQ(expected.high_resolution_point_cloud,
            actual.high_resolution_point_cloud);
  EXPECT_EQ(expected.low_resolution_point_cloud,
            actual.low_resolution_point_cloud);
  EXPECT_EQ(expected.rotational_scan_matcher_histogram,
            actual.rotational_scan_matcher_histogram);
  EXPECT_THAT(actual.local_pose,
              transform::IsNearly(expected.local_pose, 1e-9));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
