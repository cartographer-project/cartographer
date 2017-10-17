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

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data) {
  proto::TrajectoryNodeData proto;
  proto.set_timestamp(common::ToUniversal(constant_data.time));
  *proto.mutable_gravity_alignment() =
      transform::ToProto(constant_data.gravity_alignment);
  *proto.mutable_filtered_gravity_aligned_point_cloud() =
      sensor::CompressedPointCloud(
          constant_data.filtered_gravity_aligned_point_cloud)
          .ToProto();
  *proto.mutable_high_resolution_point_cloud() =
      sensor::CompressedPointCloud(constant_data.high_resolution_point_cloud)
          .ToProto();
  *proto.mutable_low_resolution_point_cloud() =
      sensor::CompressedPointCloud(constant_data.low_resolution_point_cloud)
          .ToProto();
  for (Eigen::VectorXf::Index i = 0;
       i != constant_data.rotational_scan_matcher_histogram.size(); ++i) {
    proto.add_rotational_scan_matcher_histogram(
        constant_data.rotational_scan_matcher_histogram(i));
  }
  *proto.mutable_local_pose() = transform::ToProto(constant_data.local_pose);
  return proto;
}

TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto) {
  Eigen::VectorXf rotational_scan_matcher_histogram(
      proto.rotational_scan_matcher_histogram_size());
  for (int i = 0; i != proto.rotational_scan_matcher_histogram_size(); ++i) {
    rotational_scan_matcher_histogram(i) =
        proto.rotational_scan_matcher_histogram(i);
  }
  return TrajectoryNode::Data{
      common::FromUniversal(proto.timestamp()),
      transform::ToEigen(proto.gravity_alignment()),
      sensor::CompressedPointCloud(proto.filtered_gravity_aligned_point_cloud())
          .Decompress(),
      sensor::CompressedPointCloud(proto.high_resolution_point_cloud())
          .Decompress(),
      sensor::CompressedPointCloud(proto.low_resolution_point_cloud())
          .Decompress(),
      rotational_scan_matcher_histogram,
      transform::ToRigid3(proto.local_pose())};
}

}  // namespace mapping
}  // namespace cartographer
