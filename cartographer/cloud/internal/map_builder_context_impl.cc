/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/cloud/internal/map_builder_server.h"

#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/mapping/internal/2d/local_slam_result_2d.h"
#include "cartographer/mapping/internal/3d/local_slam_result_3d.h"

namespace cartographer {
namespace cloud {

template <>
std::unique_ptr<mapping::LocalSlamResultData>
MapBuilderContext<mapping::Submap2D>::ProcessLocalSlamResultData(
    const std::string& sensor_id, common::Time time,
    const mapping::proto::LocalSlamResultData& proto) {
  CHECK_GE(proto.submaps().size(), 1);
  CHECK(proto.submaps(0).has_submap_2d());
  std::vector<std::shared_ptr<const mapping::Submap2D>> submaps;
  for (const auto& submap_proto : proto.submaps()) {
    submaps.push_back(submap_controller_.UpdateSubmap(submap_proto));
  }
  return common::make_unique<mapping::LocalSlamResult2D>(
      sensor_id, time,
      std::make_shared<const mapping::TrajectoryNode::Data>(
          mapping::FromProto(proto.node_data())),
      submaps);
}

template <>
std::unique_ptr<mapping::LocalSlamResultData>
MapBuilderContext<mapping::Submap3D>::ProcessLocalSlamResultData(
    const std::string& sensor_id, common::Time time,
    const mapping::proto::LocalSlamResultData& proto) {
  CHECK_GE(proto.submaps().size(), 1);
  CHECK(proto.submaps(0).has_submap_3d());
  std::vector<std::shared_ptr<const mapping::Submap3D>> submaps;
  for (const auto& submap_proto : proto.submaps()) {
    submaps.push_back(submap_controller_.UpdateSubmap(submap_proto));
  }
  return common::make_unique<mapping::LocalSlamResult3D>(
      sensor_id, time,
      std::make_shared<const mapping::TrajectoryNode::Data>(
          mapping::FromProto(proto.node_data())),
      submaps);
}

}  // namespace cloud
}  // namespace cartographer
