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

#include "local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

LocalSlamResultData::LocalSlamResultData(const std::string& sensor_id,
                                         common::Time time)
    : Data(sensor_id), time_(time) {}

LocalSlamResult2D::LocalSlamResult2D(
    const std::string& sensor_id, common::Time time,
    std::shared_ptr<const mapping::TrajectoryNode::Data> node_data,
    const std::vector<std::shared_ptr<const mapping_2d::Submap>>&
        insertion_submaps)
    : LocalSlamResultData(sensor_id, time),
      node_data_(node_data),
      insertion_submaps_(insertion_submaps) {}

void LocalSlamResult2D::AddToTrajectoryBuilder(
    mapping::TrajectoryBuilderInterface* const trajectory_builder) {
  trajectory_builder->AddLocalSlamResultData(
      common::make_unique<LocalSlamResult2D>(*this));
}

void LocalSlamResult2D::AddToPoseGraph(int trajectory_id,
                                       mapping::PoseGraph* pose_graph) const {
  DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph));
  PoseGraph2D* pose_graph_2d = static_cast<PoseGraph2D*>(pose_graph);
  pose_graph_2d->AddNode(node_data_, trajectory_id, insertion_submaps_);
}

LocalSlamResult3D::LocalSlamResult3D(
    const std::string& sensor_id, common::Time time,
    std::shared_ptr<const mapping::TrajectoryNode::Data> node_data,
    const std::vector<std::shared_ptr<const mapping_3d::Submap>>&
        insertion_submaps)
    : LocalSlamResultData(sensor_id, time),
      node_data_(node_data),
      insertion_submaps_(insertion_submaps) {}

void LocalSlamResult3D::AddToTrajectoryBuilder(
    mapping::TrajectoryBuilderInterface* const trajectory_builder) {
  trajectory_builder->AddLocalSlamResultData(
      common::make_unique<LocalSlamResult3D>(*this));
}

void LocalSlamResult3D::AddToPoseGraph(int trajectory_id,
                                       mapping::PoseGraph* pose_graph) const {
  DCHECK(dynamic_cast<mapping_3d::PoseGraph*>(pose_graph));
  mapping_3d::PoseGraph* pose_graph_3d =
      static_cast<mapping_3d::PoseGraph*>(pose_graph);
  pose_graph_3d->AddNode(node_data_, trajectory_id, insertion_submaps_);
}

}  // namespace mapping
}  // namespace cartographer
