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

#include "cartographer/mapping/internal/3d/local_slam_result_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"

namespace cartographer {
namespace mapping {

void LocalSlamResult3D::AddToTrajectoryBuilder(
    TrajectoryBuilderInterface* const trajectory_builder) {
  trajectory_builder->AddLocalSlamResultData(
      common::make_unique<LocalSlamResult3D>(*this));
}

void LocalSlamResult3D::AddToPoseGraph(int trajectory_id,
                                       PoseGraph* pose_graph) const {
  DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph));
  static_cast<PoseGraph3D*>(pose_graph)
      ->AddNode(node_data_, trajectory_id, insertion_submaps_);
}

}  // namespace mapping
}  // namespace cartographer
