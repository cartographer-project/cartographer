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

#include "cartographer/mapping/internal/2d/local_slam_result_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"

namespace cartographer {
namespace mapping {

void LocalSlamResult2D::AddToTrajectoryBuilder(
    TrajectoryBuilderInterface* const trajectory_builder) {
  trajectory_builder->AddLocalSlamResultData(
      common::make_unique<LocalSlamResult2D>(*this));
}

void LocalSlamResult2D::AddToPoseGraph(int trajectory_id,
                                       PoseGraph* pose_graph) const {
  DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph));
  static_cast<PoseGraph2D*>(pose_graph)
      ->AddNode(node_data_, trajectory_id, insertion_submaps_);
}

}  // namespace mapping
}  // namespace cartographer
