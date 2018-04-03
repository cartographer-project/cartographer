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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_SLAM_RESULT_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_SLAM_RESULT_2D_H_

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

class LocalSlamResult2D : public LocalSlamResultData {
 public:
  LocalSlamResult2D(
      const std::string& sensor_id, common::Time time,
      std::shared_ptr<const TrajectoryNode::Data> node_data,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      : LocalSlamResultData(sensor_id, time),
        node_data_(node_data),
        insertion_submaps_(insertion_submaps) {}

  void AddToTrajectoryBuilder(
      TrajectoryBuilderInterface* const trajectory_builder) override;
  void AddToPoseGraph(int trajectory_id, PoseGraph* pose_graph) const override;

 private:
  std::shared_ptr<const TrajectoryNode::Data> node_data_;
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_SLAM_RESULT_2D_H_
