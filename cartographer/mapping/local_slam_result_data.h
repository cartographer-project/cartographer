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

#ifndef CARTOGRAPHER_MAPPING_LOCAL_SLAM_RESULT_DATA_H
#define CARTOGRAPHER_MAPPING_LOCAL_SLAM_RESULT_DATA_H

#include "cartographer/mapping_2d/pose_graph_2d.h"
#include "cartographer/mapping_3d/pose_graph.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace mapping {

class TrajectoryBuilderInterface;
class LocalSlamResultData : public cartographer::sensor::Data {
 public:
  LocalSlamResultData(const std::string& sensor_id, common::Time time);

  common::Time GetTime() const override { return time_; }
  virtual void AddToPoseGraph(int trajectory_id,
                              mapping::PoseGraph* pose_graph) const = 0;

 private:
  common::Time time_;
};

class LocalSlamResult2D : public LocalSlamResultData {
 public:
  LocalSlamResult2D(
      const std::string& sensor_id, common::Time time,
      std::shared_ptr<const mapping::TrajectoryNode::Data> node_data,
      const std::vector<std::shared_ptr<const mapping_2d::Submap>>&
          insertion_submaps);

  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface* const trajectory_builder) override;
  void AddToPoseGraph(int trajectory_id,
                      mapping::PoseGraph* pose_graph) const override;

 private:
  std::shared_ptr<const mapping::TrajectoryNode::Data> node_data_;
  std::vector<std::shared_ptr<const mapping_2d::Submap>> insertion_submaps_;
};

class LocalSlamResult3D : public LocalSlamResultData {
 public:
  LocalSlamResult3D(
      const std::string& sensor_id, common::Time time,
      std::shared_ptr<const mapping::TrajectoryNode::Data> node_data,
      const std::vector<std::shared_ptr<const mapping_3d::Submap>>&
          insertion_submaps);

  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface* const trajectory_builder) override;
  void AddToPoseGraph(int trajectory_id,
                      mapping::PoseGraph* pose_graph) const override;

 private:
  std::shared_ptr<const mapping::TrajectoryNode::Data> node_data_;
  std::vector<std::shared_ptr<const mapping_3d::Submap>> insertion_submaps_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_LOCAL_SLAM_RESULT_DATA_H
