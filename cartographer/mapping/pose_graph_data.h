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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_DATA_H
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_DATA_H

#include "cartographer/mapping_2d/pose_graph.h"
#include "cartographer/mapping_3d/pose_graph.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace mapping {

class PoseGraphData : public cartographer::sensor::Data {
 public:
  PoseGraphData(const std::string& sensor_id) : Data(sensor_id) {}

  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface* const trajectory_builder) override {
    trajectory_builder->AddPoseGraphData(*this);
  }

  virtual void AddToPoseGraph(int trajectory_id,
                              mapping::PoseGraph* pose_graph) const = 0;
};

class LocalSlamResult2D : public PoseGraphData {
 public:
  LocalSlamResult2D(const std::string& sensor_id) : PoseGraphData(sensor_id) {}

  void AddToPoseGraph(int trajectory_id,
                      mapping::PoseGraph* pose_graph) const override {
    mapping_2d::PoseGraph* pose_graph_2d =
        static_cast<mapping_2d::PoseGraph*>(pose_graph);
    pose_graph_2d->AddNode(constant_data, trajectory_id, insertion_submaps);
  }

 private:
  std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
  std::vector<std::shared_ptr<const mapping_2d::Submap>> insertion_submaps;
};

class LocalSlamResult3D : public PoseGraphData {
 public:
  LocalSlamResult3D(const std::string& sensor_id) : PoseGraphData(sensor_id) {}

  void AddToPoseGraph(int trajectory_id,
                      mapping::PoseGraph* pose_graph) const override {
    mapping_3d::PoseGraph* pose_graph_3d =
        static_cast<mapping_3d::PoseGraph*>(pose_graph);
    pose_graph_3d->AddNode(constant_data, trajectory_id, insertion_submaps);
  }

 private:
  std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
  std::vector<std::shared_ptr<const mapping_3d::Submap>> insertion_submaps;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_DATA_H
