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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include "cartographer/mapping/map_builder_interface.h"

#include <memory>
#include <unordered_map>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping_2d/pose_graph.h"
#include "cartographer/mapping_3d/pose_graph.h"
#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder : public MapBuilderInterface {
 public:
  MapBuilder(const proto::MapBuilderOptions& options);
  ~MapBuilder() override;

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  int AddTrajectoryBuilder(
      const std::unordered_set<std::string>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization() override;

  mapping::TrajectoryBuilder* GetTrajectoryBuilder(
      int trajectory_id) const override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId& submap_id,
                            proto::SubmapQuery::Response* response) override;

  void SerializeState(io::ProtoStreamWriter* writer) override;

  void LoadMap(io::ProtoStreamReader* reader) override;

  int num_trajectory_builders() const override;

  mapping::PoseGraphInterface* pose_graph() override;

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<mapping_2d::PoseGraph> pose_graph_2d_;
  std::unique_ptr<mapping_3d::PoseGraph> pose_graph_3d_;
  mapping::PoseGraph* pose_graph_;

  sensor::Collator sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilder>> trajectory_builders_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
