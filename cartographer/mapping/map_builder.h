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

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/mapping_2d/pose_graph.h"
#include "cartographer/mapping_3d/pose_graph.h"
#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder {
 public:
  using LocalSlamResultCallback =
      GlobalTrajectoryBuilderInterface::LocalSlamResultCallback;

  MapBuilder(const proto::MapBuilderOptions& options,
             const LocalSlamResultCallback& local_slam_result_callback);
  ~MapBuilder();

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // Creates a new trajectory builder and returns its index.
  int AddTrajectoryBuilder(
      const std::unordered_set<std::string>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options);

  // Creates a new trajectory and returns its index. Querying the trajectory
  // builder for it will return 'nullptr'.
  int AddTrajectoryForDeserialization();

  // Returns the TrajectoryBuilder corresponding to the specified
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  mapping::TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id) const;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  void FinishTrajectory(int trajectory_id);

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the MapBuilder is
  // unblocked.
  int GetBlockingTrajectoryId() const;

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
  // error string on failure, or an empty string on success.
  std::string SubmapToProto(const SubmapId& submap_id,
                            proto::SubmapQuery::Response* response);

  // Serializes the current state to a proto stream.
  void SerializeState(io::ProtoStreamWriter* writer);

  // Loads submaps from a proto stream into a new frozen trajectory.
  void LoadMap(io::ProtoStreamReader* reader);

  int num_trajectory_builders() const;

  mapping::PoseGraph* pose_graph();

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<mapping_2d::PoseGraph> pose_graph_2d_;
  std::unique_ptr<mapping_3d::PoseGraph> pose_graph_3d_;
  mapping::PoseGraph* pose_graph_;

  LocalSlamResultCallback local_slam_result_callback_;

  sensor::Collator sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilder>> trajectory_builders_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
