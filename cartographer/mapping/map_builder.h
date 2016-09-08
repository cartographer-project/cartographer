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

#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/local_trajectory_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a SparsePoseGraph for loop closure.
class MapBuilder {
 public:
  MapBuilder(const proto::MapBuilderOptions& options,
             std::deque<mapping::TrajectoryNode::ConstantData>* constant_data);
  ~MapBuilder();

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // Create a new trajectory and return its index.
  int AddTrajectoryBuilder();

  // Returns the TrajectoryBuilder corresponding to the specified
  // 'trajectory_id' or 'trajectory' pointer.
  mapping::GlobalTrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const;
  mapping::GlobalTrajectoryBuilderInterface* GetTrajectoryBuilder(
      const mapping::Submaps* trajectory) const;

  // Returns the trajectory ID for 'trajectory'.
  int GetTrajectoryId(const mapping::Submaps* trajectory) const;

  // Returns the trajectory connectivity.
  proto::TrajectoryConnectivity GetTrajectoryConnectivity();

  int num_trajectory_builders() const;

  mapping::SparsePoseGraph* sparse_pose_graph();

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<mapping_2d::SparsePoseGraph> sparse_pose_graph_2d_;
  std::unique_ptr<mapping_3d::SparsePoseGraph> sparse_pose_graph_3d_;
  mapping::SparsePoseGraph* sparse_pose_graph_;

  std::vector<std::unique_ptr<mapping::GlobalTrajectoryBuilderInterface>>
      trajectory_builders_;
  std::unordered_map<const mapping::Submaps*, int> trajectory_ids_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
