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

#ifndef CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_

#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/mapping/proto/sparse_pose_graph_options.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class SparsePoseGraph {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      double translation_weight;
      double rotation_weight;
    };

    mapping::SubmapId submap_id;  // 'i' in the paper.
    mapping::NodeId node_id;      // 'j' in the paper.

    // Pose of the scan 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where scan 'j' was inserted into
    // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  SparsePoseGraph() {}
  virtual ~SparsePoseGraph() {}

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Computes optimized poses.
  virtual void RunFinalOptimization() = 0;

  // Get the current trajectory clusters.
  virtual std::vector<std::vector<int>> GetConnectedTrajectories() = 0;

  // Returns the current optimized transforms for the given 'trajectory_id'.
  virtual std::vector<transform::Rigid3d> GetSubmapTransforms(
      int trajectory_id) = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  virtual transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) = 0;

  // Returns the current optimized trajectories.
  virtual std::vector<std::vector<TrajectoryNode>> GetTrajectoryNodes() = 0;

  // Serializes the constraints and trajectories.
  proto::SparsePoseGraph ToProto();

  // Returns the collection of constraints.
  virtual std::vector<Constraint> constraints() = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
