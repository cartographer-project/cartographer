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
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/mapping/proto/sparse_pose_graph_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// TrajectoryNodes are provided in a flat vector, but serialization requires
// that we group them by trajectory. This groups the elements of
// 'trajectory_nodes' into 'grouped_nodes' (so that (*grouped_nodes)[i]
// contains a complete single trajectory). The re-indexing done is stored in
// 'new_indices', such that 'trajectory_nodes[i]' landed in
// '(*grouped_nodes)[new_indices[i].first][new_indices[i].second]'.
void GroupTrajectoryNodes(
    const std::vector<TrajectoryNode>& trajectory_nodes,
    const std::unordered_map<const Submaps*, int>& trajectory_ids,
    std::vector<std::vector<TrajectoryNode>>* grouped_nodes,
    std::vector<std::pair<int, int>>* new_indices);

class SparsePoseGraph {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;
      Eigen::Matrix<double, 6, 6> sqrt_Lambda_ij;
    };

    mapping::SubmapId submap_id;  // 'i' in the paper.

    // Scan index.
    int j;

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

  // Returns the current optimized transforms for the given 'trajectory'.
  virtual std::vector<transform::Rigid3d> GetSubmapTransforms(
      const Submaps* trajectory) = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      const Submaps* submaps) = 0;

  // Returns the current optimized trajectory.
  virtual std::vector<TrajectoryNode> GetTrajectoryNodes() = 0;

  // Serializes the constraints and trajectories.
  proto::SparsePoseGraph ToProto();

 protected:
  // Returns the collection of constraints.
  virtual std::vector<Constraint> constraints() = 0;

  // Returns the mapping from Submaps* to trajectory IDs.
  virtual const std::unordered_map<const Submaps*, int>& trajectory_ids() = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
