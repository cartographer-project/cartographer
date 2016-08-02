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

#include "cartographer/mapping/sparse_pose_graph.h"

#include <unordered_map>

#include "cartographer/mapping/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping/sparse_pose_graph/optimization_problem_options.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

std::vector<std::vector<TrajectoryNode>> SplitTrajectoryNodes(
    const std::vector<TrajectoryNode>& trajectory_nodes) {
  std::vector<std::vector<TrajectoryNode>> trajectories;
  std::unordered_map<const mapping::Submaps*, int> trajectory_ids;
  for (const auto& node : trajectory_nodes) {
    const auto* trajectory = node.constant_data->trajectory;
    if (trajectory_ids.emplace(trajectory, trajectories.size()).second) {
      trajectories.push_back({node});
    } else {
      trajectories[trajectory_ids[trajectory]].push_back(node);
    }
  }
  return trajectories;
}

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SparsePoseGraphOptions options;
  options.set_optimize_every_n_scans(
      parameter_dictionary->GetInt("optimize_every_n_scans"));
  options.set_also_match_to_new_submaps(
      parameter_dictionary->GetBool("also_match_to_new_submaps"));
  *options.mutable_constraint_builder_options() =
      sparse_pose_graph::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  *options.mutable_optimization_problem_options() =
      mapping::sparse_pose_graph::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_global_sampling_ratio(
      parameter_dictionary->GetDouble("global_sampling_ratio"));
  return options;
}

}  // namespace mapping
}  // namespace cartographer
