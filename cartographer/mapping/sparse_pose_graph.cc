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

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping/sparse_pose_graph/optimization_problem_options.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::SparsePoseGraph::Constraint::Tag ToProto(
    const SparsePoseGraph::Constraint::Tag& tag) {
  switch (tag) {
    case SparsePoseGraph::Constraint::Tag::INTRA_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTRA_SUBMAP;
    case SparsePoseGraph::Constraint::Tag::INTER_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

void GroupTrajectoryNodes(
    const std::vector<TrajectoryNode>& trajectory_nodes,
    const std::unordered_map<const Submaps*, int>& trajectory_ids,
    std::vector<std::vector<TrajectoryNode>>* grouped_nodes,
    std::vector<std::pair<int, int>>* new_indices) {
  CHECK_NOTNULL(grouped_nodes)->clear();
  CHECK_NOTNULL(new_indices)->clear();

  grouped_nodes->resize(trajectory_ids.size());

  for (const auto& node : trajectory_nodes) {
    const int id = trajectory_ids.at(node.constant_data->trajectory);
    new_indices->emplace_back(id, (*grouped_nodes)[id].size());
    (*grouped_nodes)[id].push_back(node);
  }
}

// TODO(macmason): This function is very nearly a copy of GroupTrajectoryNodes.
// Consider refactoring them to share an implementation.
void GroupSubmapStates(
    const std::vector<SparsePoseGraph::SubmapState>& submap_states,
    const std::unordered_map<const Submaps*, int>& trajectory_ids,
    std::vector<std::pair<int, int>>* new_indices) {
  CHECK_NOTNULL(new_indices)->clear();
  std::vector<int> submap_group_sizes(trajectory_ids.size(), 0);

  for (const auto& submap_state : submap_states) {
    const int id = trajectory_ids.at(submap_state.trajectory);
    new_indices->emplace_back(id, submap_group_sizes[id]);
    submap_group_sizes[id]++;
  }
}

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SparsePoseGraphOptions options;
  options.set_optimize_every_n_scans(
      parameter_dictionary->GetInt("optimize_every_n_scans"));
  *options.mutable_constraint_builder_options() =
      sparse_pose_graph::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  *options.mutable_optimization_problem_options() =
      sparse_pose_graph::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_global_sampling_ratio(
      parameter_dictionary->GetDouble("global_sampling_ratio"));
  return options;
}

proto::SparsePoseGraph SparsePoseGraph::ToProto() {
  proto::SparsePoseGraph proto;

  std::vector<std::vector<TrajectoryNode>> grouped_nodes;
  std::vector<std::pair<int, int>> grouped_node_indices;
  GroupTrajectoryNodes(GetTrajectoryNodes(), trajectory_ids(), &grouped_nodes,
                       &grouped_node_indices);

  std::vector<std::pair<int, int>> grouped_submap_indices;
  GroupSubmapStates(GetSubmapStates(), trajectory_ids(),
                    &grouped_submap_indices);

  for (const auto& constraint : constraints()) {
    auto* const constraint_proto = proto.add_constraint();
    *constraint_proto->mutable_relative_pose() =
        transform::ToProto(constraint.pose.zbar_ij);
    constraint_proto->mutable_sqrt_lambda()->Reserve(36);
    for (int i = 0; i != 36; ++i) {
      constraint_proto->mutable_sqrt_lambda()->Add(0.);
    }
    Eigen::Map<Eigen::Matrix<double, 6, 6>>(
        constraint_proto->mutable_sqrt_lambda()->mutable_data()) =
        constraint.pose.sqrt_Lambda_ij;

    constraint_proto->mutable_submap_id()->set_trajectory_id(
        grouped_submap_indices[constraint.i].first);
    constraint_proto->mutable_submap_id()->set_submap_index(
        grouped_submap_indices[constraint.i].second);

    constraint_proto->mutable_scan_id()->set_trajectory_id(
        grouped_node_indices[constraint.j].first);
    constraint_proto->mutable_scan_id()->set_scan_index(
        grouped_node_indices[constraint.j].second);

    constraint_proto->set_tag(mapping::ToProto(constraint.tag));
  }

  for (const auto& group : grouped_nodes) {
    auto* trajectory_proto = proto.add_trajectory();
    for (const auto& node : group) {
      auto* node_proto = trajectory_proto->add_node();
      node_proto->set_timestamp(common::ToUniversal(node.constant_data->time));
      *node_proto->mutable_pose() =
          transform::ToProto(node.pose * node.constant_data->tracking_to_pose);
    }

    const Submaps* const trajectory = group[0].constant_data->trajectory;
    for (const auto& transform : GetSubmapTransforms(trajectory)) {
      *trajectory_proto->add_submap()->mutable_pose() =
          transform::ToProto(transform);
    }
  }

  return proto;
}

}  // namespace mapping
}  // namespace cartographer
