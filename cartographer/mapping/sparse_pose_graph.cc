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

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SparsePoseGraphOptions options;
  options.set_optimize_every_n_scans(
      parameter_dictionary->GetInt("optimize_every_n_scans"));
  *options.mutable_constraint_builder_options() =
      sparse_pose_graph::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  options.set_matcher_translation_weight(
      parameter_dictionary->GetDouble("matcher_translation_weight"));
  options.set_matcher_rotation_weight(
      parameter_dictionary->GetDouble("matcher_rotation_weight"));
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

  std::map<NodeId, NodeId> node_id_remapping;  // Due to trimming.

  const auto all_trajectory_nodes = GetTrajectoryNodes();
  for (size_t trajectory_id = 0; trajectory_id != all_trajectory_nodes.size();
       ++trajectory_id) {
    const auto& single_trajectory_nodes = all_trajectory_nodes[trajectory_id];
    auto* trajectory_proto = proto.add_trajectory();

    for (size_t old_node_index = 0;
         old_node_index != single_trajectory_nodes.size(); ++old_node_index) {
      const auto& node = single_trajectory_nodes[old_node_index];
      if (!node.trimmed()) {
        node_id_remapping[NodeId{static_cast<int>(trajectory_id),
                                 static_cast<int>(old_node_index)}] =
            NodeId{static_cast<int>(trajectory_id),
                   static_cast<int>(trajectory_proto->node_size())};
        auto* node_proto = trajectory_proto->add_node();
        node_proto->set_timestamp(
            common::ToUniversal(node.constant_data->time));
        *node_proto->mutable_pose() = transform::ToProto(
            node.pose * node.constant_data->tracking_to_pose);
      }
    }

    if (!single_trajectory_nodes.empty()) {
      for (const auto& transform : GetSubmapTransforms(trajectory_id)) {
        *trajectory_proto->add_submap()->mutable_pose() =
            transform::ToProto(transform);
      }
    }
  }

  for (const auto& constraint : constraints()) {
    auto* const constraint_proto = proto.add_constraint();
    *constraint_proto->mutable_relative_pose() =
        transform::ToProto(constraint.pose.zbar_ij);
    constraint_proto->set_translation_weight(
        constraint.pose.translation_weight);
    constraint_proto->set_rotation_weight(constraint.pose.rotation_weight);
    constraint_proto->mutable_submap_id()->set_trajectory_id(
        constraint.submap_id.trajectory_id);
    constraint_proto->mutable_submap_id()->set_submap_index(
        constraint.submap_id.submap_index);

    const NodeId node_id = node_id_remapping.at(constraint.node_id);
    constraint_proto->mutable_scan_id()->set_trajectory_id(
        node_id.trajectory_id);
    constraint_proto->mutable_scan_id()->set_scan_index(node_id.node_index);

    constraint_proto->set_tag(mapping::ToProto(constraint.tag));
  }

  return proto;
}

}  // namespace mapping
}  // namespace cartographer
