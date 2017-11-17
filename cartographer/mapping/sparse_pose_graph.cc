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

SparsePoseGraph::Constraint::Tag FromProto(
    const proto::SparsePoseGraph::Constraint::Tag& proto) {
  switch (proto) {
    case proto::SparsePoseGraph::Constraint::INTRA_SUBMAP:
      return SparsePoseGraph::Constraint::Tag::INTRA_SUBMAP;
    case proto::SparsePoseGraph::Constraint::INTER_SUBMAP:
      return SparsePoseGraph::Constraint::Tag::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

std::vector<SparsePoseGraph::Constraint> FromProto(
    const ::google::protobuf::RepeatedPtrField<
        ::cartographer::mapping::proto::SparsePoseGraph::Constraint>&
        constraint_protos) {
  std::vector<SparsePoseGraph::Constraint> constraints;
  for (const auto& constraint_proto : constraint_protos) {
    const mapping::SubmapId submap_id{
        constraint_proto.submap_id().trajectory_id(),
        constraint_proto.submap_id().submap_index()};
    const mapping::NodeId node_id{constraint_proto.node_id().trajectory_id(),
                                  constraint_proto.node_id().node_index()};
    const SparsePoseGraph::Constraint::Pose pose{
        transform::ToRigid3(constraint_proto.relative_pose()),
        constraint_proto.translation_weight(),
        constraint_proto.rotation_weight()};
    const SparsePoseGraph::Constraint::Tag tag =
        FromProto(constraint_proto.tag());
    constraints.push_back(
        SparsePoseGraph::Constraint{submap_id, node_id, pose, tag});
  }
  return constraints;
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
  options.set_log_residual_histograms(
      parameter_dictionary->GetBool("log_residual_histograms"));
  options.set_global_constraint_search_after_n_seconds(
      parameter_dictionary->GetDouble(
          "global_constraint_search_after_n_seconds"));
  return options;
}

proto::SparsePoseGraph SparsePoseGraph::ToProto() {
  proto::SparsePoseGraph proto;

  std::map<int, proto::Trajectory* const> trajectory_protos;
  const auto trajectory = [&proto, &trajectory_protos](
                              const int trajectory_id) -> proto::Trajectory* {
    if (trajectory_protos.count(trajectory_id) == 0) {
      auto* const trajectory_proto = proto.add_trajectory();
      trajectory_proto->set_trajectory_id(trajectory_id);
      CHECK(trajectory_protos.emplace(trajectory_id, trajectory_proto).second);
    }
    return trajectory_protos.at(trajectory_id);
  };

  for (const auto& node_id_data : GetTrajectoryNodes()) {
    CHECK(node_id_data.data.constant_data != nullptr);
    auto* const node_proto =
        trajectory(node_id_data.id.trajectory_id)->add_node();
    node_proto->set_node_index(node_id_data.id.node_index);
    node_proto->set_timestamp(
        common::ToUniversal(node_id_data.data.constant_data->time));
    *node_proto->mutable_pose() =
        transform::ToProto(node_id_data.data.global_pose);
  }

  for (const auto& submap_id_data : GetAllSubmapData()) {
    CHECK(submap_id_data.data.submap != nullptr);
    auto* const submap_proto =
        trajectory(submap_id_data.id.trajectory_id)->add_submap();
    submap_proto->set_submap_index(submap_id_data.id.submap_index);
    *submap_proto->mutable_pose() =
        transform::ToProto(submap_id_data.data.pose);
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

    constraint_proto->mutable_node_id()->set_trajectory_id(
        constraint.node_id.trajectory_id);
    constraint_proto->mutable_node_id()->set_node_index(
        constraint.node_id.node_index);

    constraint_proto->set_tag(mapping::ToProto(constraint.tag));
  }

  return proto;
}

}  // namespace mapping
}  // namespace cartographer
