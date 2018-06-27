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

#include "cartographer/ground_truth/autogenerate_ground_truth.h"

#include <string>
#include <vector>

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace ground_truth {
namespace {

std::vector<double> ComputeCoveredDistance(
    const mapping::proto::Trajectory& trajectory) {
  std::vector<double> covered_distance;
  covered_distance.push_back(0.);
  CHECK_GT(trajectory.node_size(), 0)
      << "Trajectory does not contain any nodes.";
  for (int i = 1; i < trajectory.node_size(); ++i) {
    const auto last_pose = transform::ToRigid3(trajectory.node(i - 1).pose());
    const auto this_pose = transform::ToRigid3(trajectory.node(i).pose());
    covered_distance.push_back(
        covered_distance.back() +
        (last_pose.inverse() * this_pose).translation().norm());
  }
  return covered_distance;
}

// We pick the representative node in the middle of the submap.
//
// TODO(whess): Should we consider all nodes inserted into the submap and
// exclude, e.g. based on large relative linear or angular distance?
std::vector<int> ComputeSubmapRepresentativeNode(
    const mapping::proto::PoseGraph& pose_graph) {
  std::vector<int> submap_to_node_index;
  for (const auto& constraint : pose_graph.constraint()) {
    if (constraint.tag() !=
        mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
      continue;
    }
    CHECK_EQ(constraint.submap_id().trajectory_id(), 0);
    CHECK_EQ(constraint.node_id().trajectory_id(), 0);

    const int next_submap_index = static_cast<int>(submap_to_node_index.size());
    const int submap_index = constraint.submap_id().submap_index();
    if (submap_index <= next_submap_index) {
      continue;
    }

    CHECK_EQ(submap_index, next_submap_index + 1);
    submap_to_node_index.push_back(constraint.node_id().node_index());
  }
  return submap_to_node_index;
}

}  // namespace

proto::GroundTruth GenerateGroundTruth(
    const mapping::proto::PoseGraph& pose_graph,
    const double min_covered_distance, const double outlier_threshold_meters,
    const double outlier_threshold_radians) {
  const mapping::proto::Trajectory& trajectory = pose_graph.trajectory(0);
  const std::vector<double> covered_distance =
      ComputeCoveredDistance(trajectory);

  const std::vector<int> submap_to_node_index =
      ComputeSubmapRepresentativeNode(pose_graph);

  int num_outliers = 0;
  proto::GroundTruth ground_truth;
  for (const auto& constraint : pose_graph.constraint()) {
    // We're only interested in loop closure constraints.
    if (constraint.tag() ==
        mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
      continue;
    }

    // For some submaps at the very end, we have not chosen a representative
    // node, but those should not be part of loop closure anyway.
    CHECK_EQ(constraint.submap_id().trajectory_id(), 0);
    CHECK_EQ(constraint.node_id().trajectory_id(), 0);
    if (constraint.submap_id().submap_index() >=
        static_cast<int>(submap_to_node_index.size())) {
      continue;
    }
    const int matched_node = constraint.node_id().node_index();
    const int representative_node =
        submap_to_node_index.at(constraint.submap_id().submap_index());

    // Covered distance between the two should not be too small.
    double covered_distance_in_constraint =
        std::abs(covered_distance.at(matched_node) -
                 covered_distance.at(representative_node));
    if (covered_distance_in_constraint < min_covered_distance) {
      continue;
    }

    // Compute the transform between the nodes according to the solution and
    // the constraint.
    const transform::Rigid3d solution_pose1 =
        transform::ToRigid3(trajectory.node(representative_node).pose());
    const transform::Rigid3d solution_pose2 =
        transform::ToRigid3(trajectory.node(matched_node).pose());
    const transform::Rigid3d solution =
        solution_pose1.inverse() * solution_pose2;

    const transform::Rigid3d submap_solution = transform::ToRigid3(
        trajectory.submap(constraint.submap_id().submap_index()).pose());
    const transform::Rigid3d submap_solution_to_node_solution =
        solution_pose1.inverse() * submap_solution;
    const transform::Rigid3d node_to_submap_constraint =
        transform::ToRigid3(constraint.relative_pose());
    const transform::Rigid3d expected =
        submap_solution_to_node_solution * node_to_submap_constraint;

    const transform::Rigid3d error = solution * expected.inverse();

    if (error.translation().norm() > outlier_threshold_meters ||
        transform::GetAngle(error) > outlier_threshold_radians) {
      ++num_outliers;
      continue;
    }
    auto* const new_relation = ground_truth.add_relation();
    new_relation->set_timestamp1(
        trajectory.node(representative_node).timestamp());
    new_relation->set_timestamp2(trajectory.node(matched_node).timestamp());
    *new_relation->mutable_expected() = transform::ToProto(expected);
    new_relation->set_covered_distance(covered_distance_in_constraint);
  }
  LOG(INFO) << "Generated " << ground_truth.relation_size()
            << " relations and ignored " << num_outliers << " outliers.";
  return ground_truth;
}

}  // namespace ground_truth
}  // namespace cartographer
