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

#include "cartographer/mapping_2d/sparse_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

SparsePoseGraph::SparsePoseGraph(
    const mapping::proto::SparsePoseGraphOptions& options,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options()),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

SparsePoseGraph::~SparsePoseGraph() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(scan_queue_ == nullptr);
}

void SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const std::vector<const mapping::Submap*>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const mapping::SubmapId first_submap_id = GetSubmapId(insertion_submaps[0]);
  const int trajectory_id = first_submap_id.trajectory_id;
  CHECK_GE(trajectory_id, 0);
  const auto& submap_data = optimization_problem_.submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    CHECK_EQ(first_submap_id.submap_index, 0);
    if (static_cast<size_t>(trajectory_id) >= submap_data.size() ||
        submap_data[trajectory_id].empty()) {
      optimization_problem_.AddSubmap(trajectory_id,
                                      transform::Rigid2d::Identity());
    }
    return;
  }
  CHECK_EQ(2, insertion_submaps.size());
  const int next_submap_index = submap_data.at(trajectory_id).size();
  // CHECK that we have a index for the second submap.
  const mapping::SubmapId second_submap_id = GetSubmapId(insertion_submaps[1]);
  CHECK_EQ(second_submap_id.trajectory_id, trajectory_id);
  CHECK_LE(second_submap_id.submap_index, next_submap_index);
  // Extrapolate if necessary.
  if (second_submap_id.submap_index == next_submap_index) {
    const auto& first_submap_pose =
        submap_data.at(trajectory_id).at(first_submap_id.submap_index).pose;
    optimization_problem_.AddSubmap(
        trajectory_id,
        first_submap_pose *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0])
                .inverse() *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[1]));
  }
}

void SparsePoseGraph::AddScan(
    common::Time time, const transform::Rigid3d& tracking_to_pose,
    const sensor::RangeData& range_data_in_pose, const transform::Rigid2d& pose,
    const int trajectory_id, const mapping::Submap* const matching_submap,
    const std::vector<const mapping::Submap*>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * transform::Embed3D(pose));

  common::MutexLocker locker(&mutex_);
  trajectory_nodes_.Append(
      trajectory_id,
      mapping::TrajectoryNode{
          std::make_shared<const mapping::TrajectoryNode::Data>(
              mapping::TrajectoryNode::Data{
                  time, range_data_in_pose,
                  Compress(sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}}),
                  tracking_to_pose}),
          optimized_pose});
  ++num_trajectory_nodes_;
  trajectory_connectivity_.Add(trajectory_id);

  if (submap_ids_.count(insertion_submaps.back()) == 0) {
    const mapping::SubmapId submap_id =
        submap_data_.Append(trajectory_id, SubmapData());
    submap_ids_.emplace(insertion_submaps.back(), submap_id);
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }
  const mapping::Submap* const finished_submap =
      insertion_submaps.front()->finished_probability_grid != nullptr
          ? insertion_submaps.front()
          : nullptr;

  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }

  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(matching_submap, insertion_submaps,
                              finished_submap, pose);
  });
}

void SparsePoseGraph::AddWorkItem(std::function<void()> work_item) {
  if (scan_queue_ == nullptr) {
    work_item();
  } else {
    scan_queue_->push_back(work_item);
  }
}

void SparsePoseGraph::AddImuData(const int trajectory_id, common::Time time,
                                 const Eigen::Vector3d& linear_acceleration,
                                 const Eigen::Vector3d& angular_velocity) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddImuData(trajectory_id, time, linear_acceleration,
                                     angular_velocity);
  });
}

void SparsePoseGraph::ComputeConstraint(const mapping::NodeId& node_id,
                                        const mapping::SubmapId& submap_id) {
  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Only globally match against submaps not in this trajectory.
  if (node_id.trajectory_id != submap_id.trajectory_id &&
      global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap, node_id,
        &trajectory_nodes_.at(node_id).constant_data->range_data_2d.returns,
        &trajectory_connectivity_);
  } else {
    const bool scan_and_submap_trajectories_connected =
        reverse_connected_components_.count(node_id.trajectory_id) > 0 &&
        reverse_connected_components_.count(submap_id.trajectory_id) > 0 &&
        reverse_connected_components_.at(node_id.trajectory_id) ==
            reverse_connected_components_.at(submap_id.trajectory_id);
    if (node_id.trajectory_id == submap_id.trajectory_id ||
        scan_and_submap_trajectories_connected) {
      const transform::Rigid2d initial_relative_pose =
          optimization_problem_.submap_data()
              .at(submap_id.trajectory_id)
              .at(submap_id.submap_index)
              .pose.inverse() *
          optimization_problem_.node_data()
              .at(node_id.trajectory_id)
              .at(node_id.node_index)
              .point_cloud_pose;

      constraint_builder_.MaybeAddConstraint(
          submap_id, submap_data_.at(submap_id).submap, node_id,
          &trajectory_nodes_.at(node_id).constant_data->range_data_2d.returns,
          initial_relative_pose);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(
    const mapping::Submap* submap) {
  const auto submap_id = GetSubmapId(submap);
  const auto& submap_data = submap_data_.at(submap_id);

  const auto& node_data = optimization_problem_.node_data();
  for (size_t trajectory_id = 0; trajectory_id != node_data.size();
       ++trajectory_id) {
    for (size_t node_index = 0; node_index != node_data[trajectory_id].size();
         ++node_index) {
      const mapping::NodeId node_id{static_cast<int>(trajectory_id),
                                    static_cast<int>(node_index)};
      if (submap_data.node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, submap_id);
      }
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForScan(
    const mapping::Submap* matching_submap,
    std::vector<const mapping::Submap*> insertion_submaps,
    const mapping::Submap* finished_submap, const transform::Rigid2d& pose) {
  GrowSubmapTransformsAsNeeded(insertion_submaps);
  const mapping::SubmapId matching_id = GetSubmapId(matching_submap);
  const transform::Rigid2d optimized_pose =
      optimization_problem_.submap_data()
          .at(matching_id.trajectory_id)
          .at(matching_id.submap_index)
          .pose *
      sparse_pose_graph::ComputeSubmapPose(*matching_submap).inverse() * pose;
  const mapping::NodeId node_id{
      matching_id.trajectory_id,
      static_cast<size_t>(matching_id.trajectory_id) <
              optimization_problem_.node_data().size()
          ? static_cast<int>(optimization_problem_.node_data()
                                 .at(matching_id.trajectory_id)
                                 .size())
          : 0};
  const auto& scan_data = trajectory_nodes_.at(node_id).constant_data;
  optimization_problem_.AddTrajectoryNode(
      matching_id.trajectory_id, scan_data->time, pose, optimized_pose);
  for (const mapping::Submap* submap : insertion_submaps) {
    const mapping::SubmapId submap_id = GetSubmapId(submap);
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid2d constraint_transform =
        sparse_pose_graph::ComputeSubmapPose(*submap).inverse() * pose;
    constraints_.push_back(Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                       options_.matcher_translation_weight(),
                                       options_.matcher_rotation_weight()},
                                      Constraint::INTRA_SUBMAP});
  }

  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      const mapping::SubmapId submap_id{trajectory_id, submap_index};
      if (submap_data_.at(submap_id).state == SubmapState::kFinished) {
        CHECK_EQ(submap_data_.at(submap_id).node_ids.count(node_id), 0);
        ComputeConstraint(node_id, submap_id);
      }
    }
  }

  if (finished_submap != nullptr) {
    const mapping::SubmapId finished_submap_id = GetSubmapId(finished_submap);
    SubmapData& finished_submap_data = submap_data_.at(finished_submap_id);
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old scans.
    ComputeConstraintsForOldScans(finished_submap);
  }
  constraint_builder_.NotifyEndOfScan();
  ++num_scans_since_last_loop_closure_;
  if (options_.optimize_every_n_scans() > 0 &&
      num_scans_since_last_loop_closure_ > options_.optimize_every_n_scans()) {
    CHECK(!run_loop_closure_);
    run_loop_closure_ = true;
    // If there is a 'scan_queue_' already, some other thread will take care.
    if (scan_queue_ == nullptr) {
      scan_queue_ = common::make_unique<std::deque<std::function<void()>>>();
      HandleScanQueue();
    }
  }
}

void SparsePoseGraph::HandleScanQueue() {
  constraint_builder_.WhenDone(
      [this](const sparse_pose_graph::ConstraintBuilder::Result& result) {
        {
          common::MutexLocker locker(&mutex_);
          constraints_.insert(constraints_.end(), result.begin(), result.end());
        }
        RunOptimization();

        common::MutexLocker locker(&mutex_);
        num_scans_since_last_loop_closure_ = 0;
        run_loop_closure_ = false;
        while (!run_loop_closure_) {
          if (scan_queue_->empty()) {
            LOG(INFO) << "We caught up. Hooray!";
            scan_queue_.reset();
            return;
          }
          scan_queue_->front()();
          scan_queue_->pop_front();
        }
        // We have to optimize again.
        HandleScanQueue();
      });
}

void SparsePoseGraph::WaitForAllComputations() {
  bool notification = false;
  common::MutexLocker locker(&mutex_);
  const int num_finished_scans_at_start =
      constraint_builder_.GetNumFinishedScans();
  while (!locker.AwaitWithTimeout(
      [this]() REQUIRES(mutex_) {
        return constraint_builder_.GetNumFinishedScans() ==
               num_trajectory_nodes_;
      },
      common::FromSeconds(1.))) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. *
                         (constraint_builder_.GetNumFinishedScans() -
                          num_finished_scans_at_start) /
                         (num_trajectory_nodes_ - num_finished_scans_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone(
      [this, &notification](
          const sparse_pose_graph::ConstraintBuilder::Result& result) {
        common::MutexLocker locker(&mutex_);
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        notification = true;
      });
  locker.Await([&notification]() { return notification; });
}

void SparsePoseGraph::AddTrimmer(
    std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) {
  common::MutexLocker locker(&mutex_);
  // C++11 does not allow us to move a unique_ptr into a lambda.
  mapping::PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  AddWorkItem([this, trimmer_ptr]()
                  REQUIRES(mutex_) { trimmers_.emplace_back(trimmer_ptr); });
}

void SparsePoseGraph::RunFinalOptimization() {
  WaitForAllComputations();
  optimization_problem_.SetMaxNumIterations(
      options_.max_num_final_iterations());
  RunOptimization();
  optimization_problem_.SetMaxNumIterations(
      options_.optimization_problem_options()
          .ceres_solver_options()
          .max_num_iterations());
}

void SparsePoseGraph::RunOptimization() {
  if (optimization_problem_.submap_data().empty()) {
    return;
  }
  optimization_problem_.Solve(constraints_);
  common::MutexLocker locker(&mutex_);

  const auto& node_data = optimization_problem_.node_data();
  for (int trajectory_id = 0;
       trajectory_id != static_cast<int>(node_data.size()); ++trajectory_id) {
    int node_index = 0;
    const int num_nodes = trajectory_nodes_.num_indices(trajectory_id);
    for (; node_index != static_cast<int>(node_data[trajectory_id].size());
         ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      trajectory_nodes_.at(node_id).pose = transform::Embed3D(
          node_data[trajectory_id][node_index].point_cloud_pose);
    }
    // Extrapolate all point cloud poses that were added later.
    const auto new_submap_transforms = ExtrapolateSubmapTransforms(
        optimization_problem_.submap_data(), trajectory_id);
    const auto old_submap_transforms = ExtrapolateSubmapTransforms(
        optimized_submap_transforms_, trajectory_id);
    CHECK_EQ(new_submap_transforms.size(), old_submap_transforms.size());
    const transform::Rigid3d extrapolation_transform =
        new_submap_transforms.back() * old_submap_transforms.back().inverse();
    for (; node_index < num_nodes; ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      trajectory_nodes_.at(node_id).pose =
          extrapolation_transform * trajectory_nodes_.at(node_id).pose;
    }
  }
  optimized_submap_transforms_ = optimization_problem_.submap_data();
  connected_components_ = trajectory_connectivity_.ConnectedComponents();
  reverse_connected_components_.clear();
  for (size_t i = 0; i != connected_components_.size(); ++i) {
    for (const int trajectory_id : connected_components_[i]) {
      reverse_connected_components_.emplace(trajectory_id, i);
    }
  }

  TrimmingHandle trimming_handle(this);
  for (auto& trimmer : trimmers_) {
    trimmer->Trim(&trimming_handle);
  }
}

std::vector<std::vector<mapping::TrajectoryNode>>
SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_.data();
}

std::vector<SparsePoseGraph::Constraint> SparsePoseGraph::constraints() {
  common::MutexLocker locker(&mutex_);
  return constraints_;
}

transform::Rigid3d SparsePoseGraph::GetLocalToGlobalTransform(
    const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  if (submap_data_.num_trajectories() <= trajectory_id ||
      submap_data_.num_indices(trajectory_id) == 0) {
    return transform::Rigid3d::Identity();
  }
  const auto extrapolated_submap_transforms =
      ExtrapolateSubmapTransforms(optimized_submap_transforms_, trajectory_id);
  return extrapolated_submap_transforms.back() *
         submap_data_
             .at(mapping::SubmapId{
                 trajectory_id,
                 static_cast<int>(extrapolated_submap_transforms.size()) - 1})
             .submap->local_pose.inverse();
}

std::vector<std::vector<int>> SparsePoseGraph::GetConnectedTrajectories() {
  common::MutexLocker locker(&mutex_);
  return connected_components_;
}

std::vector<transform::Rigid3d> SparsePoseGraph::GetSubmapTransforms(
    const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  return ExtrapolateSubmapTransforms(optimized_submap_transforms_,
                                     trajectory_id);
}

std::vector<transform::Rigid3d> SparsePoseGraph::ExtrapolateSubmapTransforms(
    const std::vector<std::vector<sparse_pose_graph::SubmapData>>&
        submap_transforms,
    const int trajectory_id) const {
  if (trajectory_id >= submap_data_.num_trajectories()) {
    return {transform::Rigid3d::Identity()};
  }

  // Submaps for which we have optimized poses.
  std::vector<transform::Rigid3d> result;
  for (int submap_index = 0;
       submap_index != submap_data_.num_indices(trajectory_id);
       ++submap_index) {
    const mapping::SubmapId submap_id{trajectory_id, submap_index};
    const auto& submap_data = submap_data_.at(submap_id);
    if (static_cast<size_t>(trajectory_id) < submap_transforms.size() &&
        result.size() < submap_transforms.at(trajectory_id).size()) {
      // Submaps for which we have optimized poses.
      result.push_back(
          Embed3D(submap_transforms.at(trajectory_id).at(result.size()).pose));
    } else if (result.empty()) {
      result.push_back(transform::Rigid3d::Identity());
    } else {
      // Extrapolate to the remaining submaps. Accessing 'local_pose' in
      // Submaps is okay, since the member is const.
      const mapping::SubmapId previous_submap_id{
          trajectory_id, static_cast<int>(result.size()) - 1};
      result.push_back(
          result.back() *
          submap_data_.at(previous_submap_id).submap->local_pose.inverse() *
          submap_data.submap->local_pose);
    }
  }

  if (result.empty()) {
    result.push_back(transform::Rigid3d::Identity());
  }
  return result;
}

SparsePoseGraph::TrimmingHandle::TrimmingHandle(SparsePoseGraph* const parent)
    : parent_(parent) {}

int SparsePoseGraph::TrimmingHandle::num_submaps(
    const int trajectory_id) const {
  return parent_->optimization_problem_.submap_data().at(trajectory_id).size();
}

void SparsePoseGraph::TrimmingHandle::MarkSubmapAsTrimmed(
    const mapping::SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  std::set<mapping::NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }
  // Remove all 'constraints_' related to 'submap_id'.
  std::set<mapping::NodeId> nodes_to_remove;
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (constraint.submap_id == submap_id) {
        if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
            nodes_to_retain.count(constraint.node_id) == 0) {
          // This node will no longer be INTRA_SUBMAP contrained and has to be
          // removed.
          nodes_to_remove.insert(constraint.node_id);
        }
      } else {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }
  // Remove all 'constraints_' related to 'nodes_to_remove'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->constraints_) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      }
    }
    parent_->constraints_ = std::move(constraints);
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  parent_->submap_data_.at(submap_id).state = SubmapState::kTrimmed;
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  // TODO(hrapp): Make 'Submap' object thread safe and remove submap data in
  // there.

  // TODO(whess): Mark the 'nodes_to_remove' as pruned and remove their data.
  // Also make sure we no longer try to scan match against it.

  // TODO(whess): The optimization problem should no longer include the submap
  // and the removed nodes.

  // TODO(whess): If the first submap is gone, we want to tie the first not
  // yet trimmed submap to be set fixed to its current pose.
}

}  // namespace mapping_2d
}  // namespace cartographer
