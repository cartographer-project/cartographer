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

#include "cartographer/mapping_3d/sparse_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
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
namespace mapping_3d {

SparsePoseGraph::SparsePoseGraph(
    const mapping::proto::SparsePoseGraphOptions& options,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options(),
                            sparse_pose_graph::OptimizationProblem::FixZ::kNo),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

SparsePoseGraph::~SparsePoseGraph() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(scan_queue_ == nullptr);
}

void SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const std::vector<const Submap*>& insertion_submaps) {
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
                                      transform::Rigid3d::Identity());
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
        trajectory_id, first_submap_pose *
                           insertion_submaps[0]->local_pose.inverse() *
                           insertion_submaps[1]->local_pose);
  }
}

void SparsePoseGraph::AddScan(
    common::Time time, const sensor::RangeData& range_data_in_tracking,
    const transform::Rigid3d& pose, const int trajectory_id,
    const Submap* const matching_submap,
    const std::vector<const Submap*>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * pose);

  common::MutexLocker locker(&mutex_);
  trajectory_nodes_.Append(
      trajectory_id,
      mapping::TrajectoryNode{
          std::make_shared<const mapping::TrajectoryNode::Data>(
              mapping::TrajectoryNode::Data{
                  time, sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}},
                  sensor::Compress(range_data_in_tracking),
                  transform::Rigid3d::Identity()}),
          optimized_pose});
  ++num_trajectory_nodes_;
  trajectory_connectivity_.Add(trajectory_id);

  if (submap_ids_.count(insertion_submaps.back()) == 0) {
    const mapping::SubmapId submap_id =
        submap_data_.Append(trajectory_id, SubmapData());
    submap_ids_.emplace(insertion_submaps.back(), submap_id);
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }
  const Submap* const finished_submap =
      insertion_submaps.front()->finished ? insertion_submaps.front() : nullptr;

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

  const transform::Rigid3d inverse_submap_pose =
      optimization_problem_.submap_data()
          .at(submap_id.trajectory_id)
          .at(submap_id.submap_index)
          .pose.inverse();

  const transform::Rigid3d initial_relative_pose =
      inverse_submap_pose * optimization_problem_.node_data()
                                .at(node_id.trajectory_id)
                                .at(node_id.node_index)
                                .point_cloud_pose;

  std::vector<mapping::TrajectoryNode> submap_nodes;
  for (const mapping::NodeId& submap_node_id :
       submap_data_.at(submap_id).node_ids) {
    submap_nodes.push_back(mapping::TrajectoryNode{
        trajectory_nodes_.at(submap_node_id).constant_data,
        inverse_submap_pose * trajectory_nodes_.at(submap_node_id).pose});
  }

  // Only globally match against submaps not in this trajectory.
  if (node_id.trajectory_id != submap_id.trajectory_id &&
      global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    // In this situation, 'initial_relative_pose' is:
    //
    // submap <- global map 2 <- global map 1 <- tracking
    //               (agreeing on gravity)
    //
    // Since they possibly came from two disconnected trajectories, the only
    // guaranteed connection between the tracking and the submap frames is
    // an agreement on the direction of gravity. Therefore, excluding yaw,
    // 'initial_relative_pose.rotation()' is a good estimate of the relative
    // orientation of the point cloud in the submap frame. Finding the correct
    // yaw component will be handled by the matching procedure in the
    // FastCorrelativeScanMatcher, and the given yaw is essentially ignored.
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap, node_id,
        &trajectory_nodes_.at(node_id).constant_data->range_data_3d.returns,
        submap_nodes, initial_relative_pose.rotation(),
        &trajectory_connectivity_);
  } else {
    const bool scan_and_submap_trajectories_connected =
        reverse_connected_components_.count(node_id.trajectory_id) > 0 &&
        reverse_connected_components_.count(submap_id.trajectory_id) > 0 &&
        reverse_connected_components_.at(node_id.trajectory_id) ==
            reverse_connected_components_.at(submap_id.trajectory_id);
    if (node_id.trajectory_id == submap_id.trajectory_id ||
        scan_and_submap_trajectories_connected) {
      constraint_builder_.MaybeAddConstraint(
          submap_id, submap_data_.at(submap_id).submap, node_id,
          &trajectory_nodes_.at(node_id).constant_data->range_data_3d.returns,
          submap_nodes, initial_relative_pose);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(const Submap* submap) {
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
    const Submap* matching_submap, std::vector<const Submap*> insertion_submaps,
    const Submap* finished_submap, const transform::Rigid3d& pose) {
  GrowSubmapTransformsAsNeeded(insertion_submaps);
  const mapping::SubmapId matching_id = GetSubmapId(matching_submap);
  const transform::Rigid3d optimized_pose =
      optimization_problem_.submap_data()
          .at(matching_id.trajectory_id)
          .at(matching_id.submap_index)
          .pose *
      matching_submap->local_pose.inverse() * pose;
  const mapping::NodeId node_id{
      matching_id.trajectory_id,
      static_cast<size_t>(matching_id.trajectory_id) <
              optimization_problem_.node_data().size()
          ? static_cast<int>(optimization_problem_.node_data()
                                 .at(matching_id.trajectory_id)
                                 .size())
          : 0};
  const auto& scan_data = trajectory_nodes_.at(node_id).constant_data;
  optimization_problem_.AddTrajectoryNode(matching_id.trajectory_id,
                                          scan_data->time, optimized_pose);
  for (const Submap* submap : insertion_submaps) {
    const mapping::SubmapId submap_id = GetSubmapId(submap);
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid3d constraint_transform =
        submap->local_pose.inverse() * pose;
    constraints_.push_back(
        Constraint{submap_id,
                   node_id,
                   {constraint_transform, options_.matcher_translation_weight(),
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
      trajectory_nodes_.at(node_id).pose =
          node_data[trajectory_id][node_index].point_cloud_pose;
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
          submap_transforms.at(trajectory_id).at(result.size()).pose);
    } else if (result.empty()) {
      result.push_back(transform::Rigid3d::Identity());
    } else {
      // Extrapolate to the remaining submaps. Accessing 'local_pose' in Submaps
      // is okay, since the member is const.
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

}  // namespace mapping_3d
}  // namespace cartographer
