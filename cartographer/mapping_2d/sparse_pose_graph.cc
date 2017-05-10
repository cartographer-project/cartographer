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
  CHECK_LT(optimization_problem_.submap_data().size(),
           std::numeric_limits<int>::max());
  const int next_submap_index = optimization_problem_.submap_data().size();
  // Verify that we have an index for the first submap.
  const int first_submap_index = GetSubmapIndex(insertion_submaps[0]);
  CHECK_LE(first_submap_index, next_submap_index);
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for this submap, add one.
    if (first_submap_index == next_submap_index) {
      optimization_problem_.AddSubmap(
          submap_states_[first_submap_index].trajectory,
          transform::Rigid2d::Identity());
    }
    return;
  }
  CHECK_EQ(2, insertion_submaps.size());
  // CHECK that we have a index for the second submap.
  const int second_submap_index = GetSubmapIndex(insertion_submaps[1]);
  CHECK_LE(second_submap_index, next_submap_index);
  // Extrapolate if necessary.
  if (second_submap_index == next_submap_index) {
    const auto& first_submap_pose =
        optimization_problem_.submap_data().at(first_submap_index).pose;
    optimization_problem_.AddSubmap(
        submap_states_[second_submap_index].trajectory,
        first_submap_pose *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0])
                .inverse() *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[1]));
  }
}

void SparsePoseGraph::AddScan(
    common::Time time, const transform::Rigid3d& tracking_to_pose,
    const sensor::RangeData& range_data_in_pose, const transform::Rigid2d& pose,
    const kalman_filter::Pose2DCovariance& covariance,
    const mapping::Submaps* const trajectory,
    const mapping::Submap* const matching_submap,
    const std::vector<const mapping::Submap*>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory) * transform::Embed3D(pose));

  common::MutexLocker locker(&mutex_);
  trajectory_ids_.emplace(trajectory, trajectory_ids_.size());
  const int j = trajectory_nodes_.size();
  CHECK_LT(j, std::numeric_limits<int>::max());

  constant_node_data_.push_back(mapping::TrajectoryNode::ConstantData{
      time, range_data_in_pose,
      Compress(sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}}), trajectory,
      tracking_to_pose});
  trajectory_nodes_.push_back(mapping::TrajectoryNode{
      &constant_node_data_.back(), optimized_pose,
  });
  trajectory_connectivity_.Add(trajectory);

  if (submap_indices_.count(insertion_submaps.back()) == 0) {
    submap_indices_.emplace(insertion_submaps.back(),
                            static_cast<int>(submap_indices_.size()));
    submap_states_.emplace_back();
    submap_states_.back().submap = insertion_submaps.back();
    submap_states_.back().trajectory = trajectory;
    CHECK_EQ(submap_states_.size(), submap_indices_.size());
  }
  const mapping::Submap* const finished_submap =
      insertion_submaps.front()->finished_probability_grid != nullptr
          ? insertion_submaps.front()
          : nullptr;

  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory]) {
    global_localization_samplers_[trajectory] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }

  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(j, matching_submap, insertion_submaps,
                              finished_submap, pose, covariance);
  });
}

void SparsePoseGraph::AddWorkItem(std::function<void()> work_item) {
  if (scan_queue_ == nullptr) {
    work_item();
  } else {
    scan_queue_->push_back(work_item);
  }
}

void SparsePoseGraph::AddImuData(const mapping::Submaps* trajectory,
                                 common::Time time,
                                 const Eigen::Vector3d& linear_acceleration,
                                 const Eigen::Vector3d& angular_velocity) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddImuData(trajectory, time, linear_acceleration,
                                     angular_velocity);
  });
}

void SparsePoseGraph::ComputeConstraint(const int scan_index,
                                        const int submap_index) {
  const transform::Rigid2d relative_pose =
      optimization_problem_.submap_data().at(submap_index).pose.inverse() *
      optimization_problem_.node_data().at(scan_index).point_cloud_pose;

  const mapping::Submaps* const scan_trajectory =
      trajectory_nodes_[scan_index].constant_data->trajectory;
  const mapping::Submaps* const submap_trajectory =
      submap_states_[submap_index].trajectory;
  // Only globally match against submaps not in this trajectory.
  if (scan_trajectory != submap_trajectory &&
      global_localization_samplers_[scan_trajectory]->Pulse()) {
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_index, submap_states_[submap_index].submap, scan_index,
        scan_trajectory, submap_trajectory, &trajectory_connectivity_,
        &trajectory_nodes_[scan_index].constant_data->range_data_2d.returns);
  } else {
    const bool scan_and_submap_trajectories_connected =
        reverse_connected_components_.count(scan_trajectory) > 0 &&
        reverse_connected_components_.count(submap_trajectory) > 0 &&
        reverse_connected_components_.at(scan_trajectory) ==
            reverse_connected_components_.at(submap_trajectory);
    if (scan_trajectory == submap_trajectory ||
        scan_and_submap_trajectories_connected) {
      constraint_builder_.MaybeAddConstraint(
          submap_index, submap_states_[submap_index].submap, scan_index,
          &trajectory_nodes_[scan_index].constant_data->range_data_2d.returns,
          relative_pose);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(
    const mapping::Submap* submap) {
  const int submap_index = GetSubmapIndex(submap);
  const auto& node_data = optimization_problem_.node_data();
  CHECK_GT(node_data.size(), 0);
  CHECK_LT(node_data.size(), std::numeric_limits<int>::max());
  const int num_nodes = node_data.size();
  for (int scan_index = 0; scan_index < num_nodes; ++scan_index) {
    if (submap_states_[submap_index].scan_indices.count(scan_index) == 0) {
      ComputeConstraint(scan_index, submap_index);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForScan(
    const int scan_index, const mapping::Submap* matching_submap,
    std::vector<const mapping::Submap*> insertion_submaps,
    const mapping::Submap* finished_submap, const transform::Rigid2d& pose,
    const kalman_filter::Pose2DCovariance& covariance) {
  GrowSubmapTransformsAsNeeded(insertion_submaps);
  const int matching_index = GetSubmapIndex(matching_submap);
  const transform::Rigid2d optimized_pose =
      optimization_problem_.submap_data().at(matching_index).pose *
      sparse_pose_graph::ComputeSubmapPose(*matching_submap).inverse() * pose;
  CHECK_EQ(scan_index, optimization_problem_.node_data().size());
  const mapping::TrajectoryNode::ConstantData* const scan_data =
      trajectory_nodes_[scan_index].constant_data;
  optimization_problem_.AddTrajectoryNode(
      scan_data->trajectory, scan_data->time, pose, optimized_pose);
  for (const mapping::Submap* submap : insertion_submaps) {
    const int submap_index = GetSubmapIndex(submap);
    CHECK(!submap_states_[submap_index].finished);
    submap_states_[submap_index].scan_indices.emplace(scan_index);
    // Unchanged covariance as (submap <- map) is a translation.
    const transform::Rigid2d constraint_transform =
        sparse_pose_graph::ComputeSubmapPose(*submap).inverse() * pose;
    constexpr double kFakePositionCovariance = 1e-6;
    constexpr double kFakeOrientationCovariance = 1e-6;
    constraints_.push_back(Constraint{
        submap_index,
        scan_index,
        {transform::Embed3D(constraint_transform),
         common::ComputeSpdMatrixSqrtInverse(
             kalman_filter::Embed3D(covariance, kFakePositionCovariance,
                                    kFakeOrientationCovariance),
             options_.constraint_builder_options()
                 .lower_covariance_eigenvalue_bound())},
        Constraint::INTRA_SUBMAP});
  }

  CHECK_LT(submap_states_.size(), std::numeric_limits<int>::max());
  const int num_submaps = submap_states_.size();
  for (int submap_index = 0; submap_index != num_submaps; ++submap_index) {
    if (submap_states_[submap_index].finished) {
      CHECK_EQ(submap_states_[submap_index].scan_indices.count(scan_index), 0);
      ComputeConstraint(scan_index, submap_index);
    }
  }

  if (finished_submap != nullptr) {
    const int finished_submap_index = GetSubmapIndex(finished_submap);
    SubmapState& finished_submap_state = submap_states_[finished_submap_index];
    CHECK(!finished_submap_state.finished);
    // We have a new completed submap, so we look into adding constraints for
    // old scans.
    ComputeConstraintsForOldScans(finished_submap);
    finished_submap_state.finished = true;
  }
  constraint_builder_.NotifyEndOfScan(scan_index);
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
        constraints_.insert(constraints_.end(), result.begin(), result.end());
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
        return static_cast<size_t>(constraint_builder_.GetNumFinishedScans()) ==
               trajectory_nodes_.size();
      },
      common::FromSeconds(1.))) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. *
                         (constraint_builder_.GetNumFinishedScans() -
                          num_finished_scans_at_start) /
                         (trajectory_nodes_.size() -
                          num_finished_scans_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone(
      [this, &notification](
          const sparse_pose_graph::ConstraintBuilder::Result& result) {
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        common::MutexLocker locker(&mutex_);
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
  const size_t num_optimized_poses = node_data.size();
  for (size_t i = 0; i != num_optimized_poses; ++i) {
    trajectory_nodes_[i].pose =
        transform::Rigid3d(transform::Embed3D(node_data[i].point_cloud_pose));
  }
  // Extrapolate all point cloud poses that were added later.
  std::unordered_map<const mapping::Submaps*, transform::Rigid3d>
      extrapolation_transforms;
  std::vector<transform::Rigid2d> submap_transforms;
  for (const auto& submap_data : optimization_problem_.submap_data()) {
    submap_transforms.push_back(submap_data.pose);
  }
  for (size_t i = num_optimized_poses; i != trajectory_nodes_.size(); ++i) {
    const mapping::Submaps* trajectory =
        trajectory_nodes_[i].constant_data->trajectory;
    if (extrapolation_transforms.count(trajectory) == 0) {
      const auto new_submap_transforms =
          ExtrapolateSubmapTransforms(submap_transforms, trajectory);
      const auto old_submap_transforms =
          ExtrapolateSubmapTransforms(optimized_submap_transforms_, trajectory);
      CHECK_EQ(new_submap_transforms.size(), old_submap_transforms.size());
      extrapolation_transforms[trajectory] =
          transform::Rigid3d(new_submap_transforms.back() *
                             old_submap_transforms.back().inverse());
    }
    trajectory_nodes_[i].pose =
        extrapolation_transforms[trajectory] * trajectory_nodes_[i].pose;
  }
  optimized_submap_transforms_ = submap_transforms;
  connected_components_ = trajectory_connectivity_.ConnectedComponents();
  reverse_connected_components_.clear();
  for (size_t i = 0; i != connected_components_.size(); ++i) {
    for (const auto* trajectory : connected_components_[i]) {
      reverse_connected_components_.emplace(trajectory, i);
    }
  }
}

std::vector<mapping::TrajectoryNode> SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_;
}

std::vector<SparsePoseGraph::SubmapState> SparsePoseGraph::GetSubmapStates() {
  common::MutexLocker locker(&mutex_);
  return submap_states_;
}

std::vector<SparsePoseGraph::Constraint> SparsePoseGraph::constraints() {
  common::MutexLocker locker(&mutex_);
  return constraints_;
}

const std::unordered_map<const mapping::Submaps*, int>&
SparsePoseGraph::trajectory_ids() {
  common::MutexLocker locker(&mutex_);
  return trajectory_ids_;
}

transform::Rigid3d SparsePoseGraph::GetLocalToGlobalTransform(
    const mapping::Submaps* const submaps) {
  const auto extrapolated_submap_transforms = GetSubmapTransforms(submaps);
  return extrapolated_submap_transforms.back() *
         submaps->Get(extrapolated_submap_transforms.size() - 1)
             ->local_pose()
             .inverse();
}

std::vector<std::vector<const mapping::Submaps*>>
SparsePoseGraph::GetConnectedTrajectories() {
  common::MutexLocker locker(&mutex_);
  return connected_components_;
}

std::vector<transform::Rigid3d> SparsePoseGraph::GetSubmapTransforms(
    const mapping::Submaps* const trajectory) {
  common::MutexLocker locker(&mutex_);
  return ExtrapolateSubmapTransforms(optimized_submap_transforms_, trajectory);
}

std::vector<transform::Rigid3d> SparsePoseGraph::ExtrapolateSubmapTransforms(
    const std::vector<transform::Rigid2d>& submap_transforms,
    const mapping::Submaps* const trajectory) const {
  std::vector<transform::Rigid3d> result;
  size_t flat_index = 0;
  size_t flat_index_of_result_back = -1;

  // Submaps for which we have optimized poses.
  for (; flat_index != submap_states_.size(); ++flat_index) {
    const auto& state = submap_states_[flat_index];
    if (state.trajectory != trajectory) {
      continue;
    }
    if (flat_index >= submap_transforms.size()) {
      break;
    }
    result.push_back(transform::Embed3D(submap_transforms[flat_index]));
    flat_index_of_result_back = flat_index;
  }

  // Extrapolate to the remaining submaps.
  for (; flat_index != submap_states_.size(); ++flat_index) {
    const auto& state = submap_states_[flat_index];
    if (state.trajectory != trajectory) {
      continue;
    }
    if (result.empty()) {
      result.push_back(transform::Rigid3d::Identity());
    } else {
      // Accessing local_pose() in Submaps is okay, since the member is const.
      result.push_back(result.back() *
                       submap_states_[flat_index_of_result_back]
                           .submap->local_pose()
                           .inverse() *
                       state.submap->local_pose());
    }
    flat_index_of_result_back = flat_index;
  }
  if (result.empty()) {
    result.push_back(transform::Rigid3d::Identity());
  }
  return result;
}

}  // namespace mapping_2d
}  // namespace cartographer
