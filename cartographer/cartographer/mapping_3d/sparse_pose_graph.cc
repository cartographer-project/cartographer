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
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/proto/scan_matching_progress.pb.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/sensor/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

SparsePoseGraph::SparsePoseGraph(
    const mapping::proto::SparsePoseGraphOptions& options,
    common::ThreadPool* thread_pool,
    std::deque<mapping::TrajectoryNode::ConstantData>* constant_node_data)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options()),
      constraint_builder_(options_.constraint_builder_options(), thread_pool),
      constant_node_data_(constant_node_data) {}

SparsePoseGraph::~SparsePoseGraph() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(scan_queue_ == nullptr);
}

void SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const std::vector<const Submap*>& submaps) {
  CHECK(!submaps.empty());
  CHECK_LT(submap_transforms_.size(), std::numeric_limits<int>::max());
  const int next_transform_index = submap_transforms_.size();
  // Verify that we have an index for the first submap.
  const int first_transform_index = GetSubmapIndex(submaps[0]);
  if (submaps.size() == 1) {
    // If we don't already have an entry for this submap, add one.
    if (first_transform_index == next_transform_index) {
      submap_transforms_.push_back(transform::Rigid3d::Identity());
    }
    return;
  }
  CHECK_EQ(2, submaps.size());
  // CHECK that we have a index for the second submap.
  const int second_transform_index = GetSubmapIndex(submaps[1]);
  CHECK_LE(second_transform_index, next_transform_index);
  // Extrapolate if necessary.
  if (second_transform_index == next_transform_index) {
    const auto& first_submap_transform =
        submap_transforms_[first_transform_index];
    submap_transforms_.push_back(first_submap_transform *
                                 submaps[0]->local_pose().inverse() *
                                 submaps[1]->local_pose());
  }
}

int SparsePoseGraph::AddScan(
    common::Time time, const sensor::LaserFan3D& laser_fan_in_tracking,
    const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance, const Submaps* submaps,
    const Submap* const matching_submap,
    const std::vector<const Submap*>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(GetOdometryToMapTransform(*submaps) *
                                          pose);

  common::MutexLocker locker(&mutex_);
  const int j = trajectory_nodes_.size();
  CHECK_LT(j, std::numeric_limits<int>::max());

  constant_node_data_->push_back(mapping::TrajectoryNode::ConstantData{
      time, sensor::LaserFan{Eigen::Vector2f::Zero(), {}, {}},
      sensor::Compress(laser_fan_in_tracking), submaps,
      transform::Rigid3d::Identity()});
  trajectory_nodes_.push_back(
      mapping::TrajectoryNode{&constant_node_data_->back(), optimized_pose});
  trajectory_connectivity_.Add(submaps);

  if (submap_indices_.count(insertion_submaps.back()) == 0) {
    submap_indices_.emplace(insertion_submaps.back(),
                            static_cast<int>(submap_indices_.size()));
    submap_states_.emplace_back();
    submap_states_.back().submap = insertion_submaps.back();
    submap_states_.back().trajectory = submaps;
    CHECK_EQ(submap_states_.size(), submap_indices_.size());
  }
  const Submap* const finished_submap =
      insertion_submaps.front()->finished ? insertion_submaps.front() : nullptr;

  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[submaps]) {
    global_localization_samplers_[submaps] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }

  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(time, j, submaps, matching_submap,
                              insertion_submaps, finished_submap, pose,
                              covariance);
  });
  return j;
}

void SparsePoseGraph::AddWorkItem(std::function<void()> work_item) {
  if (scan_queue_ == nullptr) {
    work_item();
  } else {
    scan_queue_->push_back(work_item);
  }
}

void SparsePoseGraph::AddImuData(common::Time time,
                                 const Eigen::Vector3d& linear_acceleration,
                                 const Eigen::Vector3d& angular_velocity) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddImuData(time, linear_acceleration,
                                     angular_velocity);
  });
}

void SparsePoseGraph::ComputeConstraintsForOldScans(const Submap* submap) {
  const int submap_index = GetSubmapIndex(submap);
  const auto& node_data = optimization_problem_.node_data();
  CHECK_GT(node_data.size(), 0);
  CHECK_LT(node_data.size(), std::numeric_limits<int>::max());
  const int num_nodes = node_data.size();
  for (int scan_index = 0; scan_index < num_nodes; ++scan_index) {
    if (submap_states_[submap_index].scan_indices.count(scan_index) == 0) {
      const transform::Rigid3d relative_pose =
          submap_transforms_[submap_index].inverse() *
          node_data[scan_index].point_cloud_pose;
      constraint_builder_.MaybeAddConstraint(submap_index, submap, scan_index,
                                             trajectory_nodes_, relative_pose);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForScan(
    const common::Time time, const int scan_index,
    const Submaps* scan_trajectory, const Submap* matching_submap,
    std::vector<const Submap*> insertion_submaps, const Submap* finished_submap,
    const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  GrowSubmapTransformsAsNeeded(insertion_submaps);
  const int matching_index = GetSubmapIndex(matching_submap);
  const transform::Rigid3d optimized_pose =
      submap_transforms_[matching_index] *
      matching_submap->local_pose().inverse() * pose;
  CHECK_EQ(scan_index, optimization_problem_.node_data().size());
  optimization_problem_.AddTrajectoryNode(time, pose, optimized_pose);
  for (const Submap* submap : insertion_submaps) {
    const int submap_index = GetSubmapIndex(submap);
    CHECK(!submap_states_[submap_index].finished);
    submap_states_[submap_index].scan_indices.emplace(scan_index);
    // Unchanged covariance as (submap <- map) is a translation.
    const transform::Rigid3d constraint_transform =
        submap->local_pose().inverse() * pose;
    constraints_.push_back(Constraint3D{
        submap_index,
        scan_index,
        {constraint_transform,
         common::ComputeSpdMatrixSqrtInverse(
             covariance,
             options_.constraint_builder_options().max_covariance_trace(),
             options_.constraint_builder_options()
                 .lower_covariance_eigenvalue_bound())},
        Constraint3D::INTRA_SUBMAP});
  }

  // Determine if this scan should be globally localized.
  const bool run_global_localization =
      global_localization_samplers_[scan_trajectory]->Pulse();

  CHECK_LT(submap_states_.size(), std::numeric_limits<int>::max());
  const int num_submaps = submap_states_.size();
  for (int submap_index = 0; submap_index != num_submaps; ++submap_index) {
    if (submap_states_[submap_index].finished) {
      CHECK_EQ(submap_states_[submap_index].scan_indices.count(scan_index), 0);
      const transform::Rigid3d relative_pose =
          submap_transforms_[submap_index].inverse() * optimized_pose;

      const auto* submap_trajectory = submap_states_[submap_index].trajectory;

      // Only globally match against submaps not in this trajectory.
      if (run_global_localization && scan_trajectory != submap_trajectory) {
        constraint_builder_.MaybeAddGlobalConstraint(
            submap_index, submap_states_[submap_index].submap, scan_index,
            scan_trajectory, submap_trajectory, &trajectory_connectivity_,
            trajectory_nodes_);
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
              trajectory_nodes_, relative_pose);
        }
      }
    }
  }

  if (finished_submap != nullptr) {
    const int finished_submap_index = GetSubmapIndex(finished_submap);
    SubmapState& finished_submap_state = submap_states_[finished_submap_index];
    CHECK(!finished_submap_state.finished);
    if (options_.also_match_to_new_submaps()) {
      // We have a new completed submap, so we look into adding constraints for
      // old scans.
      ComputeConstraintsForOldScans(finished_submap);
    }
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
      scan_queue_.reset(new std::deque<std::function<void()>>);
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
                  << 100. * (constraint_builder_.GetNumFinishedScans() -
                             num_finished_scans_at_start) /
                         (trajectory_nodes_.size() -
                          num_finished_scans_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone([this, &notification](
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
}

void SparsePoseGraph::RunOptimization() {
  if (!submap_transforms_.empty()) {
    transform::Rigid3d submap_0_pose;
    std::vector<const mapping::Submaps*> trajectories;
    {
      common::MutexLocker locker(&mutex_);
      CHECK(!submap_states_.empty());
      submap_0_pose = submap_states_.front().submap->local_pose();
      for (const auto& trajectory_node : trajectory_nodes_) {
        trajectories.push_back(trajectory_node.constant_data->trajectory);
      }
    }

    optimization_problem_.Solve(constraints_, submap_0_pose, trajectories,
                                &submap_transforms_);
    common::MutexLocker locker(&mutex_);
    has_new_optimized_poses_ = true;

    const auto& node_data = optimization_problem_.node_data();
    const size_t num_optimized_poses = node_data.size();
    for (size_t i = 0; i != num_optimized_poses; ++i) {
      trajectory_nodes_[i].pose =
          transform::Rigid3d(node_data[i].point_cloud_pose);
    }
    // Extrapolate all point cloud poses that were added later.
    std::unordered_map<const mapping::Submaps*, transform::Rigid3d>
        extrapolation_transforms;
    for (size_t i = num_optimized_poses; i != trajectory_nodes_.size(); ++i) {
      const mapping::Submaps* trajectory =
          trajectory_nodes_[i].constant_data->trajectory;
      if (extrapolation_transforms.count(trajectory) == 0) {
        extrapolation_transforms[trajectory] = transform::Rigid3d(
            ExtrapolateSubmapTransforms(submap_transforms_, *trajectory)
                .back() *
            ExtrapolateSubmapTransforms(optimized_submap_transforms_,
                                        *trajectory)
                .back()
                .inverse());
      }
      trajectory_nodes_[i].pose =
          extrapolation_transforms[trajectory] * trajectory_nodes_[i].pose;
    }
    optimized_submap_transforms_ = submap_transforms_;
    connected_components_ = trajectory_connectivity_.ConnectedComponents();
    reverse_connected_components_.clear();
    for (size_t i = 0; i != connected_components_.size(); ++i) {
      for (const auto* trajectory : connected_components_[i]) {
        reverse_connected_components_.emplace(trajectory, i);
      }
    }
  }
}

bool SparsePoseGraph::HasNewOptimizedPoses() {
  common::MutexLocker locker(&mutex_);
  if (!has_new_optimized_poses_) {
    return false;
  }
  has_new_optimized_poses_ = false;
  return true;
}

mapping::proto::ScanMatchingProgress
SparsePoseGraph::GetScanMatchingProgress() {
  mapping::proto::ScanMatchingProgress progress;
  common::MutexLocker locker(&mutex_);
  progress.set_num_scans_total(trajectory_nodes_.size());
  progress.set_num_scans_finished(constraint_builder_.GetNumFinishedScans());
  return progress;
}

std::vector<mapping::TrajectoryNode> SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_;
}

std::vector<SparsePoseGraph::Constraint2D> SparsePoseGraph::constraints_2d() {
  return {};
}

std::vector<SparsePoseGraph::Constraint3D> SparsePoseGraph::constraints_3d() {
  return constraints_;
}

transform::Rigid3d SparsePoseGraph::GetOdometryToMapTransform(
    const mapping::Submaps& submaps) {
  return GetSubmapTransforms(submaps).back() *
         submaps.Get(submaps.size() - 1)->local_pose().inverse();
}

std::vector<std::vector<const mapping::Submaps*>>
SparsePoseGraph::GetConnectedTrajectories() {
  common::MutexLocker locker(&mutex_);
  return connected_components_;
}

std::vector<transform::Rigid3d> SparsePoseGraph::GetSubmapTransforms(
    const mapping::Submaps& submaps) {
  common::MutexLocker locker(&mutex_);
  return ExtrapolateSubmapTransforms(optimized_submap_transforms_, submaps);
}

std::vector<transform::Rigid3d> SparsePoseGraph::ExtrapolateSubmapTransforms(
    const std::vector<transform::Rigid3d>& submap_transforms,
    const mapping::Submaps& submaps) const {
  std::vector<transform::Rigid3d> result;
  int i = 0;
  // Submaps for which we have optimized poses.
  for (; i < submaps.size(); ++i) {
    const mapping::Submap* submap = submaps.Get(i);
    const auto submap_and_index = submap_indices_.find(submap);
    // Determine if we have a loop-closed transform for this submap.
    if (submap_and_index == submap_indices_.end() ||
        static_cast<size_t>(submap_and_index->second) >=
            submap_transforms.size()) {
      break;
    }
    result.push_back(submap_transforms[submap_and_index->second]);
  }

  // Extrapolate to the remaining submaps.
  for (; i < submaps.size(); ++i) {
    if (i == 0) {
      result.push_back(transform::Rigid3d::Identity());
      continue;
    }
    result.push_back(result.back() *
                     submaps.Get(result.size() - 1)->local_pose().inverse() *
                     submaps.Get(result.size())->local_pose());
  }
  return result;
}

}  // namespace mapping_3d
}  // namespace cartographer
