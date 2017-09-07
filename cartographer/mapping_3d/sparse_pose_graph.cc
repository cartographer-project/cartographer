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
  CHECK(work_queue_ == nullptr);
}

std::vector<mapping::SubmapId> SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_.submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (static_cast<size_t>(trajectory_id) >= submap_data.size() ||
        submap_data[trajectory_id].empty()) {
      optimization_problem_.AddSubmap(trajectory_id,
                                      insertion_submaps[0]->local_pose());
    }
    const mapping::SubmapId submap_id{
        trajectory_id, static_cast<int>(submap_data[trajectory_id].size()) - 1};
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const mapping::SubmapId last_submap_id{
      trajectory_id,
      static_cast<int>(submap_data.at(trajectory_id).size() - 1)};
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of 'insertions_submaps.front()'
    // and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose =
        submap_data.at(trajectory_id).at(last_submap_id.submap_index).pose;
    optimization_problem_.AddSubmap(
        trajectory_id, first_submap_pose *
                           insertion_submaps[0]->local_pose().inverse() *
                           insertion_submaps[1]->local_pose());
    return {last_submap_id,
            mapping::SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
  const mapping::SubmapId front_submap_id{trajectory_id,
                                          last_submap_id.submap_index - 1};
  CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

void SparsePoseGraph::AddScan(
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data,
    const transform::Rigid3d& pose, const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * pose);
  common::MutexLocker locker(&mutex_);
  trajectory_nodes_.Append(
      trajectory_id, mapping::TrajectoryNode{constant_data, optimized_pose});
  ++num_trajectory_nodes_;
  trajectory_connectivity_.Add(trajectory_id);

  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (trajectory_id >= submap_data_.num_trajectories() ||
      submap_data_.num_indices(trajectory_id) == 0 ||
      submap_data_
              .at(mapping::SubmapId{
                  trajectory_id, submap_data_.num_indices(trajectory_id) - 1})
              .submap != insertion_submaps.back()) {
    // We grow 'submap_data_' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const mapping::SubmapId submap_id =
        submap_data_.Append(trajectory_id, SubmapData());
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }

  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }

  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  const bool newly_finished_submap = insertion_submaps.front()->finished();
  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(trajectory_id, insertion_submaps,
                              newly_finished_submap, pose);
  });
}

void SparsePoseGraph::AddWorkItem(const std::function<void()>& work_item) {
  if (work_queue_ == nullptr) {
    work_item();
  } else {
    work_queue_->push_back(work_item);
  }
}

void SparsePoseGraph::AddImuData(const int trajectory_id,
                                 const sensor::ImuData& imu_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddImuData(trajectory_id, imu_data);
  });
}

void SparsePoseGraph::AddOdometerData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddOdometerData(trajectory_id, odometry_data);
  });
}

void SparsePoseGraph::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddFixedFramePoseData(trajectory_id,
                                                fixed_frame_pose_data);
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
                                .pose;

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
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get(), submap_nodes,
        initial_relative_pose.rotation(), &trajectory_connectivity_);
  } else {
    const bool scan_and_submap_trajectories_connected =
        reverse_connected_components_.count(node_id.trajectory_id) > 0 &&
        reverse_connected_components_.count(submap_id.trajectory_id) > 0 &&
        reverse_connected_components_.at(node_id.trajectory_id) ==
            reverse_connected_components_.at(submap_id.trajectory_id);
    if (node_id.trajectory_id == submap_id.trajectory_id ||
        scan_and_submap_trajectories_connected) {
      constraint_builder_.MaybeAddConstraint(
          submap_id, submap_data_.at(submap_id).submap.get(), node_id,
          trajectory_nodes_.at(node_id).constant_data.get(), submap_nodes,
          initial_relative_pose);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(
    const mapping::SubmapId& submap_id) {
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
    const int trajectory_id,
    std::vector<std::shared_ptr<const Submap>> insertion_submaps,
    const bool newly_finished_submap, const transform::Rigid3d& pose) {
  const std::vector<mapping::SubmapId> submap_ids =
      GrowSubmapTransformsAsNeeded(trajectory_id, insertion_submaps);
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  const mapping::SubmapId matching_id = submap_ids.front();
  const transform::Rigid3d optimized_pose =
      optimization_problem_.submap_data()
          .at(matching_id.trajectory_id)
          .at(matching_id.submap_index)
          .pose *
      insertion_submaps.front()->local_pose().inverse() * pose;
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
  for (size_t i = 0; i < insertion_submaps.size(); ++i) {
    const mapping::SubmapId submap_id = submap_ids[i];
    // Even if this was the last scan added to 'submap_id', the submap will only
    // be marked as finished in 'submap_data_' further below.
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid3d constraint_transform =
        insertion_submaps[i]->local_pose().inverse() * pose;
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

  if (newly_finished_submap) {
    const mapping::SubmapId finished_submap_id = submap_ids.front();
    SubmapData& finished_submap_data = submap_data_.at(finished_submap_id);
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old scans.
    ComputeConstraintsForOldScans(finished_submap_id);
  }
  constraint_builder_.NotifyEndOfScan();
  ++num_scans_since_last_loop_closure_;
  if (options_.optimize_every_n_scans() > 0 &&
      num_scans_since_last_loop_closure_ > options_.optimize_every_n_scans()) {
    CHECK(!run_loop_closure_);
    run_loop_closure_ = true;
    // If there is a 'work_queue_' already, some other thread will take care.
    if (work_queue_ == nullptr) {
      work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
      HandleWorkQueue();
    }
  }
}

void SparsePoseGraph::HandleWorkQueue() {
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
          if (work_queue_->empty()) {
            LOG(INFO) << "We caught up. Hooray!";
            work_queue_.reset();
            return;
          }
          work_queue_->front()();
          work_queue_->pop_front();
        }
        // We have to optimize again.
        HandleWorkQueue();
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

void SparsePoseGraph::FreezeTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

void SparsePoseGraph::AddSubmapFromProto(const int trajectory_id,
                                         const transform::Rigid3d& initial_pose,
                                         const mapping::proto::Submap& submap) {
  if (!submap.has_submap_3d()) {
    return;
  }

  std::shared_ptr<const Submap> submap_ptr =
      std::make_shared<const Submap>(submap.submap_3d());

  common::MutexLocker locker(&mutex_);
  const mapping::SubmapId submap_id =
      submap_data_.Append(trajectory_id, SubmapData());
  submap_data_.at(submap_id).submap = submap_ptr;
  // Immediately show the submap at the optimized pose.
  CHECK_GE(static_cast<size_t>(submap_data_.num_trajectories()),
           optimized_submap_transforms_.size());
  optimized_submap_transforms_.resize(submap_data_.num_trajectories());
  CHECK_EQ(optimized_submap_transforms_.at(trajectory_id).size(),
           submap_id.submap_index);
  optimized_submap_transforms_.at(trajectory_id)
      .push_back(sparse_pose_graph::SubmapData{initial_pose});
  AddWorkItem([this, submap_id, initial_pose]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(submap_id.trajectory_id), 1);
    submap_data_.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_.AddSubmap(submap_id.trajectory_id, initial_pose);
  });
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

void SparsePoseGraph::LogResidualHistograms() {
  common::Histogram rotational_residual;
  common::Histogram translational_residual;
  for (const Constraint& constraint : constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
      const cartographer::transform::Rigid3d optimized_node_to_map =
          trajectory_nodes_.at(constraint.node_id).pose;
      const cartographer::transform::Rigid3d node_to_submap_constraint =
          constraint.pose.zbar_ij;
      const cartographer::transform::Rigid3d optimized_submap_to_map =
          optimized_submap_transforms_.at(constraint.submap_id.trajectory_id)
              .at(constraint.submap_id.submap_index)
              .pose;
      const cartographer::transform::Rigid3d optimized_node_to_submap =
          optimized_submap_to_map.inverse() * optimized_node_to_map;
      const cartographer::transform::Rigid3d residual =
          node_to_submap_constraint.inverse() * optimized_node_to_submap;
      rotational_residual.Add(
          common::NormalizeAngleDifference(transform::GetAngle(residual)));
      translational_residual.Add(residual.translation().norm());
    }
  }
  LOG(INFO) << "Translational residuals histogram:\n"
            << translational_residual.ToString(10);
  LOG(INFO) << "Rotational residuals histogram:\n"
            << rotational_residual.ToString(10);
}

void SparsePoseGraph::RunOptimization() {
  if (optimization_problem_.submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_, constraints_ and
  // frozen_trajectories_ when executing the Solve. Solve is time consuming, so
  // not taking the mutex before Solve to avoid blocking foreground processing.
  optimization_problem_.Solve(constraints_, frozen_trajectories_);
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
          node_data[trajectory_id][node_index].pose;
    }
    // Extrapolate all point cloud poses that were added later.
    const auto local_to_new_global = ComputeLocalToGlobalTransform(
        optimization_problem_.submap_data(), trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        optimized_submap_transforms_, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();
    for (; node_index < num_nodes; ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      trajectory_nodes_.at(node_id).pose =
          old_global_to_new_global * trajectory_nodes_.at(node_id).pose;
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

  // Log the histograms for the pose residuals.
  if (options_.log_residual_histograms()) {
    LogResidualHistograms();
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
  return ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                       trajectory_id);
}

std::vector<std::vector<int>> SparsePoseGraph::GetConnectedTrajectories() {
  common::MutexLocker locker(&mutex_);
  return connected_components_;
}

int SparsePoseGraph::num_submaps(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  if (trajectory_id >= submap_data_.num_trajectories()) {
    return 0;
  }
  return submap_data_.num_indices(trajectory_id);
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapData(
    const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
SparsePoseGraph::GetAllSubmapData() {
  common::MutexLocker locker(&mutex_);
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
      all_submap_data(submap_data_.num_trajectories());
  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    all_submap_data[trajectory_id].reserve(
        submap_data_.num_indices(trajectory_id));
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      all_submap_data[trajectory_id].emplace_back(GetSubmapDataUnderLock(
          mapping::SubmapId{trajectory_id, submap_index}));
    }
  }
  return all_submap_data;
}

transform::Rigid3d SparsePoseGraph::ComputeLocalToGlobalTransform(
    const std::vector<std::vector<sparse_pose_graph::SubmapData>>&
        submap_transforms,
    const int trajectory_id) const {
  if (trajectory_id >= static_cast<int>(submap_transforms.size()) ||
      submap_transforms.at(trajectory_id).empty()) {
    return transform::Rigid3d::Identity();
  }
  const mapping::SubmapId last_optimized_submap_id{
      trajectory_id,
      static_cast<int>(submap_transforms.at(trajectory_id).size() - 1)};
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return submap_transforms.at(trajectory_id).back().pose *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapDataUnderLock(
    const mapping::SubmapId& submap_id) {
  auto submap = submap_data_.at(submap_id).submap;
  // We already have an optimized pose.
  if (submap_id.trajectory_id <
          static_cast<int>(optimized_submap_transforms_.size()) &&
      submap_id.submap_index < static_cast<int>(optimized_submap_transforms_
                                                    .at(submap_id.trajectory_id)
                                                    .size())) {
    return {submap, optimized_submap_transforms_.at(submap_id.trajectory_id)
                        .at(submap_id.submap_index)
                        .pose};
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

SparsePoseGraph::TrimmingHandle::TrimmingHandle(SparsePoseGraph* const parent)
    : parent_(parent) {}

int SparsePoseGraph::TrimmingHandle::num_submaps(
    const int trajectory_id) const {
  LOG(FATAL) << "Not yet implemented for 3D.";
}

void SparsePoseGraph::TrimmingHandle::MarkSubmapAsTrimmed(
    const mapping::SubmapId& submap_id) {
  LOG(FATAL) << "Not yet implemented for 3D.";
}

}  // namespace mapping_3d
}  // namespace cartographer
