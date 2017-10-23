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
#include <iterator>
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
  CHECK(work_queue_ == nullptr);
}

std::vector<mapping::SubmapId> SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_.submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      optimization_problem_.AddSubmap(
          trajectory_id,
          sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0]));
    }
    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
    const mapping::SubmapId submap_id{trajectory_id, 0};
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  const mapping::SubmapId last_submap_id = std::prev(end_it)->id;
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of 'insertions_submaps.front()'
    // and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id).pose;
    optimization_problem_.AddSubmap(
        trajectory_id,
        first_submap_pose *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[0])
                .inverse() *
            sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[1]));
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
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(trajectory_id);
  const mapping::NodeId node_id = trajectory_nodes_.Append(
      trajectory_id, mapping::TrajectoryNode{constant_data, optimized_pose});
  ++num_trajectory_nodes_;

  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (submap_data_.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(submap_data_.EndOfTrajectory(trajectory_id))->data.submap !=
          insertion_submaps.back()) {
    // We grow 'submap_data_' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const mapping::SubmapId submap_id =
        submap_data_.Append(trajectory_id, SubmapData());
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }

  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  const bool newly_finished_submap = insertion_submaps.front()->finished();
  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(node_id, insertion_submaps,
                              newly_finished_submap);
  });
}

void SparsePoseGraph::AddWorkItem(const std::function<void()>& work_item) {
  if (work_queue_ == nullptr) {
    work_item();
  } else {
    work_queue_->push_back(work_item);
  }
}

void SparsePoseGraph::AddTrajectoryIfNeeded(const int trajectory_id) {
  trajectory_connectivity_state_.Add(trajectory_id);
  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
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
  LOG(FATAL) << "Not yet implemented for 2D.";
}

void SparsePoseGraph::ComputeConstraint(const mapping::NodeId& node_id,
                                        const mapping::SubmapId& submap_id) {
  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  const common::Time scan_time = GetLatestScanTime(node_id, submap_id);
  const common::Time last_connection_time =
      trajectory_connectivity_state_.LastConnectionTime(
          node_id.trajectory_id, submap_id.trajectory_id);
  if (node_id.trajectory_id == submap_id.trajectory_id ||
      scan_time <
          last_connection_time +
              common::FromSeconds(
                  options_.global_constraint_search_after_n_seconds())) {
    // If the scan and the submap belong to the same trajectory or if there has
    // been a recent global constraint that ties that scan's trajectory to the
    // submap's trajectory, it suffices to do a match constrained to a local
    // search window.
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_.submap_data().at(submap_id).pose.inverse() *
        optimization_problem_.node_data().at(node_id).pose;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get(),
        initial_relative_pose);
  } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get());
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(
    const mapping::SubmapId& submap_id) {
  const auto& submap_data = submap_data_.at(submap_id);
  for (const auto& node_id_data : optimization_problem_.node_data()) {
    const mapping::NodeId& node_id = node_id_data.id;
    if (submap_data.node_ids.count(node_id) == 0) {
      ComputeConstraint(node_id, submap_id);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForScan(
    const mapping::NodeId& node_id,
    std::vector<std::shared_ptr<const Submap>> insertion_submaps,
    const bool newly_finished_submap) {
  const std::vector<mapping::SubmapId> submap_ids =
      GrowSubmapTransformsAsNeeded(node_id.trajectory_id, insertion_submaps);
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  const mapping::SubmapId matching_id = submap_ids.front();
  const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
  const transform::Rigid2d pose = transform::Project2D(
      constant_data->local_pose *
      transform::Rigid3d::Rotation(constant_data->gravity_alignment.inverse()));
  const transform::Rigid2d optimized_pose =
      optimization_problem_.submap_data().at(matching_id).pose *
      sparse_pose_graph::ComputeSubmapPose(*insertion_submaps.front())
          .inverse() *
      pose;
  optimization_problem_.AddTrajectoryNode(
      matching_id.trajectory_id, constant_data->time, pose, optimized_pose,
      constant_data->gravity_alignment);
  for (size_t i = 0; i < insertion_submaps.size(); ++i) {
    const mapping::SubmapId submap_id = submap_ids[i];
    // Even if this was the last scan added to 'submap_id', the submap will only
    // be marked as finished in 'submap_data_' further below.
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid2d constraint_transform =
        sparse_pose_graph::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
        pose;
    constraints_.push_back(Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                       options_.matcher_translation_weight(),
                                       options_.matcher_rotation_weight()},
                                      Constraint::INTRA_SUBMAP});
  }

  for (const auto& submap_id_data : submap_data_) {
    if (submap_id_data.data.state == SubmapState::kFinished) {
      CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
      ComputeConstraint(node_id, submap_id_data.id);
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

common::Time SparsePoseGraph::GetLatestScanTime(
    const mapping::NodeId& node_id, const mapping::SubmapId& submap_id) const {
  common::Time time = trajectory_nodes_.at(node_id).constant_data->time;
  const SubmapData& submap_data = submap_data_.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    const mapping::NodeId last_submap_node_id =
        *submap_data_.at(submap_id).node_ids.rbegin();
    time = std::max(
        time, trajectory_nodes_.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void SparsePoseGraph::UpdateTrajectoryConnectivity(
    const sparse_pose_graph::ConstraintBuilder::Result& result) {
  for (const Constraint& constraint : result) {
    CHECK_EQ(constraint.tag,
             mapping::SparsePoseGraph::Constraint::INTER_SUBMAP);
    const common::Time time =
        GetLatestScanTime(constraint.node_id, constraint.submap_id);
    trajectory_connectivity_state_.Connect(constraint.node_id.trajectory_id,
                                           constraint.submap_id.trajectory_id,
                                           time);
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
        UpdateTrajectoryConnectivity(result);
        TrimmingHandle trimming_handle(this);
        for (auto& trimmer : trimmers_) {
          trimmer->Trim(&trimming_handle);
        }

        num_scans_since_last_loop_closure_ = 0;
        run_loop_closure_ = false;
        while (!run_loop_closure_) {
          if (work_queue_->empty()) {
            work_queue_.reset();
            return;
          }
          work_queue_->front()();
          work_queue_->pop_front();
        }
        LOG(INFO) << "Remaining work items in queue: " << work_queue_->size();
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
  trajectory_connectivity_state_.Add(trajectory_id);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

void SparsePoseGraph::AddSubmapFromProto(const transform::Rigid3d& global_pose,
                                         const mapping::proto::Submap& submap) {
  if (!submap.has_submap_2d()) {
    return;
  }

  const mapping::SubmapId submap_id = {submap.submap_id().trajectory_id(),
                                       submap.submap_id().submap_index()};
  std::shared_ptr<const Submap> submap_ptr =
      std::make_shared<const Submap>(submap.submap_2d());
  const transform::Rigid2d global_pose_2d = transform::Project2D(global_pose);

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(submap_id.trajectory_id);
  submap_data_.Insert(submap_id, SubmapData());
  submap_data_.at(submap_id).submap = submap_ptr;
  // Immediately show the submap at the optimized pose.
  optimized_submap_transforms_.Insert(
      submap_id, sparse_pose_graph::SubmapData{global_pose_2d});
  AddWorkItem([this, submap_id, global_pose_2d]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(submap_id.trajectory_id), 1);
    submap_data_.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_.InsertSubmap(submap_id, global_pose_2d);
  });
}

void SparsePoseGraph::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                       const mapping::proto::Node& node) {
  const mapping::NodeId node_id = {node.node_id().trajectory_id(),
                                   node.node_id().node_index()};
  std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data =
      std::make_shared<const mapping::TrajectoryNode::Data>(
          mapping::FromProto(node.node_data()));

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(node_id.trajectory_id);
  trajectory_nodes_.Insert(node_id,
                           mapping::TrajectoryNode{constant_data, global_pose});

  AddWorkItem([this, node_id, global_pose]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(node_id.trajectory_id), 1);
    const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
    const auto gravity_alignment_inverse = transform::Rigid3d::Rotation(
        constant_data->gravity_alignment.inverse());
    optimization_problem_.InsertTrajectoryNode(
        node_id, constant_data->time,
        transform::Project2D(constant_data->local_pose *
                             gravity_alignment_inverse),
        transform::Project2D(global_pose * gravity_alignment_inverse),
        constant_data->gravity_alignment);
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

void SparsePoseGraph::RunOptimization() {
  if (optimization_problem_.submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_, constraints_ and
  // frozen_trajectories_ when executing the Solve. Solve is time consuming, so
  // not taking the mutex before Solve to avoid blocking foreground processing.
  optimization_problem_.Solve(constraints_, frozen_trajectories_);
  common::MutexLocker locker(&mutex_);

  const auto& submap_data = optimization_problem_.submap_data();
  const auto& node_data = optimization_problem_.node_data();
  for (const int trajectory_id : node_data.trajectory_ids()) {
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      auto& mutable_trajectory_node = trajectory_nodes_.at(node.id);
      mutable_trajectory_node.global_pose =
          transform::Embed3D(node.data.pose) *
          transform::Rigid3d::Rotation(
              mutable_trajectory_node.constant_data->gravity_alignment);
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        optimized_submap_transforms_, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    const mapping::NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    auto node_it = std::next(trajectory_nodes_.find(last_optimized_node_id));
    for (; node_it != trajectory_nodes_.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = trajectory_nodes_.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }
  optimized_submap_transforms_ = submap_data;
}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNode>
SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_;
}

std::vector<SparsePoseGraph::Constraint> SparsePoseGraph::constraints() {
  std::vector<Constraint> result;
  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : constraints_) {
    result.push_back(Constraint{
        constraint.submap_id, constraint.node_id,
        Constraint::Pose{constraint.pose.zbar_ij *
                             transform::Rigid3d::Rotation(
                                 trajectory_nodes_.at(constraint.node_id)
                                     .constant_data->gravity_alignment),
                         constraint.pose.translation_weight,
                         constraint.pose.rotation_weight},
        constraint.tag});
  }
  return result;
}

transform::Rigid3d SparsePoseGraph::GetLocalToGlobalTransform(
    const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  return ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                       trajectory_id);
}

std::vector<std::vector<int>> SparsePoseGraph::GetConnectedTrajectories() {
  return trajectory_connectivity_state_.Components();
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapData(
    const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

mapping::MapById<mapping::SubmapId, mapping::SparsePoseGraph::SubmapData>
SparsePoseGraph::GetAllSubmapData() {
  common::MutexLocker locker(&mutex_);
  mapping::MapById<mapping::SubmapId, mapping::SparsePoseGraph::SubmapData>
      submaps;
  for (const auto& submap_id_data : submap_data_) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

transform::Rigid3d SparsePoseGraph::ComputeLocalToGlobalTransform(
    const mapping::MapById<mapping::SubmapId, sparse_pose_graph::SubmapData>&
        submap_transforms,
    const int trajectory_id) const {
  auto begin_it = submap_transforms.BeginOfTrajectory(trajectory_id);
  auto end_it = submap_transforms.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    return transform::Rigid3d::Identity();
  }
  const mapping::SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return transform::Embed3D(
             submap_transforms.at(last_optimized_submap_id).pose) *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapDataUnderLock(
    const mapping::SubmapId& submap_id) {
  const auto it = submap_data_.find(submap_id);
  if (it == submap_data_.end()) {
    return {};
  }
  auto submap = it->data.submap;
  if (optimized_submap_transforms_.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap, transform::Embed3D(
                        optimized_submap_transforms_.at(submap_id).pose)};
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
  const auto& submap_data = parent_->optimization_problem_.submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
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
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);
  parent_->submap_data_.Trim(submap_id);
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_.TrimSubmap(submap_id);

  // Remove the 'nodes_to_remove' from the sparse pose graph and the
  // optimization problem.
  for (const mapping::NodeId& node_id : nodes_to_remove) {
    parent_->trajectory_nodes_.Trim(node_id);
    parent_->optimization_problem_.TrimTrajectoryNode(node_id);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
