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

#include "cartographer/mapping/internal/3d/pose_graph_3d.h"

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
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseGraph3D::PoseGraph3D(
    const proto::PoseGraphOptions& options,
    std::unique_ptr<optimization::OptimizationProblem3D> optimization_problem,
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

PoseGraph3D::~PoseGraph3D() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(work_queue_ == nullptr);
}

std::vector<SubmapId> PoseGraph3D::InitializeGlobalSubmapPoses(
    const int trajectory_id, const common::Time time,
    const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_->submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      if (initial_trajectory_poses_.count(trajectory_id) > 0) {
        trajectory_connectivity_state_.Connect(
            trajectory_id,
            initial_trajectory_poses_.at(trajectory_id).to_trajectory_id, time);
      }
      optimization_problem_->AddSubmap(
          trajectory_id,
          ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id) *
              insertion_submaps[0]->local_pose());
    }
    CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));
    const SubmapId submap_id{trajectory_id, 0};
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  const SubmapId last_submap_id = std::prev(end_it)->id;
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of 'insertions_submaps.front()'
    // and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
    optimization_problem_->AddSubmap(
        trajectory_id, first_submap_pose *
                           insertion_submaps[0]->local_pose().inverse() *
                           insertion_submaps[1]->local_pose());
    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

NodeId PoseGraph3D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps) {
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(trajectory_id);
  const NodeId node_id = trajectory_nodes_.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  ++num_trajectory_nodes_;

  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (submap_data_.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(submap_data_.EndOfTrajectory(trajectory_id))->data.submap !=
          insertion_submaps.back()) {
    // We grow 'submap_data_' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const SubmapId submap_id =
        submap_data_.Append(trajectory_id, InternalSubmapData());
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }

  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  const bool newly_finished_submap = insertion_submaps.front()->finished();
  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForNode(node_id, insertion_submaps,
                              newly_finished_submap);
  });
  return node_id;
}

void PoseGraph3D::AddWorkItem(const std::function<void()>& work_item) {
  if (work_queue_ == nullptr) {
    work_item();
  } else {
    work_queue_->push_back(work_item);
  }
}

void PoseGraph3D::AddTrajectoryIfNeeded(const int trajectory_id) {
  trajectory_connectivity_state_.Add(trajectory_id);
  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }
}

void PoseGraph3D::AddImuData(const int trajectory_id,
                             const sensor::ImuData& imu_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_->AddImuData(trajectory_id, imu_data);
  });
}

void PoseGraph3D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
  });
}

void PoseGraph3D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_->AddFixedFramePoseData(trajectory_id,
                                                 fixed_frame_pose_data);
  });
}

void PoseGraph3D::AddLandmarkData(int trajectory_id,
                                  const sensor::LandmarkData& landmark_data)
    EXCLUDES(mutex_) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    for (const auto& observation : landmark_data.landmark_observations) {
      landmark_nodes_[observation.id].landmark_observations.emplace_back(
          PoseGraphInterface::LandmarkNode::LandmarkObservation{
              trajectory_id, landmark_data.time,
              observation.landmark_to_tracking_transform,
              observation.translation_weight, observation.rotation_weight});
    }
  });
}

void PoseGraph3D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) {
  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  const transform::Rigid3d global_node_pose =
      optimization_problem_->node_data().at(node_id).global_pose;

  const transform::Rigid3d global_submap_pose =
      optimization_problem_->submap_data().at(submap_id).global_pose;

  const transform::Rigid3d global_submap_pose_inverse =
      global_submap_pose.inverse();

  std::vector<TrajectoryNode> submap_nodes;
  for (const NodeId& submap_node_id : submap_data_.at(submap_id).node_ids) {
    submap_nodes.push_back(
        TrajectoryNode{trajectory_nodes_.at(submap_node_id).constant_data,
                       global_submap_pose_inverse *
                           trajectory_nodes_.at(submap_node_id).global_pose});
  }

  const common::Time node_time = GetLatestNodeTime(node_id, submap_id);
  const common::Time last_connection_time =
      trajectory_connectivity_state_.LastConnectionTime(
          node_id.trajectory_id, submap_id.trajectory_id);
  if (node_id.trajectory_id == submap_id.trajectory_id ||
      node_time <
          last_connection_time +
              common::FromSeconds(
                  options_.global_constraint_search_after_n_seconds())) {
    // If the node and the submap belong to the same trajectory or if there has
    // been a recent global constraint that ties that node's trajectory to the
    // submap's trajectory, it suffices to do a match constrained to a local
    // search window.
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get(), submap_nodes,
        global_node_pose, global_submap_pose);
  } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    // In this situation, 'global_node_pose' and 'global_submap_pose' have
    // orientations agreeing on gravity. Their relationship regarding yaw is
    // arbitrary. Finding the correct yaw component will be handled by the
    // matching procedure in the FastCorrelativeScanMatcher, and the given yaw
    // is essentially ignored.
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        trajectory_nodes_.at(node_id).constant_data.get(), submap_nodes,
        global_node_pose.rotation(), global_submap_pose.rotation());
  }
}

void PoseGraph3D::ComputeConstraintsForOldNodes(const SubmapId& submap_id) {
  const auto& submap_data = submap_data_.at(submap_id);
  for (const auto& node_id_data : optimization_problem_->node_data()) {
    const NodeId& node_id = node_id_data.id;
    if (submap_data.node_ids.count(node_id) == 0) {
      ComputeConstraint(node_id, submap_id);
    }
  }
}

void PoseGraph3D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const Submap3D>> insertion_submaps,
    const bool newly_finished_submap) {
  const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
  const std::vector<SubmapId> submap_ids = InitializeGlobalSubmapPoses(
      node_id.trajectory_id, constant_data->time, insertion_submaps);
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  const SubmapId matching_id = submap_ids.front();
  const transform::Rigid3d& local_pose = constant_data->local_pose;
  const transform::Rigid3d global_pose =
      optimization_problem_->submap_data().at(matching_id).global_pose *
      insertion_submaps.front()->local_pose().inverse() * local_pose;
  optimization_problem_->AddTrajectoryNode(
      matching_id.trajectory_id,
      optimization::NodeSpec3D{constant_data->time, local_pose, global_pose});
  for (size_t i = 0; i < insertion_submaps.size(); ++i) {
    const SubmapId submap_id = submap_ids[i];
    // Even if this was the last node added to 'submap_id', the submap will only
    // be marked as finished in 'submap_data_' further below.
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    const transform::Rigid3d constraint_transform =
        insertion_submaps[i]->local_pose().inverse() * local_pose;
    constraints_.push_back(
        Constraint{submap_id,
                   node_id,
                   {constraint_transform, options_.matcher_translation_weight(),
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
    const SubmapId finished_submap_id = submap_ids.front();
    InternalSubmapData& finished_submap_data =
        submap_data_.at(finished_submap_id);
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old nodes.
    ComputeConstraintsForOldNodes(finished_submap_id);
  }
  constraint_builder_.NotifyEndOfNode();
  ++num_nodes_since_last_loop_closure_;
  CHECK(!run_loop_closure_);
  if (options_.optimize_every_n_nodes() > 0 &&
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
    DispatchOptimization();
  }
}

void PoseGraph3D::DispatchOptimization() {
  run_loop_closure_ = true;
  // If there is a 'work_queue_' already, some other thread will take care.
  if (work_queue_ == nullptr) {
    work_queue_ = common::make_unique<std::deque<std::function<void()>>>();
    constraint_builder_.WhenDone(
        std::bind(&PoseGraph3D::HandleWorkQueue, this, std::placeholders::_1));
  }
}

common::Time PoseGraph3D::GetLatestNodeTime(const NodeId& node_id,
                                            const SubmapId& submap_id) const {
  common::Time time = trajectory_nodes_.at(node_id).constant_data->time;
  const InternalSubmapData& submap_data = submap_data_.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    const NodeId last_submap_node_id =
        *submap_data_.at(submap_id).node_ids.rbegin();
    time = std::max(
        time, trajectory_nodes_.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void PoseGraph3D::UpdateTrajectoryConnectivity(const Constraint& constraint) {
  CHECK_EQ(constraint.tag, PoseGraphInterface::Constraint::INTER_SUBMAP);
  const common::Time time =
      GetLatestNodeTime(constraint.node_id, constraint.submap_id);
  trajectory_connectivity_state_.Connect(constraint.node_id.trajectory_id,
                                         constraint.submap_id.trajectory_id,
                                         time);
}

void PoseGraph3D::HandleWorkQueue(
    const constraints::ConstraintBuilder3D::Result& result) {
  {
    common::MutexLocker locker(&mutex_);
    constraints_.insert(constraints_.end(), result.begin(), result.end());
  }
  RunOptimization();

  if (global_slam_optimization_callback_) {
    std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
    std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
    {
      common::MutexLocker locker(&mutex_);
      const auto& submap_data = optimization_problem_->submap_data();
      const auto& node_data = optimization_problem_->node_data();
      for (const int trajectory_id : node_data.trajectory_ids()) {
        trajectory_id_to_last_optimized_node_id[trajectory_id] =
            std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
        trajectory_id_to_last_optimized_submap_id[trajectory_id] =
            std::prev(submap_data.EndOfTrajectory(trajectory_id))->id;
      }
    }
    global_slam_optimization_callback_(
        trajectory_id_to_last_optimized_submap_id,
        trajectory_id_to_last_optimized_node_id);
  }

  common::MutexLocker locker(&mutex_);
  for (const Constraint& constraint : result) {
    UpdateTrajectoryConnectivity(constraint);
  }
  TrimmingHandle trimming_handle(this);
  for (auto& trimmer : trimmers_) {
    trimmer->Trim(&trimming_handle);
  }
  trimmers_.erase(
      std::remove_if(trimmers_.begin(), trimmers_.end(),
                     [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                       return trimmer->IsFinished();
                     }),
      trimmers_.end());

  num_nodes_since_last_loop_closure_ = 0;
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
  constraint_builder_.WhenDone(
      std::bind(&PoseGraph3D::HandleWorkQueue, this, std::placeholders::_1));
}

void PoseGraph3D::WaitForAllComputations() {
  bool notification = false;
  common::MutexLocker locker(&mutex_);
  const int num_finished_nodes_at_start =
      constraint_builder_.GetNumFinishedNodes();
  while (!locker.AwaitWithTimeout(
      [this]() REQUIRES(mutex_) {
        return ((constraint_builder_.GetNumFinishedNodes() ==
                 num_trajectory_nodes_) &&
                !work_queue_);
      },
      common::FromSeconds(1.))) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. *
                         (constraint_builder_.GetNumFinishedNodes() -
                          num_finished_nodes_at_start) /
                         (num_trajectory_nodes_ - num_finished_nodes_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone(
      [this,
       &notification](const constraints::ConstraintBuilder3D::Result& result) {
        common::MutexLocker locker(&mutex_);
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        notification = true;
      });
  locker.Await([&notification]() { return notification; });
}

void PoseGraph3D::FinishTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(finished_trajectories_.count(trajectory_id), 0);
    finished_trajectories_.insert(trajectory_id);

    for (const auto& submap : submap_data_.trajectory(trajectory_id)) {
      submap_data_.at(submap.id).state = SubmapState::kFinished;
    }
    CHECK(!run_loop_closure_);
    DispatchOptimization();
  });
}

bool PoseGraph3D::IsTrajectoryFinished(const int trajectory_id) const {
  return finished_trajectories_.count(trajectory_id) > 0;
}

void PoseGraph3D::FreezeTrajectory(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  trajectory_connectivity_state_.Add(trajectory_id);
  AddWorkItem([this, trajectory_id]() REQUIRES(mutex_) {
    CHECK_EQ(frozen_trajectories_.count(trajectory_id), 0);
    frozen_trajectories_.insert(trajectory_id);
  });
}

bool PoseGraph3D::IsTrajectoryFrozen(const int trajectory_id) const {
  return frozen_trajectories_.count(trajectory_id) > 0;
}

void PoseGraph3D::AddSubmapFromProto(
    const transform::Rigid3d& global_submap_pose, const proto::Submap& submap) {
  if (!submap.has_submap_3d()) {
    return;
  }

  const SubmapId submap_id = {submap.submap_id().trajectory_id(),
                              submap.submap_id().submap_index()};
  std::shared_ptr<const Submap3D> submap_ptr =
      std::make_shared<const Submap3D>(submap.submap_3d());

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(submap_id.trajectory_id);
  submap_data_.Insert(submap_id, InternalSubmapData());
  submap_data_.at(submap_id).submap = submap_ptr;
  // Immediately show the submap at the 'global_submap_pose'.
  global_submap_poses_.Insert(submap_id,
                              optimization::SubmapSpec3D{global_submap_pose});
  AddWorkItem([this, submap_id, global_submap_pose]() REQUIRES(mutex_) {
    submap_data_.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_->InsertSubmap(submap_id, global_submap_pose);
  });
}

void PoseGraph3D::AddNodeFromProto(const transform::Rigid3d& global_pose,
                                   const proto::Node& node) {
  const NodeId node_id = {node.node_id().trajectory_id(),
                          node.node_id().node_index()};
  std::shared_ptr<const TrajectoryNode::Data> constant_data =
      std::make_shared<const TrajectoryNode::Data>(FromProto(node.node_data()));

  common::MutexLocker locker(&mutex_);
  AddTrajectoryIfNeeded(node_id.trajectory_id);
  trajectory_nodes_.Insert(node_id, TrajectoryNode{constant_data, global_pose});

  AddWorkItem([this, node_id, global_pose]() REQUIRES(mutex_) {
    const auto& constant_data = trajectory_nodes_.at(node_id).constant_data;
    optimization_problem_->InsertTrajectoryNode(
        node_id,
        optimization::NodeSpec3D{constant_data->time, constant_data->local_pose,
                                 global_pose});
  });
}

void PoseGraph3D::SetTrajectoryDataFromProto(
    const proto::TrajectoryData& data) {
  TrajectoryData trajectory_data;
  trajectory_data.gravity_constant = data.gravity_constant();
  trajectory_data.imu_calibration = {
      {data.imu_calibration().w(), data.imu_calibration().x(),
       data.imu_calibration().y(), data.imu_calibration().z()}};
  if (data.has_fixed_frame_origin_in_map()) {
    trajectory_data.fixed_frame_origin_in_map =
        transform::ToRigid3(data.fixed_frame_origin_in_map());
  }

  const int trajectory_id = data.trajectory_id();
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, trajectory_id, trajectory_data]() REQUIRES(mutex_) {
    optimization_problem_->SetTrajectoryData(trajectory_id, trajectory_data);
  });
}

void PoseGraph3D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, node_id, submap_id]() REQUIRES(mutex_) {
    submap_data_.at(submap_id).node_ids.insert(node_id);
  });
}

void PoseGraph3D::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([this, constraints]() REQUIRES(mutex_) {
    for (const auto& constraint : constraints) {
      CHECK(trajectory_nodes_.Contains(constraint.node_id));
      CHECK(submap_data_.Contains(constraint.submap_id));
      CHECK(trajectory_nodes_.at(constraint.node_id).constant_data != nullptr);
      CHECK(submap_data_.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(submap_data_.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          UpdateTrajectoryConnectivity(constraint);
          break;
      }
      constraints_.push_back(constraint);
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
  });
}

void PoseGraph3D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
  common::MutexLocker locker(&mutex_);
  // C++11 does not allow us to move a unique_ptr into a lambda.
  PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  AddWorkItem([this, trimmer_ptr]()
                  REQUIRES(mutex_) { trimmers_.emplace_back(trimmer_ptr); });
}

void PoseGraph3D::RunFinalOptimization() {
  {
    common::MutexLocker locker(&mutex_);
    AddWorkItem([this]() REQUIRES(mutex_) {
      optimization_problem_->SetMaxNumIterations(
          options_.max_num_final_iterations());
      DispatchOptimization();
    });
    AddWorkItem([this]() REQUIRES(mutex_) {
      optimization_problem_->SetMaxNumIterations(
          options_.optimization_problem_options()
              .ceres_solver_options()
              .max_num_iterations());
    });
  }
  WaitForAllComputations();
}

void PoseGraph3D::LogResidualHistograms() const {
  common::Histogram rotational_residual;
  common::Histogram translational_residual;
  for (const Constraint& constraint : constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
      const cartographer::transform::Rigid3d optimized_node_to_map =
          trajectory_nodes_.at(constraint.node_id).global_pose;
      const cartographer::transform::Rigid3d node_to_submap_constraint =
          constraint.pose.zbar_ij;
      const cartographer::transform::Rigid3d optimized_submap_to_map =
          global_submap_poses_.at(constraint.submap_id).global_pose;
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

void PoseGraph3D::RunOptimization() {
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_, constraints_,
  // frozen_trajectories_ and landmark_nodes_ when executing the Solve. Solve is
  // time consuming, so not taking the mutex before Solve to avoid blocking
  // foreground processing.
  optimization_problem_->Solve(constraints_, frozen_trajectories_,
                               landmark_nodes_);
  common::MutexLocker locker(&mutex_);

  const auto& submap_data = optimization_problem_->submap_data();
  const auto& node_data = optimization_problem_->node_data();
  for (const int trajectory_id : node_data.trajectory_ids()) {
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      trajectory_nodes_.at(node.id).global_pose = node.data.global_pose;
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global =
        ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    auto node_it = std::next(trajectory_nodes_.find(last_optimized_node_id));
    for (; node_it != trajectory_nodes_.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = trajectory_nodes_.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }
  for (const auto& landmark : optimization_problem_->landmark_data()) {
    landmark_nodes_[landmark.first].global_landmark_pose = landmark.second;
  }
  global_submap_poses_ = submap_data;

  // Log the histograms for the pose residuals.
  if (options_.log_residual_histograms()) {
    LogResidualHistograms();
  }
}

MapById<NodeId, TrajectoryNode> PoseGraph3D::GetTrajectoryNodes() const {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_;
}

MapById<NodeId, TrajectoryNodePose> PoseGraph3D::GetTrajectoryNodePoses()
    const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  common::MutexLocker locker(&mutex_);
  for (const auto& node_id_data : trajectory_nodes_) {
    common::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<std::string, transform::Rigid3d> PoseGraph3D::GetLandmarkPoses()
    const {
  std::map<std::string, transform::Rigid3d> landmark_poses;
  common::MutexLocker locker(&mutex_);
  for (const auto& landmark : landmark_nodes_) {
    // Landmark without value has not been optimized yet.
    if (!landmark.second.global_landmark_pose.has_value()) continue;
    landmark_poses[landmark.first] =
        landmark.second.global_landmark_pose.value();
  }
  return landmark_poses;
}

void PoseGraph3D::SetLandmarkPose(const std::string& landmark_id,
                                  const transform::Rigid3d& global_pose) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    landmark_nodes_[landmark_id].global_landmark_pose = global_pose;
  });
}

sensor::MapByTime<sensor::ImuData> PoseGraph3D::GetImuData() const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->imu_data();
}

sensor::MapByTime<sensor::OdometryData> PoseGraph3D::GetOdometryData() const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->odometry_data();
}

sensor::MapByTime<sensor::FixedFramePoseData>
PoseGraph3D::GetFixedFramePoseData() const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->fixed_frame_pose_data();
}

std::map<std::string /* landmark ID */, PoseGraphInterface::LandmarkNode>
PoseGraph3D::GetLandmarkNodes() const {
  common::MutexLocker locker(&mutex_);
  return landmark_nodes_;
}

std::map<int, PoseGraphInterface::TrajectoryData>
PoseGraph3D::GetTrajectoryData() const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->trajectory_data();
}

std::vector<PoseGraphInterface::Constraint> PoseGraph3D::constraints() const {
  common::MutexLocker locker(&mutex_);
  return constraints_;
}

void PoseGraph3D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                           const int to_trajectory_id,
                                           const transform::Rigid3d& pose,
                                           const common::Time time) {
  common::MutexLocker locker(&mutex_);
  initial_trajectory_poses_[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

transform::Rigid3d PoseGraph3D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(trajectory_nodes_.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = trajectory_nodes_.lower_bound(trajectory_id, time);
  if (it == trajectory_nodes_.BeginOfTrajectory(trajectory_id)) {
    return trajectory_nodes_.BeginOfTrajectory(trajectory_id)->data.global_pose;
  }
  if (it == trajectory_nodes_.EndOfTrajectory(trajectory_id)) {
    return std::prev(trajectory_nodes_.EndOfTrajectory(trajectory_id))
        ->data.global_pose;
  }
  return transform::Interpolate(
             transform::TimestampedTransform{std::prev(it)->data.time(),
                                             std::prev(it)->data.global_pose},
             transform::TimestampedTransform{it->data.time(),
                                             it->data.global_pose},
             time)
      .transform;
}

transform::Rigid3d PoseGraph3D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  common::MutexLocker locker(&mutex_);
  return ComputeLocalToGlobalTransform(global_submap_poses_, trajectory_id);
}

std::vector<std::vector<int>> PoseGraph3D::GetConnectedTrajectories() const {
  return trajectory_connectivity_state_.Components();
}

PoseGraphInterface::SubmapData PoseGraph3D::GetSubmapData(
    const SubmapId& submap_id) const {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph3D::GetAllSubmapData() const {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, PoseGraphInterface::SubmapPose>
PoseGraph3D::GetAllSubmapPoses() const {
  common::MutexLocker locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : submap_data_) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        PoseGraphInterface::SubmapPose{submap_data.submap->num_range_data(),
                                       submap_data.pose});
  }
  return submap_poses;
}

transform::Rigid3d PoseGraph3D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec3D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    const auto it = initial_trajectory_poses_.find(trajectory_id);
    if (it != initial_trajectory_poses_.end()) {
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    } else {
      return transform::Rigid3d::Identity();
    }
  }
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  return global_submap_poses.at(last_optimized_submap_id).global_pose *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

PoseGraphInterface::SubmapData PoseGraph3D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = submap_data_.find(submap_id);
  if (it == submap_data_.end()) {
    return {};
  }
  auto submap = it->data.submap;
  if (global_submap_poses_.Contains(submap_id)) {
    // We already have an optimized pose.
    return {submap, global_submap_poses_.at(submap_id).global_pose};
  }
  // We have to extrapolate.
  return {submap, ComputeLocalToGlobalTransform(global_submap_poses_,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

PoseGraph3D::TrimmingHandle::TrimmingHandle(PoseGraph3D* const parent)
    : parent_(parent) {}

int PoseGraph3D::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

std::vector<SubmapId> PoseGraph3D::TrimmingHandle::GetSubmapIds(
    int trajectory_id) const {
  std::vector<SubmapId> submap_ids;
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  for (const auto& it : submap_data.trajectory(trajectory_id)) {
    submap_ids.push_back(it.id);
  }
  return submap_ids;
}
MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph3D::TrimmingHandle::GetOptimizedSubmapData() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : parent_->submap_data_) {
    if (submap_id_data.data.state != SubmapState::kFinished ||
        !parent_->global_submap_poses_.Contains(submap_id_data.id)) {
      continue;
    }
    submaps.Insert(
        submap_id_data.id,
        SubmapData{
            submap_id_data.data.submap,
            parent_->global_submap_poses_.at(submap_id_data.id).global_pose});
  }
  return submaps;
}

const MapById<NodeId, TrajectoryNode>&
PoseGraph3D::TrimmingHandle::GetTrajectoryNodes() const {
  return parent_->trajectory_nodes_;
}

const std::vector<PoseGraphInterface::Constraint>&
PoseGraph3D::TrimmingHandle::GetConstraints() const {
  return parent_->constraints_;
}

bool PoseGraph3D::TrimmingHandle::IsFinished(const int trajectory_id) const {
  return parent_->IsTrajectoryFinished(trajectory_id);
}

void PoseGraph3D::TrimmingHandle::MarkSubmapAsTrimmed(
    const SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->submap_data_.at(submap_id).state == SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained once the submap
  // with 'submap_id' is gone.
  std::set<NodeId> nodes_to_retain;
  for (const Constraint& constraint : parent_->constraints_) {
    if (constraint.tag == Constraint::Tag::INTRA_SUBMAP &&
        constraint.submap_id != submap_id) {
      nodes_to_retain.insert(constraint.node_id);
    }
  }
  // Remove all 'constraints_' related to 'submap_id'.
  std::set<NodeId> nodes_to_remove;
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
  parent_->optimization_problem_->TrimSubmap(submap_id);

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  for (const NodeId& node_id : nodes_to_remove) {
    parent_->trajectory_nodes_.Trim(node_id);
    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
}

MapById<SubmapId, PoseGraphInterface::SubmapData>
PoseGraph3D::GetSubmapDataUnderLock() const {
  MapById<SubmapId, PoseGraphInterface::SubmapData> submaps;
  for (const auto& submap_id_data : submap_data_) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

void PoseGraph3D::SetGlobalSlamOptimizationCallback(
    PoseGraphInterface::GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

}  // namespace mapping
}  // namespace cartographer
